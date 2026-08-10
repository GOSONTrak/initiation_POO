#pragma once
#include <Arduino.h>
#include <math.h>
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  MPC.hpp — Contrôle Prédictif par Tir Aléatoire
//
//  Principe (Random Shooting MPC) :
//  ┌─────────────────────────────────────────────────────────┐
//  │ À chaque cycle (10 Hz) :                                │
//  │  1. Tire N_SAMPLES séquences aléatoires de N_HORIZON   │
//  │     commandes {l, r} dans une table d'actions discrètes │
//  │  2. Simule la cinématique différentielle pour chaque    │
//  │     séquence → états futurs s_0 … s_N                  │
//  │  3. Calcule le coût cumulé (escompté) de chaque chemin  │
//  │  4. Applique la PREMIÈRE commande du meilleur chemin    │
//  └─────────────────────────────────────────────────────────┘
//
//  Fonctionnalités intégrées (numérotées dans le code) :
//
//   Feature 1 — Contrainte bord arène
//     Pénalité quadratique dès que l'EKF prédit que le robot
//     dépasse EDGE_WARN_MM du centre. Mur dur à DOHYO_R_MM.
//     → le robot évite le bord AVANT que les capteurs de ligne
//       ne réagissent (prédiction vs réaction).
//
//   Feature 2 — Leading shot (anticipation adversaire)
//     La position-cible n'est PAS la position lidar courante
//     mais la position PRÉDITE de l'adversaire à LEAD_S dans
//     le futur, estimée par filtre EWMA sur la dérivée lidar.
//     → vise où il SERA, pas où il EST.
//
//   Feature 3 — Recherche optimale (mode SEARCH)
//     En absence de cible le MPC maximise la vitesse angulaire
//     tout en minimisant le déplacement translationnel.
//     → pivotement rapide sur place ; couvre 360° plus vite
//       que le WAIT/ROTATE à pas fixes de SeekStrategy.
//
//   Feature 4 — Impact radial optimal
//     La poussée la plus efficace est radiale (centre→bord).
//     Un terme aligne le cap robot sur le vecteur centre→cible,
//     ajoutant un angle de frappe perpendiculaire au bord.
//     → maximise la force appliquée HORS du dohyo.
//
//  Budget CPU :
//   N_SAMPLES=60, N_HORIZON=5 → 300 simulations × ~25 flops
//   ≈ 2–3 ms sur ESP32 @ 240 MHz. Largement dans 100 ms/cycle.
//
//  Limitation connue :
//   L'EKF a pour origine la position de démarrage du robot.
//   Pour que Feature 1 soit précise, reset l'EKF avec le robot
//   AU CENTRE du dohyo → (0,0) = centre → seuil 300mm valide.
//   En démarrage bord-à-bord standard, la pénalité reste utile
//   comme heuristique de stabilisation de trajectoire.
// ═══════════════════════════════════════════════════════════════

class MPCStrategy : public StrategyInterface {
public:

    const char* name()       const override { return "MPC"; }
    LedColor    color()      const override { return LedColor::CYAN; }
    bool        needsLidar() const override { return true; }

    // ── reset() : appelé à chaque activation de la stratégie ─
    void reset() override {
        _hasTarget = false;
        _oppX = _oppY = 0.0f;
        _oppVx = _oppVy = 0.0f;
        _prevOppX = _prevOppY = 0.0f;
        _lastOppMs = 0;
        _graceEnd = 0;
        _rng = 87654321UL ^ (uint32_t)millis();
    }

    // ─────────────────────────────────────────────────────────
    //  execute() — point d'entrée appelé à 10 Hz par taskStrategy
    // ─────────────────────────────────────────────────────────
    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;

        // ── Table des actions discrètes [PWM_gauche, PWM_droit] ──
        // static constexpr dans la fonction = valide C++14, pas d'ODR
        struct Ctrl { int l, r; };
        static constexpr Ctrl ACTIONS[] = {
            { 220,  220},   // avance droit (modéré)
            { 255,  255},   // charge pleine
            { 150,  220},   // courbe gauche légère
            { 220,  150},   // courbe droite légère
            {  70,  220},   // courbe gauche prononcée
            { 220,   70},   // courbe droite prononcée
            {-150,  150},   // pivotement gauche (recherche)
            { 150, -150},   // pivotement droite (recherche)
            { 255,  170},   // sprint + correction droite
            { 170,  255},   // sprint + correction gauche
            {   0,    0},   // stop (évasion bord)
            { 180,  180},   // avance souple
        };
        static constexpr int N_ACT = (int)(sizeof(ACTIONS) / sizeof(ACTIONS[0]));

        // ── Lecture capteurs ──────────────────────────────────
        const EKF::Pose  pose  = ctx.getPose();
        const SensorData lidar = ctx.getLidarData();
        const SensorData tofFL = ctx.getTOFData(SensorPosition::FRONT_LEFT);
        const SensorData tofFR = ctx.getTOFData(SensorPosition::FRONT_RIGHT);

        // ── Mise à jour position + vitesse adversaire ─────────
        _updateTarget(pose, lidar, tofFL, tofFR);

        // ── Mode courant ──────────────────────────────────────
        const bool attacking = _hasTarget;
        ctx.setState(attacking ? RobotConstants::State::ATTACK
                               : RobotConstants::State::SEARCH);

        // ── Feature 2 : position prédite de l'adversaire ─────
        // On extrapole la position à LEAD_S dans le futur via la
        // vitesse estimée → vise où il sera, pas où il est.
        const float predOppX = _oppX + _oppVx * LEAD_S;
        const float predOppY = _oppY + _oppVy * LEAD_S;

        // ── Random Shooting MPC ───────────────────────────────
        struct State3 { float x, y, theta; };
        const State3 s0 = {pose.x, pose.y, pose.theta};

        float bestCost = 1e9f;
        Ctrl  bestFirst = {0, 0};

        for (int trial = 0; trial < N_SAMPLES; ++trial) {
            State3 s    = s0;
            float  cost = 0.0f;
            float  disc = 1.0f;   // facteur d'escompte temporel
            Ctrl   first = {0, 0};

            for (int k = 0; k < N_HORIZON; ++k) {
                // Action tirée aléatoirement dans la table
                const Ctrl& ctrl = ACTIONS[_rand() % N_ACT];
                if (k == 0) first = ctrl;

                // Simulation cinématique différentielle (dt = 100 ms)
                {
                    const float vL = (float)ctrl.l / 255.0f * MAX_V_MMS;
                    const float vR = (float)ctrl.r / 255.0f * MAX_V_MMS;
                    const float v  = (vL + vR) * 0.5f;
                    const float w  = (vR - vL) / WHEELBASE;
                    const float mt = s.theta + w * DT * 0.5f;  // angle milieu-pas
                    s = {
                        s.x     + v * cosf(mt) * DT,
                        s.y     + v * sinf(mt) * DT,
                        _wrap(s.theta + w * DT)
                    };
                }

                // ──────────────────────────────────────────────
                //  Feature 1 — Pénalité bord arène
                // ──────────────────────────────────────────────
                {
                    const float d    = sqrtf(s.x*s.x + s.y*s.y);
                    const float over = d - EDGE_WARN_MM;
                    if (over > 0.0f)
                        cost += disc * W_EDGE * over * over;
                    if (d > DOHYO_R_MM)
                        cost += disc * 50000.0f;  // mur dur : jamais sélectionné
                }

                if (attacking) {
                    // ──────────────────────────────────────────
                    //  Feature 2 — Cap vers position prédite
                    // ──────────────────────────────────────────
                    {
                        const float dx  = predOppX - s.x;
                        const float dy  = predOppY - s.y;
                        const float des = atan2f(dy, dx);
                        const float err = _wrap(des - s.theta);
                        cost += disc * W_HEADING * err * err;
                    }

                    // ──────────────────────────────────────────
                    //  Feature 4 — Alignement impact radial
                    //  Idéal : robot head = vecteur centre→cible
                    //  → pousse l'adversaire vers l'extérieur
                    // ──────────────────────────────────────────
                    {
                        const float oppDir = atan2f(predOppY, predOppX);
                        const float impErr = _wrap(s.theta - oppDir);
                        cost += disc * W_IMPACT * impErr * impErr;
                    }

                } else {
                    // ──────────────────────────────────────────
                    //  Feature 3 — Maximise balayage angulaire
                    //  Négatif : minimiser = maximiser la rotation
                    // ──────────────────────────────────────────
                    {
                        const float dTheta = fabsf(_wrap(s.theta - s0.theta));
                        cost -= disc * W_SWEEP * dTheta;

                        // Pénalise déplacement translationnel
                        // (garde le robot vers le centre pendant la recherche)
                        const float dx = s.x - s0.x;
                        const float dy = s.y - s0.y;
                        cost += disc * W_DRIFT * (dx*dx + dy*dy) * 1e-3f;
                    }
                }

                disc *= GAMMA;
            }

            if (cost < bestCost) {
                bestCost  = cost;
                bestFirst = first;
            }
        }

        return AC{bestFirst.l, bestFirst.r};
    }

private:

    // ── Paramètres physiques ──────────────────────────────────
    static constexpr float MAX_V_MMS = 350.0f;    // vitesse max à PWM=255 (mm/s)
    static constexpr float DT        = 0.10f;     // durée d'un pas de simulation (s)
    static constexpr float WHEELBASE = RobotConstants::WHEELBASE_MM;

    // ── Optimiseur ────────────────────────────────────────────
    static constexpr int   N_SAMPLES  = 60;    // séquences évaluées par cycle
    static constexpr int   N_HORIZON  = 5;     // pas d'anticipation (= 500 ms)
    static constexpr float GAMMA      = 0.85f; // escompte temporel

    // ── Feature 1 : bord arène ────────────────────────────────
    static constexpr float EDGE_WARN_MM = 300.0f;  // alerte à 85mm du bord
    static constexpr float DOHYO_R_MM   = 385.0f;  // rayon dohyo (mm)
    static constexpr float W_EDGE       = 6.0f;

    // ── Feature 2 : leading shot ──────────────────────────────
    static constexpr float LEAD_S       = 0.25f;   // horizon anticipation (s)
    static constexpr float OPP_ALPHA    = 0.4f;    // EWMA vitesse adversaire
    static constexpr float OPP_VEL_MAX  = 500.0f;  // clamp vitesse (mm/s)
    static constexpr float MIN_DIST_M   = 0.12f;   // portée min lidar (m)
    static constexpr float MAX_DIST_M   = 0.80f;   // portée max lidar (m)
    static constexpr float FRONT_DEG    = 10.0f;   // offset avant lidar (degrés)
    static constexpr float LOCK_GRACE_MS = 400.0f; // grace period cible perdue (ms)

    // ── Feature 2 + 4 : attaque ───────────────────────────────
    static constexpr float W_HEADING = 2.0f;   // poids erreur de cap
    static constexpr float W_IMPACT  = 1.0f;   // poids alignement radial

    // ── Feature 3 : recherche ─────────────────────────────────
    static constexpr float W_SWEEP = 0.8f;  // poids rotation totale
    static constexpr float W_DRIFT = 0.3f;  // poids déplacement pendant recherche

    // ── État interne ──────────────────────────────────────────
    bool     _hasTarget = false;
    float    _oppX = 0.0f,     _oppY = 0.0f;     // position adversaire (mm, repère EKF)
    float    _oppVx = 0.0f,    _oppVy = 0.0f;    // vitesse estimée (mm/s)
    float    _prevOppX = 0.0f, _prevOppY = 0.0f;
    uint32_t _lastOppMs = 0;
    uint32_t _graceEnd  = 0;
    uint32_t _rng = 87654321UL;   // graine PRNG (xorshift32)

    // ─────────────────────────────────────────────────────────
    //  _updateTarget — fusionne lidar + TOF pour estimer la
    //  position adversaire et sa vitesse (Features 2 + 4)
    // ─────────────────────────────────────────────────────────
    void _updateTarget(const EKF::Pose& pose,
                       const SensorData& lidar,
                       const SensorData& tofFL,
                       const SensorData& tofFR) {
        float    newX = 0.0f, newY = 0.0f;
        bool     found = false;

        // ── Lidar (longue portée) ─────────────────────────────
        if (lidar.isValid) {
            const float dist  = lidar.value.vector.x;   // mètres
            const float angle = lidar.value.vector.y;   // degrés

            if (dist >= MIN_DIST_M && dist <= MAX_DIST_M) {
                // Conversion repère lidar → repère monde EKF.
                // Convention YDLIDAR : angle croissant = CW.
                // FRONT_DEG = axe avant du robot dans le repère lidar.
                const float wAngle = pose.theta
                    - (angle - FRONT_DEG) * (float)M_PI / 180.0f;
                const float distMm = dist * 1000.0f;
                newX  = pose.x + distMm * cosf(wAngle);
                newY  = pose.y + distMm * sinf(wAngle);
                found = true;
            }
        }

        // ── TOF fallback (zone morte lidar ou cible très proche) ─
        if (!found) {
            const bool fl = tofFL.isValid
                         && tofFL.value.scalar >  80.0f
                         && tofFL.value.scalar < 280.0f;
            const bool fr = tofFR.isValid
                         && tofFR.value.scalar >  80.0f
                         && tofFR.value.scalar < 280.0f;

            if (fl || fr) {
                // Angle estimé dans le repère robot
                const float angOff = (fl && fr) ?  0.0f
                                   : fl          ? -20.0f * (float)M_PI / 180.0f
                                                 :  20.0f * (float)M_PI / 180.0f;
                const float distMm = (fl && fr)
                    ? (tofFL.value.scalar + tofFR.value.scalar) * 0.5f
                    : (fl ? tofFL.value.scalar : tofFR.value.scalar);
                const float wAngle = pose.theta + angOff;
                newX  = pose.x + distMm * cosf(wAngle);
                newY  = pose.y + distMm * sinf(wAngle);
                found = true;
            }
        }

        if (found) {
            // ── Estimation vitesse adversaire (EWMA) ─────────
            // Feature 2 : différence finie filtrée pour "leading"
            const uint32_t now = millis();
            if (_lastOppMs != 0) {
                const float dt = (float)(now - _lastOppMs) * 0.001f;
                if (dt > 0.02f && dt < 0.5f) {
                    const float rvx = constrain(
                        (newX - _prevOppX) / dt, -OPP_VEL_MAX, OPP_VEL_MAX);
                    const float rvy = constrain(
                        (newY - _prevOppY) / dt, -OPP_VEL_MAX, OPP_VEL_MAX);
                    _oppVx = OPP_ALPHA * rvx + (1.0f - OPP_ALPHA) * _oppVx;
                    _oppVy = OPP_ALPHA * rvy + (1.0f - OPP_ALPHA) * _oppVy;
                }
            }
            _prevOppX  = newX;
            _prevOppY  = newY;
            _lastOppMs = now;
            _oppX      = newX;
            _oppY      = newY;
            _hasTarget = true;
            _graceEnd  = now + (uint32_t)LOCK_GRACE_MS;

        } else {
            // Cible perdue — grace period avant de passer en SEARCH
            if (millis() >= _graceEnd) {
                _hasTarget = false;
                _oppVx = _oppVy = 0.0f;  // reset vitesse : donnée trop vieille
            }
        }
    }

    // ── PRNG xorshift32 (déterministe, ~0 overhead) ──────────
    uint32_t _rand() {
        _rng ^= _rng << 13;
        _rng ^= _rng >> 17;
        _rng ^= _rng << 5;
        return _rng;
    }

    // ── Normalisation angle ∈ ]-π, π] ────────────────────────
    static float _wrap(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }
};

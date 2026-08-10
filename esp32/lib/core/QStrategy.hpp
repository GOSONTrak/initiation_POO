#pragma once
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  QStrategy.hpp — Stratégie par renforcement (Q-learning discret)
//
//  Principe :
//  - Table Q[8 états][4 actions] — valeur apprise de chaque action
//  - Mise à jour Bellman : Q[s][a] += α × (r + γ·max Q[s'] - Q[s][a])
//  - Sélection ε-greedy : explore (random) ou exploite (argmax)
//  - ε décroit de 0.85 → 0.10 sur la durée des matchs
//  - Q-table en RAM uniquement (reset au reboot)
//
//  États (priorité décroissante) :
//  0  BORD        : capteur de ligne actif (danger)
//  1  DOS_ENNEMI  : lidar valide, angle ~0° (ennemi derrière)
//  2  FLANC_GAUCHE: lidar/TOF, ennemi à gauche
//  3  FLANC_DROIT : lidar/TOF, ennemi à droite
//  4  ENNEMI_PRES : lidar aligné < 25cm ou double TOF
//  5  ENNEMI_LOIN : lidar aligné 25–67cm
//  6  CONTACT_TOF : double TOF < 28cm (zone morte lidar)
//  7  RIEN        : aucun signal
//
//  Actions :
//  0  CHARGE       : (255, 255)  — avancer pleine puissance
//  1  RETRAITE     : (-200,-200) — reculer
//  2  PIVOT_GAUCHE : (180,-180)  — tourner gauche sur place
//  3  PIVOT_DROITE : (-180, 180) — tourner droite sur place
// ═══════════════════════════════════════════════════════════════

class QStrategy : public StrategyInterface {
public:
    const char* name()       const override { return "Q"; }
    LedColor    color()      const override { return LedColor::BLUE; }
    bool        needsLidar() const override { return true; }
    bool        needsLQR()   const override { return false; }

    void reset() override {
        _firstStep  = true;
        _prevState  = 7;
        _prevAction = 0;
        _stepCount  = 0;
        _evadeUntil = 0;
        _episodeCount++;
        LOGF("[Q] session %u  eps=%.3f\n", _episodeCount, _epsilon);
    }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;
        uint32_t now = millis();

        // ── 0. EVADE verrouillé — priorité absolue sur Q ──────────
        // Q-learning ne peut pas choisir "fonce dans le bord".
        // La pénalité -5 est quand même apprise pour que Q évite l'état 0.
        if (now < _evadeUntil) {
            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // ── 1. État courant ───────────────────────────────────────
        uint8_t s = _computeState(ctx);

        // ── 1b. Border détecté → armer l'évitement et punir Q ────
        if (s == 0) {
            auto lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
            auto lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);
            auto lineB  = ctx.getLineData(SensorPosition::BACK);
            bool fl = lineFL.isValid && lineFL.value.scalar > 0.0f;
            bool fr = lineFR.isValid && lineFR.value.scalar > 0.0f;
            bool b  = lineB.isValid  && lineB.value.scalar  > 0.0f;

            if (b) {
                _evadeCmd   = AC{  220,  220 };   // bord arrière → avancer
            } else if (fl && fr) {
                _evadeCmd   = AC{ -200, -200 };   // bord avant les deux → reculer droit
            } else if (fl) {
                _evadeCmd   = AC{ -200, -140 };   // bord avant gauche → reculer en virant droite
            } else {
                _evadeCmd   = AC{ -140, -200 };   // bord avant droite → reculer en virant gauche
            }
            _evadeUntil = now + EVADE_MS;

            // Mettre à jour Q avec pénalité même pendant l'évitement forcé
            if (!_firstStep) {
                _updateQ(_prevState, _prevAction, -5.0f, s);
            }
            _firstStep  = false;
            _prevState  = s;
            _prevAction = 1;  // assimile à RETRAITE pour la prochaine mise à jour

            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // ── 2. Mise à jour Bellman (pas le premier cycle) ─────────
        if (!_firstStep) {
            float r = _computeReward(s);
            _updateQ(_prevState, _prevAction, r, s);
        }
        _firstStep = false;

        // ── 3. Sélection ε-greedy ─────────────────────────────────
        uint8_t a;
        if (_randFloat() < _epsilon) {
            a = (uint8_t)(esp_random() % N_ACTIONS);
        } else {
            a = _argmax(s);
        }

        // ── 4. Décroissance ε ─────────────────────────────────────
        if (_epsilon > EPSILON_MIN)
            _epsilon *= EPSILON_DECAY;

        // ── 5. Log périodique ─────────────────────────────────────
        if (++_stepCount % LOG_EVERY == 0)
            LOGF("[Q] s=%u a=%u eps=%.3f Q=%.2f\n", s, a, _epsilon, _q[s][a]);

        // ── 6. Mémoriser pour le prochain cycle ───────────────────
        _prevState  = s;
        _prevAction = a;

        // ── 7. Exécuter ───────────────────────────────────────────
        ctx.setState(s == 0 ? RobotConstants::State::EVADE  :
                     s == 7 ? RobotConstants::State::SEARCH :
                               RobotConstants::State::ATTACK);
        switch (a) {
            case 1:  return {-200, -200};
            case 2:  return { 180, -180};
            case 3:  return {-180,  180};
            default: return { 255,  255};
        }
    }

private:
    static constexpr uint8_t  N_STATES      = 8;
    static constexpr uint8_t  N_ACTIONS     = 4;
    static constexpr float    ALPHA         = 0.10f;
    static constexpr float    GAMMA         = 0.90f;
    static constexpr float    EPSILON_MIN   = 0.10f;
    static constexpr float    EPSILON_DECAY = 0.9997f;
    static constexpr uint32_t LOG_EVERY     = 200;
    static constexpr uint32_t EVADE_MS      = 350;   // durée évitement bord (ms)

    // Seuils capteurs
    static constexpr float TOF_MIN_MM   =  80.0f;
    static constexpr float TOF_CLOSE_MM = 280.0f;
    static constexpr float LIDAR_MIN_M  =  0.055f;
    static constexpr float LIDAR_MAX_M  =  0.67f;
    static constexpr float LIDAR_NEAR_M =  0.25f;
    static constexpr float ARC_DEG      =  30.0f;

    // ── Q-table et hyperparamètres ────────────────────────────────
    float    _q[N_STATES][N_ACTIONS] = {};
    float    _epsilon      = 0.85f;
    uint32_t _episodeCount = 0;

    // ── État interne ──────────────────────────────────────────────
    uint8_t  _prevState  = 7;
    uint8_t  _prevAction = 0;
    bool     _firstStep  = true;
    uint32_t _stepCount  = 0;
    uint32_t _evadeUntil = 0;
    RobotConstants::ActionCommand _evadeCmd = {};

    // ── Encodage de l'état ────────────────────────────────────────
    uint8_t _computeState(RobotContext& ctx) const {
        // Priorité 1 : bords
        auto lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
        auto lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);
        auto lineB  = ctx.getLineData(SensorPosition::BACK);
        if ((lineFL.isValid && lineFL.value.scalar > 0.0f) ||
            (lineFR.isValid && lineFR.value.scalar > 0.0f) ||
            (lineB.isValid  && lineB.value.scalar  > 0.0f))
            return 0;

        // Priorité 2 : contact TOF bilatéral
        auto tofL  = ctx.getTOFData(SensorPosition::FRONT_LEFT);
        auto tofR  = ctx.getTOFData(SensorPosition::FRONT_RIGHT);
        bool closeL = tofL.isValid && tofL.value.scalar > TOF_MIN_MM && tofL.value.scalar < TOF_CLOSE_MM;
        bool closeR = tofR.isValid && tofR.value.scalar > TOF_MIN_MM && tofR.value.scalar < TOF_CLOSE_MM;
        if (closeL && closeR) return 6;

        // Priorité 3 : lidar
        auto lidar = ctx.getLidarData();
        if (lidar.isValid) {
            float dist  = lidar.value.vector.x;
            float angle = lidar.value.vector.y;
            if (dist >= LIDAR_MIN_M && dist <= LIDAR_MAX_M) {
                float diff = angle - 180.0f;
                if (diff >  180.0f) diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;

                if (fabsf(diff) > 150.0f)               return 1;  // dos à l'ennemi
                if (fabsf(diff) <= ARC_DEG && dist < LIDAR_NEAR_M) return 4;  // aligné proche
                if (fabsf(diff) <= ARC_DEG)              return 5;  // aligné loin
                return (diff < 0.0f) ? 2 : 3;                      // flanc G/D
            }
        }

        if (closeL) return 2;
        if (closeR) return 3;
        return 7;
    }

    // ── Récompense immédiate ──────────────────────────────────────
    float _computeReward(uint8_t s) const {
        switch (s) {
            case 0: return -5.0f;
            case 1: return -1.5f;
            case 2: return -0.3f;
            case 3: return -0.3f;
            case 4: return  1.5f;
            case 5: return  0.5f;
            case 6: return  2.0f;
            default: return -0.1f;
        }
    }

    // ── Bellman ───────────────────────────────────────────────────
    void _updateQ(uint8_t s, uint8_t a, float r, uint8_t sp) {
        float maxNext = _q[sp][0];
        for (uint8_t i = 1; i < N_ACTIONS; i++)
            if (_q[sp][i] > maxNext) maxNext = _q[sp][i];
        _q[s][a] += ALPHA * (r + GAMMA * maxNext - _q[s][a]);
    }

    uint8_t _argmax(uint8_t s) const {
        uint8_t best = 0;
        for (uint8_t i = 1; i < N_ACTIONS; i++)
            if (_q[s][i] > _q[s][best]) best = i;
        return best;
    }

    float _randFloat() const {
        return (float)(esp_random() & 0xFFFF) / 65536.0f;
    }
};

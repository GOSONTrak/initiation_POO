#pragma once
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  TrackStrategy — Charge vers tout obstacle détecté par le lidar
//
//  Priorités (décroissantes) :
//  1. ÉVADE   : bord → 2 phases :
//                 Phase 1 (EVADE_BACKUP_MS) : recul (temps)
//                 Phase 2 (EVADE_PIVOT_ANGLE EKF) : pivot 90°
//               + grace period EVADE_GRACE_MS post-pivot
//               → mémoire ennemi effacée à chaque EVADE
//  2. ATTACK  : lidar [100–670 mm] → charge pleine avec steering
//  3. MEMORY  : lidar perdu → se réorienter vers dernier θ connu
//  4. SEARCH  : rien → rotation rapide continue (EKF demi-tour)
//
//  Toutes les consignes de rotation s'appuient sur EKF θ.
// ═══════════════════════════════════════════════════════════════

class TrackStrategy : public StrategyInterface {
public:
    const char* name()       const override { return "Track"; }
    LedColor    color()      const override { return LedColor::MAGENTA; }
    bool        needsLidar() const override { return true; }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;

        uint32_t now = millis();

        // ── 0. EVADE phase 1 : recul linéaire (temps) ─────────
        if (now < _evadeUntil) {
            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // ── 0b. EVADE phase 2 : pivot 90° (EKF θ) ─────────────
        // Angle sauvegardé au premier cycle après la fin du recul.
        if (_pivoting) {
            if (!_pivotAngleSaved) {
                _pivotStartTheta = ctx.getPose().theta;
                _pivotAngleSaved = true;
            }
            float dTheta = ctx.getPose().theta - _pivotStartTheta;
            while (dTheta >  (float)M_PI) dTheta -= 2.0f * (float)M_PI;
            while (dTheta < -(float)M_PI) dTheta += 2.0f * (float)M_PI;
            if (fabsf(dTheta) < EVADE_PIVOT_ANGLE) {
                ctx.setState(RobotConstants::State::EVADE);
                return _pivotCmd;
            }
            _pivoting        = false;
            _pivotAngleSaved = false;
            _graceUntil      = millis() + EVADE_GRACE_MS;
        }

        // ── 1. Bords — priorité absolue ───────────────────────
        SensorData lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
        SensorData lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);
        SensorData lineB  = ctx.getLineData(SensorPosition::BACK);

        bool fl = lineFL.isValid && lineFL.value.scalar > 0.0f;
        bool fr = lineFR.isValid && lineFR.value.scalar > 0.0f;
        bool b  = lineB.isValid  && lineB.value.scalar  > 0.0f;

        if (b || fl || fr) {
            // Effacer la mémoire ennemi : évite de se réaligner vers
            // le bord qu'on vient de quitter après le pivot.
            _hasLastEnemy    = false;
            _spinning        = false;
            _evadeUntil      = now + EVADE_BACKUP_MS;

            if (b) {
                _pivoting        = false;
                _pivotAngleSaved = false;
                _graceUntil      = now + EVADE_BACKUP_MS + EVADE_GRACE_MS;
                _evadeCmd        = AC{ SPEED_FWD, SPEED_FWD };
            } else {
                // Recul avec légère correction latérale
                _evadeCmd = (fl && fr) ? AC{ -SPEED_FWD,  -SPEED_FWD  }
                            : fl       ? AC{ -SPEED_FWD,  -SPEED_SLOW }
                                       : AC{ -SPEED_SLOW, -SPEED_FWD  };
                // Pivot après recul : s'écarter du bord détecté
                _pivoting        = true;
                _pivotAngleSaved = false;
                _pivotCmd = fl ? AC{  SPEED_EVADE, -SPEED_EVADE }  // FL → pivot CW
                               : AC{ -SPEED_EVADE,  SPEED_EVADE }; // FR → pivot CCW
            }
            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // ── Grace period post-EVADE ────────────────────────────
        // Interdit l'attaque lidar pendant EVADE_GRACE_MS pour laisser
        // le temps de s'éloigner et éviter un rechargement immédiat
        // vers la même paroi.
        bool inGrace = (now < _graceUntil);

        // ── 2. Lidar — charge immédiate ───────────────────────
        if (!inGrace) {
            SensorData lidar = ctx.getLidarData();

            if (lidar.isValid) {
                float dist  = lidar.value.vector.x;
                float angle = lidar.value.vector.y;

                if (dist >= MIN_DIST_M && dist <= MAX_DIST_M) {
                    float diff = angle - 180.0f;
                    if (diff >  180.0f) diff -= 360.0f;
                    if (diff < -180.0f) diff += 360.0f;

                    // Mémoriser le cap global de l'ennemi (EKF)
                    _spinning = false;
                    EKF::Pose pose = ctx.getPose();
                    _lastEnemyTheta = pose.theta + diff * ((float)M_PI / 180.0f);
                    _hasLastEnemy   = true;

                    float diffNorm = constrain(diff / 180.0f, -1.0f, 1.0f);
                    ctx.setState(RobotConstants::State::ATTACK);
                    int l = constrain((int)(SPEED_FWD * (1.0f + diffNorm)), -255, 255);
                    int r = constrain((int)(SPEED_FWD * (1.0f - diffNorm)), -255, 255);
                    return AC{ l, r };
                }
            }
        }

        // ── 3. Recherche ──────────────────────────────────────
        ctx.setState(RobotConstants::State::SEARCH);

        // 3a. Mémoire ennemi — se réorienter vers le dernier cap connu
        if (_hasLastEnemy && !inGrace) {
            EKF::Pose pose = ctx.getPose();
            float diff = _lastEnemyTheta - pose.theta;
            while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
            while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
            if (fabsf(diff) > 0.25f) {
                return diff > 0 ? AC{  SPEED_SEARCH, -SPEED_SEARCH }
                                : AC{ -SPEED_SEARCH,  SPEED_SEARCH };
            }
            _hasLastEnemy = false;  // aligné → reprise du scan
        }

        // 3b. Scan rapide — demi-tour EKF puis reset (balayage continu)
        if (!_spinning) {
            _spinning       = true;
            _spinStartTheta = ctx.getPose().theta;
        }
        EKF::Pose pose = ctx.getPose();
        float turned = pose.theta - _spinStartTheta;
        while (turned >  (float)M_PI) turned -= 2.0f * (float)M_PI;
        while (turned < -(float)M_PI) turned += 2.0f * (float)M_PI;
        if (fabsf(turned) >= SPIN_TARGET_RAD) {
            _spinning = false;  // reset → nouveau demi-tour
        }

        return AC{ SPEED_SEARCH, -SPEED_SEARCH };
    }

private:
    // ── EVADE phase 1 (recul linéaire, temps) ────────────────
    uint32_t                       _evadeUntil = 0;
    uint32_t                       _graceUntil = 0;
    RobotConstants::ActionCommand  _evadeCmd   = {0, 0};
    RobotConstants::ActionCommand  _pivotCmd   = {0, 0};
    static constexpr uint32_t      EVADE_BACKUP_MS = 200;   // recul avant pivot
    static constexpr uint32_t      EVADE_GRACE_MS  = 400;   // fenêtre de sécurité post-pivot

    // ── EVADE phase 2 — pivot 90° (EKF θ) ────────────────────
    bool  _pivoting        = false;
    bool  _pivotAngleSaved = false;
    float _pivotStartTheta = 0.0f;
    static constexpr float EVADE_PIVOT_ANGLE = (float)M_PI / 2.0f;  // 90°
    static constexpr int   SPEED_EVADE       = 200;

    // ── Scan rapide (EKF θ) ───────────────────────────────────
    bool  _spinning       = false;
    float _spinStartTheta = 0.0f;
    static constexpr float SPIN_TARGET_RAD = 0.95f * (float)M_PI;  // ~171°

    // ── Mémoire dernière position ennemi ─────────────────────
    bool  _hasLastEnemy   = false;
    float _lastEnemyTheta = 0.0f;

    // ── Distances lidar ───────────────────────────────────────
    // MIN_DIST_M = 0.055m : filtre uniquement la valeur plancher 0.050m
    // que le capteur retourne quand aucune cible valide n'est en vue.
    // Les vraies détections commencent à ~0.064m et doivent passer.
    static constexpr float MIN_DIST_M  = 0.055f;
    static constexpr float MAX_DIST_M  = 0.67f;

    // ── Vitesses ──────────────────────────────────────────────
    static constexpr int SPEED_FWD    = 255;
    static constexpr int SPEED_SLOW   = 100;
    static constexpr int SPEED_SEARCH = 180;
};

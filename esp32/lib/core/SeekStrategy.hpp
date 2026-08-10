#pragma once
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  SeekStrategy — Approche l'objet le plus proche détecté par le lidar
//
//  États :
//  1. ATTACK : objet détecté dans l'arc avant (±70° autour de 10°)
//  2. SEARCH : rien détecté → machine d'états WAIT/ROTATE
//               WAIT   : robot immobile SEARCH_WAIT_MS (scan propre)
//               ROTATE : pivote de SEARCH_STEP_RAD (30°) via EKF θ
//              → couvre 360° en 12 pas sans distorsion lidar
// ═══════════════════════════════════════════════════════════════

class SeekStrategy : public StrategyInterface {
public:
    const char* name()       const override { return "Seek"; }
    LedColor    color()      const override { return LedColor::YELLOW; }
    bool        needsLidar() const override { return true; }

    void reset() override {
        _searchPhase    = SearchPhase::WAIT;
        _searchDeadline = 0;
        _rotateStart    = 0.0f;
        _wasInAttack    = false;
        _hasLock        = false;
        _lockLostAt     = 0;
        _chargeMode     = false;
    }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;

        SensorData lidar = ctx.getLidarData();

        if (lidar.isValid) {
            float dist  = lidar.value.vector.x;
            float angle = lidar.value.vector.y;

            if (dist >= MIN_DIST_M && dist <= MAX_DIST_M) {
                float diff = angle - FRONT_DEG;
                if (diff >  180.0f) diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;

                if (fabsf(diff) <= FRONT_ARC_DEG) {
                    float diffNorm = constrain(diff / 90.0f, -1.0f, 1.0f);
                    int l = constrain((int)(SPEED_FWD * (1.0f + diffNorm)), 0, 255);
                    int r = constrain((int)(SPEED_FWD * (1.0f - diffNorm)), 0, 255);

                    if (fabsf(diffNorm) < 0.15f) {
                        auto encL = ctx.getEncoderData(SensorPosition::LEFT);
                        auto encR = ctx.getEncoderData(SensorPosition::RIGHT);
                        if (encL.isValid && encR.isValid) {
                            int corr = constrain((int)((encL.rpm - encR.rpm) * ENC_KP), -30, 30);
                            l = constrain(l - corr, 0, 255);
                            r = constrain(r + corr, 0, 255);
                        }
                    }

                    // Verrouillage : mémorise le dernier angle valide
                    _hasLock    = true;
                    _lockAngle  = angle;
                    _lockLostAt = 0;
                    _wasInAttack = true;
                    ctx.setState(RobotConstants::State::ATTACK);
                    return AC{ l, r };
                }
            }
        }

        // ── TOF : détection courte portée ────────────────────────
        // Complémente le lidar quand l'adversaire est très proche
        // ou dans la zone morte lidar. Si un seul capteur détecte,
        // on tourne vers lui pour le ramener dans l'axe.
        {
            SensorData tofFL = ctx.getTOFData(SensorPosition::FRONT_LEFT);
            SensorData tofFR = ctx.getTOFData(SensorPosition::FRONT_RIGHT);
            bool tofL = tofFL.isValid
                     && tofFL.value.scalar > TOF_MIN_MM
                     && tofFL.value.scalar < TOF_MAX_MM;
            bool tofR = tofFR.isValid
                     && tofFR.value.scalar > TOF_MIN_MM
                     && tofFR.value.scalar < TOF_MAX_MM;

            if (tofL && tofR) {
                // Les deux TOF confirment → cible verrouillée en face → charge
                _hasLock     = true;
                _lockLostAt  = 0;
                _chargeMode  = true;
                _wasInAttack = true;
                ctx.setState(RobotConstants::State::ATTACK);
                return AC{ SPEED_CHARGE, SPEED_CHARGE };
            }

            if (tofL || tofR) {
                // Un seul TOF → cible de côté → aligner
                _hasLock    = true;
                _lockLostAt = 0;
                _lockAngle  = tofL ? FRONT_DEG - 45.0f : FRONT_DEG + 45.0f;
                _wasInAttack = true;
                int l = tofL ? SPEED_TOF_TURN : SPEED_FWD;
                int r = tofR ? SPEED_TOF_TURN : SPEED_FWD;
                ctx.setState(RobotConstants::State::ATTACK);
                return AC{ l, r };
            }
        }

        // ── Grace period : cible perdue mais lock actif ───────────
        // Continue l'attaque vers le dernier angle connu pendant
        // LOCK_GRACE_MS avant de passer en SEARCH.
        if (_hasLock) {
            if (_lockLostAt == 0) _lockLostAt = millis();

            if (millis() - _lockLostAt < LOCK_GRACE_MS) {
                ctx.setState(RobotConstants::State::ATTACK);
                if (_chargeMode) {
                    // Charge confirmée → maintenir pleine vitesse
                    return AC{ SPEED_CHARGE, SPEED_CHARGE };
                }
                // Alignement → steering vers dernier angle connu
                float diff = _lockAngle - FRONT_DEG;
                if (diff >  180.0f) diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;
                float diffNorm = constrain(diff / 90.0f, -1.0f, 1.0f);
                int l = constrain((int)(SPEED_FWD * (1.0f + diffNorm)), 0, 255);
                int r = constrain((int)(SPEED_FWD * (1.0f - diffNorm)), 0, 255);
                return AC{ l, r };
            }
            _hasLock    = false;
            _chargeMode = false;
        }

        // ── SEARCH : rotation par pas de 30° ─────────────────────
        if (_wasInAttack) {
            _wasInAttack    = false;
            _searchPhase    = SearchPhase::WAIT;
            _searchDeadline = millis() + SEARCH_WAIT_MS;
        }

        ctx.setState(RobotConstants::State::SEARCH);

        if (_searchPhase == SearchPhase::WAIT) {
            if (_searchDeadline == 0)
                _searchDeadline = millis() + SEARCH_WAIT_MS;

            if (millis() >= _searchDeadline) {
                _rotateStart = ctx.getPose().theta;
                _searchPhase = SearchPhase::ROTATE;
            }
            return AC{0, 0};
        }

        // ROTATE : pivote jusqu'à atteindre SEARCH_STEP_RAD
        float dTheta = ctx.getPose().theta - _rotateStart;
        while (dTheta >  (float)M_PI) dTheta -= 2.0f * (float)M_PI;
        while (dTheta < -(float)M_PI) dTheta += 2.0f * (float)M_PI;

        if (fabsf(dTheta) >= SEARCH_STEP_RAD) {
            _searchDeadline = millis() + SEARCH_WAIT_MS;
            _searchPhase    = SearchPhase::WAIT;
            return AC{0, 0};
        }

     
     
        return AC{SPEED_SEARCH, -SPEED_SEARCH};
    }

private:
    static constexpr float    FRONT_DEG        =  10.0f;
    static constexpr float    FRONT_ARC_DEG    =  90.0f;
    static constexpr float    MIN_DIST_M       =  0.15f;
    static constexpr float    MAX_DIST_M       =  0.8f;
    static constexpr int      SPEED_FWD        = 180;
    static constexpr float    ENC_KP           = 0.5f;

    static constexpr uint32_t SEARCH_WAIT_MS   = 150;
    static constexpr float    SEARCH_STEP_RAD  = (float)M_PI / 6.0f;  // 30°
    static constexpr int      SPEED_SEARCH     = 150;
    static constexpr uint32_t LOCK_GRACE_MS    = 500;
    static constexpr float    TOF_MIN_MM       =  80.0f;
    static constexpr float    TOF_MAX_MM       = 280.0f;
    static constexpr int      SPEED_TOF_TURN   =  80;
    static constexpr int      SPEED_CHARGE     = 255;

    enum class SearchPhase : uint8_t { WAIT, ROTATE };

    SearchPhase _searchPhase    = SearchPhase::WAIT;
    uint32_t    _searchDeadline = 0;
    float       _rotateStart    = 0.0f;
    bool        _wasInAttack    = false;

    bool        _hasLock        = false;
    float       _lockAngle      = 0.0f;
    uint32_t    _lockLostAt     = 0;
    bool        _chargeMode     = false;
};

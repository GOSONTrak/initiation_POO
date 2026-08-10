#pragma once
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  AdamantineStrategy.hpp — Stratégie défensive
//
//  Philosophie : survivre d'abord, contre-attaquer ensuite
//
//  Priorités (décroissantes) :
//  1. ÉVADE   : bord détecté → récupération immédiate
//  2. IMPACT  : choc IMU > 2g + robot repoussé → reculer et repositionner
//  3. RECUL   : ennemi très proche (TOF < 15 cm) → reculer + pivoter
//  4. ATTAQUE : ennemi aligné et proche (Lidar < 50 cm) → contre-attaque
//  5. RECUL   : ennemi détecté (Lidar 50–90 cm) → recul latéral
//  6. SEARCH  : rien détecté → rotation lente de surveillance
//
//  Capteurs utilisés :
//  - LineSensors FRONT_LEFT / FRONT_RIGHT / BACK → bords
//  - IMU ay/az                                  → détection d'impact
//  - TOF FRONT_LEFT / FRONT_RIGHT               → détection rapprochée
//  - Lidar FRONT                                → distance + angle ennemi
// ═══════════════════════════════════════════════════════════════

class AdamantineStrategy : public StrategyInterface {
public:

    const char* name()       const override { return "Adamantine"; }
    LedColor    color()      const override { return LedColor::CYAN; }
    bool        needsLidar() const override { return true; }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {

        using AC = RobotConstants::ActionCommand;

        // ── 0. EVADE verrouillé — durée minimale garantie ─────
        uint32_t now = millis();
        if (now < _evadeUntil) {
            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // ── 0b. Retour au centre après évitement ──────────────
        // Une fois l'évitement terminé, on pousse vers le centre
        // pendant RECENTER_MS avant de reprendre la stratégie.
        if (_recentering) {
            EKF::Pose pose = ctx.getPose();
            float dist = sqrtf(pose.x * pose.x + pose.y * pose.y);
            if (dist < RECENTER_STOP_MM || now > _recenterUntil) {
                _recentering = false;
            } else {
                ctx.setState(RobotConstants::State::EVADE);
                float angleToCenter = atan2f(-pose.y, -pose.x);
                float diff = angleToCenter - pose.theta;
                while (diff >  M_PI) diff -= 2.0f * M_PI;
                while (diff < -M_PI) diff += 2.0f * M_PI;
                float diffNorm = constrain(diff / M_PI, -1.0f, 1.0f);
                int l = constrain((int)(SPEED_EVADE * (1.0f + diffNorm)), -255, 255);
                int r = constrain((int)(SPEED_EVADE * (1.0f - diffNorm)), -255, 255);
                return AC{ l, r };
            }
        }

        // ── 1. Bords — priorité absolue ───────────────────────
        SensorData lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
        SensorData lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);
        SensorData lineB  = ctx.getLineData(SensorPosition::BACK);

        bool fl = lineFL.isValid && lineFL.value.scalar > 0.0f;
        bool fr = lineFR.isValid && lineFR.value.scalar > 0.0f;
        bool b  = lineB.isValid  && lineB.value.scalar  > 0.0f;

        if (b || fl || fr) {
            _evadeUntil  = now + EVADE_MIN_MS;
            _justEvaded  = true;   // activera le recentrage après l'évitement
            _recentering = false;
            if (b) { _evadeCmd = AC{  SPEED_CHARGE,  SPEED_CHARGE }; }
            else   { _evadeCmd = AC{ -SPEED_EVADE,  -SPEED_EVADE }; }  // recul droit dans tous les cas
            ctx.setState(RobotConstants::State::EVADE);
            return _evadeCmd;
        }

        // Sortie d'évitement → activer le retour au centre
        if (_justEvaded) {
            _justEvaded    = false;
            _recentering   = true;
            _recenterUntil = now + RECENTER_MS;
        }

        // ── 2. Impact IMU — choc détecté → repositionnement ───
        SensorData imu = ctx.getIMUData();
        if (imu.isValid) {
            float impact = sqrtf(imu.value.imu.ay * imu.value.imu.ay
                               + imu.value.imu.az * imu.value.imu.az);
            if (impact > IMPACT_THRESHOLD_G) {
                // Choc ! Le robot est en train d'être repoussé
                // → reculer et pivoter pour changer d'angle d'approche
                ctx.setState(RobotConstants::State::EVADE);
                // Pivoter du côté opposé à la force latérale
                if (imu.value.imu.ay > 0.0f) {
                    return AC{ -SPEED_EVADE, -SPEED_SLOW };  // recul virant gauche
                }
                return AC{ -SPEED_SLOW, -SPEED_EVADE };      // recul virant droite
            }
        }

        // ── 3. TOF — ennemi très proche (< 15 cm) ─────────────
        // Réaction rapide avant que le Lidar puisse traiter
        SensorData tofFL = ctx.getTOFData(SensorPosition::FRONT_LEFT);
        SensorData tofFR = ctx.getTOFData(SensorPosition::FRONT_RIGHT);

        bool tofCloseL = tofFL.isValid && tofFL.value.scalar > TOF_MIN_MM && tofFL.value.scalar < TOF_CLOSE_MM;
        bool tofCloseR = tofFR.isValid && tofFR.value.scalar > TOF_MIN_MM && tofFR.value.scalar < TOF_CLOSE_MM;

        // ── 4. TOF — les deux capteurs doivent confirmer (évite faux positifs)
        if (tofCloseL && tofCloseR) {
            ctx.setState(RobotConstants::State::EVADE);
            return AC{ -SPEED_EVADE, -SPEED_EVADE };
        }

        // ── 3. Lidar — analyse de la distance ennemi ──────────
        SensorData lidar = ctx.getLidarData();

        if (lidar.isValid) {
            float dist  = lidar.value.vector.x;
            float angle = lidar.value.vector.y;

            // Contre-attaque si ennemi aligné et proche
            if (dist < LIDAR_CLOSE_M) {
                ctx.setState(RobotConstants::State::ATTACK);

                // Lidar 0° = arrière robot, 180° = avant robot
                float diff = angle - 180.0f;
                if (diff >  180.0f) diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;

                // Steering proportionnel : diff normalisé -1..+1
                float diffNorm = constrain(diff / 180.0f, -1.0f, 1.0f);
                int l = constrain((int)(SPEED_CHARGE * (1.0f + diffNorm)), -255, 255);
                int r = constrain((int)(SPEED_CHARGE * (1.0f - diffNorm)), -255, 255);
                return AC{ l, r };
            }

            // Ennemi à distance moyenne → recul latéral pour esquiver
            if (dist < LIDAR_MID_M) {
                ctx.setState(RobotConstants::State::EVADE);
                if (angle < 180.0f) {
                    return AC{ -SPEED_SLOW, -SPEED_TURN };  // recul virant gauche
                }
                return AC{ -SPEED_TURN, -SPEED_SLOW };      // recul virant droite
            }
        }

        // ── 5. EKF — anticiper le bord si trop loin du centre ──
        EKF::Pose pose = ctx.getPose();
        float distCentre = sqrtf(pose.x * pose.x + pose.y * pose.y);
        if (distCentre > EDGE_WARN_MM) {
            ctx.setState(RobotConstants::State::EVADE);
            float angleToCenter = atan2f(-pose.y, -pose.x);
            float diff = angleToCenter - pose.theta;
            while (diff >  M_PI) diff -= 2.0f * M_PI;
            while (diff < -M_PI) diff += 2.0f * M_PI;
            if (fabsf(diff) <= 0.35f) return AC{ SPEED_CHARGE, SPEED_CHARGE };
            return diff > 0 ? AC{ SPEED_TURN, SPEED_CHARGE }
                            : AC{ SPEED_CHARGE, SPEED_TURN };
        }

        // ── 6. Rien détecté → rotation lente de surveillance ──
        ctx.setState(RobotConstants::State::SEARCH);
        return AC{ SPEED_PATROL, -SPEED_PATROL };
    }

private:
    // ── EVADE verrouillé ─────────────────────────────────────
    uint32_t                       _evadeUntil = 0;
    RobotConstants::ActionCommand  _evadeCmd   = {0, 0};
    static constexpr uint32_t      EVADE_MIN_MS = 500;

    // ── Retour au centre après évitement ─────────────────────
    bool     _justEvaded    = false;
    bool     _recentering   = false;
    uint32_t _recenterUntil = 0;
    static constexpr uint32_t RECENTER_MS      = 600;   // durée max du retour au centre
    static constexpr float    RECENTER_STOP_MM = 100.0f; // arrêt si déjà proche du centre

    // ── Seuils ────────────────────────────────────────────────
    static constexpr float IMPACT_THRESHOLD_G = 3.0f;    // seuil choc IMU dynamique ay/az (m/s²)
    static constexpr float TOF_MIN_MM        = 350.0f;  // ignore les lectures < 350mm (sol/structure)
    static constexpr float TOF_CLOSE_MM      = 450.0f;  // TOF : contact imminent (mm)
    static constexpr float LIDAR_CLOSE_M     =   0.35f; // Lidar : contre-attaque (m)
    static constexpr float LIDAR_MID_M       =   0.55f; // Lidar : orienter vers ennemi (m)
    static constexpr float EDGE_WARN_MM      = 295.0f;  // EKF : > 295mm du centre → retour (cercle noir r=385mm, robot r=50mm → limite=335mm)

    // ── Vitesses ──────────────────────────────────────────────
    static constexpr int SPEED_CHARGE =  255;  // contre-attaque pleine puissance
    static constexpr int SPEED_EVADE  =  200;  // récupération bord
    static constexpr int SPEED_TURN   =  120;  // virage correction
    static constexpr int SPEED_SLOW   =   80;  // correction douce
    static constexpr int SPEED_PATROL =  100;  // rotation surveillance
};

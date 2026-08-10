#pragma once
#include <cmath>
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  SquareStrategy v2 — Pure Pursuit
//
//  Principe :
//    5 waypoints définissent le carré dans le repère monde EKF.
//    À chaque cycle (20 Hz) le contrôleur Pure Pursuit calcule
//    la courbure κ = 2·y_local / L² vers le lookahead point,
//    puis la convertit en différentiel PWM.
//
//  Avantages vs v1 (machine à états bang-bang) :
//    - Trajectoire continue, aucun arrêt aux coins
//    - Virages arrondis naturellement (cercles inscrits)
//    - Un seul paramètre à régler : LOOKAHEAD_MM
//
//  Waypoints (repère monde, carré CCW) :
//    WP[0] = départ
//    WP[1] = départ + SIDE le long du cap initial
//    WP[2] = WP[1] tourné de 90° à gauche
//    WP[3] = WP[2] tourné de 90° à gauche
//    WP[4] = WP[0]  (retour au départ)
// ═══════════════════════════════════════════════════════════════

class SquareStrategy : public StrategyInterface {
public:
    const char* name()     const override { return "Square"; }
    LedColor    color()    const override { return LedColor::CYAN; }
    bool        needsLQR() const override { return false; }  // Pure Pursuit gère lui-même

    void reset() override {
        _phase    = Phase::IDLE;
        _seg      = 0;
        _deadline = 0;
    }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;
        const EKF::Pose pose = ctx.getPose();

        switch (_phase) {

            // ── IDLE : construit les waypoints dans le repère monde ───────
            case Phase::IDLE: {
                const float c = cosf(pose.theta);
                const float s = sinf(pose.theta);
                // Carré CCW : chaque côté tourne de 90° vers la gauche
                _wp[0] = { pose.x,                              pose.y                             };
                _wp[1] = { pose.x + SIDE_MM*c,                 pose.y + SIDE_MM*s                 };
                _wp[2] = { pose.x + SIDE_MM*c - SIDE_MM*s,     pose.y + SIDE_MM*s + SIDE_MM*c     };
                _wp[3] = { pose.x             - SIDE_MM*s,     pose.y             + SIDE_MM*c     };
                _wp[4] = { pose.x,                              pose.y                             };
                _seg      = 0;
                _deadline = millis() + TOTAL_TIMEOUT_MS;
                _phase    = Phase::RUNNING;
                ctx.setState(RobotConstants::State::ATTACK);
                return AC{ FWD_SPEED, FWD_SPEED };
            }

            // ── RUNNING : Pure Pursuit ────────────────────────────────────
            case Phase::RUNNING: {
                if (millis() > _deadline) {
                    _phase = Phase::DONE;
                    return AC{ 0, 0 };
                }

                // Avance le segment courant selon le critère adapté :
                // - Segments 0-2 : projection le long du segment >= SIDE_MM - ARRIVAL_MM
                // - Segment 3 (retour au départ) : distance euclidienne à WP[4]
                //   pour garantir que le robot revient réellement à l'origine
                while (_seg < 4) {
                    bool advance = false;
                    if (_seg == 3) {
                        // Dernier côté : critère de distance au point de départ
                        const float dx = _wp[4].x - pose.x;
                        const float dy = _wp[4].y - pose.y;
                        advance = (sqrtf(dx*dx + dy*dy) < ARRIVAL_MM);
                    } else {
                        // Côtés 0-2 : critère de projection
                        const float sdx = _wp[_seg+1].x - _wp[_seg].x;
                        const float sdy = _wp[_seg+1].y - _wp[_seg].y;
                        const float len = sqrtf(sdx*sdx + sdy*sdy);
                        const float ux  = sdx / len, uy = sdy / len;
                        const float progress = (pose.x - _wp[_seg].x)*ux
                                             + (pose.y - _wp[_seg].y)*uy;
                        advance = (progress >= SIDE_MM - ARRIVAL_MM);
                    }
                    if (advance) {
                        _seg++;
                        if (_seg >= 4) { _phase = Phase::DONE; return AC{ 0, 0 }; }
                    } else break;
                }

                // Cherche le lookahead point sur le chemin
                const WP lh = _lookahead(pose.x, pose.y);

                // Transforme dans le repère robot (rotation de -theta)
                const float dx      = lh.x - pose.x;
                const float dy      = lh.y - pose.y;
                const float y_local = -dx * sinf(pose.theta) + dy * cosf(pose.theta);

                // κ = 2·y_local / L²  →  différentiel PWM = κ · WHEELBASE/2 · vitesse
                const float diff = constrain(
                    FWD_SPEED * y_local * WHEELBASE_MM / (LOOKAHEAD_MM * LOOKAHEAD_MM),
                    (float)-MAX_DIFF, (float)MAX_DIFF
                );

                ctx.setState(RobotConstants::State::ATTACK);
                return AC{ (int)(FWD_SPEED - diff), (int)(FWD_SPEED + diff) };
            }

            case Phase::DONE:
            default:
                ctx.setState(RobotConstants::State::STANDBY);
                return AC{ 0, 0 };
        }
    }

private:
    enum class Phase : uint8_t { IDLE, RUNNING, DONE };
    struct WP { float x, y; };

    Phase    _phase    = Phase::IDLE;
    int      _seg      = 0;
    WP       _wp[5]    = {};
    uint32_t _deadline = 0;

    static constexpr float    SIDE_MM          = 435.0f;
    static constexpr float    LOOKAHEAD_MM     = 100.0f;  // L — réduire pour plus précis, augmenter pour plus lisse
    static constexpr float    ARRIVAL_MM       = 60.0f;   // seuil d'arrivée au waypoint
    static constexpr int      FWD_SPEED        = 150;
    static constexpr int      MAX_DIFF         = 80;      // différentiel max PWM gauche/droite
    static constexpr uint32_t TOTAL_TIMEOUT_MS = 25000;   // 25s max pour le carré complet
    static constexpr float    WHEELBASE_MM     = RobotConstants::WHEELBASE_MM;

    // Intersection arc de cercle (rayon L, centre robot) avec le segment courant.
    // Retourne le point sur le chemin à distance LOOKAHEAD_MM du robot.
    WP _lookahead(float rx, float ry) const {
        for (int i = _seg; i < 4; i++) {
            const float p1x = _wp[i].x,     p1y = _wp[i].y;
            const float p2x = _wp[i+1].x,   p2y = _wp[i+1].y;
            const float dx  = p2x - p1x,    dy  = p2y - p1y;
            const float fx  = p1x - rx,     fy  = p1y - ry;
            const float a   = dx*dx + dy*dy;
            const float b   = 2.0f*(fx*dx + fy*dy);
            const float c   = fx*fx + fy*fy - LOOKAHEAD_MM*LOOKAHEAD_MM;
            const float dis = b*b - 4.0f*a*c;
            if (dis >= 0.0f) {
                const float t = (-b + sqrtf(dis)) / (2.0f*a);
                if (t >= 0.0f && t <= 1.0f)
                    return { p1x + t*dx, p1y + t*dy };
            }
        }
        // Fallback : prochain waypoint
        return _wp[(_seg + 1 < 5) ? _seg + 1 : 4];
    }
};

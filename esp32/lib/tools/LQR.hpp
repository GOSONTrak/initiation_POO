#pragma once
#include <Arduino.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════
//  LQR.hpp — Régulateur Linéaire Quadratique
//
//  Deux modes de régulation :
//
//  ① compute()      — cap fixe (ligne droite, L == R)
//     État   : x = [θ_err (rad), ω (rad/s)]
//     Loi    : u = -(K1·θ_err + K2·ω)
//     Gains DARE : K1=310, K2=3
//
//  ② computeCurve() — courbure constante (arc, L ≠ R)
//     État   : x = ω_err = ω_ref − ω_meas  (scalaire)
//     Loi    : u = K_OMEGA · ω_err
//     Gain   : K_OMEGA = 40 PWM/(rad/s)  ← dérivé de b = 0.01482
//
//  ── Modèle commun ───────────────────────────────────────────
//    b = MAX_V_MMS / (255 × WHEELBASE_MM)
//      = 350 / (255 × 92.66) ≈ 0.01482  (rad/s)/PWM
//    ω_ref (arc) = (cmdR − cmdL) / 255 × MAX_V_MMS / WHEELBASE_MM
//               = (cmdR − cmdL) × b
//
//  ── Application dans taskEncoders ───────────────────────────
//    targetL += u × RPM_PER_PWM   (gauche plus vite = CW, corrige CCW)
//    targetR -= u × RPM_PER_PWM
//
//  Avantage vs PID :
//    - Utilise ω mesuré (IMU), pas de dérivée numérique
//    - Pas d'intégrateur → pas de windup
// ═══════════════════════════════════════════════════════════════

class LQR {
public:

    // ── Gains pré-calculés ────────────────────────────────────
    static constexpr float K1           = 310.0f;  // PWM / rad        (cap fixe)
    static constexpr float K2           =   3.0f;  // PWM / (rad/s)    (amortissement)
    static constexpr float K_OMEGA      =  40.0f;  // PWM / (rad/s)    (courbure)
    static constexpr float U_MAX        =  40.0f;  // saturation PWM
    static constexpr float DEADBAND     =  0.009f; // rad ≈ 0.5°  (compute)
    static constexpr float DEADBAND_OMEGA = 0.05f; // rad/s ≈ 3°/s (computeCurve)

    // ─────────────────────────────────────────────────────────
    //  compute() — appelé à 200 Hz dans taskEncoders
    //
    //  theta_err : θ_cible − θ_mesuré  (rad, normalisé ]-π,π])
    //  omega     : gx × DEG_TO_RAD     (rad/s, positif = lacet gauche)
    //
    //  Retourne la correction en unité PWM :
    //    targetL += retour * (SPEED_MAX_RPM_L/255),  targetR -= retour * (SPEED_MAX_RPM_L/255)
    // ─────────────────────────────────────────────────────────
    float compute(float theta_err, float omega) const {
        if (fabsf(theta_err) < DEADBAND) return 0.0f;

        float u = -(K1 * theta_err + K2 * omega);

        if (u >  U_MAX) u =  U_MAX;
        if (u < -U_MAX) u = -U_MAX;
        return u;
    }

    // ─────────────────────────────────────────────────────────
    //  computeCurve() — régulation de courbure (L ≠ R, arc/cercle)
    //
    //  Maintient un taux de rotation ω_ref pendant un arc.
    //  Loi proportionnelle : u = K_OMEGA × (ω_ref − ω_meas)
    //
    //  omega_ref  : taux cible (rad/s) = (cmdR−cmdL) × b
    //               positif = CCW (virage gauche)
    //  omega_meas : gxRad compensé (rad/s), depuis IMU
    //
    //  Retourne la correction PWM (même convention que compute()) :
    //    targetL += u × RPM_PER_PWM
    //    targetR -= u × RPM_PER_PWM
    // ─────────────────────────────────────────────────────────
    float computeCurve(float omega_ref, float omega_meas) const {
        float omega_err = omega_ref - omega_meas;
        if (fabsf(omega_err) < DEADBAND_OMEGA) return 0.0f;
        float u = -K_OMEGA * omega_err;  // négatif = plus de différentiel CCW, positif = CW
        if (u >  U_MAX) u =  U_MAX;
        if (u < -U_MAX) u = -U_MAX;
        return u;
    }

    void reset() { /* sans état interne — rien à réinitialiser */ }
};

#pragma once
#include <Arduino.h>
#include <math.h>
#include "RobotConstants.hpp"

// ═══════════════════════════════════════════════════════════════
//  EKF.hpp — Filtre de Kalman Étendu pour localisation 2D
//
//  État estimé : [x (mm), y (mm), θ (rad)]
//
//  Sources de mesure fusionnées :
//  - Encodeurs (odométrie différentielle) → prédiction
//  - IMU gx (gyroscope X = axe lacet)    → correction du cap
//
//  Paramètres robot (depuis RobotConstants.hpp) :
//  - Diamètre roue    : WHEEL_DIAMETER_MM  (30 mm)
//  - Entraxe          : WHEELBASE_MM       (56 mm)
//  - Pulses/tour      : PULSES_PER_REV     (1400)
//
//  Usage :
//      EKF ekf;
//      ekf.reset();
//      ekf.predict(pulsesLeft, pulsesRight);   // depuis encodeurs
//      ekf.updateIMU(gz, dtSec);               // depuis IMU
//      EKF::Pose pose = ekf.getPose();
// ═══════════════════════════════════════════════════════════════

class EKF {
public:

    struct Pose {
        float x     = 0.0f;  // mm
        float y     = 0.0f;  // mm
        float theta = 0.0f;  // rad
    };

    EKF() { reset(); }

    // ── Remise à zéro ─────────────────────────────────────────
    void reset() {
        _x = _y = _theta = _thetaPrev = 0.0f;
        _cov[0][0] = 1.0f;  _cov[0][1] = 0.0f;  _cov[0][2] = 0.0f;
        _cov[1][0] = 0.0f;  _cov[1][1] = 1.0f;  _cov[1][2] = 0.0f;
        _cov[2][0] = 0.0f;  _cov[2][1] = 0.0f;  _cov[2][2] = 0.1f;
    }

    // ── Prédiction — odométrie encodeurs ─────────────────────
    //  Appeler à chaque cycle taskEncoders (200 Hz)
    //  @param dl  distance roue gauche en mm (signée)
    //  @param dr  distance roue droite en mm (signée)
    void predict(float dl, float dr) {
        const float d      = (dl + dr) * 0.5f;
        // Correction du glissement latéral pendant les virages brusques.
        // Les roues glissent quand dl et dr sont de signe opposé (pivot sur place).
        // Calibrer : faire tourner 360° → SLIP_FACTOR = angle_physique_deg / 360.
        constexpr float SLIP_FACTOR = 1.0f;  // à ajuster après test 360°
        const bool  sharpTurn = (dl * dr < 0.0f);
        const float dTheta    = (dr - dl) / WHEELBASE * (sharpTurn ? SLIP_FACTOR : 1.0f);
        const float midTheta = _theta + dTheta * 0.5f;

        _thetaPrev = _theta;
        _x        += d * cosf(midTheta);
        _y        += d * sinf(midTheta);
        _theta     = _wrapAngle(_theta + dTheta);

        // Jacobien F = linearisation du modele de mouvement
        float F[3][3] = {
            { 1.0f, 0.0f, -d * sinf(midTheta) },
            { 0.0f, 1.0f,  d * cosf(midTheta) },
            { 0.0f, 0.0f,  1.0f               }
        };

        // Bruit de processus Q proportionnel au mouvement
        const float absD = fabsf(d) + fabsf(dTheta) * WHEELBASE * 0.5f;
        float Q[3][3] = {
            { _qXY * absD, 0.0f,        0.0f                             },
            { 0.0f,        _qXY * absD, 0.0f                             },
            { 0.0f,        0.0f,        _qTheta * fabsf(dTheta) + 1e-6f }
        };

        // P = F * P * F^T + Q
        float FP[3][3], FPFT[3][3];
        _mat3Mul(F, _cov, FP);
        _mat3MulT(FP, F, FPFT);
        _mat3Add(FPFT, Q, _cov);
    }

    // ── Mise à jour — gyroscope IMU ───────────────────────────
    //  Appeler à chaque cycle taskEncoder (200 Hz)
    //  @param gyroRate  vitesse angulaire gx (lacet) en rad/s — signe : CCW > 0
    //  @param dtSec     intervalle de temps en secondes
    void updateIMU(float gyroRate, float dtSec) {
        const float zMeasured  = gyroRate * dtSec;
        const float dThetaEnc  = _theta - _thetaPrev;
        const float innovation = _wrapAngle(zMeasured - dThetaEnc);

        const float S  = _cov[2][2] + _rIMU;
        const float K2 = _cov[2][2] / S;

        // Le gyro mesure uniquement l'angle — K[0]=K[1]=0 : x et y non corrigés.
        // Corriger x,y avec une mesure angulaire polluerait la position quand
        // les covariances croisées cov[0][2]/cov[1][2] deviennent non nulles.
        _theta = _wrapAngle(_theta + K2 * innovation);

        // P = (I - KH)P avec K=[0,0,K2], H=[0,0,1] → seule la ligne 2 change
        const float s = 1.0f - K2;
        for (int j = 0; j < 3; j++) _cov[2][j] *= s;
        // Restaure la symétrie : colonne 2 = transposée de la ligne 2
        for (int i = 0; i < 3; i++) _cov[i][2] = _cov[2][i];

        // Symétrie numérique globale (accumulation d'erreurs float sur toutes les itérations)
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 3; j++) {
                float avg = (_cov[i][j] + _cov[j][i]) * 0.5f;
                _cov[i][j] = _cov[j][i] = avg;
            }
    }

    // ── Accesseurs ────────────────────────────────────────────
    Pose  getPose()             const { return { _x, _y, _theta }; }
    float getX()                const { return _x; }
    float getY()                const { return _y; }
    float getTheta()            const { return _theta; }
    float getUncertaintyX()     const { return _cov[0][0]; }
    float getUncertaintyY()     const { return _cov[1][1]; }
    float getUncertaintyTheta() const { return _cov[2][2]; }

    // ── Réglage des bruits (à calibrer selon le robot réel) ───
    void setProcessNoise(float qXY, float qTheta) { _qXY = qXY; _qTheta = qTheta; }
    void setMeasurementNoiseIMU(float rIMU)        { _rIMU = rIMU; }

private:

    float _x = 0.0f, _y = 0.0f, _theta = 0.0f, _thetaPrev = 0.0f;
    float _cov[3][3] = {};   // matrice de covariance (evite le conflit avec le macro _P de newlib)

    float _qXY    = 0.021f;  // bruit processus position  (mm²/mm parcouru) — calibré test droit: cross-track var=11.2mm² / 540mm
    float _qTheta = 0.1f;    // bruit processus cap       (rad²/rad tourné)
    float _rIMU   = 1e-4f;   // bruit mesure gyroscope (rad²) — calibré: K≈85% pendant virage rapide

    static constexpr float WHEELBASE = RobotConstants::WHEELBASE_MM;

    static float _wrapAngle(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }

    static void _mat3Mul(const float A[3][3], const float B[3][3], float C[3][3]) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                C[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) C[i][j] += A[i][k] * B[k][j];
            }
    }

    static void _mat3MulT(const float A[3][3], const float B[3][3], float C[3][3]) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                C[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) C[i][j] += A[i][k] * B[j][k];
            }
    }

    static void _mat3Add(const float A[3][3], const float B[3][3], float C[3][3]) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) C[i][j] = A[i][j] + B[i][j];
    }
};

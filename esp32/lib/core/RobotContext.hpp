#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "SensorTypes.hpp"
#include "RobotConstants.hpp"
#include "EKF.hpp"

// ═══════════════════════════════════════════════════════════════
//  RobotContext.hpp — État centralisé et thread-safe du robot
//
//  OOP appliqué :
//  - Singleton    : une seule instance accessible partout
//  - Encapsulation: données privées, accès via getters/setters
//  - Thread-safe  : mutex FreeRTOS sur chaque accès
//
//  Flux de données entre tâches :
//  - taskSensors  → set*(...)         (écrit)
//  - taskStrategy → get*() + setState (lit + écrit)
//  - taskMotors   → getMotorSpeeds()  (lit)
//  - taskComm     → getBattData()     (lit)
// ═══════════════════════════════════════════════════════════════

class RobotContext {
public:

    // ── Singleton (Meyer's) ────────────────────────────────────
    static RobotContext& instance() {
        static RobotContext inst;
        return inst;
    }

    RobotContext(const RobotContext&)            = delete;
    RobotContext& operator=(const RobotContext&) = delete;

    // ───────────────────────────────────────────────────────────
    //  État de la machine à états
    // ───────────────────────────────────────────────────────────

    void setState(RobotConstants::State s) {
        _lock(); _state = s; _unlock();
    }

    RobotConstants::State getState() {
        _lock(); auto s = _state; _unlock(); return s;
    }

    // ───────────────────────────────────────────────────────────
    //  Lidar — données + activation
    // ───────────────────────────────────────────────────────────

    void setLidarData(const SensorData& d) {
        _lock(); _lidarData = d; _unlock();
    }

    SensorData getLidarData() {
        _lock(); auto d = _lidarData; _unlock(); return d;
    }

    // Contrôle on/off du lidar — lu par taskLidar pour start()/stop().
    // taskStrategy positionne ce flag avant chaque executeStrategy() ;
    // n'importe quelle autre tâche peut l'écraser à tout moment.
    void setLidarEnabled(bool en) {
        _lock(); _lidarEnabled = en; _unlock();
    }

    bool isLidarEnabled() {
        _lock(); bool en = _lidarEnabled; _unlock(); return en;
    }

    void setLqrEnabled(bool en) {
        _lock(); _lqrEnabled = en; _unlock();
    }

    bool isLqrEnabled() {
        _lock(); bool en = _lqrEnabled; _unlock(); return en;
    }

    // Vrai dès que la calibration automatique de la zone morte est terminée.
    void setLidarCalibDone(bool done) {
        _lock(); _lidarCalibDone = done; _unlock();
    }

    bool isLidarCalibDone() {
        _lock(); bool v = _lidarCalibDone; _unlock(); return v;
    }

    // ───────────────────────────────────────────────────────────
    //  Capteurs de ligne (LEFT, RIGHT, BACK)
    // ───────────────────────────────────────────────────────────

    void setLineData(SensorPosition pos, const SensorData& d) {
        _lock(); _lineData[_posToIndex(pos)] = d; _unlock();
    }

    SensorData getLineData(SensorPosition pos) {
        _lock(); auto d = _lineData[_posToIndex(pos)]; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  TOF — un capteur par position (FRONT, LEFT, RIGHT, BACK, CENTER)
    // ───────────────────────────────────────────────────────────

    void setTOFData(SensorPosition pos, const SensorData& d) {
        _lock(); _tofData[_posToIndex(pos)] = d; _unlock();
    }

    SensorData getTOFData(SensorPosition pos) {
        _lock(); auto d = _tofData[_posToIndex(pos)]; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  IMU (MPU6050 — ax/ay/az + gx/gy/gz)
    // ───────────────────────────────────────────────────────────

    void setIMUData(const SensorData& d) {
        _lock(); _imuData = d; _unlock();
    }

    SensorData getIMUData() {
        _lock(); auto d = _imuData; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  Bouton start (PressType encodé en scalar)
    // ───────────────────────────────────────────────────────────

    void setSwitchData(const SensorData& d) {
        _lock(); _switchData = d; _unlock();
    }

    SensorData getSwitchData() {
        _lock(); auto d = _switchData; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  Encodeurs — vitesse (RPM) + position absolue (pulses)
    //  LEFT = index 0, RIGHT = index 1
    // ───────────────────────────────────────────────────────────

    struct EncoderData {
        float   rpm        = 0.0f;
        int32_t totalCount = 0;
        bool    isValid    = false;
    };

    void setEncoderData(SensorPosition side, const EncoderData& d) {
        _lock(); _encoderData[_sideToIndex(side)] = d; _unlock();
    }

    EncoderData getEncoderData(SensorPosition side) {
        _lock(); auto d = _encoderData[_sideToIndex(side)]; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  Batterie
    // ───────────────────────────────────────────────────────────

    void setBattData(const SensorData& d) {
        _lock(); _battData = d; _unlock();
    }

    SensorData getBattData() {
        _lock(); auto d = _battData; _unlock(); return d;
    }

    // ───────────────────────────────────────────────────────────
    //  Pose EKF — position estimée [x (mm), y (mm), θ (rad)]
    //  Écrit par taskEKF, lu par taskStrategy
    // ───────────────────────────────────────────────────────────

    void setPose(const EKF::Pose& pose) {
        _lock(); _pose = pose; _unlock();
    }

    EKF::Pose getPose() {
        _lock(); auto p = _pose; _unlock(); return p;
    }

    // ── Cap idéal pour la prochaine ligne droite ──────────────
    // Permet à une stratégie de forcer headingTarget dans taskEncoders
    // au lieu de capturer pose.theta (qui peut être légèrement erroné
    // après un virage). consumeIdealHeading() retourne false si non défini.
    void setIdealHeading(float theta) {
        _lock(); _idealHeading = theta; _idealHeadingSet = true; _unlock();
    }

    bool consumeIdealHeading(float& out) {
        _lock();
        bool s = _idealHeadingSet;
        out = _idealHeading;
        _idealHeadingSet = false;
        _unlock();
        return s;
    }

    // ───────────────────────────────────────────────────────────
    //  Commande moteurs
    // ───────────────────────────────────────────────────────────

    void setMotorSpeeds(const RobotConstants::ActionCommand& cmd) {
        _lock(); _motorSpeeds = cmd; _unlock();
    }

    RobotConstants::ActionCommand getMotorSpeeds() {
        _lock(); auto cmd = _motorSpeeds; _unlock(); return cmd;
    }

private:

    RobotContext() {
        _mutex = xSemaphoreCreateMutex();
    }

    // ── Données internes ──────────────────────────────────────
    RobotConstants::State         _state       = RobotConstants::State::STANDBY;
    SensorData                    _lidarData;
    bool                          _lidarEnabled    = false;
    bool                          _lqrEnabled      = true;
    bool                          _lidarCalibDone  = false;
    SensorData                    _lineData[9];     // indexé par _posToIndex
    SensorData                    _tofData[9];      // indexé par _posToIndex
    SensorData                    _imuData;
    SensorData                    _switchData;
    EncoderData                   _encoderData[2];  // LEFT=0, RIGHT=1
    SensorData                    _battData;
    EKF::Pose                     _pose;
    RobotConstants::ActionCommand _motorSpeeds;
    float                         _idealHeading    = 0.0f;
    bool                          _idealHeadingSet = false;

    // ── Mutex FreeRTOS ────────────────────────────────────────
    SemaphoreHandle_t _mutex = nullptr;

    void _lock()   { if (_mutex) xSemaphoreTake(_mutex, portMAX_DELAY); }
    void _unlock() { if (_mutex) xSemaphoreGive(_mutex); }

    // ── Mapping position → index ──────────────────────────────
    static uint8_t _posToIndex(SensorPosition pos) {
        switch (pos) {
            case SensorPosition::LEFT:         return 0;
            case SensorPosition::RIGHT:        return 1;
            case SensorPosition::FRONT:        return 2;
            case SensorPosition::BACK:         return 3;
            case SensorPosition::CENTER:       return 4;
            case SensorPosition::FRONT_LEFT:   return 5;
            case SensorPosition::FRONT_RIGHT:  return 6;
            case SensorPosition::BATTERY:      return 7;
            default:                           return 8;
        }
    }

    // LEFT / RIGHT uniquement pour les encodeurs
    static uint8_t _sideToIndex(SensorPosition side) {
        return (side == SensorPosition::RIGHT) ? 1 : 0;
    }
};

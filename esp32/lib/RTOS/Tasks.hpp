#pragma once
#include "RobotContext.hpp"
#include "RobotConstants.hpp"
#include "DriverManager.hpp"
#include "DriverLedRGB.hpp"
#include "SensorManger.hpp"
#include "StrategyManager.hpp"
#include "CommunicationManager.hpp"
#include "CommandParser.hpp"
#include "Queues.hpp"
#include "Encoder.hpp"
#include "EKF.hpp"
#include "PID.hpp"
#include "LQR.hpp"
#include "LidarSensor.hpp"
#include <ArduinoOTA.h>

extern TaskHandle_t g_commTask;

// ═══════════════════════════════════════════════════════════════
//  Paramètres passés à taskEncoders (intègre EKF + PID vitesse)
// ═══════════════════════════════════════════════════════════════
struct EKFParams {
    EKF*            ekf;
    Encoder*        left;
    Encoder*        right;
    DriverManager*  driver;
    PID*            pidLeft;
    PID*            pidRight;
    PID*            pidYaw;          // correcteur de cap (legacy — remplacé par lqrYaw)
    LQR             lqrYaw;          // LQR cap : u = -(K1·θ_err + K2·ω)
    volatile int    lastPwmLeft  = 0;
    volatile int    lastPwmRight = 0;
    float           headingTarget = 0.0f;   // cap enregistré au début d'un déplacement droit
    bool            headingLocked = false;  // true dès que le cap est enregistré
};

// ───────────────────────────────────────────────────────────────
//  Paramètres de taskLed
// ───────────────────────────────────────────────────────────────
struct LedTaskParams {
    DriverLedRGB*    led;
    StrategyManager* manager;
};

// ───────────────────────────────────────────────────────────────
//  ledStartup — séquence visuelle de démarrage (5 s)
//  Rouge → Vert → Bleu, ~1667 ms chacun.
//  À appeler au début de chaque tâche, avant son for(;;).
// ───────────────────────────────────────────────────────────────
static inline void ledStartup(DriverLedRGB* led) {
    constexpr TickType_t STEP = pdMS_TO_TICKS(5000 / 3);
    led->setColor(LedColor::RED);   vTaskDelay(STEP);
    led->setColor(LedColor::GREEN); vTaskDelay(STEP);
    led->setColor(LedColor::BLUE);  vTaskDelay(STEP);
    led->off();
}

// ───────────────────────────────────────────────────────────────
//  taskLed — 10 Hz, Core 0, Prio 0
//  STANDBY → YELLOW
//  Actif   → couleur de la stratégie courante
// ───────────────────────────────────────────────────────────────
void taskLed(void* pvParameters)
{
    LedTaskParams* p = static_cast<LedTaskParams*>(pvParameters);
    ledStartup(p->led);

    for (;;)
    {
        if (RobotContext::instance().getState() == RobotConstants::State::STANDBY) {
            p->led->setColor(LedColor::YELLOW);
        } else {
            p->led->setColor(p->manager->currentColor());
        }
        vTaskDelay(RTOSConfig::PERIOD_LED);
    }
}

// ───────────────────────────────────────────────────────────────
//  taskMotors — 100 Hz, Core 1, Prio 4
//  Gère UNIQUEMENT l'arrêt moteur (cmd == 0).
//  Les commandes non-nulles sont pilotées exclusivement par
//  taskEncoders (PID feedforward) — écrire en parallèle sur le
//  driver écrasait la correction PID toutes les 10ms et causait
//  une oscillation PWM → pics de courant → brownout reset.
// ───────────────────────────────────────────────────────────────
void taskMotors(void* pvParameters)
{
    DriverManager* devices = static_cast<DriverManager*>(pvParameters);
    for (;;)
    {
        RobotConstants::ActionCommand cmd = RobotContext::instance().getMotorSpeeds();
        if (cmd.isStop()) {
            devices->setSpeedAll(0, 0);
        }
        vTaskDelay(RTOSConfig::PERIOD_MOTORS);
    }
}

// ───────────────────────────────────────────────────────────────
//  taskSensors — 50 Hz, Core 1, Prio 3
//  Met à jour tous les capteurs et publie dans RobotContext
// ───────────────────────────────────────────────────────────────
void taskSensors(void* pvParameters)
{
    SensorManager* sensors = static_cast<SensorManager*>(pvParameters);

    for (;;)
    {
        sensors->updateAll();

        auto& ctx = RobotContext::instance();

        // TOF — avant-gauche et avant-droit (filtre par type pour éviter
        // la collision avec les LineSensors à la même position)
        ctx.setTOFData(SensorPosition::FRONT_LEFT,
            sensors->getDataByPositionAndType(SensorPosition::FRONT_LEFT,  "TOF")
        );
        ctx.setTOFData(SensorPosition::FRONT_RIGHT,
            sensors->getDataByPositionAndType(SensorPosition::FRONT_RIGHT, "TOF")
        );

        // LineSensors — avant-gauche, avant-droit, arrière
        ctx.setLineData(SensorPosition::FRONT_LEFT,
            sensors->getDataByPositionAndType(SensorPosition::FRONT_LEFT,  "LINE")
        );
        ctx.setLineData(SensorPosition::FRONT_RIGHT,
            sensors->getDataByPositionAndType(SensorPosition::FRONT_RIGHT, "LINE")
        );
        ctx.setLineData(SensorPosition::BACK,
            sensors->getDataByPosition(SensorPosition::BACK)
        );

        // IMU (CENTER) — ax/ay/az + gx/gy/gz
        ctx.setIMUData(
            sensors->getDataByPosition(SensorPosition::CENTER)
        );

        // Bouton start
        ctx.setSwitchData(
            sensors->getDataByPosition(SensorPosition::UNKNOWN)
        );

        // Batterie
        ctx.setBattData(
            sensors->getDataByPosition(SensorPosition::BATTERY)
        );

        vTaskDelay(RTOSConfig::PERIOD_SENSORS);
    }
}

// ───────────────────────────────────────────────────────────────
//  Paramètres de taskLidar
// ───────────────────────────────────────────────────────────────
struct LidarTaskParams {
    LidarSensor*          lidar;
    CommunicationManager* comm;
};

// ───────────────────────────────────────────────────────────────
//  taskLidar — DMA UART, Core 1, Prio 2
//  Attend un paquet lidar (bloquant), met à jour RobotContext.
//  Publie le scan brut sur robot/lidar_scan à 1 Hz.
// ───────────────────────────────────────────────────────────────
void taskLidar(void* pvParameters)
{
    LidarTaskParams* p = static_cast<LidarTaskParams*>(pvParameters);
    p->lidar->init();

    uint32_t lastScanPub = 0;
    bool     lastEnabled = false;

    for (;;) {
        auto& ctx = RobotContext::instance();
        bool enabled = ctx.isLidarEnabled();

        // 1. Start/Stop sur transition du flag — avant waitAndProcess()
        //    pour éviter que _handleStartup() envoie CMD_START alors que
        //    le lidar doit rester éteint.
        if (enabled != lastEnabled) {
            if (enabled) {
                p->lidar->start();
                LOG("[taskLidar] Démarrage LiDAR.");
            } else {
                p->lidar->stop();
                LOG("[taskLidar] Arrêt LiDAR.");
            }
            lastEnabled = enabled;
        }

        // 2. Traiter les paquets seulement si actif
        //    (waitAndProcess appelle _handleStartup qui enverrait CMD_START)
        if (enabled) {
            p->lidar->waitAndProcess();
            ctx.setLidarData(p->lidar->getData());
            ctx.setLidarCalibDone(p->lidar->isCalibDone());
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 3. Publier le scan brut (Toutes les 2000 ms)
        uint32_t now = millis();
        if (p->comm->isConnected() && (now - lastScanPub) >= 2000) {
            lastScanPub = now;

            if (enabled) {
                const LidarSensor::ScanPoint* pts = p->lidar->getRawScan();
                uint16_t n = p->lidar->getRawScanCount();

                static constexpr float BIN_DEG = 5.0f;
                static constexpr uint16_t NBINS = 72;
                float binDist[NBINS];
                uint8_t binQual[NBINS];
                bool binFilled[NBINS];
                memset(binFilled, 0, sizeof(binFilled));

                for (uint16_t i = 0; i < n; i++) {
                    if (pts[i].dist < 0.001f) continue;
                    uint16_t b = (uint16_t)(pts[i].angle / BIN_DEG) % NBINS;
                    if (!binFilled[b] || pts[i].dist < binDist[b]) {
                        binDist[b]   = pts[i].dist;
                        binQual[b]   = pts[i].quality;
                        binFilled[b] = true;
                    }
                }

                std::string msg = "{\"type\":\"LIDAR_SCAN\",\"points\":[";
                char tmp[48];
                bool first = true;
                for (uint16_t b = 0; b < NBINS; b++) {
                    if (!binFilled[b]) continue;
                    snprintf(tmp, sizeof(tmp), "%s{\"a\":%.0f,\"d\":%.3f,\"q\":%u}",
                             first ? "" : ",",
                             b * BIN_DEG, binDist[b], binQual[b]);
                    msg += tmp;
                    first = false;
                }
                msg += "]}";
                p->comm->sendTo("robot/lidar_scan", msg);
            }
        }
    }
}
// ───────────────────────────────────────────────────────────────
//  Paramètres de taskOTA
// ───────────────────────────────────────────────────────────────
struct OTATaskParams {
    DriverLedRGB* led;
};

// ───────────────────────────────────────────────────────────────
//  taskOTA — Core 0, Prio 4
//  Attend la connexion WiFi, configure ArduinoOTA puis gère les
//  requêtes de mise à jour en boucle.
//
//  Sécurité  : mot de passe "sumo2026" (à renseigner dans PlatformIO).
//  Sécurité  : moteurs stoppés à l'entrée du flash (onStart).
//  Feedback  : LED ORANGE pendant le flash + progression Serial.
// ───────────────────────────────────────────────────────────────
static volatile bool _otaFlashing = false;

void taskOTA(void* pvParameters)
{
    OTATaskParams* p = static_cast<OTATaskParams*>(pvParameters);

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ArduinoOTA.setHostname("mini-sumo");
    ArduinoOTA.setPassword("sumo2026");

    ArduinoOTA.onStart([]() {
        _otaFlashing = true;
        RobotContext::instance().setMotorSpeeds({});  // sécurité mécanique
        if (g_commTask) vTaskSuspend(g_commTask);
        Serial.println("[OTA] Mise a jour en cours...");
    });

    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        Serial.printf("[OTA] %3u%%\r", done * 100u / total);
    });

    ArduinoOTA.onEnd([]() {
        _otaFlashing = false;
        Serial.println("\n[OTA] Termine — redemarrage");
        if (g_commTask) vTaskResume(g_commTask);
    });

    ArduinoOTA.onError([](ota_error_t err) {
        _otaFlashing = false;
        Serial.printf("[OTA] Erreur [%u]\n", err);
        if (g_commTask) vTaskResume(g_commTask);
    });

    ArduinoOTA.begin();
    LOG("[OTA] Pret — hostname: mini-sumo  mot de passe: sumo2026");

    for (;;) {
        ArduinoOTA.handle();
        // LED ORANGE pendant le flash — taskLed (Prio 0) ne concurrence pas
        if (p && p->led)
            p->led->setColor(_otaFlashing ? LedColor::ORANGE : LedColor::OFF);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ───────────────────────────────────────────────────────────────
//  taskStrategy — 20 Hz, Core 1, Prio 2
//  Lit RobotContext, gère le bouton start, exécute la stratégie
// ───────────────────────────────────────────────────────────────
void taskStrategy(void* pvParameters)
{
    StrategyManager* manager = static_cast<StrategyManager*>(pvParameters);

    for (;;)
    {
        auto& ctx   = RobotContext::instance();
        auto  state = ctx.getState();

        // ── Gestion bouton start ──────────────────────────────
        SensorData sw = ctx.getSwitchData();
        if (sw.isValid) {
            PressType press = static_cast<PressType>(
                                static_cast<uint8_t>(sw.value.scalar));

            // Appui court en STANDBY → délai 5s réglementaire puis démarrage
            if (press == PressType::SHORT &&
                state  == RobotConstants::State::STANDBY) {
                // Attendre la fin de la calibration lidar avant de démarrer
                if (!ctx.isLidarCalibDone()) {
                    LOG("[Strategy] Attente calibration zone morte lidar...");
                    uint32_t _calibTimeout = millis() + 3000;
                    while (!ctx.isLidarCalibDone() && millis() < _calibTimeout)
                        vTaskDelay(pdMS_TO_TICKS(50));
                    if (!ctx.isLidarCalibDone())
                        LOG("[Strategy] Timeout calibration lidar — démarrage quand même");
                }
                LOG("[Strategy] Départ dans 5 secondes...");
                ctx.setMotorSpeeds({});           // moteurs arrêtés pendant l'attente
                vTaskDelay(pdMS_TO_TICKS(5000));  // délai réglementaire
                ctx.setState(RobotConstants::State::SEARCH);
                LOG("[Strategy] GO !");
            }

            // Appui long → arrêt d'urgence
            if (press == PressType::LONG) {
                LOG("[Strategy] Arrêt d'urgence");
                ctx.setState(RobotConstants::State::STANDBY);
                ctx.setMotorSpeeds({});
                ctx.setLidarEnabled(false);
            }

            // Double appui rapide → cycle de stratégie
            if (press == PressType::RAPID &&
                state != RobotConstants::State::STANDBY) {
                manager->nextStrategy();
                LOGF("[Strategy] Stratégie : %s\n", manager->currentName());
            }
        }

        // ── Protection batterie ───────────────────────────────
        SensorData batt = ctx.getBattData();
        if (batt.isValid && batt.value.scalar > 0.0f && batt.value.scalar <= 5.80f &&
            state != RobotConstants::State::STANDBY) {
            LOG("[Strategy] BATTERIE CRITIQUE → STANDBY force");
            ctx.setState(RobotConstants::State::STANDBY);
            ctx.setMotorSpeeds({});
            vTaskDelay(RTOSConfig::PERIOD_STRATEGY);
            continue;
        }

        // ── Exécution stratégie (seulement hors STANDBY) ─────
        if (state != RobotConstants::State::STANDBY) {
            ctx.setLidarEnabled(manager->currentNeedsLidar());
            ctx.setLqrEnabled(manager->currentNeedsLQR());
            RobotConstants::ActionCommand cmd = manager->executeStrategy(ctx);
            ctx.setMotorSpeeds(cmd);
        } else {
            ctx.setLidarEnabled(false);
        }

        vTaskDelay(RTOSConfig::PERIOD_STRATEGY);
    }
}

// ───────────────────────────────────────────────────────────────
//  taskComm — 10 Hz, Core 0, Prio 1
//  Gère BLE/UART : envoie la telémétrie, reçoit les commandes
// ───────────────────────────────────────────────────────────────
void taskComm(void* pvParameters)
{
    CommunicationManager* comm = static_cast<CommunicationManager*>(pvParameters);

    comm->onReceive([](const std::string& msg) {
        RobotQueues::sendCommand(msg.c_str());
    });

    comm->begin();  // connexion WiFi + MQTT (bloquant jusqu'à connexion)

    for (;;)
    {
        // 1. Traiter les messages entrants + autoSelect BLE/UART
        comm->update();

        // 2. Envoyer la télémétrie si connecté
        if (comm->isConnected()) {
            auto& ctx        = RobotContext::instance();
            EKF::Pose pose   = ctx.getPose();
            SensorData batt  = ctx.getBattData();
            SensorData lidar = ctx.getLidarData();

            // Lidar — adversaire
            float enemyDist  = lidar.isValid ? lidar.value.vector.x : -1.0f;
            float enemyAngle = lidar.isValid ? lidar.value.vector.y : -1.0f;

            // Capteurs de ligne : 1=bord, 0=ok, -1=invalide
            SensorData lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
            SensorData lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);
            SensorData lineB  = ctx.getLineData(SensorPosition::BACK);
            auto lineVal = [](const SensorData& d) -> int {
                if (!d.isValid) return -1;
                return d.value.scalar > 0.0f ? 1 : 0;
            };

            // TOF
            SensorData tofFL = ctx.getTOFData(SensorPosition::FRONT_LEFT);
            SensorData tofFR = ctx.getTOFData(SensorPosition::FRONT_RIGHT);

            // IMU
            SensorData imu = ctx.getIMUData();
            float ax = imu.isValid ? imu.value.imu.ax : 0.0f;
            float ay = imu.isValid ? imu.value.imu.ay : 0.0f;
            float az = imu.isValid ? imu.value.imu.az : 0.0f;
            float gx = imu.isValid ? imu.value.imu.gx : 0.0f;
            float gy = imu.isValid ? imu.value.imu.gy : 0.0f;
            float gz = imu.isValid ? imu.value.imu.gz : 0.0f;

            // Encodeurs
            RobotContext::EncoderData encL = ctx.getEncoderData(SensorPosition::LEFT);
            RobotContext::EncoderData encR = ctx.getEncoderData(SensorPosition::RIGHT);

            // Commande moteur
            RobotConstants::ActionCommand cmd = ctx.getMotorSpeeds();

            // Incertitude EKF
            // (stockée dans EKF directement — accès via pose uniquement ici)

            // ── Calculs complémentaires pour le protocole V1 ──────────
            // Batterie : percent et critical calculés localement
            // (BattSensor::getPercent/isCritical ne sont pas dans RobotContext)
            float  batVoltage  = batt.isValid ? batt.value.scalar : 0.0f;
            int    batPercent  = batt.isValid
                                 ? (int)((batVoltage - 6.0f) / (8.4f - 6.0f) * 100.0f)
                                 : 0;
            // Clamping 0-100
            if (batPercent < 0)   batPercent = 0;
            if (batPercent > 100) batPercent = 100;
            bool batCritical = batt.isValid && (batVoltage <= 6.60f);
 
            // Lidar : validité explicite (le HMI attend un bool "valid")
            bool lidarValid = lidar.isValid && (lidar.value.vector.x > 0.0f);
            float lidarDist  = lidarValid ? lidar.value.vector.x : 0.0f;
            float lidarAngle = lidarValid ? lidar.value.vector.y : 0.0f;
 
            // Capteurs de ligne : conversion int (0/1/-1) → bool
            // lineVal() existe déjà dans la tâche, on l'utilise directement
            bool lineFLbool = (lineVal(lineFL) == 1);
            bool lineFRbool = (lineVal(lineFR) == 1);
            bool lineBbool  = (lineVal(lineB)  == 1);

            // TOF
            float tofFLmm = tofFL.isValid ? tofFL.value.scalar : -1.0f;
            float tofFRmm = tofFR.isValid ? tofFR.value.scalar : -1.0f;

            char buf[800];
            snprintf(buf, sizeof(buf),
                "{"
                    "\"type\":\"TELEMETRY\","
                    "\"payload\":{"
                        "\"ts\":%lu,"
                        "\"battery\":{"
                            "\"voltage\":%.2f,"
                            "\"percent\":%d,"
                            "\"critical\":%s"
                        "},"
                        "\"line\":{"
                            "\"frontLeft\":%s,"
                            "\"frontRight\":%s,"
                            "\"backLeft\":false,"
                            "\"back\":%s"
                        "},"
                        "\"tof\":{"
                            "\"frontLeft\":%.0f,"
                            "\"frontRight\":%.0f"
                        "},"
                        "\"lidar\":{"
                            "\"dist\":%.3f,"
                            "\"angle\":%.1f,"
                            "\"valid\":%s"
                        "},"
                        "\"imu\":{"
                            "\"ax\":%.3f,"
                            "\"ay\":%.3f,"
                            "\"az\":%.3f,"
                            "\"gx\":%.3f,"
                            "\"gy\":%.3f,"
                            "\"gz\":%.3f,"
                            "\"valid\":%s"
                        "},"
                        "\"motors\":{"
                            "\"left\":%d,"
                            "\"right\":%d"
                        "},"
                        "\"pose\":{"
                            "\"x\":%.1f,"
                            "\"y\":%.1f,"
                            "\"theta\":%.3f"
                        "},"
                        "\"encoders\":{"
                            "\"leftRpm\":%.1f,"
                            "\"rightRpm\":%.1f"
                        "}"
                    "}"
                "}",
                (unsigned long)millis(),
                batVoltage,
                batPercent,
                batCritical  ? "true" : "false",
                lineFLbool   ? "true" : "false",
                lineFRbool   ? "true" : "false",
                lineBbool    ? "true" : "false",
                tofFLmm,
                tofFRmm,
                lidarDist,
                lidarAngle,
                lidarValid   ? "true" : "false",
                ax, ay, az,
                gx, gy, gz,
                imu.isValid  ? "true" : "false",
                cmd.leftSpeed, cmd.rightSpeed,
                pose.x, pose.y, pose.theta,
                encL.rpm, encR.rpm
            );
            comm->send(std::string(buf));
        }

        vTaskDelay(RTOSConfig::PERIOD_COMM);
    }
}

// ───────────────────────────────────────────────────────────────
//  taskEncoders — 200 Hz, Core 1, Prio 5
//  Lit les impulsions, calcule RPM, met à jour l'EKF et publie
//  dans RobotContext. Intègre taskEKF pour éviter un double
//  getAndReset() sur les mêmes encodeurs.
// ───────────────────────────────────────────────────────────────
void taskEncoders(void* pvParameters)
{
    // Vitesses maxi par roue extrapolées à PWM=255 (mesurées à PWM=150, batterie ~7.4V)
    static constexpr float SPEED_MAX_RPM_L = 150.0f;  // roue gauche, gear 50:1  (88.7 RPM @ PWM150)
    static constexpr float SPEED_MAX_RPM_R = 142.0f;  // roue droite, gear 50:1 CH-N20-3 — ffR=159 PWM à cmd=150 (interpolé entre 137/165)
    // FF_SCALE_R = L/R > 1 → ffR > ffL. Pour ajuster :
    // ↑ Si robot tourne à gauche  → AUGMENTER SPEED_MAX_RPM_R (réduit FF_SCALE_R, réduit ffR)
    // ↑ Si robot tourne à droite  → DIMINUER  SPEED_MAX_RPM_R (augmente FF_SCALE_R, augmente ffR)

    EKFParams* p = static_cast<EKFParams*>(pvParameters);

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t   lastMs   = millis();

    // Calibration du bias gyro — robot immobile au démarrage (200 cycles × 5 ms = 1 s)
    float gxBiasDps = 0.0f;
    {
        constexpr int N_CAL = 200;
        int nValid = 0;
        for (int i = 0; i < N_CAL; i++) {
            vTaskDelay(pdMS_TO_TICKS(5));
            SensorData imu = RobotContext::instance().getIMUData();
            if (imu.isValid) {
                gxBiasDps += imu.value.imu.gx;
                nValid++;
            }
        }
        if (nValid > 0) gxBiasDps /= (float)nValid;
        LOGF("[EKF] gx_bias calibre: %.2f deg/s (%d samples)\n", gxBiasDps, nValid);
    }
    lastWake = xTaskGetTickCount();   // réinitialise après la calibration
    lastMs   = millis();

    for (;;)
    {
        const float deltaMs = RTOSConfig::PERIOD_ENCODER * portTICK_PERIOD_MS;

        // 1. Lire et remettre à zéro atomiquement (une seule fois)
        int32_t pulsesLeft  = -p->left->getAndReset();   // encodeur gauche câblé inversé
        int32_t pulsesRight =  p->right->getAndReset();

        // 2. Calculer RPM et publier dans RobotContext
        RobotContext::EncoderData left, right;

        left.rpm        = p->left->getRPM(pulsesLeft, deltaMs);
        left.totalCount = p->left->getTotalCount();
        left.isValid    = true;

        right.rpm        = p->right->getRPM(pulsesRight, deltaMs);
        right.totalCount = p->right->getTotalCount();
        right.isValid    = true;

        auto& ctx = RobotContext::instance();
        ctx.setEncoderData(SensorPosition::LEFT,  left);
        ctx.setEncoderData(SensorPosition::RIGHT, right);

        // 3. Prédiction EKF odométrie — conversion pulses → mm par roue (PPR différents)
        p->ekf->predict(p->left->toMM(pulsesLeft), p->right->toMM(pulsesRight));

        // 4. Correction EKF gyroscope
        SensorData imuData = ctx.getIMUData();
        float gxRad = 0.0f;   // taux de lacet mesuré (rad/s) — utilisé aussi par LQR
        if (imuData.isValid) {
            uint32_t now = millis();
            float dtSec  = (now - lastMs) / 1000.0f;
            lastMs       = now;
            static constexpr float GYRO_EKF_DEADBAND_RAD = 1.0f * (float)DEG_TO_RAD;  // ±1°/s — bruit résiduel post-calibration
            gxRad = (imuData.value.imu.gx - gxBiasDps) * DEG_TO_RAD;
            if (fabsf(gxRad) > GYRO_EKF_DEADBAND_RAD) {
                p->ekf->updateIMU(gxRad, dtSec);
            }
        }

        // 5. Publier la pose estimée
        ctx.setPose(p->ekf->getPose());

        // 6. Asservissement de vitesse — feedforward + correction PID ──
        RobotConstants::ActionCommand cmd = ctx.getMotorSpeeds();

        // Feedforward + correction PID pour égaliser les vitesses
        static RobotConstants::ActionCommand prevCmd = {0, 0};
        if (cmd.leftSpeed != prevCmd.leftSpeed || cmd.rightSpeed != prevCmd.rightSpeed) {
            p->pidLeft->reset();
            p->pidRight->reset();
            prevCmd = cmd;
        }

        if (cmd.isStop()) {
            p->driver->setSpeedAll(0, 0);
            p->pidLeft->reset();
            p->pidRight->reset();
            p->pidYaw->reset();
            p->headingLocked = false;
        } else {
            // Cible identique en RPM roue pour les deux moteurs
            float targetL = (cmd.leftSpeed  / 255.0f) * SPEED_MAX_RPM_L;
            float targetR = (cmd.rightSpeed / 255.0f) * SPEED_MAX_RPM_L;

            // LQR cap — modifie les consignes RPM AVANT les PIDs
            // Désactivé si la stratégie courante ne nécessite pas de tenue de cap
            // (ex: Q-learning qui change de direction trop fréquemment).
            bool lqrOn = ctx.isLqrEnabled();
            if (lqrOn && cmd.leftSpeed == cmd.rightSpeed && imuData.isValid) {
                EKF::Pose pose = p->ekf->getPose();
                if (!p->headingLocked) {
                    float ideal;
                    if (ctx.consumeIdealHeading(ideal))
                        p->headingTarget = ideal;
                    else
                        p->headingTarget = pose.theta;
                    p->headingLocked = true;
                    p->lqrYaw.reset();
                }
                float thetaErr = p->headingTarget - pose.theta;
                while (thetaErr >  (float)M_PI) thetaErr -= 2.0f * (float)M_PI;
                while (thetaErr < -(float)M_PI) thetaErr += 2.0f * (float)M_PI;

                // u (PWM) → RPM : les PIDs enforceront la correction
                // EKF : dTheta = (dr - dl) / wb → gauche plus vite = CW (theta diminue)
                // Donc pour corriger CCW (theta↑) : gauche↑, droite↓ → targetL+=, targetR-=
                static constexpr float RPM_PER_PWM = SPEED_MAX_RPM_L / 255.0f;
                float corrRPM = p->lqrYaw.compute(thetaErr, gxRad) * RPM_PER_PWM;
                targetL += corrRPM;
                targetR -= corrRPM;
            } else if (lqrOn && imuData.isValid) {
                // LQR courbure — maintient ω_ref pendant un arc (L ≠ R)
                p->headingLocked = false;
                static constexpr float B_RAD = 350.0f / (255.0f * 92.66f); // (rad/s)/PWM
                float omegaRef = (cmd.rightSpeed - cmd.leftSpeed) * B_RAD;
                static constexpr float RPM_PER_PWM_C = SPEED_MAX_RPM_L / 255.0f;
                float corrRPM = p->lqrYaw.computeCurve(omegaRef, gxRad) * RPM_PER_PWM_C;
                targetL += corrRPM;
                targetR -= corrRPM;
            } else {
                p->lqrYaw.reset();
                p->headingLocked = false;
            }

            p->pidLeft->setSetpoint(targetL);
            p->pidRight->setSetpoint(targetR);

            // Feedforward pré-échelonné : R a besoin de moins de PWM pour la même vitesse roue
            static constexpr float FF_SCALE_R = SPEED_MAX_RPM_L / SPEED_MAX_RPM_R;
            int ffL = cmd.leftSpeed;
            int ffR = (int)roundf(cmd.rightSpeed * FF_SCALE_R);

            int pwmL = ffL + (int)p->pidLeft->compute(left.rpm);
            int pwmR = ffR + (int)p->pidRight->compute(right.rpm);

            p->lastPwmLeft  = constrain(pwmL, -255, 255);
            p->lastPwmRight = constrain(pwmR, -255, 255);
            p->driver->setSpeedAll(p->lastPwmLeft, p->lastPwmRight);
        }

        vTaskDelayUntil(&lastWake, RTOSConfig::PERIOD_ENCODER);
    }
}

// ───────────────────────────────────────────────────────────────
//  Paramètres de taskCommand
// ───────────────────────────────────────────────────────────────
struct CommandTaskParams {
    StrategyManager* manager;

    // Callbacks vers main.cpp (résolution des dépendances locales)
    void (*onStrategy)(const char* name);       // STRATEGY:xxx
    void (*onLed)(uint8_t r, uint8_t g, uint8_t b); // LED:R:G:B
    void (*onPidKp)(float v);                   // PID:KP     (moteurs gauche+droit)
    void (*onPidKi)(float v);                   // PID:KI
    void (*onPidKd)(float v);                   // PID:KD
    void (*onPidYawKp)(float v);               // PID:YAW:KP (correcteur de cap)
    void (*onPidYawKi)(float v);               // PID:YAW:KI
    void (*onPidYawKd)(float v);               // PID:YAW:KD
    void (*onPoseReset)();                       // POSE:RESET / RESET
};

// ───────────────────────────────────────────────────────────────
//  taskCommand — event-driven, Core 1, Prio 2
//  Lit la queue de commandes HMI et les applique au robot
//  Gère aussi l'exécution des séquences de mouvement
// ───────────────────────────────────────────────────────────────
void taskCommand(void* pvParameters)
{
    CommandTaskParams* p = static_cast<CommandTaskParams*>(pvParameters);
    StrategyManager* manager = p->manager;

    char rawCmd[RobotQueues::CMD_MAX_LEN];

    for (;;)
    {
        // Bloque jusqu'à réception d'une commande (pas de busy-wait)
        if (!RobotQueues::receiveCommand(rawCmd)) continue;

        RobotCommand cmd = parseCommand(rawCmd);
        auto& ctx = RobotContext::instance();

        switch (cmd.type) {

            case CommandType::START:
                if (ctx.getState() == RobotConstants::State::STANDBY) {
                    LOG("[CMD] Départ dans 5 secondes...");
                    ctx.setMotorSpeeds({});
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    ctx.setState(RobotConstants::State::SEARCH);
                    LOG("[CMD] GO !");
                }
                break;

            case CommandType::STOP:
                ctx.setState(RobotConstants::State::STANDBY);
                ctx.setMotorSpeeds({});
                ctx.setLidarEnabled(false);
                break;

            case CommandType::RESET:
            case CommandType::POSE_RESET:
                ctx.setPose({ 0.0f, 0.0f, 0.0f });
                if (p->onPoseReset) p->onPoseReset();
                LOG("[CMD] Pose réinitialisée");
                break;

            case CommandType::STRATEGY:
                LOGF("[CMD] Stratégie : %s\n", cmd.strategyName);
                if (p->onStrategy) p->onStrategy(cmd.strategyName);
                break;

            case CommandType::MOTOR_DIRECT:
                ctx.setMotorSpeeds({ cmd.motorLeft, cmd.motorRight });
                break;

            case CommandType::MOTOR_STOP:
                ctx.setMotorSpeeds({});
                break;

            case CommandType::PID_KP:
                LOGF("[CMD] PID KP = %.3f\n", cmd.pidValue);
                if (p->onPidKp) p->onPidKp(cmd.pidValue);
                break;

            case CommandType::PID_KI:
                LOGF("[CMD] PID KI = %.3f\n", cmd.pidValue);
                if (p->onPidKi) p->onPidKi(cmd.pidValue);
                break;

            case CommandType::PID_KD:
                LOGF("[CMD] PID KD = %.3f\n", cmd.pidValue);
                if (p->onPidKd) p->onPidKd(cmd.pidValue);
                break;

            case CommandType::PID_YAW_KP:
                LOGF("[CMD] PID YAW KP = %.3f\n", cmd.pidValue);
                if (p->onPidYawKp) p->onPidYawKp(cmd.pidValue);
                break;

            case CommandType::PID_YAW_KI:
                LOGF("[CMD] PID YAW KI = %.3f\n", cmd.pidValue);
                if (p->onPidYawKi) p->onPidYawKi(cmd.pidValue);
                break;

            case CommandType::PID_YAW_KD:
                LOGF("[CMD] PID YAW KD = %.3f\n", cmd.pidValue);
                if (p->onPidYawKd) p->onPidYawKd(cmd.pidValue);
                break;

            case CommandType::LED:
                LOGF("[CMD] LED R=%d G=%d B=%d\n", cmd.ledR, cmd.ledG, cmd.ledB);
                if (p->onLed) p->onLed(cmd.ledR, cmd.ledG, cmd.ledB);
                break;

            case CommandType::SEQUENCE:
                LOGF("[CMD] SEQ %d steps\n", cmd.seqLength);
                ctx.setState(RobotConstants::State::STANDBY);
                for (uint8_t i = 0; i < cmd.seqLength; i++) {
                    const MoveStep& step = cmd.seqSteps[i];
                    ctx.setMotorSpeeds({ step.leftSpeed, step.rightSpeed });

                    if (step.targetMm > 0.0f) {
                        // Arrêt sur distance EKF (timeout 8s)
                        EKF::Pose startPose = ctx.getPose();
                        const uint32_t tMax = millis() + 8000;
                        while (millis() < tMax) {
                            EKF::Pose pose = ctx.getPose();
                            float dx = pose.x - startPose.x;
                            float dy = pose.y - startPose.y;
                            if (sqrtf(dx * dx + dy * dy) >= step.targetMm) break;
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(step.durationMs));
                    }
                }
                ctx.setMotorSpeeds({});
                break;

            default:
                LOGF("[CMD] Commande inconnue : %s\n", rawCmd);
                break;
        }
    }
}

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ═══════════════════════════════════════════════════════════════
//  RTOSConfig.hpp — Configuration FreeRTOS mini sumo ESP32
// ═══════════════════════════════════════════════════════════════

// ─── Debug (commenter pour désactiver tout log) ────────────────
#define DEBUG_ENABLED

// ───────────────────────────────────────────────────────────────
//  Constantes des tâches
// ───────────────────────────────────────────────────────────────
namespace RTOSConfig {

    // ── Stack sizes (bytes) ─────────────────────────────────────
    // ESP32 FreeRTOS : minimum recommandé 2048, éviter < 1024
    constexpr uint32_t STACK_SENSORS   = 6144;  // Lidar parser + TOF + ligne
    constexpr uint32_t STACK_STRATEGY  = 8192;  // Logique sumo (MPC : sinf/cosf × 300 iter)
    constexpr uint32_t STACK_MOTORS    = 3072;  // PWM + queue commandes
    constexpr uint32_t STACK_COMM      = 20480; // WiFi + MQTT (gros stack requis)
    constexpr uint32_t STACK_LOGGER    = 2048;  // Serial.print centralisé
    constexpr uint32_t STACK_ENCODER   = 4096;  // Lecture compteurs interruption + LQR float
    constexpr uint32_t STACK_COMMAND   = 3072;  // Parsing commandes + séquences
    constexpr uint32_t STACK_LED       = 1024;  // LED RGB — état robot
    constexpr uint32_t STACK_LIDAR     = 4096;  // DMA UART lidar parser
    constexpr uint32_t STACK_OTA       = 8192;  // ArduinoOTA handler (MD5 + réseau)

    // ── Priorités (0=plus basse, 24=plus haute sur ESP32) ───────
    // Règle : plus le timing est critique, plus la priorité est haute
    constexpr UBaseType_t PRIO_ENCODER   = 5;   // Maximale — compteurs impulsions
    constexpr UBaseType_t PRIO_MOTORS    = 4;   // Critique — contrôle moteurs
    constexpr UBaseType_t PRIO_SENSORS   = 3;   // Haute    — lecture capteurs
    constexpr UBaseType_t PRIO_STRATEGY  = 2;   // Normale  — décision robot
    constexpr UBaseType_t PRIO_COMMAND   = 2;   // Normale  — commandes HMI
    constexpr UBaseType_t PRIO_COMM      = 1;   // Basse    — WiFi/MQTT
    constexpr UBaseType_t PRIO_LOGGER    = 1;   // Basse    — Serial debug
    constexpr UBaseType_t PRIO_LED       = 0;   // Minimale — affichage état
    constexpr UBaseType_t PRIO_LIDAR     = 2;   // Normale  — réception DMA lidar
    constexpr UBaseType_t PRIO_OTA       = 4;   // Haute    — transfert OTA prioritaire

    // ── Affectation des cœurs ───────────────────────────────────
    // Core 0 → WiFi/BT réservé par ESP-IDF (ne pas y mettre logique temps-réel)
    // Core 1 → logique robot
    constexpr BaseType_t CORE_SENSORS   = 1;
    constexpr BaseType_t CORE_STRATEGY  = 1;
    constexpr BaseType_t CORE_MOTORS    = 1;
    constexpr BaseType_t CORE_COMM      = 0;    // obligatoire pour WiFi
    constexpr BaseType_t CORE_LOGGER    = 0;
    constexpr BaseType_t CORE_ENCODER   = 1;    // temps-réel → Core 1
    constexpr BaseType_t CORE_COMMAND   = 1;    // traitement commandes → Core 1
    constexpr BaseType_t CORE_LED       = 0;    // non temps-réel → Core 0
    constexpr BaseType_t CORE_LIDAR     = 1;    // temps-réel → Core 1
    constexpr BaseType_t CORE_OTA       = 0;    // réseau → Core 0

    // ── Périodes de mise à jour (ms → ticks) ────────────────────
    constexpr TickType_t PERIOD_SENSORS   = pdMS_TO_TICKS(20);   // 50 Hz
    constexpr TickType_t PERIOD_STRATEGY  = pdMS_TO_TICKS(50);   // 20 Hz
    constexpr TickType_t PERIOD_MOTORS    = pdMS_TO_TICKS(10);   // 100 Hz
    constexpr TickType_t PERIOD_COMM      = pdMS_TO_TICKS(100);  // 10 Hz
    constexpr TickType_t PERIOD_ENCODER   = pdMS_TO_TICKS(5);    // 200 Hz
    constexpr TickType_t PERIOD_LED       = pdMS_TO_TICKS(100); // 10 Hz

    // ── Logger queue ─────────────────────────────────────────────
    constexpr uint8_t  LOG_QUEUE_SIZE   = 16;   // nb messages en attente max
    constexpr uint8_t  LOG_MSG_MAX_LEN  = 128;  // longueur max d'un message
}

// ───────────────────────────────────────────────────────────────
//  Mutex global — protège Serial (défini dans main.cpp)
// ───────────────────────────────────────────────────────────────
extern SemaphoreHandle_t g_logMutex;

// ───────────────────────────────────────────────────────────────
//  Macros LOG thread-safe
//  → Attente max 5 ms pour obtenir le mutex, sinon message ignoré
//    (évite le blocage des tâches temps-réel)
// ───────────────────────────────────────────────────────────────
#ifdef DEBUG_ENABLED

  #define LOG(msg)                                                        \
    do {                                                                  \
        if (g_logMutex &&                                                 \
            xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {    \
            Serial.println(msg);                                          \
            xSemaphoreGive(g_logMutex);                                   \
        }                                                                 \
    } while(0)

  #define LOGF(fmt, ...)                                                  \
    do {                                                                  \
        if (g_logMutex &&                                                 \
            xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {    \
            Serial.printf(fmt, ##__VA_ARGS__);                            \
            xSemaphoreGive(g_logMutex);                                   \
        }                                                                 \
    } while(0)

#else
  #define LOG(msg)          // désactivé — zéro coût
  #define LOGF(fmt, ...)    // désactivé — zéro coût
#endif

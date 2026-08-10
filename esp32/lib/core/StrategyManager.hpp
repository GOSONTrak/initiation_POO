#pragma once
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"
#include "RTOSConfig.hpp"

// ═══════════════════════════════════════════════════════════════
//  StrategyManager.hpp — Gestionnaire de stratégies
//
//  OOP appliqué :
//  - Composition    : contient un pointeur vers StrategyInterface
//  - Polymorphisme  : executeStrategy() appelle la bonne execute()
//                     sans connaître la stratégie concrète
//  - Encapsulation  : le changement de stratégie est contrôlé
//
//  Utilisation dans l'architecture FreeRTOS :
//  - taskStrategy appelle executeStrategy() toutes les 50 ms
// ═══════════════════════════════════════════════════════════════

class StrategyManager {
public:

    StrategyManager() = default;

    // ── Enregistrer une stratégie (ordre = ordre de cycle) ────
    void registerStrategy(StrategyInterface* s) {
        if (_count < MAX_STRATEGIES) {
            _strategies[_count++] = s;
            if (_count == 1) setStrategy(s);  // première = défaut
        }
    }

    // ── Changer de stratégie ──────────────────────────────────
    void setStrategy(StrategyInterface* strategy) {
        _current = strategy;
        if (_current) {
            _current->reset();
            LOGF("[StrategyManager] Stratégie active : %s\n", _current->name());
        }
    }

    // ── Passer à la stratégie suivante (cycle) ────────────────
    void nextStrategy() {
        if (_count == 0) return;
        _currentIdx = (_currentIdx + 1) % _count;
        setStrategy(_strategies[_currentIdx]);
    }

    // ── Sélectionner par nom (commande HMI) ───────────────────
    bool setByName(const char* name) {
        for (uint8_t i = 0; i < _count; i++) {
            if (strcasecmp(_strategies[i]->name(), name) == 0) {
                _currentIdx = i;
                setStrategy(_strategies[i]);
                return true;
            }
        }
        LOGF("[StrategyManager] Stratégie inconnue : %s\n", name);
        return false;
    }

    // ── Exécuter la stratégie courante ────────────────────────
    RobotConstants::ActionCommand executeStrategy(RobotContext& ctx) {
        if (!_current) return RobotConstants::ActionCommand{};
        return _current->execute(ctx);
    }

    // ── Accesseurs ────────────────────────────────────────────
    const char* currentName()       const { return _current ? _current->name()       : "none"; }
    LedColor    currentColor()      const { return _current ? _current->color()      : LedColor::YELLOW; }
    bool        hasStrategy()       const { return _current != nullptr; }
    bool        currentNeedsLidar() const { return _current ? _current->needsLidar() : false; }
    bool        currentNeedsLQR()   const { return _current ? _current->needsLQR()   : true;  }

private:
    static constexpr uint8_t MAX_STRATEGIES = 10;
    StrategyInterface* _strategies[MAX_STRATEGIES] = {};
    uint8_t            _count      = 0;
    uint8_t            _currentIdx = 0;
    StrategyInterface* _current    = nullptr;
};

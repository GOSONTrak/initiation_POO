#pragma once
#include "RobotConstants.hpp"
#include "DriverLedRGB.hpp"

// ═══════════════════════════════════════════════════════════════
//  StrategyInterface.hpp — Interface abstraite des stratégies
//
//  OOP appliqué :
//  - Classe abstraite pure (aucune implémentation concrète ici)
//  - Polymorphisme : StrategyManager manipule uniquement ce type
//  - Destructeur virtuel : requis pour delete via pointeur de base
//
//  Utilisation du forward declare :
//  - RobotContext est forward-déclaré (pas besoin de l'include complet)
//    → évite la dépendance circulaire
//    → execute() prend RobotContext& (référence → pointeur opaque ok)
// ═══════════════════════════════════════════════════════════════

// Forward declaration — évite d'inclure RobotContext.hpp ici
// (RobotContext.hpp inclura StrategyInterface indirectement)
class RobotContext;

class StrategyInterface {
public:
    virtual ~StrategyInterface() = default;

    // ── Méthode principale ────────────────────────────────────
    //  Reçoit l'état complet du robot, retourne une commande moteur.
    //  Appelée par StrategyManager::executeStrategy() depuis taskStrategy.
    virtual RobotConstants::ActionCommand execute(RobotContext& ctx) = 0;

    // ── Appelé à chaque activation de la stratégie ───────────
    //  Implémentation par défaut vide — surcharger si nécessaire.
    virtual void reset() {}

    // ── Nom de la stratégie ───────────────────────────────────
    virtual const char* name() const = 0;

    // ── Couleur LED associée ──────────────────────────────────
    virtual LedColor color() const = 0;

    // ── Lidar requis ? ────────────────────────────────────────
    //  false par défaut — stratégies géométriques (Square, Circle…)
    //  Override avec return true si la stratégie lit getLidarData()
    virtual bool needsLidar() const { return false; }

    // ── LQR cap requis ? ──────────────────────────────────────
    //  true par défaut — désactiver pour les stratégies qui changent
    //  de direction trop fréquemment (Q-learning, etc.)
    virtual bool needsLQR()   const { return true; }
};

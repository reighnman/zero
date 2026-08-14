#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when the target is weak enough - both relative to us and in absolute terms - that ending
// the fight is a better bet than breaking off, even though our own energy is low enough that we'd
// normally be retreating.
//
// This is the exception to the hard engage floor: never fight below the floor *unless* we know they
// are lower than we are and we can actually finish them. Both halves of that matter. "Lower than us"
// alone is not enough at 14% against 13% - that's a coin flip, not a kill - so a target also has to
// be near death in its own right before this opens.
//
// Both sides are percentages of their own ship's true maximum energy, the same normalization
// EnergyDisadvantageNode uses, because raw energy means something different on every ship and
// bounty.
//
// The constants deliberately leave a wide margin. Our own energy is exact, but the target's is a
// HeuristicEnergyTracker estimate that is capped well below their true maximum, so their percent
// reads *lower* than reality - the error points toward believing a target is finishable when it
// isn't, which is the direction that gets us killed. A tight threshold here would be systematically
// wrong in the dangerous direction.
struct FinishableTargetNode : public behavior::BehaviorNode {
  FinishableTargetNode(const char* target_player_key, const char* target_energy_key, float max_relative_percent,
                       float max_absolute_percent)
      : target_player_key(target_player_key),
        target_energy_key(target_energy_key),
        max_relative_percent(max_relative_percent),
        max_absolute_percent(max_absolute_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_target_energy) return behavior::ExecuteResult::Failure;

    float self_max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    float target_max_energy = (float)ctx.bot->game->connection.settings.ShipSettings[target->ship].MaximumEnergy;
    if (self_max_energy <= 0.0f || target_max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    float self_percent = self->energy / self_max_energy;
    float target_percent = *opt_target_energy / target_max_energy;

    if (target_percent >= max_absolute_percent) return behavior::ExecuteResult::Failure;
    if (target_percent >= self_percent * max_relative_percent) return behavior::ExecuteResult::Failure;

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* target_energy_key = nullptr;
  float max_relative_percent = 0.5f;
  float max_absolute_percent = 0.2f;
};

}  // namespace nexus
}  // namespace zero

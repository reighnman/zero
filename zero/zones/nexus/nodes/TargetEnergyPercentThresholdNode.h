#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Threshold check against a target's energy as a percent of *their* true maximum energy, instead
// of the target's raw estimated energy value against a ship-specific magic number. The energy level
// worth committing to a kill at is consistent as a percent of max energy across ships/bounties, not
// as an absolute value - the same percent math EnergyDisadvantageNode already uses internally for its own
// self-vs-target comparison, exposed here as a standalone threshold check for target-only use
// (rush/press/thor triggers that don't need the full hysteresis state machine).
struct TargetEnergyPercentThresholdNode : public behavior::BehaviorNode {
  TargetEnergyPercentThresholdNode(const char* target_player_key, const char* target_energy_key, float percent)
      : target_player_key(target_player_key), target_energy_key(target_energy_key), percent(percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_target_energy) return behavior::ExecuteResult::Failure;

    float target_max_energy = (float)ctx.bot->game->connection.settings.ShipSettings[target->ship].MaximumEnergy;
    if (target_max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    float target_percent = *opt_target_energy / target_max_energy;

    return target_percent < percent ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  const char* target_energy_key = nullptr;
  float percent = 0.2f;
};

}  // namespace nexus
}  // namespace zero

#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Gates a weapon's use behind the (self energy%, distance-to-target) band it was actually observed
// used in across 21 real 4v4 SVS league replays (ZeroReplayAnalyzer), instead of each fire sequence
// re-deriving its own ad hoc PlayerEnergyPercentThresholdNode + DistanceThresholdNode pair with
// unrelated magic numbers. `max_distance < 0` means no distance ceiling (some weapons, like
// bullets, showed no real upper range limit in the data - they're just used whenever a shot lines
// up, out to the edge of aiming range).
struct WeaponEnergyBandNode : public behavior::BehaviorNode {
  WeaponEnergyBandNode(float min_self_energy_percent, const char* target_position_key = nullptr,
                        float max_distance = -1.0f)
      : min_self_energy_percent(min_self_energy_percent),
        target_position_key(target_position_key),
        max_distance(max_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    if (max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    if (self->energy / max_energy < min_self_energy_percent) return behavior::ExecuteResult::Failure;

    if (target_position_key && max_distance >= 0.0f) {
      auto opt_position = ctx.blackboard.Value<Vector2f>(target_position_key);
      if (!opt_position) return behavior::ExecuteResult::Failure;
      if (self->position.Distance(*opt_position) > max_distance) return behavior::ExecuteResult::Failure;
    }

    return behavior::ExecuteResult::Success;
  }

  float min_self_energy_percent = 0.0f;
  const char* target_position_key = nullptr;
  float max_distance = -1.0f;
};

}  // namespace nexus
}  // namespace zero

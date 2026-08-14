#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Publishes how far to break off, scaled by how badly hurt we are, for FleeNode to consume through
// its target_distance_key constructor.
//
// THE PROBLEM THIS FIXES. FleeNode's distance was a constant 30 tiles, and it is a leash rather
// than an escape: past target_distance + max_overshoot it seeks back IN toward the standoff point,
// between the two it holds broadside without thrusting, and inside it retreats until it is back
// out to 30. So an injured bot is actively pinned to a 30-35 tile band, and a bot knocked further
// out than that flies back toward the enemy to rejoin it.
//
// 30 tiles is a sound kiting standoff at full energy. It is the wrong number when hurt, and the
// replay says so precisely - measuring the approach that leads to each kill:
//
//     T-3.0s   killer-victim range  30.8 tiles     <- the leash distance
//     T-2.0s                        21.4
//     T-1.0s                        17.3
//     T-0.5s                        13.6
//     T-0.0s                         3.8
//
// Killers begin their run at almost exactly the distance the leash holds an injured bot at, and
// cover it in three seconds. The bot spends its whole recharge sitting in the band where kills
// start, and FleeNode's own low-energy getaway (which does abandon the leash and simply open
// distance) is keyed to an energy percent it only reaches once the killer is already inside.
//
// So "far enough" has to be a function of how hurt we are rather than a constant. At full energy
// this returns healthy_distance and nothing changes; as energy falls toward hurt_energy_percent it
// ramps to hurt_distance, which sits outside both effective bullet range and the observed kill
// funnel. Below that the ramp is done and FleeNode's own panic threshold takes over with no leash
// at all.
//
// Ramping rather than switching matters: a step change would produce a visible lurch outward at one
// energy value, and would be one more threshold to tune. A ramp means the bot gives ground
// continuously as it loses energy, which is also what it looks like when a person does it.
struct FleeDistanceNode : public behavior::BehaviorNode {
  FleeDistanceNode(const char* output_key, float healthy_distance, float hurt_distance, float hurt_energy_percent)
      : output_key(output_key),
        healthy_distance(healthy_distance),
        hurt_distance(hurt_distance),
        hurt_energy_percent(hurt_energy_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    if (max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    float energy_percent = self->energy / max_energy;

    // 1 at full energy, 0 at or below the hurt threshold.
    float t = 1.0f;

    if (hurt_energy_percent < 1.0f) {
      t = (energy_percent - hurt_energy_percent) / (1.0f - hurt_energy_percent);
    }

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    ctx.blackboard.Set<float>(output_key, hurt_distance + (healthy_distance - hurt_distance) * t);

    return behavior::ExecuteResult::Success;
  }

  const char* output_key = nullptr;

  float healthy_distance = 30.0f;
  float hurt_distance = 55.0f;
  float hurt_energy_percent = 0.35f;
};

}  // namespace nexus
}  // namespace zero

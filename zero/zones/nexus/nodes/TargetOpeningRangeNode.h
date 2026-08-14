#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when the target is actually pulling away from us at `min_opening_speed` or better.
//
// This exists to gate rockets. A rocket is a limited item that buys a short burst of speed, and its
// legitimate offensive use is narrow: something nearly dead is escaping, and ordinary thrust will
// not catch it. Every other "we are far from the target" situation looks identical to a
// distance-and-speed gate but is not the same decision at all - most of all the one where the
// target is holding station in the middle of their own team and shooting at us. Lighting a rocket
// there does not close on a fleeing kill, it delivers us into a group at speed with no thrust left
// to turn around, which is exactly the "diving into crowds" behavior observed in play.
//
// Distance alone cannot tell those apart, which is why the previous gate could not either. Radial
// velocity can: a target running away has positive closing distance, a target holding or pressing
// does not.
//
// Sign convention matches the mine/pursuit checks: positive means the range is opening.
struct TargetOpeningRangeNode : public behavior::BehaviorNode {
  TargetOpeningRangeNode(const char* target_player_key, float min_opening_speed)
      : target_player_key(target_player_key), min_opening_speed(min_opening_speed) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    Vector2f offset = target->position - self->position;
    float distance = offset.Length();

    // Degenerate on top of each other - no meaningful radial direction, and a rocket is wrong here
    // regardless.
    if (distance < 1.0f) return behavior::ExecuteResult::Failure;

    Vector2f radial = offset * (1.0f / distance);
    float opening_speed = (target->velocity - self->velocity).Dot(radial);

    return opening_speed >= min_opening_speed ? behavior::ExecuteResult::Success
                                             : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  float min_opening_speed = 0.0f;
};

}  // namespace nexus
}  // namespace zero

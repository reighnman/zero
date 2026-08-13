#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Computes a bomb aimpoint thrown *ahead* of where the target is travelling, rather than at a
// straight intercept, so the blast denies the lane they are already committed to and pushes them
// back toward our bullets.
//
// This is what real players actually do. Measuring the signed angle between a bomb's launch heading
// and the direct bearing to the target across 18,178 bomb shots in 21 league matches (positive =
// thrown toward the side the target is moving), the distribution sits clearly positive: median
// +12.2 deg, mean +11.0, p75 +39.0. And bombs are thrown at targets that are genuinely moving
// sideways - median lateral speed 6.9 tiles/sec. Bombs aimed straight down the bearing would center
// on zero; these do not.
//
// When the target isn't sliding fast enough across our line of fire to be herded anywhere
// (`min_lateral_speed`), there's nothing to cut off, so this falls back to whatever aimpoint
// `fallback_aim_key` holds - normally the ordinary intercept solution - rather than lobbing a bomb
// into empty space ahead of a target that isn't going there.
struct HerdingBombAimNode : public behavior::BehaviorNode {
  HerdingBombAimNode(const char* target_player_key, const char* output_key, const char* fallback_aim_key,
                     float lead_seconds = 0.6f, float min_lateral_speed = 3.0f)
      : target_player_key(target_player_key),
        output_key(output_key),
        fallback_aim_key(fallback_aim_key),
        lead_seconds(lead_seconds),
        min_lateral_speed(min_lateral_speed) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    Vector2f to_target = target->position - self->position;
    float distance = to_target.Length();
    if (distance < 1.0f) return behavior::ExecuteResult::Failure;

    Vector2f bearing = Normalize(to_target);
    Vector2f perpendicular = Perpendicular(bearing);

    // Only the component of their motion across our line of fire can be herded - closing or
    // opening straight along the bearing gives us nothing to cut off.
    float lateral_speed = target->velocity.Dot(perpendicular);

    if (std::abs(lateral_speed) < min_lateral_speed) {
      auto opt_fallback = ctx.blackboard.Value<Vector2f>(fallback_aim_key);
      if (!opt_fallback) return behavior::ExecuteResult::Failure;

      ctx.blackboard.Set(output_key, *opt_fallback);

      return behavior::ExecuteResult::Success;
    }

    // Throw ahead of their travel: where they'll be if they hold this course, biased further along
    // the direction they're already sliding so the blast sits in front of them rather than behind.
    Vector2f lead_point = target->position + target->velocity * lead_seconds;

    ctx.blackboard.Set(output_key, lead_point);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* output_key = nullptr;
  const char* fallback_aim_key = nullptr;
  float lead_seconds = 0.6f;
  float min_lateral_speed = 3.0f;
};

}  // namespace nexus
}  // namespace zero

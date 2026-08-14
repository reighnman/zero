#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Copies an aim point but refuses to let it drift more than `max_lead_distance` tiles from where the
// target actually is right now.
//
// Predictive aim solves for where a target will be after the projectile's flight time, and that is
// the right answer only as long as the target keeps doing what it is doing. Flight time grows with
// range, so the lead grows with range, and the CONFIDENCE in it falls at the same time - by the far
// end of a 78-tile lob the solver is extrapolating a second or more ahead and placing the aim point
// tens of tiles from the ship it is aiming at. That aim point is frequently somewhere the target was
// never going, and often inside terrain, which then fails the line-of-sight gate and quietly kills
// the shot as well as missing with it.
//
// Clamping keeps the useful part and discards the speculative part. Inside the cap the lead is
// untouched, so ordinary short-range prediction is unaffected; beyond it the aim collapses back
// toward the target's present position. The shot becomes "near where they are, slightly ahead"
// rather than "exactly where a constant-velocity model insists they will be", which at this range is
// the better bet: a long lob is an area weapon thrown at a group that has not necessarily noticed
// it, and catching someone who did not dodge beats precisely missing someone who did.
//
// Direction is preserved when clamping rather than snapping to the target, so the shot still leads
// the correct way - it is only the magnitude that is capped.
struct ClampedLeadAimNode : public behavior::BehaviorNode {
  ClampedLeadAimNode(const char* target_player_key, const char* aim_key, const char* output_key,
                     float max_lead_distance)
      : target_player_key(target_player_key),
        aim_key(aim_key),
        output_key(output_key),
        max_lead_distance(max_lead_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    auto opt_aim = ctx.blackboard.Value<Vector2f>(aim_key);
    if (!opt_aim.has_value()) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;

    Vector2f lead = *opt_aim - target->position;
    float lead_distance = lead.Length();

    Vector2f aim = *opt_aim;

    if (lead_distance > max_lead_distance && lead_distance > 0.0001f) {
      aim = target->position + lead * (max_lead_distance / lead_distance);
    }

    ctx.blackboard.Set<Vector2f>(output_key, aim);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* aim_key = nullptr;
  const char* output_key = nullptr;
  float max_lead_distance = 0.0f;
};

}  // namespace nexus
}  // namespace zero

#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Circles a target at roughly `orbit_distance` instead of closing straight in to melee range.
//
// Weapon speed is self velocity plus base weapon speed along the ship's *heading*
// (PredictiveAimNode), and FaceNode already points the heading at the aimshot independently of
// whatever this node does for movement - so a bot that always just seeks straight at its target
// always fires from the same closing-velocity profile with zero lateral movement of its own,
// making both its bullets and its own position completely predictable. Thrusting tangentially
// instead varies the approach angle shot to shot and gets evasive movement "for free" out of the
// attack pattern itself, at the cost of less of our velocity landing along the fire heading.
//
// Commits to one rotation direction per engagement (`direction_key`, persisted like "rushing")
// instead of recomputing it every tick, which would just cancel out into no net rotation. Then
// aims at a lead point slightly ahead of our current angular position on the orbit circle and
// seeks toward it - since that point keeps moving around the circle each tick, chasing it traces
// an actual circular path instead of homing in on one fixed spot.
struct OrbitNode : public behavior::BehaviorNode {
  OrbitNode(const char* position_key, float orbit_distance, const char* direction_key)
      : position_key(position_key), orbit_distance(orbit_distance), direction_key(direction_key) {}

  // Reads the radius from the blackboard instead of a fixed value, so EngagementRangeNode can move
  // it in response to local head-count and the in-and-out pump.
  OrbitNode(const char* position_key, const char* orbit_distance_key, const char* direction_key)
      : position_key(position_key), orbit_distance_key(orbit_distance_key), direction_key(direction_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_position = ctx.blackboard.Value<Vector2f>(position_key);
    if (!opt_position.has_value()) return behavior::ExecuteResult::Failure;

    float orbit_distance = this->orbit_distance;

    if (orbit_distance_key) {
      auto opt_distance = ctx.blackboard.Value<float>(orbit_distance_key);
      if (!opt_distance.has_value()) return behavior::ExecuteResult::Failure;

      orbit_distance = *opt_distance;
    }

    Vector2f target_position = *opt_position;
    Vector2f to_self = self->position - target_position;

    if (to_self.LengthSq() < 1.0f) {
      // Degenerate - right on top of the target, any direction is as good as any other to start
      // opening up an orbit radius.
      to_self = Vector2f(1, 0);
    }

    float direction = GetOrbitDirection(ctx, to_self);

    float current_angle = atan2f(to_self.y, to_self.x);
    float lead_angle = current_angle + direction * kLeadAngleRadians;

    Vector2f lead_point = target_position + Vector2f(cosf(lead_angle), sinf(lead_angle)) * orbit_distance;

    auto& game = *ctx.bot->game;
    auto& steering = ctx.bot->bot_controller->steering;

    steering.Seek(game, lead_point);
    steering.AvoidWalls(game);

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
  const char* direction_key = nullptr;
  const char* orbit_distance_key = nullptr;
  float orbit_distance = 0.0f;

 private:
  // How far ahead (radians) along the orbit circle to aim the chase point. Small enough that Seek
  // is always pulling mostly tangentially rather than cutting across toward the target.
  static constexpr float kLeadAngleRadians = 0.6f;

  // Picks whichever rotation direction is already closer to our current heading, so committing to
  // it doesn't cost a hard turn, then holds that choice for the rest of the engagement.
  float GetOrbitDirection(behavior::ExecuteContext& ctx, const Vector2f& to_self) const {
    auto opt_direction = ctx.blackboard.Value<float>(direction_key);
    if (opt_direction) return *opt_direction;

    Player* self = ctx.bot->game->player_manager.GetSelf();

    Vector2f tangent = Perpendicular(Normalize(to_self));
    float direction = tangent.Dot(self->GetHeading()) >= 0.0f ? 1.0f : -1.0f;

    ctx.blackboard.Set<float>(direction_key, direction);

    return direction;
  }
};

}  // namespace nexus
}  // namespace zero

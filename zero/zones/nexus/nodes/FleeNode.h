#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Retreats away from a target, for defensive kiting/leashing.
//
// Plain Seek-based retreats push straight away from the target with no regard for the map, so a
// bot backing away in a straight line can get driven into a wall or corner and pinned there. This
// blends in gentle wall avoidance while retreating, but relies on WallAvoidanceNode being checked
// ahead of it in the tree to override and steer clear before that happens - this node alone isn't
// enough to get out of a corner once actually wedged in one.
//
// Once already at or beyond the desired distance, it no longer needs to burn speed retreating
// further, so it turns broadside to the target instead of continuing to move. Subspace ships only
// thrust forward/backward along their current heading, so a dodge is fastest when that heading is
// already perpendicular to the threat; reactively dodging from a face-on orientation costs a
// rotation before any real lateral velocity builds up. Facing broadside in advance means
// DodgeIncomingDamage can convert straight into forward/backward thrust the instant it's needed.
//
// Holding broadside doesn't cancel momentum though, so a ship that was still accelerating away
// when it crossed the leash distance keeps coasting outward. Past `target_distance +
// max_overshoot`, pull back in toward the leash instead of just holding broadside, so residual
// drift doesn't strand us too far from the fight to quickly re-engage.
struct FleeNode : public behavior::BehaviorNode {
  FleeNode(const char* position_key, float target_distance, float max_overshoot = 5.0f)
      : position_key(position_key), target_distance(target_distance), max_overshoot(max_overshoot) {}
  FleeNode(const char* position_key, const char* target_distance_key, float max_overshoot = 5.0f)
      : position_key(position_key), target_distance_key(target_distance_key), max_overshoot(max_overshoot) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_position = ctx.blackboard.Value<Vector2f>(position_key);
    if (!opt_position.has_value()) return behavior::ExecuteResult::Failure;

    float distance = target_distance;

    if (target_distance_key) {
      auto opt_distance = ctx.blackboard.Value<float>(target_distance_key);
      if (!opt_distance) return behavior::ExecuteResult::Failure;

      distance = *opt_distance;
    }

    Vector2f threat_position = *opt_position;
    auto& game = *ctx.bot->game;
    auto& steering = ctx.bot->bot_controller->steering;

    Vector2f to_threat = threat_position - self->position;
    float distance_sq = to_threat.LengthSq();
    float max_distance = distance + max_overshoot;

    if (distance_sq > max_distance * max_distance) {
      // Drifted past the leash margin on momentum alone - pull back in toward the standoff point
      // instead of continuing to hold broadside and coast further away.
      Vector2f standoff_point = threat_position + Normalize(self->position - threat_position) * distance;

      steering.Seek(game, standoff_point);
      steering.AvoidWalls(game);

      return behavior::ExecuteResult::Success;
    }

    if (distance_sq > distance * distance) {
      // Within the leash margin — hold here broadside instead of continuing to move, so we're
      // ready to dodge along this axis the instant it's needed.
      Vector2f broadside_direction = GetBroadsideDirection(*self, threat_position);

      steering.Face(game, self->position + broadside_direction);
      steering.AvoidWalls(game);

      return behavior::ExecuteResult::Success;
    }

    // Same stand-off behavior as Seek, but blends in wall avoidance so retreating away from the
    // target steers around walls instead of being driven straight into them.
    steering.Seek(game, threat_position, distance);
    steering.AvoidWalls(game);

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
  const char* target_distance_key = nullptr;
  float target_distance = 0.0f;
  float max_overshoot = 5.0f;

 private:
  // Returns whichever perpendicular-to-threat heading is closer to our current facing, so turning
  // to face it costs the smaller rotation. Either direction along that axis is equally useful for
  // a forward/backward dodge, since Actuator already picks whichever of forward/backward thrust
  // requires less rotation to reach.
  static Vector2f GetBroadsideDirection(const Player& self, const Vector2f& threat_position) {
    Vector2f away_direction = Normalize(self.position - threat_position);
    Vector2f broadside = Perpendicular(away_direction);

    if (broadside.Dot(self.GetHeading()) < 0.0f) {
      broadside = -broadside;
    }

    return broadside;
  }
};

}  // namespace nexus
}  // namespace zero

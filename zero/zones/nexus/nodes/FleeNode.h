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
//
// While actively retreating (not yet holding broadside), faces the threat instead of leaving
// orientation to fall out of the retreat force. Ships thrust exactly as hard backward as forward,
// so Actuator already picks Backward whenever the desired movement is behind current heading -
// pointing the nose at the threat means retreating never costs a 180-degree turn or the aim that
// comes with it, unlike leaving orientation to default to the movement direction.
//
// Below `low_energy_percent`, skips the leash altogether and keeps actively retreating no matter
// how far past `target_distance` that goes. At that point this isn't a stall-for-recharge posture
// anymore, it's a getaway - there's no "far enough" to settle for and hold broadside at when a
// single hit could be fatal; every extra unit of distance is still worth having.
//
// `pressing_target_energy_key`, if given, names the energy of the target actually being pressed -
// not necessarily the same player as `position_key`'s threat, since that may be the nearest enemy
// rather than the one we're fighting. Still having more energy than that target overrides the
// low-energy panic entirely: the fight itself is still winnable, so breaking off to run from a
// closer, unrelated threat would be throwing away an advantage for no reason.
struct FleeNode : public behavior::BehaviorNode {
  FleeNode(const char* position_key, float target_distance, float max_overshoot = 5.0f,
           float low_energy_percent = 0.2f, const char* pressing_target_energy_key = nullptr)
      : position_key(position_key),
        target_distance(target_distance),
        max_overshoot(max_overshoot),
        low_energy_percent(low_energy_percent),
        pressing_target_energy_key(pressing_target_energy_key) {}
  FleeNode(const char* position_key, const char* target_distance_key, float max_overshoot = 5.0f,
           float low_energy_percent = 0.2f, const char* pressing_target_energy_key = nullptr)
      : position_key(position_key),
        target_distance_key(target_distance_key),
        max_overshoot(max_overshoot),
        low_energy_percent(low_energy_percent),
        pressing_target_energy_key(pressing_target_energy_key) {}

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

    // Below this energy, there's no leash to settle for - just keep opening distance. Unless
    // we're still ahead of the target we're actually pressing, in which case the fight is still
    // worth finishing rather than breaking off for an unrelated nearby threat.
    float self_energy_percent = self->energy / (float)game.ship_controller.ship.energy;
    bool pressing_advantage = false;

    if (pressing_target_energy_key) {
      auto opt_target_energy = ctx.blackboard.Value<float>(pressing_target_energy_key);
      pressing_advantage = opt_target_energy.has_value() && self->energy > *opt_target_energy;
    }

    bool panicking = !pressing_advantage && self_energy_percent < low_energy_percent;

    if (!panicking && distance_sq > max_distance * max_distance) {
      // Drifted past the leash margin on momentum alone - pull back in toward the standoff point
      // instead of continuing to hold broadside and coast further away.
      Vector2f standoff_point = threat_position + Normalize(self->position - threat_position) * distance;

      steering.Face(game, threat_position);
      steering.Seek(game, standoff_point);
      steering.AvoidWalls(game);

      return behavior::ExecuteResult::Success;
    }

    if (!panicking && distance_sq > distance * distance) {
      // Within the leash margin — hold here broadside instead of continuing to move, so we're
      // ready to dodge along this axis the instant it's needed.
      Vector2f broadside_direction = GetBroadsideDirection(*self, threat_position);

      steering.Face(game, self->position + broadside_direction);
      steering.AvoidWalls(game);

      return behavior::ExecuteResult::Success;
    }

    // Same stand-off behavior as Seek, but blends in wall avoidance so retreating away from the
    // target steers around walls instead of being driven straight into them. Facing the threat
    // while doing so means the retreat force ends up behind our heading, so Actuator backs us
    // away with reverse thrust instead of turning around to face the retreat direction.
    steering.Face(game, threat_position);

    if (panicking) {
      // Seek's 3-arg overload switches to closing back in once past `distance`, which is exactly
      // wrong while panicking - seek an away point instead, so it keeps opening distance no matter
      // how far out that goes.
      Vector2f away_direction = Normalize(self->position - threat_position);
      steering.Seek(game, self->position + away_direction * 1000.0f);
    } else {
      steering.Seek(game, threat_position, distance);
    }

    steering.AvoidWalls(game);

    // steering.force is a single accumulator shared by every node that runs this tick, and nodes
    // earlier in the tree (DodgeIncomingDamage's minor nudge, in particular) blend into it whether
    // or not they're the reason movement is happening. Seek's own contribution above shrinks toward
    // zero once already cruising at retreat speed, which can be most of an active retreat - once it
    // does, a small unrelated nudge is free to decide the sign of the combined force on its own.
    // Actuator picks Forward whenever that combined force reads as ahead of heading, which here
    // means thrusting into the threat instead of away from it. Clamp the heading-aligned component
    // so this branch's "away" guarantee holds regardless of what else added to force this tick.
    float heading_component = steering.force.Dot(self->GetHeading());
    if (heading_component > -kMinRetreatForce) {
      steering.force -= self->GetHeading() * (heading_component + kMinRetreatForce);
    }

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
  const char* target_distance_key = nullptr;
  const char* pressing_target_energy_key = nullptr;
  float target_distance = 0.0f;
  float max_overshoot = 5.0f;
  float low_energy_percent = 0.2f;

 private:
  // Minimum backward-facing force to guarantee during active retreat, so Actuator can never read
  // the combined steering.force as pointing toward the threat once Seek's own contribution decays.
  static constexpr float kMinRetreatForce = 1.0f;

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

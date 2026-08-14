#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>
#include <zero/zones/nexus/nodes/BroadsideFaceNode.h>

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
// `team_position_key`, if given, curves the retreat back toward that position (the team's centre of
// mass) instead of running dead away from the threat. Retreating along the pure away-vector is what
// converts a fight the bot is losing into the isolation that actually kills it: measured over the
// bot-vs-human replays, a bot ends a sustained retreat a median 3-7 tiles further from its nearest
// teammate than it started, across 33-62 retreats a match, while the human ends his at +1.4. His
// retreats also point much less away from his own team (median 84 degrees off the bearing to it,
// against the bots' 101-140).
//
// The bias is capped at `max_team_bias_radians` off the away-vector so the retreat never turns back
// into the threat - at 60 degrees we still open range at half rate while arcing toward support,
// which is the "slowly path back to the team" shape rather than a straight line either way.
struct FleeNode : public behavior::BehaviorNode {
  FleeNode(const char* position_key, float target_distance, float max_overshoot = 5.0f,
           float low_energy_percent = 0.2f, const char* pressing_target_energy_key = nullptr,
           const char* team_position_key = nullptr, float max_team_bias_radians = 1.05f)
      : position_key(position_key),
        target_distance(target_distance),
        max_overshoot(max_overshoot),
        low_energy_percent(low_energy_percent),
        pressing_target_energy_key(pressing_target_energy_key),
        team_position_key(team_position_key),
        max_team_bias_radians(max_team_bias_radians) {}
  FleeNode(const char* position_key, const char* target_distance_key, float max_overshoot = 5.0f,
           float low_energy_percent = 0.2f, const char* pressing_target_energy_key = nullptr,
           const char* team_position_key = nullptr, float max_team_bias_radians = 1.05f)
      : position_key(position_key),
        target_distance_key(target_distance_key),
        max_overshoot(max_overshoot),
        low_energy_percent(low_energy_percent),
        pressing_target_energy_key(pressing_target_energy_key),
        team_position_key(team_position_key),
        max_team_bias_radians(max_team_bias_radians) {}

  // Straight away from the threat, rotated toward the team's centre of mass by at most
  // `max_team_bias_radians`. The cap is what keeps this a retreat: staying within 60 degrees of the
  // away-vector still opens range, it just does so on an arc that ends up back with the team
  // instead of alone in a corner of the map.
  Vector2f GetRetreatDirection(behavior::ExecuteContext& ctx, Player& self, const Vector2f& threat_position) {
    Vector2f away = Normalize(self.position - threat_position);

    if (!team_position_key) return away;

    auto opt_team = ctx.blackboard.Value<Vector2f>(team_position_key);
    if (!opt_team.has_value()) return away;

    Vector2f to_team = *opt_team - self.position;
    if (to_team.LengthSq() < 1.0f) return away;

    to_team = Normalize(to_team);

    float cos_angle = away.Dot(to_team);
    if (cos_angle > 1.0f) cos_angle = 1.0f;
    if (cos_angle < -1.0f) cos_angle = -1.0f;

    float angle = acosf(cos_angle);
    if (angle <= 0.0001f) return away;

    float rotation = angle < max_team_bias_radians ? angle : max_team_bias_radians;

    // Rotate toward the team, whichever way round that is.
    float cross = away.x * to_team.y - away.y * to_team.x;
    if (cross < 0.0f) rotation = -rotation;

    float cos_r = cosf(rotation);
    float sin_r = sinf(rotation);

    return Vector2f(away.x * cos_r - away.y * sin_r, away.x * sin_r + away.y * cos_r);
  }

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

    // Which way "away" actually means - straight back from the threat, or arced toward our team.
    Vector2f retreat_direction = GetRetreatDirection(ctx, *self, threat_position);

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
      Vector2f standoff_point = threat_position + retreat_direction * distance;

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
      steering.Seek(game, self->position + retreat_direction * 1000.0f);
    } else {
      // Equivalent to Seek(game, threat_position, distance) - which resolves to the point at
      // `distance` from the threat on our side - except that the side is the biased retreat
      // direction rather than dead away.
      steering.Seek(game, threat_position + retreat_direction * distance);
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
  const char* team_position_key = nullptr;
  float target_distance = 0.0f;
  float max_overshoot = 5.0f;
  float low_energy_percent = 0.2f;
  float max_team_bias_radians = 1.05f;

 private:
  // Minimum backward-facing force to guarantee during active retreat, so Actuator can never read
  // the combined steering.force as pointing toward the threat once Seek's own contribution decays.
  static constexpr float kMinRetreatForce = 1.0f;
};

}  // namespace nexus
}  // namespace zero

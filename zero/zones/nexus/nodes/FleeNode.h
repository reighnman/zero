#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
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
    Vector2f desired = away;

    if (team_position_key) {
      auto opt_team = ctx.blackboard.Value<Vector2f>(team_position_key);

      if (opt_team.has_value()) {
        Vector2f to_team = *opt_team - self.position;

        if (to_team.LengthSq() >= 1.0f) {
          to_team = Normalize(to_team);

          float cos_angle = away.Dot(to_team);
          if (cos_angle > 1.0f) cos_angle = 1.0f;
          if (cos_angle < -1.0f) cos_angle = -1.0f;

          float angle = acosf(cos_angle);

          if (angle > 0.0001f) {
            float rotation = angle < max_team_bias_radians ? angle : max_team_bias_radians;

            // Rotate toward the team, whichever way round that is.
            float cross = away.x * to_team.y - away.y * to_team.x;
            if (cross < 0.0f) rotation = -rotation;

            desired = Rotate(away, rotation);
          }
        }
      }
    }

    return ChooseEscapeDirection(ctx, self, desired);
  }

  static Vector2f Rotate(const Vector2f& v, float radians) {
    float c = cosf(radians);
    float s = sinf(radians);

    return Vector2f(v.x * c - v.y * s, v.x * s + v.y * c);
  }

  // Pick an escape that is actually escapable: not into their other players, and not into a wall.
  //
  // Everything upstream of this reasons only about the enemy we are running FROM. The direction that
  // opens the most range from them is frequently the direction their teammate is holding, and a bot
  // reversing at 13 tiles/sec into a waiting second enemy is how a break-off turns into a kill for
  // the other side. The team-centroid bias helps by accident when our side happens to sit opposite
  // theirs, and not at all otherwise.
  //
  // Terrain belongs in the same decision, not in a separate pass, because the two are used together:
  // an enemy holding station beside a wall is funnelling, and the wall is doing as much of the work
  // as they are. Score them apart and each looks survivable on its own - the lane past the enemy has
  // room, the lane along the wall has no enemy in it - while the pair of them leaves only one way to
  // go. WallAvoidanceNode still runs ahead of this and still owns the cornered case; what it cannot
  // do is choose, because it is a correction applied to a heading that has already been picked.
  //
  // This is a check on the direction of TRAVEL, which is not the direction the ship is pointing.
  // While retreating FleeNode deliberately holds the nose on the threat and lets the Actuator resolve
  // the movement as reverse thrust, so the hull faces backwards along the escape the whole way and
  // nothing about the heading tells you what is in front.
  //
  // Candidate directions are scored rather than the desired one being corrected afterwards, because a
  // correction applied to an already-chosen heading only ever nudges - it cannot conclude that a
  // whole side is a bad idea. Deviation from `desired` is itself part of the cost, so with a clear
  // lane this returns `desired` unchanged.
  Vector2f ChooseEscapeDirection(behavior::ExecuteContext& ctx, Player& self, const Vector2f& desired) {
    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    // Gathered once - the scoring loop below runs over every candidate.
    Vector2f blockers[kMaxBlockers];
    size_t blocker_count = 0;

    for (size_t i = 0; i < game.player_manager.player_count && blocker_count < kMaxBlockers; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self.id) continue;
      if (player->frequency == self.frequency) continue;
      if (player->ship >= 8) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!IsSynchronized(game, *player)) continue;
      if (!region_registry.IsConnected(self.position, player->position)) continue;
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      if (player->position.DistanceSq(self.position) > kEscapeLookahead * kEscapeLookahead) continue;

      blockers[blocker_count++] = player->position;
    }

    float ship_radius = game.connection.settings.ShipSettings[self.ship].GetRadius() / 16.0f;

    Vector2f best = desired;
    float best_cost = -1.0f;

    for (int step = -kCandidateSteps; step <= kCandidateSteps; ++step) {
      float offset = step * kCandidateStepRadians;
      Vector2f candidate = Rotate(desired, offset);

      // Turning away from the best escape costs something, so a lane is only abandoned when it is
      // genuinely occupied rather than because an enemy is vaguely off to that side.
      float cost = fabsf(offset) * kDeviationWeight;

      // How far we could actually run this way before terrain stops us. Cast from the hull rather
      // than the centre so a wall we are already scraping doesn't read as zero room in every
      // direction at once.
      CastResult terrain =
          game.GetMap().Cast(self.position + candidate * ship_radius, candidate, kEscapeLookahead, self.frequency);

      float open = terrain.hit ? terrain.distance : kEscapeLookahead;
      float shortfall = 1.0f - (open / kEscapeLookahead);

      // Squared, so a wall at the far end of the lane is nearly free while one in our face is not.
      // A retreat that runs out of room in ten tiles is not a retreat.
      cost += shortfall * shortfall * kTerrainWeight;

      for (size_t i = 0; i < blocker_count; ++i) {
        Vector2f to_blocker = blockers[i] - self.position;

        float along = to_blocker.Dot(candidate);
        if (along <= 0.0f) continue;  // behind us on this heading, so not in the way

        float perp = fabsf(to_blocker.x * candidate.y - to_blocker.y * candidate.x);
        if (perp >= kEscapeCorridor) continue;

        // Worst when they sit squarely in the lane and close enough that we cannot turn out of it.
        float lane = 1.0f - (perp / kEscapeCorridor);
        float proximity = 1.0f - (along / kEscapeLookahead);
        if (proximity < 0.0f) proximity = 0.0f;

        cost += lane * proximity * kBlockerWeight;
      }

      if (best_cost < 0.0f || cost < best_cost) {
        best_cost = cost;
        best = candidate;
      }
    }

    return best;
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
    //
    // THAT POSTURE CANNOT STEER, which is why it has to be given up the moment terrain is in the
    // way. Reversing nose-locked leaves almost no directional authority, and the reason is exact:
    // Actuator blends the steering force against the rotation target, and when the two disagree by
    // more than `rotation_threshold` it pins the steering direction to within 0.1 RADIANS - about
    // 5.7 degrees - of the rotation target (Actuator.cpp, the `steering_direction.Dot(rotate_target)
    // < rotation_threshold` branch). Facing the threat while the retreat force points the opposite
    // way is the maximum possible disagreement, so it clamps every tick and the bot reverses in a
    // near-straight line. Wall avoidance can pile on as much force as it likes; the hull barely
    // turns, and we back into the wall anyway. Reported directly, twice, as getting stuck against
    // walls while flying backwards.
    //
    // So when the retreat path is blocked we turn around and RUN FORWARDS instead. Facing the
    // retreat direction puts the force ahead of the heading, which means forward thrust and the
    // full turn rate to steer with - the ship can actually round the obstacle. The cost is real
    // (a turn away from the threat, and our return fire stops bearing for its duration) but it is
    // much cheaper than being pinned, which is what has been killing them.
    bool retreat_blocked = IsPathBlocked(game, *self, retreat_direction);

    if (retreat_blocked) {
      steering.Face(game, self->position + retreat_direction * 100.0f);
    } else {
      steering.Face(game, threat_position);
    }

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
    //
    // Only in reverse mode. The clamp forces a component along -heading, which is "away from the
    // threat" only while the nose is ON the threat. Once we have turned to run forwards it would
    // subtract from the direction we are now flying, shoving us back toward the wall we just turned
    // to avoid - the exact failure this whole branch exists to fix, reintroduced one line later.
    if (!retreat_blocked) {
      float heading_component = steering.force.Dot(self->GetHeading());
      if (heading_component > -kMinRetreatForce) {
        steering.force -= self->GetHeading() * (heading_component + kMinRetreatForce);
      }
    }

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
  const char* target_distance_key = nullptr;
  const char* pressing_target_energy_key = nullptr;
  // How far down the escape we care about. Roughly a retreat's worth of travel: at the measured
  // ~13 tiles/sec retreat speed a 35-tile lane is under three seconds, about as far ahead as a
  // constant-velocity picture of where anyone is standing stays worth anything.
  static constexpr float kEscapeLookahead = 35.0f;
  // Half-width of the lane. A ship is under a tile across, but this is not a collision test - it is
  // "close enough to shoot us as we go past", and ground-truth bullet hit rate inside 10 tiles runs
  // 36-65%.
  static constexpr float kEscapeCorridor = 10.0f;
  // +-60 degrees in 15 degree steps. Wide enough to route around a blocker, narrow enough that every
  // candidate still opens range from the enemy actually chasing us.
  static constexpr int kCandidateSteps = 4;
  static constexpr float kCandidateStepRadians = 0.2618f;
  static constexpr float kDeviationWeight = 1.0f;
  // Set so one enemy squarely in the lane at point-blank (cost 4.0) outweighs the full 60 degrees of
  // deviation on offer (cost 1.05), while one loitering at the far end of the corridor does not.
  static constexpr float kBlockerWeight = 4.0f;
  // Above kBlockerWeight on purpose. A lane that dead-ends is worse than a lane with someone in it:
  // running past an enemy still opens range, running into a wall stops us dead in front of the one
  // already chasing. The ordering is what solves the funnel - beside a wall the wall lane costs up
  // to 6.0 and the enemy lane up to 4.0, so a clear lane 60 degrees off at 1.05 wins both.
  static constexpr float kTerrainWeight = 6.0f;
  static constexpr size_t kMaxBlockers = 16;

  const char* team_position_key = nullptr;
  float target_distance = 0.0f;
  float max_overshoot = 5.0f;
  float low_energy_percent = 0.2f;
  float max_team_bias_radians = 1.05f;

 private:
  // Minimum backward-facing force to guarantee during active retreat, so Actuator can never read
  // the combined steering.force as pointing toward the threat once Seek's own contribution decays.
  static constexpr float kMinRetreatForce = 1.0f;

  // How far down the retreat line to look for terrain, in seconds of travel. Has to cover the
  // distance needed to TURN rather than the distance needed to stop: a ship at retreat speed
  // carrying real momentum needs most of a second to bring its nose around, and the turn has to be
  // finished before arrival, not started at it.
  static constexpr float kRetreatLookaheadSeconds = 1.2f;
  // Floor for the above, so a bot that has just started moving - or is pinned and barely moving,
  // which is exactly the state we most need to detect - still looks far enough to see the wall it
  // is stuck against.
  static constexpr float kRetreatMinLookahead = 14.0f;

  // Is there terrain down the line we intend to retreat along?
  static bool IsPathBlocked(Game& game, Player& self, const Vector2f& direction) {
    if (direction.LengthSq() < 0.0001f) return false;

    // GetRadius() is in PIXELS and positions are in tiles - the /16 matters. Without it the ray
    // starts 14 TILES ahead of the ship and is blind to everything in between, which is the same
    // bug that made WallAvoidanceNode unable to see the walls it was meant to avoid.
    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius() / 16.0f;

    float lookahead = self.velocity.Length() * kRetreatLookaheadSeconds;
    if (lookahead < kRetreatMinLookahead) lookahead = kRetreatMinLookahead;

    CastResult result = game.GetMap().Cast(self.position + direction * radius, direction, lookahead, self.frequency);

    return result.hit;
  }

  // Same "have we actually heard from them" filter the other nexus target nodes use - a stale player
  // inside radar view left where we last saw them, and steering a retreat around a ghost is worse
  // than ignoring them.
  inline bool IsSynchronized(Game& game, Player& player) {
    if (game.radar.InRadarView(player.position)) {
      return game.player_manager.IsSynchronized(player);
    }

    return true;
  }
};

}  // namespace nexus
}  // namespace zero

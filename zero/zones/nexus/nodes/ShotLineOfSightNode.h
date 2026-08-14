#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when a shot aimed at `aim_key` actually has a clear lane to fly down.
//
// The firing checks in these trees validate shot *geometry* - RayNode builds the trajectory,
// RayRectangleInterceptNode confirms it crosses the target's bounding box - but nothing in that
// chain knows terrain exists. A target on the far side of a wall produces a perfectly valid
// intercept, so the bot lines up and fires into solid tiles. Bullets and bombs do not pass through
// walls, so every one of those shots is wasted energy, wasted cooldown, and a wasted position.
//
// Range makes it worse rather than better: the bots fire at a median 21-36 tiles, and the longer the
// lane the more likely something is in it. This is the most plausible remaining explanation for why
// their long-range hit rate sits near the noise floor while their close-range rate is excellent.
//
// BOUNCE ALLOWANCE. Ships in the Nexus zone always have bouncing bullets, so a blocked lane is
// never simply a dead bullet - the shot carries on off the wall. What it is not is an *aimed* shot:
// PredictiveAimNode solved a straight-line intercept, and the moment the bullet bounces that
// solution is void, so past the ricochet it is a random bullet. That is worth taking in tight
// geometry, where a stray bullet in an enclosed space still has real odds and denies the space
// besides, and not worth taking down a long lane where it is a lottery ticket. Hence an allowance
// bounded by `bounce_distance` rather than a blanket exemption.
//
// The capability is not checked at runtime. It is invariant for this zone, and a check could only
// ever fail in the direction of silently switching bullets to a hard gate - i.e. bots refusing to
// fire in exactly the corner fights the allowance exists to preserve. Pass 0 for a hard gate.
//
// Bombs get the hard gate on the part of the lane that terrain can actually stop them in - see the
// proximity note below. They do not reliably bounce (BombBounceCount is per-ship and commonly 0), a
// bounced bomb does reduced damage (BBombDamagePercent), and detonating one against a wall we are
// stood near is exactly the self-blast case BombBlastSafetyNode exists to prevent.
//
// Thors must NOT use this node at all - a thor travels through walls, so terrain is irrelevant to
// it and gating it here would remove the one weapon that can legitimately shoot through a wall.
// PROXIMITY WEAPONS DO NOT FUSE ON TERRAIN. A wall only stops a bomb by direct contact of the
// projectile itself - WeaponManager tests a single point, `map.IsSolid(weapon.x / 16000, weapon.y /
// 16000, ...)`, while the proximity radius is used exclusively in the *player* collision scan. So a
// bomb skimming past a wall carries on, where a bomb skimming past a ship detonates.
//
// That asymmetry matters at the far end of the lane. A bomb never has to traverse the last
// `prox_radius` tiles, because the target trips the fuse first - so terrain within that final
// stretch cannot stop the shot and must not veto it. Without this, a target standing against a wall
// was un-bombable: the ray clipped the wall beside them and the gate refused a bomb that would have
// detonated on the target before it ever got there. Pass `proximity_trigger` for bombs; bullets
// have no fuse and keep the full-length cast.
struct ShotLineOfSightNode : public behavior::BehaviorNode {
  ShotLineOfSightNode(const char* aim_key) : aim_key(aim_key), bounce_distance(0.0f) {}
  ShotLineOfSightNode(const char* aim_key, float bounce_distance) : aim_key(aim_key), bounce_distance(bounce_distance) {}
  ShotLineOfSightNode(const char* aim_key, float bounce_distance, bool proximity_trigger)
      : aim_key(aim_key), bounce_distance(bounce_distance), proximity_trigger(proximity_trigger) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aim = ctx.blackboard.Value<Vector2f>(aim_key);
    if (!opt_aim) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    Vector2f aim = *opt_aim;
    Vector2f cast_to = aim;

    if (proximity_trigger) {
      u32 bombs = game.ship_controller.ship.bombs;
      u32 level = bombs > 0 ? bombs - 1 : 0;

      // Same derivation as BombBlastSafetyNode - keep these in step.
      float prox_radius = ((game.connection.settings.ProximityDistance + level) * 18.0f - 14.0f) / 16.0f;

      if (prox_radius > 0.0f) {
        Vector2f along = aim - self->position;
        float length = along.Length();

        // The entire lane is inside fuse range, so there is no stretch terrain could stop it in.
        if (length <= prox_radius) return behavior::ExecuteResult::Success;

        cast_to = self->position + along * ((length - prox_radius) / length);
      }
    }

    // Cast to the aim point rather than to the target's current position: the aim point is where the
    // shot is actually going, and it's what the intercept test downstream is built against.
    CastResult result = game.GetMap().CastTo(self->position, cast_to, self->frequency);

    if (!result.hit) return behavior::ExecuteResult::Success;

    if (bounce_distance > 0.0f) {
      float distance_sq = self->position.DistanceSq(aim);

      if (distance_sq <= bounce_distance * bounce_distance) return behavior::ExecuteResult::Success;
    }

    return behavior::ExecuteResult::Failure;
  }

  const char* aim_key = nullptr;
  float bounce_distance = 0.0f;
  bool proximity_trigger = false;
};

}  // namespace nexus
}  // namespace zero

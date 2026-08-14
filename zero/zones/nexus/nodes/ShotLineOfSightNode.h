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
// Bombs get the hard gate. They do not reliably bounce (BombBounceCount is per-ship and commonly 0),
// a bounced bomb does reduced damage (BBombDamagePercent), and detonating one against a wall we are
// stood near is exactly the self-blast case BombBlastSafetyNode exists to prevent.
//
// Thors must NOT use this node at all - a thor travels through walls, so terrain is irrelevant to
// it and gating it here would remove the one weapon that can legitimately shoot through a wall.
struct ShotLineOfSightNode : public behavior::BehaviorNode {
  ShotLineOfSightNode(const char* aim_key) : aim_key(aim_key), bounce_distance(0.0f) {}
  ShotLineOfSightNode(const char* aim_key, float bounce_distance) : aim_key(aim_key), bounce_distance(bounce_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aim = ctx.blackboard.Value<Vector2f>(aim_key);
    if (!opt_aim) return behavior::ExecuteResult::Failure;

    Vector2f aim = *opt_aim;

    // Cast to the aim point rather than to the target's current position: the aim point is where the
    // shot is actually going, and it's what the intercept test downstream is built against.
    CastResult result = ctx.bot->game->GetMap().CastTo(self->position, aim, self->frequency);

    if (!result.hit) return behavior::ExecuteResult::Success;

    if (bounce_distance > 0.0f) {
      float distance_sq = self->position.DistanceSq(aim);

      if (distance_sq <= bounce_distance * bounce_distance) return behavior::ExecuteResult::Success;
    }

    return behavior::ExecuteResult::Failure;
  }

  const char* aim_key = nullptr;
  float bounce_distance = 0.0f;
};

}  // namespace nexus
}  // namespace zero

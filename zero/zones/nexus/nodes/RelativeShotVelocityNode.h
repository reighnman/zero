#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// The projectile's velocity relative to US - bare muzzle speed along the heading, with our own
// momentum left out.
//
// This is the counterpart to behavior::ShotVelocityQueryNode, which returns the *world* velocity
// `self->velocity + heading * weapon_speed`. Both are correct descriptions of the same bullet; which
// one a check wants depends on which frame the rest of that check lives in.
//
// The intercept test wants this one. PredictiveAimNode's `aimshot` comes out of a solve with the
// shooter's velocity subtracted, so it is a shooter-frame point, and testing it against a world-frame
// ray compares two different frames - they differ by `self->velocity * flight_time`, tens of tiles at
// combat speed. Pairing this ray with that box puts both in the shooter's frame, where the geometry
// is simply "does the nose point at the lead point", and the box's angular tolerance
// (box size / lead distance) is exactly the tolerance on heading error that determines whether the
// shot connects.
//
// WHY NOT DO THE WHOLE TEST IN THE WORLD FRAME INSTEAD. It was tried (rec40) and it is worse, for a
// reason worth recording. The two tests are equivalent for a point target under exactly constant
// velocity, but the world-frame version lets the hull point anywhere so long as our own momentum
// carries the bullet onto the target - so the bot fires while broadside. Those shots are real, but
// they are bad: their whole trajectory rests on our velocity holding for the flight time, and the
// Actuator thrusts on every tick where steering force is non-zero, so it does not hold. Measured over
// rec38/rec40, shots taken at 90+ degrees of hull offset hit 0-10% of the time with median misses of
// 7-23 tiles, against 7-30% and 4-9 tiles for shots inside 30 degrees. Going world-frame roughly
// tripled the share of those shots (3-6% -> 4-19% per bot) and dropped hit rate at every range band
// past 10 tiles.
//
// Terrain and blast damage are genuinely world-frame and must NOT use this - ShotLineOfSightNode and
// BombBlastSafetyNode take the world aim point, because the lane a bullet really flies down starts at
// us and ends where it actually meets the target on the map.
struct RelativeShotVelocityNode : public behavior::BehaviorNode {
  RelativeShotVelocityNode(WeaponType weapon_type, const char* velocity_key)
      : weapon_type(weapon_type), velocity_key(velocity_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float weapon_speed = behavior::GetWeaponSpeed(*ctx.bot->game, *self, weapon_type);

    ctx.blackboard.Set(velocity_key, self->GetHeading() * weapon_speed);

    return behavior::ExecuteResult::Success;
  }

  WeaponType weapon_type;
  const char* velocity_key = nullptr;
};

}  // namespace nexus
}  // namespace zero

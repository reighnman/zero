#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when firing a bomb RIGHT NOW would leave it nearly stationary in the world - a bomb that
// behaves like a mine we did not have to spend.
//
// A projectile inherits the ship's velocity and adds muzzle speed along the heading
// (WeaponManager.cpp:903: `weapon->velocity = vel + heading * speed`). While retreating, FleeNode
// keeps the nose pointed at the threat and lets the Actuator resolve the retreat as REVERSE thrust,
// so heading and velocity point in opposite directions. The two therefore subtract: a bomb fired in
// that state leaves at roughly (muzzle_speed - our_speed) toward the chaser, and at full retreat
// speed that can be close to zero.
//
// That is a genuinely useful weapon rather than a wasted shot. A near-stationary bomb sits in the
// wake of a ship the pursuer is driving straight toward at closing speed, which is exactly the
// geometry a mine relies on - except a mine is a limited item and this is not. It is at its best
// against a fast chaser, who has the least time to steer around it.
//
// The check is on the RESULT, not on the setup, which is what makes it safe to run anywhere in the
// retreat branch. Any state that would send the bomb off at speed - nose not actually pointed back
// at the chaser, or the broadside hold FleeNode falls into inside the standoff band, or simply not
// moving fast enough yet - produces a high ground speed and fails here, so this can never turn into
// an ordinary forward bomb fired blindly behind us.
struct SlowBombNode : public behavior::BehaviorNode {
  SlowBombNode(float max_ground_speed) : max_ground_speed(max_ground_speed) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float bomb_speed = behavior::GetWeaponSpeed(*ctx.bot->game, *self, WeaponType::Bomb);
    Vector2f bomb_velocity = self->velocity + self->GetHeading() * bomb_speed;

    return bomb_velocity.Length() <= max_ground_speed ? behavior::ExecuteResult::Success
                                                      : behavior::ExecuteResult::Failure;
  }

  float max_ground_speed = 0.0f;
};

}  // namespace nexus
}  // namespace zero

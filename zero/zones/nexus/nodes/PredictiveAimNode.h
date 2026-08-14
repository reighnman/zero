#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Nexus-specific fork of behavior::AimNode (zero/behavior/nodes/AimNode.h) that additionally
// factors in the target's recent acceleration (from TargetAccelerationNode) instead of assuming
// they'll hold their current velocity for the whole bullet flight.
//
// A true accelerating-projectile intercept has no clean closed form, so this takes a pragmatic
// shortcut instead: nudge the target's velocity forward by up to `lead_bias_seconds` worth of their
// smoothed acceleration before handing it to the same proven constant-velocity solver
// (behavior::CalculateShot) the shared AimNode uses. That bends the lead toward where the target is
// actually trending without the risk of deriving a new accelerated solver from scratch.
//
// The bias is scaled down by the estimated bullet flight time (distance / weapon speed), capped at
// `lead_bias_seconds`, rather than always applying the full amount. Close-range shots have a flight
// time far shorter than lead_bias_seconds, and close range is also exactly when a target is most
// likely mid-dodge with a large instantaneous acceleration - applying the full nudge there could
// swing the aimshot by more than the short flight time actually justifies.
//
// ---------------------------------------------------------------------------------------------
// On which projectile speed to hand the solver (this was wrong here, and still is in the shared
// behavior::AimNode this was forked from):
//
// behavior::CalculateShot solves |toTarget + v*t| = s*t with v = vTarget - vShooter. Subtracting the
// shooter's velocity puts the whole problem in the shooter's own reference frame, so `s` has to be
// the projectile's speed *in that frame*. A Subspace bullet leaves at ship velocity plus muzzle
// velocity along the heading, so relative to the ship it travels at exactly the muzzle speed -
// `s` must be the bare weapon speed.
//
// Passing the world-frame speed |self->velocity + heading * weapon_speed| instead, as the shared
// node still does, counts our own velocity twice: once by subtracting it into `v`, and again by
// inflating `s`. The solver then thinks the bullet closes faster than it does, underestimates the
// flight time, and returns a lead that is too short. The error scales with our own speed and is
// worst head-on, where the inflated speed is muzzle + full ship speed - at a typical closing speed
// that is roughly a third of the flight time, and so about a third of the lead, missing behind a
// laterally moving target by more than a ship radius at ordinary fighting range.
// ---------------------------------------------------------------------------------------------
struct PredictiveAimNode : public behavior::BehaviorNode {
  PredictiveAimNode(WeaponType weapon_type, const char* target_player_key, const char* acceleration_key,
                     const char* position_key, float lead_bias_seconds = 0.2f)
      : weapon_type(weapon_type),
        target_player_key(target_player_key),
        acceleration_key(acceleration_key),
        position_key(position_key),
        lead_bias_seconds(lead_bias_seconds) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target.has_value()) return behavior::ExecuteResult::Failure;

    Player* target = opt_target.value();
    if (!target) return behavior::ExecuteResult::Failure;

    Vector2f acceleration = ctx.blackboard.ValueOr<Vector2f>(acceleration_key, Vector2f(0, 0));

    float weapon_speed = behavior::GetWeaponSpeed(*ctx.bot->game, *self, weapon_type);

    Vector2f direction = Normalize(target->position - self->position);

    // Scale the bias down at close range instead of always applying the full lead_bias_seconds -
    // see the class comment for why.
    float distance = target->position.Distance(self->position);
    float effective_lead_bias = lead_bias_seconds;

    if (weapon_speed > 0.0f) {
      float estimated_flight_time = distance / weapon_speed;
      if (estimated_flight_time < effective_lead_bias) {
        effective_lead_bias = estimated_flight_time;
      }
    }

    // Bend the target's velocity toward where they're actually trending before handing it to the
    // constant-velocity solver below.
    Vector2f biased_velocity = target->velocity + acceleration * effective_lead_bias;

    float away_amount = biased_velocity.Dot(direction);

    Vector2f target_velocity = biased_velocity;
    // If the enemy is moving away too fast, ignore the away movement for this calculation.
    if (away_amount > weapon_speed) {
      // Remove the "away" velocity from the target so it's only moving side to side.
      target_velocity = biased_velocity - direction * away_amount;
    }

    // The projectile speed handed to the solver must be the speed *relative to us*, i.e. the bare
    // muzzle speed - see the class comment. Passing the world-frame speed here double-counts our own
    // velocity and makes every shot under-lead.
    std::optional<Vector2f> calculated_shot = behavior::CalculateShot(
        self->position, target->position, self->velocity, target_velocity, weapon_speed);

    // Default to target's position if the calculated shot fails.
    Vector2f aimshot = target->position;

    if (calculated_shot.has_value()) {
      aimshot = *calculated_shot;

      constexpr float kFarDistance = 50.0f;

      // Set the aimshot directly to the player position if it is too far away.
      if (aimshot.DistanceSq(target->position) > kFarDistance * kFarDistance) {
        aimshot = target->position;
      }

      // If the aimshot is behind us but the target isn't, just shoot at the target.
      if ((aimshot - self->position).Dot(target->position - self->position) < 0.0f) {
        aimshot = target->position;
      }
    }

    ctx.blackboard.Set(position_key, aimshot);

    return behavior::ExecuteResult::Success;
  }

  WeaponType weapon_type;
  const char* target_player_key = nullptr;
  const char* acceleration_key = nullptr;
  const char* position_key = nullptr;
  float lead_bias_seconds = 0.2f;
};

}  // namespace nexus
}  // namespace zero

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
    Vector2f weapon_velocity = self->velocity + self->GetHeading() * weapon_speed;

    Vector2f direction = Normalize(target->position - self->position);

    // Scale the bias down at close range instead of always applying the full lead_bias_seconds -
    // see the class comment for why.
    float distance = target->position.Distance(self->position);
    float weapon_velocity_length = weapon_velocity.Length();
    float effective_lead_bias = lead_bias_seconds;

    if (weapon_velocity_length > 0.0f) {
      float estimated_flight_time = distance / weapon_velocity_length;
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

    std::optional<Vector2f> calculated_shot = behavior::CalculateShot(
        self->position, target->position, self->velocity, target_velocity, weapon_velocity.Length());

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

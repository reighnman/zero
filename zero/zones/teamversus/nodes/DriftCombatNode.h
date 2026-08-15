#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

#include <cstdlib>

namespace zero {
namespace teamversus {

// The combat movement controller: hold a standoff band around the target while carrying real
// tangential momentum, and oscillate that band in and out on a human rhythm.
//
// This is built directly from what human movement actually looks like in the corpus, which is
// nothing like "seek the target" and nothing like a fixed-radius circle either:
//
//   - The nose is roughly perpendicular to the direction of travel. Median angle between heading
//     and velocity is 75-95 degrees, p90 around 150. Players are almost never flying where they are
//     pointing.
//   - Yet thrust is held down 84-93% of the time. Combined with the above, that means thrust is
//     mostly *lateral* to current motion - which is centripetal, and curves the path.
//   - The hull turns about 1.5x faster than the velocity vector does (100-167 deg/s versus 64-74).
//     Momentum genuinely lags the hull; the ship is coasting through its own turns.
//   - Velocity sits a median 65-70 degrees off the bearing to the target, and *less* off the
//     bearing to where the target is going. They are travelling across the enemy, leading them,
//     not chasing them down.
//   - Speed while closing has a median of 14.7 tiles/sec with p90 at the 20.3 cap. This is fast
//     movement, not careful positioning.
//
// Put together, that is an orbit: nose pointed inward at the enemy (which is also where the shots
// go - 60% of shots are within 30 degrees of the nose), thrust doing centripetal duty and radius
// correction, and tangential velocity maintained by momentum.
//
// This maps onto the engine almost exactly, because of how Actuator resolves a steering force
// against a rotation target. It points the hull at the force direction, but clamps that to within
// `rotation_threshold` of the rotation target, then thrusts forward or backward along the result.
// So: Face the aim point to set the rotation target, and push a force describing the desired
// orbital velocity. The radial part of that force becomes forward/backward thrust directly (the
// nose is already near-radial), and the tangential part pulls the hull up to the threshold off the
// aim and builds the sideways speed. The measured head-versus-velocity offset falls out of the
// clamp rather than having to be scripted.
//
// The in-out oscillation is the other half. Real fights are not fought at a fixed radius: sustained
// closing runs have a median of 1.7 seconds and back-off runs 1.4 seconds, each sweeping about 10
// tiles, and radial direction reverses roughly every 2.5 seconds. That pump is what makes a player
// hard to lead - the shooter's own flight time is over a second, so a target that changes radial
// direction inside that window has already invalidated the shot. A bot holding a constant radius is
// trivially easy to hit no matter how good its dodging is.
struct DriftCombatNode : public behavior::BehaviorNode {
  DriftCombatNode(const char* aimshot_key, const char* target_position_key, const char* standoff_key)
      : aimshot_key(aimshot_key), target_position_key(target_position_key), standoff_key(standoff_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aimshot = ctx.blackboard.Value<Vector2f>(aimshot_key);
    auto opt_target_position = ctx.blackboard.Value<Vector2f>(target_position_key);
    if (!opt_aimshot || !opt_target_position) return behavior::ExecuteResult::Failure;

    Vector2f aimshot = *opt_aimshot;
    Vector2f target_position = *opt_target_position;

    auto& game = *ctx.bot->game;
    auto& steering = ctx.bot->bot_controller->steering;

    float standoff = ctx.blackboard.ValueOr<float>(standoff_key, 26.0f);

    Vector2f to_self = self->position - target_position;
    float radius = to_self.Length();

    // Sitting exactly on top of the target - any outward direction will do to start opening a gap.
    if (radius < 0.5f) {
      to_self = -self->GetHeading();
      radius = 0.5f;
    }

    Vector2f radial = to_self * (1.0f / radius);
    Vector2f tangent = Perpendicular(radial) * GetOrbitDirection(ctx, radial, *self);

    float desired_radius = standoff + UpdatePump(standoff);

    // Radial correction, capped so a large distance error does not swamp the tangential component
    // and collapse the orbit into a straight-line charge.
    float radius_error = desired_radius - radius;
    float radial_speed = radius_error * radial_gain;

    float max_speed = Steering::GetMaxSpeed(game);
    if (radial_speed > max_speed) radial_speed = max_speed;
    if (radial_speed < -max_speed) radial_speed = -max_speed;

    Vector2f desired_velocity = radial * radial_speed + tangent * orbit_speed;

    // Don't ask for more than the ship can do, or the force ends up dominated by an unreachable
    // component and the direction of the request stops meaning anything.
    float desired_speed = desired_velocity.Length();
    if (desired_speed > max_speed && desired_speed > 0.0f) {
      desired_velocity *= max_speed / desired_speed;
    }

    steering.force += desired_velocity - self->velocity;

    // Rotation target is the aim point, always. Movement is expressed purely as force and is
    // allowed to pull the hull off aim only as far as the rotation threshold permits.
    steering.Face(game, aimshot);
    steering.SetRotationThreshold(rotation_threshold);

    return behavior::ExecuteResult::Success;
  }

  const char* aimshot_key = nullptr;
  const char* target_position_key = nullptr;
  const char* standoff_key = nullptr;

  // Tangential speed to try to maintain, in tiles/sec. Close to the measured median closing speed.
  float orbit_speed = 13.0f;

  // How hard to correct a radius error. Low enough that being 10 tiles out of position produces a
  // firm pull rather than a headlong charge.
  float radial_gain = 0.9f;

  // How far the standoff swings either side of its nominal value, in tiles. The measured half-cycle
  // sweeps about 10 tiles, so half of that either way.
  float pump_amplitude = 5.0f;

  // Bounds on one half of a pump cycle, in ticks. Centred on the measured 1.4-1.7 second runs, with
  // enough spread that the rhythm is not itself predictable.
  u32 pump_min_ticks = 90;
  u32 pump_max_ticks = 210;

  // How far off the aim point movement is allowed to drag the hull. 0.75 is a dot product, about 41
  // degrees, which lands close to the measured share of shots taken within 30 degrees of the nose
  // while still leaving room for real lateral thrust.
  float rotation_threshold = 0.75f;

  // Chance per pump reversal of also flipping the orbit direction. Occasionally reversing the
  // circle is a genuine evasive tool - it inverts the lead a shooter has been building - but doing
  // it often would cancel out into no net tangential movement at all.
  float reverse_chance = 0.25f;

 private:
  float orbit_direction = 0.0f;
  PlayerId orbit_target = kInvalidPlayerId;

  // Pump state. `pump_sign` is +1 while drifting outward and -1 while closing.
  float pump_sign = 1.0f;
  Tick pump_tick = 0;
  u32 pump_duration = 0;

  // Commits to one rotation direction for the engagement instead of recomputing it every tick,
  // which would just cancel out into no rotation at all. Picks whichever way we are *already
  // travelling*, so committing costs the least momentum to establish.
  //
  // Travel, not facing. Those are two different vectors and in this movement model they are
  // deliberately far apart - the measured offset between heading and direction of travel is 75-95
  // degrees, and the whole point of the orbit is that the nose stays on the target while the ship
  // moves across it. Choosing the orbit direction from where the nose points therefore says almost
  // nothing about which way is cheap to turn into, and at a 90 degree offset it is a coin flip that
  // routinely picks the direction that has to kill all our existing momentum first. Facing is only
  // used as the fallback when we are barely moving and there is no travel direction to read.
  float GetOrbitDirection(behavior::ExecuteContext& ctx, const Vector2f& radial, const Player& self) {
    PlayerId target_id = ctx.blackboard.ValueOr<PlayerId>("target_id", kInvalidPlayerId);

    if (orbit_direction == 0.0f || target_id != orbit_target) {
      Vector2f travel = self.velocity;
      if (travel.LengthSq() < 1.0f) travel = self.GetHeading();

      Vector2f tangent = Perpendicular(radial);
      orbit_direction = tangent.Dot(travel) >= 0.0f ? 1.0f : -1.0f;
      orbit_target = target_id;
    }

    return orbit_direction;
  }

  // Advances the in-out oscillation and returns the current offset to apply to the standoff.
  float UpdatePump(float standoff) {
    Tick now = GetCurrentTick();

    if (pump_duration == 0 || TICK_DIFF(now, pump_tick) >= (s32)pump_duration) {
      pump_sign = -pump_sign;
      pump_tick = now;
      pump_duration = pump_min_ticks + (u32)(rand() % (int)(pump_max_ticks - pump_min_ticks + 1));

      if ((float)rand() / (float)RAND_MAX < reverse_chance) {
        orbit_direction = -orbit_direction;
      }
    }

    float amplitude = pump_amplitude;

    // Never let the pump drive the inner edge of the swing to zero or negative - at a tight press
    // standoff that would mean repeatedly trying to occupy the same tile as the target.
    float max_amplitude = standoff * 0.5f;
    if (amplitude > max_amplitude) amplitude = max_amplitude;

    // Ease across the half-cycle rather than stepping, so the reversal reads as a change of
    // direction instead of a teleport of the target radius.
    float progress = pump_duration > 0 ? (float)TICK_DIFF(now, pump_tick) / (float)pump_duration : 0.0f;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    // Travels from -sign to +sign across the half-cycle.
    return pump_sign * amplitude * (progress * 2.0f - 1.0f);
  }
};

}  // namespace teamversus
}  // namespace zero

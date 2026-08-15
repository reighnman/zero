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

    // Aim at the chosen target; hold range against whoever is actually closest. Those are two
    // different enemies more often than not, and conflating them is what put this node's radial
    // velocity at -0.8 tiles/sec while outnumbered where humans measure +3.5. Standoff is a question
    // about the nearest threat - it is the one that decides how much damage arrives - while the aim
    // point is a question about who is worth killing. Falls back to the aim target when no separate
    // nearest enemy is published, which keeps single-opponent behavior identical.
    Vector2f target_position = ctx.blackboard.ValueOr<Vector2f>("nearest_enemy_position", *opt_target_position);

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

    float max_speed = Steering::GetMaxSpeed(game);

    // Split our current motion into the two axes that matter. Only the radial part is ours to
    // command; the tangential part is momentum, and momentum is the whole point.
    float speed_radial = self->velocity.Dot(radial);
    float speed_tangential = self->velocity.Dot(tangent);

    // --- Phase 1: get up to speed -------------------------------------------------------------
    // A ship with no lateral momentum has nothing to drift on, and it cannot acquire any while its
    // nose is locked on the target, because thrust only ever acts along the nose. So when we are not
    // yet moving across the enemy, that is the one time worth spending off-aim: point along the
    // orbit direction, open the rotation clamp so the hull can actually get there, and build the
    // momentum the rest of the fight is going to coast on.
    if (speed_tangential < min_orbit_speed) {
      steering.force += tangent * max_speed;
      steering.Face(game, self->position + tangent);
      steering.SetRotationThreshold(0.0f);

      return behavior::ExecuteResult::Success;
    }

    // --- Phase 2: drift ------------------------------------------------------------------------
    // Now the nose goes on the target and stays there, and the *only* thing thrust is used for is
    // pushing along that axis - forward to tighten in, reverse to open out. Nothing here asks for
    // tangential acceleration, which is the correction that matters: asking for it produced a force
    // pointing sideways relative to the nose, and a sideways force is exactly what Actuator cannot
    // honour while holding an aim. It clamps the steering direction back to within
    // `rotation_threshold` of the rotation target and then thrusts along *that*, so the lateral
    // request was discarded every tick and what survived was a push straight at the enemy. The bot
    // flew at whatever it was shooting at and called it an orbit.
    //
    // Expressed this way the geometry does the work instead. Nose on the target while travelling
    // across it means the nose is already roughly perpendicular to the path, so forward thrust is
    // pure centripetal - it bends the trajectory without bleeding the speed that makes the bend
    // sharp - and reverse thrust bends it the other way without ever turning around. That is the
    // whole trick: a ship at speed steers by pointing across its own path and choosing a thrust
    // sign, and it gets to keep shooting down the nose at whatever it is passing while it does it.
    //
    // Only the *direction* of the accumulated force reaches the ship - Actuator normalizes it and
    // uses it to pick forward or backward - so what is being computed here is which way along the
    // radial axis we want to accelerate, not how hard.
    float desired_radius = standoff + UpdatePump(standoff);

    // Radial velocity we would like to have, from how far off the standoff we are.
    float desired_radial_speed = (desired_radius - radius) * radial_gain;
    if (desired_radial_speed > max_speed) desired_radial_speed = max_speed;
    if (desired_radial_speed < -max_speed) desired_radial_speed = -max_speed;

    float radial_accel = (desired_radial_speed - speed_radial) * approach_gain;

    // Feed-forward the inward pull needed to hold a curve of this radius at this tangential speed.
    // Without it the orbit unwinds - a ship travelling in a straight line past a target gains range
    // every tick, so the radius controller spends its life chasing a drift it could have cancelled.
    //
    // Only applied from the standoff outwards. Inside it the term inverts the answer: v^2/r grows
    // without bound as the radius shrinks, so a bot sitting at ten tiles wanting to be at
    // twenty-three would compute more inward pull than the outward correction and thrust further in.
    // It is also the case that the tight curve it is asking to hold there is not physically
    // available - at eleven tiles/sec and ten tiles of radius the requirement is already above the
    // ship's thrust - so the honest answer when too close is simply to push out and let the geometry
    // open the range.
    if (radius >= desired_radius) {
      radial_accel -= (speed_tangential * speed_tangential) / radius;
    }

    steering.force += radial * radial_accel;

    steering.Face(game, aimshot);
    steering.SetRotationThreshold(rotation_threshold);

    return behavior::ExecuteResult::Success;
  }

  const char* aimshot_key = nullptr;
  const char* target_position_key = nullptr;
  const char* standoff_key = nullptr;

  // Lateral speed below which we stop aiming and go and get some. Not a speed to *hold* - nothing
  // holds it, momentum does - just the point below which there is no drift to work with. Set near
  // the measured median closing speed so the orbit starts out at a realistic pace.
  float min_orbit_speed = 11.0f;

  // How hard to chase a radial velocity error. Converts a speed error into an acceleration request;
  // only the sign ultimately reaches the ship, so this sets how readily the radial term outvotes the
  // centripetal one rather than how hard we push.
  float approach_gain = 1.0f;

  // How hard to correct a radius error, as a desired radial speed per tile of error.
  //
  // Calibrated against measured human radial velocity rather than picked. Humans close at a median
  // -2.7 tiles/sec while even or up, from a held range of about 31 tiles, and open at +3.5 while
  // outnumbered from about 16. Those are gentle numbers: a fight is a slow squeeze, not a charge and
  // a bolt. At 0.9 this node asked for (26-31)*0.9 = -4.5 closing and (34-16)*0.9 = 16 opening,
  // the latter clamped by top speed into a full-speed flight nobody does. At 0.35 the same two
  // situations produce -1.8 and +6.3, which sit either side of the human figures.
  float radial_gain = 0.35f;

  // How far the standoff swings either side of its nominal value, in tiles. The measured half-cycle
  // sweeps about 10 tiles, so half of that either way.
  float pump_amplitude = 5.0f;

  // Bounds on one half of a pump cycle, in ticks. Centred on the measured 1.4-1.7 second runs, with
  // enough spread that the rhythm is not itself predictable.
  u32 pump_min_ticks = 90;
  u32 pump_max_ticks = 210;

  // How far off the aim point movement is allowed to drag the hull, as a dot product. Tight now, and
  // it costs nothing to be tight: the only force this node produces during the drift lies along the
  // radial axis, which is the aim axis, so there is no lateral request left for the clamp to fight.
  // Leaving it loose would only let terrain avoidance and formation spacing - which do blend in
  // sideways forces - pull the nose off a target it could otherwise hold.
  float rotation_threshold = 0.95f;

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

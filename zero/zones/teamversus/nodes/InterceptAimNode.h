#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Solves where to point in order to hit a target, by forward-integrating the target through the
// actual constraints on a Subspace ship rather than extrapolating its velocity in a straight line.
//
// Why the straight-line version is not good enough here. Projectiles in this arena are *slower than
// ships*: bullets and bombs both travel 12.5 tiles/sec in the shooter's frame against a ship top
// speed above 20. Flight time to 20 tiles is about 1.6 seconds. Over that long, a constant-velocity
// guess lands a measured 3.3-3.8 tiles from where the player actually went - several ship radii -
// and the bullet hit rate collapses from 32% at 10-14 tiles to 11% at 20-24 in exactly the band
// where flight time crosses a second.
//
// Why a *free acceleration vector* is also not good enough. A ship's motion is far more constrained
// than a free 2D body and every constraint is observable:
//
//   - Thrust acts only along the heading. Velocity changes come from thrust, bounces, repels and
//     gravity - nothing else. Acceleration is collinear with heading, never a free vector. Turning
//     does not rotate existing velocity; it only changes where future thrust will point.
//   - Heading, and therefore its rate of change, is observable via Player::orientation.
//   - Thrust, top speed and turn rate are all bounded by arena settings for that ship type.
//
// Modelling acceleration as a free vector throws all of that away, and worse, it lets a prediction
// drift somewhere the ship physically could not reach. So this tracks the target's heading and turn
// rate, infers whether they are thrusting forward or backward from how their speed along their own
// heading is changing, and integrates that model forward.
//
// Because flight time depends on the predicted point and the predicted point depends on flight
// time, this iterates: predict at t, recompute t, re-predict. Three passes is plenty at these
// speeds.
//
// Frame convention. `behavior::CalculateShot` and every consumer of "aimshot" in this codebase work
// in the *shooter's frame* - the published point is the lead point with our own velocity already
// divided out, not the target's predicted world position. Both are published here, because they
// answer different questions: aim at `aimshot`, but check blast safety and clearance against
// `predicted_position`.
//
// Weapon speed is the bare muzzle speed, deliberately. A projectile leaves at ship velocity plus
// muzzle velocity, so relative to the shooter it moves at exactly the muzzle speed. Passing the
// world-frame speed double-counts our own velocity, underestimates flight time and under-leads
// every shot, with the error growing with our speed and worst head-on.
//
// Per-target state lives on the node instance, not on the blackboard. That is not a style
// preference: these trees run the same node type more than once per tick in some configurations,
// and a fixed blackboard key for tick-to-tick memory means each call reads back a sample belonging
// to a different player, concludes the target changed, and resets - silently producing a
// permanently zero estimate. Instance state plus an identity check plus a staleness guard is the
// shape that survives that. One ZeroBot runs per process, so instance state is per-bot state.
struct InterceptAimNode : public behavior::BehaviorNode {
  InterceptAimNode(WeaponType weapon_type, const char* target_key, const char* aimshot_key, const char* predicted_key,
                   const char* flight_time_key)
      : weapon_type(weapon_type),
        target_key(target_key),
        aimshot_key(aimshot_key),
        predicted_key(predicted_key),
        flight_time_key(flight_time_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    auto& game = *ctx.bot->game;

    float weapon_speed = behavior::GetWeaponSpeed(game, *self, weapon_type);
    if (weapon_speed <= 0.0f) return behavior::ExecuteResult::Failure;

    MotionEstimate estimate = UpdateEstimate(game, *target);
    ShipMotionLimits limits = GetShipMotionLimits(game, target->ship);

    // Seed the iteration with the straight-line flight time, then refine.
    Vector2f to_target = target->position - self->position;
    float flight_time = to_target.Length() / weapon_speed;

    Vector2f predicted = target->position;

    for (int i = 0; i < kSolveIterations; ++i) {
      predicted = PredictPosition(*target, estimate, limits, flight_time);

      // Solve in the shooter's frame: our own velocity carries the projectile, so subtract the
      // ground we cover during the flight before measuring how far the shot has to travel.
      Vector2f relative = predicted - (self->position + self->velocity * flight_time);
      float relative_distance = relative.Length();

      float next_flight_time = relative_distance / weapon_speed;

      if (fabsf(next_flight_time - flight_time) < kConvergenceSeconds) {
        flight_time = next_flight_time;
        break;
      }

      flight_time = next_flight_time;
    }

    // Beyond a certain flight time the model is extrapolating far past anything it can justify -
    // the target gets to make several independent decisions in that window. Clamp rather than
    // letting a 3-second lead put the aim point in the middle of a wall.
    if (flight_time > max_flight_time) flight_time = max_flight_time;

    predicted = PredictPosition(*target, estimate, limits, flight_time);

    Vector2f aimshot = predicted - self->velocity * flight_time;

    // Sanity guards, matching the shared AimNode's. A solution that lands absurdly far from the
    // target, or behind us while the target is in front, is the solver failing rather than a clever
    // lead - shoot at the target instead.
    if (aimshot.DistanceSq(target->position) > kMaxLeadDistance * kMaxLeadDistance) {
      aimshot = target->position;
      predicted = target->position;
    } else if ((aimshot - self->position).Dot(target->position - self->position) < 0.0f) {
      aimshot = target->position;
      predicted = target->position;
    }

    ctx.blackboard.Set<Vector2f>(aimshot_key, aimshot);
    ctx.blackboard.Set<Vector2f>(predicted_key, predicted);
    ctx.blackboard.Set<float>(flight_time_key, flight_time);

    return behavior::ExecuteResult::Success;
  }

  WeaponType weapon_type;
  const char* target_key = nullptr;
  const char* aimshot_key = nullptr;
  // Where the target is expected to be in *world* space when the shot arrives, as distinct from the
  // shooter-frame lead point. Both are needed and they are not interchangeable: aim at the lead
  // point, but test the trajectory and the blast against the world position. Each weapon gets its
  // own key, because bullets and bombs have different flight times and therefore different answers.
  const char* predicted_key = nullptr;
  const char* flight_time_key = nullptr;

  // Smoothing on the turn-rate and thrust estimates. Orientation arrives quantized to 40 steps and
  // position packets are sampled, so a single raw difference is mostly noise; this is an
  // exponential moving average over it.
  float smoothing = 0.3f;

  float max_flight_time = 2.0f;

 private:
  // How far past the target a lead point may sit before we treat the solution as nonsense.
  static constexpr float kMaxLeadDistance = 50.0f;
  static constexpr int kSolveIterations = 3;
  static constexpr float kConvergenceSeconds = 0.01f;
  // Integration step for the forward model. Fine enough that a hard turn is followed accurately,
  // coarse enough that a 2 second prediction is 20 steps.
  static constexpr float kIntegrationStep = 0.1f;
  // Discard the previous sample if it is older than this - after a long gap the difference between
  // two velocities says nothing useful about the current turn.
  static constexpr s32 kStaleTicks = 50;

  struct MotionEstimate {
    float turn_rate = 0.0f;    // radians/sec, signed
    float thrust_sign = 0.0f;  // -1, 0 or +1
  };

  // Per-target memory. Identity-checked and staleness-guarded on every use.
  PlayerId tracked_id = kInvalidPlayerId;
  float last_orientation = 0.0f;
  Vector2f last_velocity;
  Tick last_tick = 0;
  MotionEstimate estimate;

  MotionEstimate UpdateEstimate(Game& game, Player& target) {
    Tick now = GetCurrentTick();
    s32 elapsed_ticks = TICK_DIFF(now, last_tick);

    bool same_target = tracked_id == target.id;
    bool fresh = elapsed_ticks > 0 && elapsed_ticks < kStaleTicks;

    if (!same_target || !fresh) {
      // A different player, or too long since we last looked. Either way the stored sample belongs
      // to a different situation - reset instead of differentiating against it, which would read as
      // an enormous bogus turn on the first tick after a target switch.
      estimate = MotionEstimate();
    } else {
      float dt = elapsed_ticks / 100.0f;

      // Orientation is a 0..1 fraction of a revolution. Wrap the difference to the shorter way
      // around, or a target crossing the seam registers as a full-speed spin the other way.
      float delta = target.orientation - last_orientation;
      while (delta > 0.5f) delta -= 1.0f;
      while (delta < -0.5f) delta += 1.0f;

      float raw_turn_rate = (delta * 2.0f * 3.14159265f) / dt;

      ShipMotionLimits limits = GetShipMotionLimits(game, target.ship);
      if (limits.max_rotation > 0.0f) {
        if (raw_turn_rate > limits.max_rotation) raw_turn_rate = limits.max_rotation;
        if (raw_turn_rate < -limits.max_rotation) raw_turn_rate = -limits.max_rotation;
      }

      estimate.turn_rate += (raw_turn_rate - estimate.turn_rate) * smoothing;

      // Thrust acts along the heading, so the observable signature of thrusting is a change in the
      // component of velocity that lies along the heading. Reading the sign of that change is a far
      // more honest estimate than treating the whole velocity delta as free acceleration, because a
      // bounce or a repel shows up almost entirely off-heading and is correctly ignored here.
      Vector2f heading = target.GetHeading();
      float along_now = target.velocity.Dot(heading);
      float along_before = last_velocity.Dot(heading);
      float along_delta = (along_now - along_before) / dt;

      float raw_thrust_sign = 0.0f;
      if (along_delta > kThrustDetectThreshold) {
        raw_thrust_sign = 1.0f;
      } else if (along_delta < -kThrustDetectThreshold) {
        raw_thrust_sign = -1.0f;
      }

      estimate.thrust_sign += (raw_thrust_sign - estimate.thrust_sign) * smoothing;
    }

    tracked_id = target.id;
    last_orientation = target.orientation;
    last_velocity = target.velocity;
    last_tick = now;

    return estimate;
  }

  // Acceleration along the heading (tiles/sec^2) below which we call it noise rather than thrust.
  // Well under the ~11 tiles/sec^2 a real thrust produces.
  static constexpr float kThrustDetectThreshold = 3.0f;

  static Vector2f PredictPosition(const Player& target, const MotionEstimate& estimate,
                                  const ShipMotionLimits& limits, float seconds) {
    if (seconds <= 0.0f) return target.position;

    ShipMotionSample sample;
    sample.position = target.position;
    sample.velocity = target.velocity;
    sample.heading = target.GetHeading();

    float remaining = seconds;

    while (remaining > 0.0f) {
      float dt = remaining > kIntegrationStep ? kIntegrationStep : remaining;

      StepShipMotion(sample, limits, estimate.turn_rate, estimate.thrust_sign, dt);

      remaining -= dt;
    }

    return sample.position;
  }
};

}  // namespace teamversus
}  // namespace zero

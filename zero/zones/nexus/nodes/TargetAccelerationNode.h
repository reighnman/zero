#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Estimates what the current target is doing with their engine, so aim prediction can bend toward
// where they are actually trending instead of assuming they hold their present velocity for the
// whole flight time.
//
// ---------------------------------------------------------------------------------------------
// A SHIP CANNOT ACCELERATE SIDEWAYS. This node used to difference tick-to-tick velocity into a free
// 2D vector, which throws away everything the game guarantees about how ships move. Thrust acts only
// along the hull's facing, and there are exactly three things a pilot can be doing: coasting,
// thrusting forward, or thrusting in reverse. Nothing else changes velocity except wall bounces,
// repels and gravity, none of which are worth predicting into an aim point.
//
// So the estimate is a *thrust state*, not a vector: project the observed velocity change onto the
// target's own heading, take the sign, and re-express it as `heading * thrust * state`. The
// perpendicular residue - which under the old model was fed straight into the aim lead - is
// discarded, because it can only ever be measurement noise or a bounce. The magnitude comes from the
// arena's thrust ceiling for that ship rather than from the derivative, so the estimate is physically
// incapable of exceeding what a real engine can produce. An enemy's *upgraded* thrust is not
// readable, so the arena's MaximumThrust is the honest ceiling to use.
//
// ---------------------------------------------------------------------------------------------
// THE SAMPLE RATE BUG THIS ALSO FIXES. A remote player's `velocity` is not simulated - it is hard
// assigned on packet arrival (PlayerManager::OnPositionPacket) and never touched again between
// packets, because SimulatePlayer only integrates position and only alters velocity on a wall bounce.
// Position updates arrive at SendPositionDelay (5 ticks here) at best and considerably slower in
// practice, while this node runs every tick at 100 Hz.
//
// The old code differenced against *last tick* regardless, so on the one tick a packet landed it
// divided a whole packet-interval's worth of velocity change by a single tick's dt - inflating the
// acceleration by the packet interval in ticks, five-fold at the absolute best - and read zero on
// every tick in between. The exponential average then smeared that spike across the following
// ~100 ms. The result was an aim lead that lurched well ahead of the target for a few tenths of a
// second after every packet and sat behind it the rest of the time.
//
// The fix is to only differentiate across an actual velocity *change* and to measure dt from the
// previous change, so the interval matches the data. Between changes the state is held rather than
// decayed toward zero, since "no new packet" is not evidence that they stopped thrusting.
//
// ---------------------------------------------------------------------------------------------
// The previous sample lives on the node rather than on the blackboard. It used to sit under fixed
// blackboard keys ("target_accel_prev_id" and friends), which quietly broke as soon as a behavior
// tree ran more than one of these per tick: the nexus trees evaluate a target-override chain, so
// this ran once for the nearest enemy and again for the team focus target, each call overwriting the
// other's sample. Every call then saw a previous id belonging to the *other* target, treated it as a
// target change, and zeroed the estimate - so whenever the team focus target differed from the
// nearest enemy, which is most of the time, the acceleration fed to PredictiveAimNode was
// permanently zero and the aim lead bias did nothing at all. Per-instance state makes each call site
// its own independent series, which is what the code always assumed it had. Safe because one
// ZeroBot runs per process, so a node instance is per-bot state, not shared across a fleet.
struct TargetAccelerationNode : public behavior::BehaviorNode {
  TargetAccelerationNode(const char* target_player_key, const char* output_key, float smoothing = 0.35f)
      : target_player_key(target_player_key), output_key(output_key), smoothing(smoothing) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    u32 tick = GetCurrentTick();

    // ShipController applies thrust as `velocity += heading * thrust * (10/16) * dt`, so this is the
    // ship's acceleration in tiles per second squared.
    float thrust_accel =
        ctx.bot->game->connection.settings.ShipSettings[target->ship].MaximumThrust * (10.0f / 16.0f);

    s32 age = has_previous ? TICK_DIFF(tick, previous_tick) : 0;
    bool same_target = has_previous && previous_id == target->id;

    if (!same_target || age < 0 || (u32)age > kMaxSampleAgeTicks) {
      // Either a different player entirely, or we have not seen them change velocity in long enough
      // that the last sample says nothing about what they are doing now. Re-baseline rather than
      // differentiate against something stale.
      thrust_state = 0.0f;
      previous_id = target->id;
      previous_velocity = target->velocity;
      previous_tick = tick;
      has_previous = true;

      ctx.blackboard.Set(output_key, Vector2f(0, 0));
      return behavior::ExecuteResult::Success;
    }

    Vector2f delta = target->velocity - previous_velocity;

    // Only a packet moves a remote player's velocity, so anything else is the same sample we already
    // measured - differencing it again would just divide zero by a growing dt.
    if (delta.LengthSq() > 0.0f && age > 0 && thrust_accel > 0.0f) {
      float dt = age / 100.0f;

      // Only the component along the hull can be engine thrust. Everything across it is a bounce, a
      // repel, or packet noise, and none of those are worth predicting.
      float along = delta.Dot(target->GetHeading()) / dt;
      float ratio = along / thrust_accel;

      if (ratio > 1.0f) ratio = 1.0f;
      if (ratio < -1.0f) ratio = -1.0f;

      // Coast, forward, or reverse - a small reading is a pilot holding course, not a third of an
      // engine.
      if (ratio > -kThrustDeadband && ratio < kThrustDeadband) ratio = 0.0f;

      // Smooth the *state*, not the magnitude: how consistently they have been on the throttle
      // recently, which is what predicts whether they will still be on it during the flight.
      thrust_state += (ratio - thrust_state) * smoothing;

      previous_velocity = target->velocity;
      previous_tick = tick;
    }

    ctx.blackboard.Set(output_key, target->GetHeading() * (thrust_accel * thrust_state));

    return behavior::ExecuteResult::Success;
  }

  // Half a second without a velocity change. Past that the target has had time to turn completely,
  // so the old sample says nothing about what they are doing right now.
  static constexpr u32 kMaxSampleAgeTicks = 50;

  // Below a tenth of full thrust, treat it as coasting. Position packets quantize velocity, so a
  // stationary throttle still produces small non-zero deltas.
  static constexpr float kThrustDeadband = 0.1f;

  const char* target_player_key = nullptr;
  const char* output_key = nullptr;
  float smoothing = 0.35f;

  PlayerId previous_id = 0;
  Vector2f previous_velocity;
  u32 previous_tick = 0;
  bool has_previous = false;

  // Signed fraction of full thrust, -1 (full reverse) to +1 (full forward).
  float thrust_state = 0.0f;
};

}  // namespace nexus
}  // namespace zero

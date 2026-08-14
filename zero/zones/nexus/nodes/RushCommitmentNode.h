#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Bounds a dive in TIME and in how much the situation is allowed to decay while it runs, and holds a
// refractory period afterwards so an abort actually sticks.
//
// The rush sequence this gates already re-tests every one of its conditions each tick, so a dive is
// not "latched" in the flag sense. The problem it was still losing to is that every one of those
// conditions is an ABSOLUTE test of the present moment, and none of them can express the thing that
// actually goes wrong: the fight was worth diving into when we started, and stopped being worth it
// while we were on the way.
//
// THE TIME BOUND IS THE LOAD-BEARING HALF. A chase that has run for seconds without converting is
// one we are losing on speed, and every additional second is spent further from our own team.
// Nothing else in the sequence times out: the target stays weak (its estimate only recharges
// slowly), the distance test keeps passing because we keep pace, and so the dive can run as long as
// the target keeps running.
//
// The advantage-drop check is the weaker half, and rec30 showed why it has to be set generously.
// The idea was that `local_advantage >= 0` passes identically whether we committed at +2 and bled
// down to parity or sat at parity throughout, and that only the first means we are arriving alone.
// That is true, but the head-count is a discrete count over a 25-tile radius and its ordinary
// jitter is +/-1 (p25 0, median 0, p75 +1 over 15426 samples), so a threshold of 1 fires on noise.
// It fired asymmetrically too: the rush arms whenever advantage >= 0, so it committed on jitter
// peaks and aborted on reversion to the mean, which suppressed diving almost completely.
//
// It is also mostly redundant. The rush sequence re-tests the absolute `local_advantage >= 0` gate
// every tick, and that already ends the dive the moment we are actually outnumbered - which is the
// real danger, not the delta. So set max_advantage_loss high enough that it only catches a genuine
// collapse from a strong start, and let the absolute gate do the ordinary work.
//
// Returns Success while the dive remains justified and Failure once it does not, which drops the
// tree through to the ordinary press/orbit/flee branches for the tick. It deliberately does NOT
// force a retreat: EnergyDisadvantageNode already owns that decision, and an abort means "this is no
// longer worth diving for", not necessarily "we are in trouble".
//
// STATE LIVES ON THE INSTANCE, never under blackboard keys - the tree calls a given node type
// several times per tick against different targets, and fixed keys let one call read back a sample
// belonging to another. Same reasoning as TargetAccelerationNode, and safe for the same reason: one
// ZeroBot per process, so an instance is per-bot state.
struct RushCommitmentNode : public behavior::BehaviorNode {
  RushCommitmentNode(const char* target_player_key, const char* advantage_key, u32 max_rush_ticks,
                     float max_advantage_loss, u32 abort_cooldown_ticks)
      : target_player_key(target_player_key),
        advantage_key(advantage_key),
        max_rush_ticks(max_rush_ticks),
        max_advantage_loss(max_advantage_loss),
        abort_cooldown_ticks(abort_cooldown_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    Tick current_tick = GetCurrentTick();

    // An abort has to outlast the tick it happened on. The head-count fluctuates as players drift in
    // and out of the radius, so without this the very next tick can re-arm the dive we just called
    // off and we oscillate in place while still deep in their half.
    if (cooling_down) {
      if (TICK_GT(cooldown_end_tick, current_tick)) return Touch(current_tick, behavior::ExecuteResult::Failure);
      cooling_down = false;
    }

    // This node only runs when every condition ahead of it in the sequence passed, so a gap in
    // execution means the dive already ended for some other reason. Without this check `committed`
    // would survive that gap and the next genuine rush - possibly a different fight entirely - would
    // inherit a commit tick and advantage from the old one and abort immediately.
    if (committed && TICK_DIFF(current_tick, last_execute_tick) > kMaxExecuteGapTicks) committed = false;

    float advantage = ctx.blackboard.ValueOr<float>(advantage_key, 0.0f);

    if (!committed || committed_target_id != target->id) {
      committed = true;
      committed_target_id = target->id;
      commit_tick = current_tick;
      commit_advantage = advantage;
      return Touch(current_tick, behavior::ExecuteResult::Success);
    }

    bool too_long = (u32)TICK_DIFF(current_tick, commit_tick) > max_rush_ticks;
    bool support_lost = (commit_advantage - advantage) >= max_advantage_loss;

    if (too_long || support_lost) {
      committed = false;
      cooling_down = true;
      cooldown_end_tick = current_tick + abort_cooldown_ticks;
      return Touch(current_tick, behavior::ExecuteResult::Failure);
    }

    return Touch(current_tick, behavior::ExecuteResult::Success);
  }

 private:
  behavior::ExecuteResult Touch(Tick current_tick, behavior::ExecuteResult result) {
    last_execute_tick = current_tick;
    return result;
  }

  // Ticks of non-execution after which a commitment is considered finished rather than ongoing.
  // Generous enough to absorb a tick where an earlier condition flickers, short enough that a real
  // break in the dive is not mistaken for a continuous one.
  static constexpr s32 kMaxExecuteGapTicks = 10;

  const char* target_player_key = nullptr;
  const char* advantage_key = nullptr;
  u32 max_rush_ticks = 250;
  float max_advantage_loss = 1.0f;
  u32 abort_cooldown_ticks = 200;

  bool committed = false;
  u16 committed_target_id = 0;
  Tick commit_tick = 0;
  float commit_advantage = 0.0f;
  Tick last_execute_tick = 0;
  bool cooling_down = false;
  Tick cooldown_end_tick = 0;
};

}  // namespace nexus
}  // namespace zero

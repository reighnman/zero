#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

enum class EngagementPhase {
  // Cut off from the team with nothing to gain by staying out here. Movement is dominated by
  // getting back into supporting distance; we still shoot on the way.
  Regroup,
  // Losing the local trade or too weak to be in it. Open distance and recharge, keep poking.
  Recover,
  // The default. Hold a standoff, keep continuous pressure on, and wait for an opening.
  Poke,
  // An opening exists. Close and commit.
  Press,
};

// Decides which of four postures we are in this tick, once, so nothing downstream has to re-derive
// it from a scattered pile of conditions.
//
// Deriving posture in one place rather than inline in each branch matters here because the inputs
// interact. "Low energy" means something completely different at a two-man advantage than it does
// alone, and "the target is weak" is not an opening if we are the one who is cut off. A tree that
// tests those conditions independently in five different branches will always eventually disagree
// with itself.
//
// The thresholds come from measured play rather than intuition:
//
//  - Retreats in real matches start at a *median 80% energy*, not at panic levels; the median flee
//    run lasts 1.4 seconds and 25% of live time is spent in one. Backing off is a routine part of
//    the rhythm, not an emergency. That's why Recover's entry threshold is high-ish and its exit is
//    higher still rather than both sitting near zero.
//  - Bullets get fired at a median 90% energy and a p10 of 60%. Humans essentially never shoot
//    below 60%, which is what makes "recover" a real posture with its own firing discipline.
//  - Local head-count advantage dominates the exchange ratio: at -1 the trade is roughly break-even
//    (1.11 at 10-14 tiles) and at -2 it is a rout (0.58). So -1 is a reason to stop pressing and
//    -2 is a reason to actively leave.
//  - Victims are isolated: median 41 tiles from support two seconds before dying against a 27 tile
//    baseline. Regroup triggers off our own support distance for the same reason target selection
//    rewards it in the enemy.
//
// Hysteresis is applied to every transition through a minimum dwell time. Without it, a bot sitting
// exactly on a threshold flips posture every tick, and because posture drives both standoff
// distance and orbit direction, that reads as a bot vibrating in place instead of fighting.
struct EngagementPhaseNode : public behavior::BehaviorNode {
  EngagementPhaseNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;

    float self_energy_percent = GetSelfEnergyPercent(game, *self);
    float advantage = ctx.blackboard.ValueOr<float>("local_advantage", 0.0f);
    float support_distance = ctx.blackboard.ValueOr<float>("support_distance", kNoSupportDistance);
    float team_alive = ctx.blackboard.ValueOr<float>("team_alive", 0.0f);
    float target_energy_percent = ctx.blackboard.ValueOr<float>("target_energy_percent", 1.0f);
    float target_distance = ctx.blackboard.ValueOr<float>("target_distance", 1000.0f);
    float target_isolation = ctx.blackboard.ValueOr<float>("target_isolation", 0.0f);

    EngagementPhase desired = Decide(self_energy_percent, advantage, support_distance, team_alive,
                                     target_energy_percent, target_distance, target_isolation);

    Tick now = GetCurrentTick();

    if (desired != phase) {
      // Only allow a change once the current posture has had time to accomplish something. The one
      // exception is escalating into Press, which is a reaction to a transient opening and is worth
      // taking immediately - openings do not wait for a dwell timer.
      bool urgent = desired == EngagementPhase::Press;

      if (urgent || TICK_DIFF(now, phase_tick) >= (s32)min_dwell_ticks) {
        phase = desired;
        phase_tick = now;
      }
    }

    ctx.blackboard.Set<float>("phase", (float)phase);

    // Existence-only flags, so branches can gate on a posture with a plain BlackboardSetQueryNode
    // rather than an equality test against a float.
    SetFlag(ctx, "phase_regroup", phase == EngagementPhase::Regroup);
    SetFlag(ctx, "phase_recover", phase == EngagementPhase::Recover);
    SetFlag(ctx, "phase_poke", phase == EngagementPhase::Poke);
    SetFlag(ctx, "phase_press", phase == EngagementPhase::Press);

    // The standoff distance the movement controller should hold. Published here rather than baked
    // into the movement node so posture stays the single source of truth for it.
    ctx.blackboard.Set<float>("standoff_distance", GetStandoffDistance(phase));

    return behavior::ExecuteResult::Success;
  }

  // Energy at or below which we stop being willing to trade and start recovering, and the higher
  // level we must climb back to before rejoining. The gap is the hysteresis band.
  float recover_enter_energy = 0.45f;
  float recover_exit_energy = 0.75f;

  // Minimum energy to commit to a dive. Killers in real matches averaged 50% energy at the moment
  // of the kill, so anything much below this is committing to a fight we cannot finish.
  float press_min_energy = 0.5f;

  // A target this weak is worth pressing regardless of anything else - victims died at a median 10%
  // energy, and this is the band where a dive converts.
  float press_target_energy = 0.35f;

  // How much healthier than the target we have to be for the margin itself to justify closing, even
  // through incoming fire.
  //
  // The reasoning is simply that energy *is* the trade. If neither side misses, the one who started
  // with more is the one still alive at the end - so holding a margin means a straight exchange is
  // already won, and there is nothing to be gained by circling at range waiting for a cleaner
  // opening while they recharge the deficit away. Closing is what makes the damage land at all.
  //
  // The margin is a buffer rather than the win condition, and it is sized for two things that both
  // cut against us: the enemy's energy is `HeuristicEnergyTracker`'s estimate rather than a fact,
  // since SeeEnergy is off in this arena, and we do not hit as often as a good human does (bot
  // bullet accuracy runs 18-33% in these recordings against phong's 36%), so an even trade is not
  // actually even. A quarter of a tank is about two bullets of head start.
  float press_energy_margin = 0.25f;

  // A target this far from their own support is cut off, which is the strongest single predictor of
  // a kill available. Well above the 27 tile baseline so we only react to genuine isolation.
  float press_target_isolation = 38.0f;

  // Don't start a dive from further out than this; the approach itself would be spent crossing open
  // ground under fire. Kills in the corpus were set up from a median 32 tiles at T-3s.
  float press_max_distance = 36.0f;

  // Our own isolation that makes rejoining the team the priority. Sits above the median victim's
  // 41 tiles, because reacting exactly at the median would have us running home constantly.
  float regroup_support_distance = 46.0f;

  // Standoff distances per posture, in tiles. Poke sits at the range real players actually fire
  // from (bullets at a median 29 tiles), Press converges toward where kills actually land (median
  // 11 tiles), and Recover opens past effective return fire.
  float standoff_press = 10.0f;
  // Was 26, from the corpus-wide median firing range of 29 tiles. The strong player fights closer
  // than the average one: phong's median firing range is 23.2 tiles and the enemy he is actually
  // pointed at sits at 26.7, against 32-40 for these bots - and he was described as being closer to
  // the enemy than anyone in the match while finishing it 5-0. Kills in the corpus land at a median
  // 11 tiles regardless of where the poking happens from, so nothing is won by holding the outer
  // edge of bullet range.
  float standoff_poke = 23.0f;
  // Recover used to sit at 42, which no measured player ever holds. Humans down two or more heads
  // are at a median 16 tiles from their nearest enemy and *opening* at 3.5 tiles/sec - they break
  // range continuously rather than sprinting to a safe radius, because the people chasing them are
  // faster than the gap they are trying to open. 34 asks for a firm, sustained withdrawal from a
  // typical outnumbered range without demanding a full-speed flight the ship cannot win anyway.
  float standoff_recover = 34.0f;

  u32 min_dwell_ticks = 60;

 private:
  EngagementPhase phase = EngagementPhase::Poke;
  Tick phase_tick = 0;

  EngagementPhase Decide(float self_energy_percent, float advantage, float support_distance, float team_alive,
                         float target_energy_percent, float target_distance, float target_isolation) const {
    // Being alone is not "isolation" - there is nobody left to regroup with, and the rest of the
    // logic still applies. Without this, the last player alive would spend the endgame running
    // toward teammates who do not exist.
    bool has_team = team_alive > 0.0f;

    if (has_team && support_distance > regroup_support_distance && advantage <= 0.0f) {
      return EngagementPhase::Regroup;
    }

    // A two-man local disadvantage is a rout at every range measured. Leave regardless of energy -
    // unless there is no team left, in which case there is nothing to preserve ourselves for and
    // backing away from a 1v3 forever just loses it slowly.
    if (advantage <= -2.0f && has_team) return EngagementPhase::Recover;

    // The relative read, which is what was missing and why these bots never finished anyone off.
    // Every press condition used to be an *absolute* threshold - our energy above a half, theirs
    // below a third - so the ordinary situation of being meaningfully healthier than an opponent who
    // is nonetheless not yet critical produced no push at all. The bot poked, they recharged, and
    // the kill never landed.
    //
    // Energy is the whole of the trade in this game. If we hold a wide enough margin, we win the
    // exchange even flying straight into it, because they run out first - and closing is the only
    // way the damage actually lands, since kills happen at a median eleven tiles. So a margin is
    // itself an opening, and it also waives the absolute floor below: at 30% against someone on 5%,
    // pressing is correct and waiting is not.
    bool stronger = self_energy_percent >= target_energy_percent + press_energy_margin;

    bool opening = stronger || target_energy_percent <= press_target_energy ||
                   target_isolation >= press_target_isolation || advantage >= 1.0f;

    // Never push into a losing head-count. The exchange data is unambiguous that being down bodies
    // costs more than any range advantage can return, so a lone bot diving three of them is simply
    // feeding - the one exception being that our team is already gone, where the alternative is
    // losing anyway.
    bool numbers_ok = advantage >= 0.0f || !has_team;

    bool can_afford = stronger || self_energy_percent >= press_min_energy;

    if (opening && can_afford && numbers_ok && target_distance <= press_max_distance) {
      return EngagementPhase::Press;
    }

    bool recovering = phase == EngagementPhase::Recover;
    float energy_gate = recovering ? recover_exit_energy : recover_enter_energy;

    if (self_energy_percent < energy_gate) return EngagementPhase::Recover;

    return EngagementPhase::Poke;
  }

  float GetStandoffDistance(EngagementPhase p) const {
    switch (p) {
      case EngagementPhase::Press:
        return standoff_press;
      case EngagementPhase::Recover:
        return standoff_recover;
      case EngagementPhase::Regroup:
        return standoff_recover;
      case EngagementPhase::Poke:
      default:
        return standoff_poke;
    }
  }

  static void SetFlag(behavior::ExecuteContext& ctx, const char* key, bool value) {
    if (value) {
      ctx.blackboard.Set<bool>(key, true);
    } else {
      ctx.blackboard.Erase(key);
    }
  }
};

}  // namespace teamversus
}  // namespace zero

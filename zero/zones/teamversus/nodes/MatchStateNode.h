#pragma once

#include <zero/BotController.h>
#include <zero/ChatQueue.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/TeamVersus.h>

namespace zero {
namespace teamversus {

// Advances the match lifecycle and publishes it as blackboard flags.
//
// Match start is announced as an arena chat message ("GO!") and nothing in the game state marks it.
// The zone controller catches that message; this node is what turns it into something the tree can
// gate on, and is also what notices the two transitions the controller cannot see because they are
// state rather than events:
//
//  - We got put in a ship. That is the cue to arm the safety-net timer, so a missed start message
//    can't leave the bot inert for an entire round.
//  - We got put back in spec. That ends the match from our point of view regardless of what the
//    zone said, and resets everything so the next round starts clean rather than inheriting the
//    previous round's posture and target.
//
// Publishing "match_live" and "spectating" as existence flags rather than as a value keeps the
// tree's gating uniform with every other posture flag.
struct MatchStateNode : public behavior::BehaviorNode {
  MatchStateNode(u32 safety_net_ticks = 3000) : safety_net_ticks(safety_net_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_teamversus = ctx.blackboard.Value<TeamVersus*>("teamversus");
    if (!opt_teamversus || !*opt_teamversus) return behavior::ExecuteResult::Failure;

    TeamVersus* tv = *opt_teamversus;
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    Tick now = GetCurrentTick();
    bool spectating = self->ship >= 8;

    if (spectating != was_spectating) {
      if (spectating) {
        tv->match.EnterSpectate();
      } else {
        // Just came out of spec. The real start arrives later as "GO!"; this is only the fallback.
        tv->match.ArmSafetyNet(now, safety_net_ticks);
      }

      was_spectating = spectating;
    }

    bool live = !spectating && tv->match.Update(now);

    if (spectating) {
      ctx.blackboard.Set<bool>("spectating", true);
    } else {
      ctx.blackboard.Erase("spectating");
    }

    if (live) {
      ctx.blackboard.Set<bool>("match_live", true);
    } else {
      ctx.blackboard.Erase("match_live");
    }

    return behavior::ExecuteResult::Success;
  }

  u32 safety_net_ticks = 3000;

 private:
  bool was_spectating = true;
};

// Resends the self-queue command on an interval (default "?next" every minute).
//
// The tree only ever reaches this from the spectating branch, which is the intended and only
// correct place for it: sitting in spec is exactly the state that means "not in a match", and
// sending a queue command from inside a ship would ask to be pulled out of the match currently
// being played. The repetition exists because a single request at the moment we enter spec is
// fragile - the queue can drop us on an arena recycle, or the request can land while a match is
// still tearing down - and the failure mode is silent: a bot that idles in spec forever.
//
// The command text and interval come from config (TeamVersus:QueueCommand /
// TeamVersus:QueueInterval), and setting the command to empty disables queueing entirely for
// arenas where the zone pulls bots in by itself.
struct QueueCommandNode : public behavior::BehaviorNode {
  QueueCommandNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_teamversus = ctx.blackboard.Value<TeamVersus*>("teamversus");
    if (!opt_teamversus || !*opt_teamversus) return behavior::ExecuteResult::Failure;

    TeamVersus* tv = *opt_teamversus;
    if (tv->queue_command.empty()) return behavior::ExecuteResult::Failure;

    Tick now = GetCurrentTick();

    // Send immediately the first time rather than waiting out a full interval before the first
    // attempt - `last_send_tick` of zero means "never sent", not "sent at tick zero".
    if (last_send_tick != 0 && TICK_DIFF(now, last_send_tick) < (s32)tv->queue_interval_ticks) {
      return behavior::ExecuteResult::Failure;
    }

    // GetCurrentTick can legitimately be zero, which would read as "never sent" forever and turn
    // the interval into a spam loop for that one tick value.
    last_send_tick = now == 0 ? 1 : now;

    Event::Dispatch(ChatQueueEvent::Public(tv->queue_command.data()));

    return behavior::ExecuteResult::Success;
  }

 private:
  Tick last_send_tick = 0;
};

}  // namespace teamversus
}  // namespace zero

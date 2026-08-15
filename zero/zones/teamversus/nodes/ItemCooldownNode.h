#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

#include <string>

namespace zero {
namespace teamversus {

// A self-imposed minimum interval between two presses of the same item key.
//
// This exists because none of an item's effects are observable on the tick it is used. A repel takes
// time to push projectiles clear, a decoy takes time to be noticed and mis-targeted, a portal takes
// time to move us and for the new position to come back around the perception pass. Meanwhile the
// condition that triggered the item - a lethal volley still in flight - is very much still true on
// the next tick, and will be for several ticks. With nothing in between, the tree re-evaluates the
// same unchanged situation at 100Hz and spends the item again, and again, until it runs out.
//
// That is how a ship with two repels ends up with zero repels and full energy against a single bomb.
// The game's own cooldowns do not prevent it either: they gate the *weapon* systems, and several of
// them are short enough to let a second press through well before the first has resolved.
//
// So the rule is: press, then wait long enough for the effect to happen and for perception to
// reflect it, before the same key is even considered again. About a second is what the user asked
// for and it matches the timescale of every effect involved.
//
// IMPORTANT: this node *claims* the cooldown when it succeeds. It has to be the last gate in its
// sequence, immediately before the InputActionNode that presses the key - anything placed after it
// that can fail will consume the cooldown without a press ever happening, and the item will go quiet
// for a second for no reason.
//
// State lives on the blackboard rather than on the node instance, which is the opposite of the rule
// the aim and threat nodes follow, and deliberately. That rule is about per-*target* memory, where a
// shared key means two different players' samples get differenced against each other. This is
// per-*item* memory, and it genuinely must be shared: the portal key is pressed from one branch and
// the warp key from another, several item branches are mutually exclusive but not co-located, and
// two node instances holding separate timers for the same physical key would each let a press
// through.
struct ItemCooldownNode : public behavior::BehaviorNode {
  ItemCooldownNode(const char* item_name, u32 cooldown_ticks)
      : key(std::string("item_used_") + item_name), cooldown_ticks(cooldown_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Tick now = GetCurrentTick();

    auto opt_last = ctx.blackboard.Value<Tick>(key);

    if (opt_last && TICK_DIFF(now, *opt_last) < (s32)cooldown_ticks) {
      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set<Tick>(key, now);

    return behavior::ExecuteResult::Success;
  }

  std::string key;
  u32 cooldown_ticks = 100;
};

}  // namespace teamversus
}  // namespace zero

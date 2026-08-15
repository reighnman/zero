#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>
#include <zero/zones/teamversus/nodes/ThreatAssessmentNode.h>

namespace zero {
namespace teamversus {

// Spends a repel, which pushes every nearby player and projectile away at once.
//
// Repels are the scarcest thing a ship carries - two from spawn - so the entire question is when
// they are worth burning. Measured usage across 758 repels in real matches is unambiguous about
// what a repel is actually for:
//
//     energy at time of use : p10 10%   p25 20%   median 30%   p75 50%
//     distance to nearest enemy : p10 5t   p25 9t   median 13t
//     damage in the second before : ~250 raw    damage in the second after : ~100 raw
//
// That is a panic button, not a tempo tool. It gets spent at low energy, at close range, in the
// middle of taking real damage, and it works - incoming damage drops by more than half immediately
// afterward. Almost nobody spends one above 70% energy.
//
// So the gate is two conditions that only mean anything together: the damage about to land would
// kill us, and there is no longer time to dodge it. Either alone is wrong. Lethal-but-avoidable is a
// movement problem and spending a repel on it wastes the item on a shot a thrust would have cleared;
// unavoidable-but-survivable is not a problem at all.
//
// Both come from "threat_unavoidable", which ThreatAssessmentNode publishes from the single
// perception-pass scan. "No longer time to dodge" there is computed rather than assumed: each threat
// is re-evaluated at the distance we could physically open up before it arrives, given our thrust,
// our rotation rate and where our nose currently points. That last part is why a fixed reaction time
// cannot do the job - a ship already broadside to the shot displaces from tick one, while one
// pointed down the shot's path spends most of the window turning before any thrust helps, and
// treating those alike is exactly what makes a bot repel something it could have flown out of. This
// node deliberately does not rescan. It used to, with its own narrower distance, and two separately-configured scans of the
// same weapon list is exactly how a bot ends up believing a volley is fatal enough to break aim for
// but not fatal enough to repel. One scan, one verdict.
//
// The energy condition being expressed against the *threat* rather than as a fixed percentage is the
// whole point. A 750-damage bomb centred on us is fatal at 40% energy and survivable at full, and
// the same repel is correct in the first case and wasted in the second.
//
// What is left here is the part specific to the repel itself: do we have one, is it off the game's
// own cooldown, and is the thing we are trying to push actually within reach. RepelDistance bounds
// what a repel can move; one spent on a bomb outside that radius accomplishes nothing except being
// out of repels when the next one arrives.
//
// This node only decides; the tree presses the key.
struct RepelDecisionNode : public behavior::BehaviorNode {
  RepelDecisionNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;

    if (game.ship_controller.ship.repels == 0) return behavior::ExecuteResult::Failure;
    if (TICK_GT(game.ship_controller.ship.next_repel_tick, GetCurrentTick())) {
      return behavior::ExecuteResult::Failure;
    }

    // Lethal and no longer dodgeable. Both halves of that come from the one perception-pass scan.
    if (!ctx.blackboard.Has("threat_unavoidable")) return behavior::ExecuteResult::Failure;

    // Is the thing we want to push actually within push range? A repel moves what is inside
    // RepelDistance and nothing beyond it, so pressing the key against something further out spends
    // the item and changes nothing.
    Vector2f threat_origin = ctx.blackboard.ValueOr<Vector2f>("threat_origin", self->position);
    float repel_range = game.connection.settings.RepelDistance / 16.0f;

    if (threat_origin.DistanceSq(self->position) > repel_range * repel_range) {
      return behavior::ExecuteResult::Failure;
    }

    return behavior::ExecuteResult::Success;
  }
};

}  // namespace teamversus
}  // namespace zero

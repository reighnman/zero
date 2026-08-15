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
// So the gate is: something is inbound that we cannot afford, and it is close enough that a repel
// actually reaches it. The reach check matters because RepelDistance bounds what a repel can push;
// spending one on a bomb still 20 tiles out accomplishes nothing except being out of repels when
// the next one arrives.
//
// The energy condition is expressed against the *threat*, not as a fixed percentage. A 750-damage
// bomb centred on us is fatal at 40% energy and survivable at full, and the same repel is correct
// in the first case and wasted in the second.
//
// This node only decides; the tree presses the key. Keeping the decision separate from the input
// means the same judgement can gate a fallback (use the portal instead) without duplicating any of
// the reasoning above.
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

    // How far a repel actually pushes things, in tiles. Anything further out is unaffected, so
    // that bounds the scan - but only bounds it. Scanning the full radius would total up shots that
    // are still two seconds away and perfectly dodgeable, and spend a scarce item on them. Real
    // repels go out at a median 13 tiles to the nearest enemy, which is the range where dodging has
    // stopped being an option.
    float repel_range = game.connection.settings.RepelDistance / 16.0f;
    float scan_distance = repel_range < max_scan_distance ? repel_range : max_scan_distance;

    ThreatReport report = AssessThreats(ctx, self, scan_distance);
    if (report.count == 0) return behavior::ExecuteResult::Failure;

    // Still enough time to move out of the way? Then move instead. A repel spent on something we
    // could have dodged is a repel we don't have when there's no time left to dodge.
    if (report.time_to_impact > impact_horizon) return behavior::ExecuteResult::Failure;

    // Would this actually kill us, allowing for the recharge that lands before impact? Without the
    // recharge term a bot at exactly the damage threshold burns a repel on something it was going
    // to survive with energy to spare.
    float recharge_per_second = (float)game.ship_controller.ship.recharge / 10.0f;
    float energy_at_impact = self->energy + recharge_per_second * report.time_to_impact;

    float max_energy = (float)game.ship_controller.ship.energy;
    if (energy_at_impact > max_energy) energy_at_impact = max_energy;

    if (report.damage < energy_at_impact * lethal_margin) return behavior::ExecuteResult::Failure;

    return behavior::ExecuteResult::Success;
  }

  // Fire when incoming damage reaches this fraction of the energy we'll have when it lands. Below
  // 1.0 because the damage estimate is exactly that - an estimate - and being wrong in the
  // direction of "survived with 5 energy" is much worse than being wrong in the direction of
  // "spent a repel slightly early".
  float lethal_margin = 0.85f;

  // Matches the measured median range at which repels actually get used, and is about a second of
  // bullet flight - roughly the point where turning and thrusting can no longer clear the shot.
  float max_scan_distance = 14.0f;

  // Seconds. Inside this, dodging is no longer a realistic alternative.
  float impact_horizon = 0.6f;
};

}  // namespace teamversus
}  // namespace zero

#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>
#include <zero/zones/teamversus/nodes/ThreatAssessmentNode.h>

namespace zero {
namespace teamversus {

// Dodges incoming fire, at one of two intensities depending on whether the shot can actually kill
// us.
//
// The distinction is the whole point. A ship only thrusts along its heading, so a genuine sideways
// dodge costs a rotation - and a rotation costs the aim, which costs the return fire, which is how
// a bot ends up alive, pointed the wrong way, and losing the exchange anyway. Real damage taken per
// sample in the corpus is lowest when players are nose-on to the enemy (5.37) and highest when
// broadside (8.50), so turning away is not free protection; it is a trade.
//
// So:
//
//  - **Survivable threat**: add the escape force but leave rotation alone, and report Failure so the
//    aim-and-shoot branch keeps running. Because Actuator resolves the combined force into
//    forward/backward thrust along a heading that is still clamped near the aim point, what this
//    actually produces is a change in *closing speed* while staying on target. That is a real dodge
//    - a shot with over a second of flight time is thrown off by a speed change as surely as by a
//    direction change - and it costs nothing.
//
//  - **Lethal threat**: take over completely. Rotate to the escape axis, push hard, and report
//    Success so nothing downstream gets a say. Aim is worth losing if the alternative is dying.
//
// Reporting Failure in the common case is deliberate and is what lets this sit at the top of the
// aim-and-shoot Parallel rather than in front of it.
//
// This *consumes* the threat report that ThreatAssessmentNode already published during perception
// rather than rescanning the weapon list itself. Rescanning would produce the same answer at twice
// the cost, but the real reason is consistency: the reflex stage decides whether to burn a repel
// from the published "threat_lethal" flag, and this node decides whether to give up aim from its
// own notion of lethal. If the two ever computed those from separately-configured scans, the bot
// could conclude a volley was fatal enough to break formation for but not fatal enough to repel,
// which is the worst of both. One scan, one verdict, both stages agreeing by construction.
struct EvasiveManeuverNode : public behavior::BehaviorNode {
  EvasiveManeuverNode(float lethal_fraction = 1.0f) : lethal_fraction(lethal_fraction) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float threat_damage = ctx.blackboard.ValueOr<float>("threat_damage", 0.0f);
    float threat_count = ctx.blackboard.ValueOr<float>("threat_count", 0.0f);
    Vector2f escape_direction = ctx.blackboard.ValueOr<Vector2f>("threat_escape", Vector2f(0, 0));

    // Nothing genuinely on a collision course. Leave steering completely untouched: applying a
    // force derived from a degenerate zero-length threat ray produces a direction with no relation
    // to any real danger, and blending that into legitimate movement is how a bot ends up appearing
    // to stall whenever a stray bullet passes nearby.
    if (threat_count <= 0.0f || threat_damage <= 0.0f) return behavior::ExecuteResult::Failure;
    if (escape_direction.LengthSq() <= 0.0f) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& steering = ctx.bot->bot_controller->steering;

    bool lethal = threat_damage >= self->energy * lethal_fraction;

    if (!lethal) {
      // Scale with how much it would hurt, so a graze produces a nudge and a near-fatal hit
      // produces a shove, without either taking over the tick.
      float severity = threat_damage / (float)game.ship_controller.ship.energy;
      float force = minimum_force + severity * survivable_force_scale;

      steering.force += escape_direction * force;

      return behavior::ExecuteResult::Failure;
    }

    // Committed break. Rotate onto the escape axis so thrust actually points along it, rather than
    // relying on forward/backward along a heading that is still locked to the target.
    steering.Face(game, self->position + escape_direction);
    steering.force += escape_direction * lethal_force;
    steering.SetRotationThreshold(0.0f);
    steering.AvoidWalls(game);

    return behavior::ExecuteResult::Success;
  }

  // What fraction of our current energy an incoming volley has to threaten before we give up aim
  // for it. 1.0 means "only if it kills me".
  float lethal_fraction = 1.0f;

  float minimum_force = 2.0f;
  float survivable_force_scale = 12.0f;
  float lethal_force = 10000.0f;
};

}  // namespace teamversus
}  // namespace zero

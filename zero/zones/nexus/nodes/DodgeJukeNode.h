#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>
#include <zero/zones/nexus/nodes/IncomingDamageQuery.h>

namespace zero {
namespace nexus {

// Nudges thrust away from a moderate incoming threat without touching rotation, so it can run
// alongside FaceNode/firing in the aim-and-shoot Parallel instead of preempting them the way the
// full DodgeIncomingDamage escape does.
//
// Subspace ships only thrust along their current heading, so this can't produce a true sideways
// dodge while locked onto a target. What it actually does is add to the blended force that decides
// Actuator's forward/backward thrust choice, while FaceNode's rotation target keeps the heading
// clamped near the target (see Actuator::Update's rotation_threshold blending) - so the ship keeps
// its nose on the target and only its closing speed changes. That's still a real dodge: changing
// speed toward or away from the target is often enough to throw off a leaded shot without ever
// turning away from it. It's not a substitute for DodgeIncomingDamage against a shot that's already
// lined up dead-on - that still needs the full break-off escape.
struct DodgeJukeNode : public behavior::BehaviorNode {
  DodgeJukeNode(float distance, float minimum_force = 2.0f) : distance(distance), minimum_force(minimum_force) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    IncomingDamageReport report = GetIncomingDamage(ctx, self, distance);

    // Nothing on a collision course to juke away from.
    if (report.weapon_count == 0) return behavior::ExecuteResult::Failure;

    float est_damage = report.weapon_count * report.average_damage;
    float damage_percent = est_damage / (float)ctx.bot->game->ship_controller.ship.energy;

    Vector2f incoming_direction = Normalize(report.average_direction);
    Ray ray(report.average_origin, incoming_direction);
    Vector2f closest_hit = ray.GetClosestPosition(self->position);

    Vector2f side = Normalize(self->position - closest_hit);
    float force = minimum_force + damage_percent * 10.0f;

    ctx.bot->bot_controller->steering.force += side * force;

    return behavior::ExecuteResult::Success;
  }

  float distance = 0.0f;
  float minimum_force = 2.0f;
};

}  // namespace nexus
}  // namespace zero

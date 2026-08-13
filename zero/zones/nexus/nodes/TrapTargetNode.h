#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/nexus/nodes/EnclosureQuery.h>

namespace zero {
namespace nexus {

// The offensive counterpart to WallEscapeNode: works out whether the target is the one who is
// boxed in, and if so where to sit so they cannot get back out.
//
// Terrain cuts both ways. A cornered opponent cannot kite, cannot open range to recharge, and has
// only one direction left to run - which makes them worth committing to at a range we would
// otherwise consider risky, and makes their movement predictable enough to lead shots against. This
// is the situation the bot should actively seek out rather than merely surviving.
//
// Writes the blocking position to `trap_position_key` and succeeds only when the trap is real:
//   - the target is genuinely enclosed (`max_target_open_fraction` / `max_target_escape_arc`),
//   - we are not ourselves boxed in while doing it, since standing in their only exit is exactly
//     the kind of spot that becomes our own dead end,
//   - and the blocking spot is somewhere we can actually sit.
//
// The blocking position is placed between the target and the middle of their escape corridor, so
// holding it plugs the gap rather than chasing them deeper into their own pocket.
struct TrapTargetNode : public behavior::BehaviorNode {
  TrapTargetNode(const char* target_player_key, const char* trap_position_key, float probe_distance,
                 float max_target_open_fraction, float max_target_escape_arc, float block_distance)
      : target_player_key(target_player_key),
        trap_position_key(trap_position_key),
        probe_distance(probe_distance),
        max_target_open_fraction(max_target_open_fraction),
        max_target_escape_arc(max_target_escape_arc),
        block_distance(block_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    float self_radius = game.connection.settings.ShipSettings[self->ship].GetRadius();
    float target_radius = game.connection.settings.ShipSettings[target->ship].GetRadius();

    EnclosureReport target_enclosure =
        GetEnclosure(game, target->position, target->frequency, target_radius, probe_distance);

    bool target_trapped = target_enclosure.open_fraction <= max_target_open_fraction ||
                          target_enclosure.escape_arc <= max_target_escape_arc;
    if (!target_trapped) return behavior::ExecuteResult::Failure;

    // Don't wall ourselves in to wall them in.
    EnclosureReport self_enclosure =
        GetEnclosure(game, self->position, self->frequency, self_radius, probe_distance);
    if (self_enclosure.open_fraction < kMinSelfOpenFraction) return behavior::ExecuteResult::Failure;

    Vector2f block_position = target->position + target_enclosure.escape_direction * block_distance;

    if (!game.GetMap().CanOccupy(block_position, self_radius, self->frequency)) {
      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set(trap_position_key, block_position);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* trap_position_key = nullptr;
  float probe_distance = 12.0f;
  float max_target_open_fraction = 0.4f;
  float max_target_escape_arc = 1.6f;
  float block_distance = 8.0f;

 private:
  // We need meaningfully more room than the target does for the trade to be worth making.
  static constexpr float kMinSelfOpenFraction = 0.5f;
};

}  // namespace nexus
}  // namespace zero

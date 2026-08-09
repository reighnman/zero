#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Retreats away from a target while staying wall-aware, for defensive kiting/leashing.
//
// Plain Seek-based retreats push straight away from the target with no regard for the map, so a
// bot backing away in a straight line can get driven into a wall or corner and pinned there. This
// blends in wall avoidance so it curves around obstacles, and if it still ends up wedged (e.g. the
// target is blocking the only clear escape out of a corner), it detects the resulting wall bounces
// and overrides steering to push toward whatever direction has the most open space until it breaks
// free.
struct FleeNode : public behavior::BehaviorNode {
  FleeNode(const char* position_key, float target_distance)
      : position_key(position_key), target_distance(target_distance) {}
  FleeNode(const char* position_key, const char* target_distance_key)
      : position_key(position_key), target_distance_key(target_distance_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_position = ctx.blackboard.Value<Vector2f>(position_key);
    if (!opt_position.has_value()) return behavior::ExecuteResult::Failure;

    float distance = target_distance;

    if (target_distance_key) {
      auto opt_distance = ctx.blackboard.Value<float>(target_distance_key);
      if (!opt_distance) return behavior::ExecuteResult::Failure;

      distance = *opt_distance;
    }

    Vector2f threat_position = *opt_position;
    auto& game = *ctx.bot->game;
    auto& steering = ctx.bot->bot_controller->steering;

    if (IsCornered(*self, ctx)) {
      Vector2f escape_direction = steering.FindOpenDirection(game);

      steering.Face(game, self->position + escape_direction);
      // Avoid Seek here because it corrects for our current velocity, which fights the escape.
      steering.force += escape_direction * 1000.0f;

      return behavior::ExecuteResult::Success;
    }

    steering.Flee(game, threat_position, distance);

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
  const char* target_distance_key = nullptr;
  float target_distance = 0.0f;

 private:
  // Tracks consecutive wall bounces so a retreat that's fighting a wall can be detected and broken
  // out of. Kept independent of FollowPathNode's stuck detection since the two track separate
  // movement contexts and 'stuck_corner' resolution doesn't apply here.
  static bool IsCornered(const Player& self, behavior::ExecuteContext& ctx) {
    constexpr u32 kCorneredTickThreshold = 50;
    constexpr u32 kCorneredTickMax = 100;

    u32 last_bounce_tick = ctx.blackboard.ValueOr<u32>("flee_last_bounce_tick", 0U);
    u32 last_bounce_check = ctx.blackboard.ValueOr<u32>("flee_last_bounce_check", 0U);
    u32 bounce_count = ctx.blackboard.ValueOr<u32>("flee_bounce_count", 0U);

    u32 tick = GetCurrentTick();

    if (tick != last_bounce_check) {
      if (last_bounce_tick != self.last_bounce_tick) {
        if (bounce_count < kCorneredTickMax) ++bounce_count;
        ctx.blackboard.Set("flee_last_bounce_tick", self.last_bounce_tick);
      } else if (bounce_count > 0) {
        --bounce_count;
      }

      ctx.blackboard.Set("flee_bounce_count", bounce_count);
      ctx.blackboard.Set<u32>("flee_last_bounce_check", tick);
    }

    return bounce_count >= kCorneredTickThreshold;
  }
};

}  // namespace nexus
}  // namespace zero

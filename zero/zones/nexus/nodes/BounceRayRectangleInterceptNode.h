#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Extends the direct-line shot check (RayRectangleInterceptNode) with a single wall bounce.
// Bullets in this game reflect off walls with standard physics (unlike bombs, which just stop on
// impact), so a target hidden around a corner isn't necessarily out of reach - a shot bounced off
// the nearby wall can still connect. Meant to be tried as a fallback once the direct check has
// already failed, since a direct hit never depends on the bounce geometry lining up exactly right.
//
// Only simulates one bounce. A shot needing two or more bounces to connect is unlikely to still be
// dangerous by the time it arrives, and searching further bounces isn't worth it for a check redone
// from scratch every tick.
struct BounceRayRectangleInterceptNode : public behavior::BehaviorNode {
  BounceRayRectangleInterceptNode(const char* ray_key, const char* rect_key, const char* max_distance_key)
      : ray_key(ray_key), rect_key(rect_key), max_distance_key(max_distance_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    auto opt_ray = ctx.blackboard.Value<Ray>(ray_key);
    if (!opt_ray.has_value()) return behavior::ExecuteResult::Failure;

    auto opt_rect = ctx.blackboard.Value<Rectangle>(rect_key);
    if (!opt_rect.has_value()) return behavior::ExecuteResult::Failure;

    auto opt_max_distance = ctx.blackboard.Value<float>(max_distance_key);
    if (!opt_max_distance.has_value()) return behavior::ExecuteResult::Failure;

    Ray& ray = *opt_ray;
    Rectangle& rect = *opt_rect;
    float max_distance = *opt_max_distance;

    CastResult wall = ctx.bot->game->GetMap().Cast(ray.origin, ray.direction, max_distance, self->frequency);
    if (!wall.hit) return behavior::ExecuteResult::Failure;

    float remaining_distance = max_distance - wall.distance;
    if (remaining_distance <= 0.0f) return behavior::ExecuteResult::Failure;

    Ray bounce_ray(wall.position, Reflect(ray.direction, wall.normal));

    float dist = 0.0f;
    if (!RayBoxIntersect(bounce_ray, rect, &dist, nullptr)) return behavior::ExecuteResult::Failure;

    return dist <= remaining_distance ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  const char* ray_key = nullptr;
  const char* rect_key = nullptr;
  const char* max_distance_key = nullptr;
};

}  // namespace nexus
}  // namespace zero

#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Wobbles the aimshot perpendicular to the aim line. When given an acceleration_key, the spread is
// scaled by how much the target has actually been accelerating (from TargetAccelerationNode)
// instead of a blind constant spread - a target holding a steady course gets shot at precisely,
// only a genuinely maneuvering target gets spread fire to hedge against. Leaving acceleration_key
// null falls back to a constant spread.
//
// This used to be copy-pasted as a local struct in every nexus behavior .cpp file. Some copies
// were updated with the acceleration_key field and some weren't, which left multiple incompatible
// definitions of zero::nexus::ShotSpreadNode across translation units - an ODR violation that
// caused sporadic memory corruption/crashes. Keep this as the single definition; don't re-add a
// local copy in a behavior file.
struct ShotSpreadNode : public behavior::BehaviorNode {
  ShotSpreadNode(const char* aimshot_key, float max_spread, float period, const char* acceleration_key = nullptr,
                 float maneuvering_normalizer = 1.0f)
      : aimshot_key(aimshot_key),
        acceleration_key(acceleration_key),
        max_spread(max_spread),
        maneuvering_normalizer(maneuvering_normalizer),
        period(period) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aimshot = ctx.blackboard.Value<Vector2f>(aimshot_key);
    if (!opt_aimshot) return behavior::ExecuteResult::Failure;
    Vector2f aimshot = *opt_aimshot;

    float spread_scale = 1.0f;

    if (acceleration_key) {
      Vector2f acceleration = ctx.blackboard.ValueOr<Vector2f>(acceleration_key, Vector2f(0, 0));
      float maneuvering = acceleration.Length();

      spread_scale = maneuvering_normalizer > 0.0f ? maneuvering / maneuvering_normalizer : 0.0f;
      if (spread_scale > 1.0f) spread_scale = 1.0f;
      if (spread_scale < 0.0f) spread_scale = 0.0f;
    }

    float spread = max_spread * spread_scale;

    Vector2f aim_direction = Normalize(aimshot - self->position);
    Vector2f perp = Perpendicular(aim_direction);

    float use_period = period > 0.0f ? period : 1.0f;

    float t = GetTime();
    aimshot += perp * sinf(t / use_period) * spread;

    ctx.blackboard.Set(aimshot_key, aimshot);

    return behavior::ExecuteResult::Success;
  }

  inline float GetTime() { return GetMicrosecondTick() / (kTickDurationMicro * 10.0f); }

  const char* aimshot_key = nullptr;
  const char* acceleration_key = nullptr;
  float max_spread = 0.0f;
  float maneuvering_normalizer = 1.0f;
  float period = 1.0f;
};

}  // namespace nexus
}  // namespace zero

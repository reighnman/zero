#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

#include <limits>

namespace zero {
namespace teamversus {

// Reads the local tactical situation around ourselves: how many friends and enemies are close
// enough to matter, the resulting numerical advantage, and how far away our own nearest teammate is.
//
// This is the single most load-bearing measurement in the zone. Damage-exchange data from real
// 4v4 matches splits almost entirely on local head-count rather than on range:
//
//     range      exchange ratio at advantage -2 / -1 / 0 / +1
//     0-4  t          1.86     2.25     7.95     12.78
//     10-14 t         0.58     1.11     2.33      3.09
//     15-19 t         0.73     1.24     1.90      2.63
//
// Read down a column and range barely moves the number; read across a row and it changes by an
// order of magnitude. The same fight that is overwhelmingly winning at even numbers is losing at
// -1. And splitting the corpus by "was this player the only one shooting" flips close range
// entirely: a sole attacker *loses* the trade from 5 to 19 tiles (ratio 0.73-0.85) and only wins
// again past 35 tiles or at point-blank.
//
// So every posture decision downstream keys off `local_advantage` first and distance second, which
// is the opposite of how a naive "close in and fight" bot works.
//
// The 25 tile radius matches the radius the exchange table was computed at. It is not an arbitrary
// "nearby" number - widening it would count teammates who are too far away to actually contribute
// to the trade within the second the exchange is measured over.
struct TeamAdvantageNode : public behavior::BehaviorNode {
  TeamAdvantageNode(float radius = 25.0f) : radius(radius) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& pm = game.player_manager;

    float radius_sq = radius * radius;

    float friends_near = 0.0f;
    float enemies_near = 0.0f;
    float team_alive = 0.0f;
    float enemies_alive = 0.0f;

    Player* nearest_teammate = nullptr;
    float nearest_teammate_dist_sq = std::numeric_limits<float>::max();

    Player* nearest_enemy = nullptr;
    float nearest_enemy_dist_sq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* player = pm.players + i;

      if (player->id == self->id) continue;
      if (!IsEngageablePlayer(game, *player)) continue;

      float dist_sq = player->position.DistanceSq(self->position);
      bool same_team = player->frequency == self->frequency;

      if (same_team) {
        team_alive += 1.0f;

        if (dist_sq < nearest_teammate_dist_sq) {
          nearest_teammate_dist_sq = dist_sq;
          nearest_teammate = player;
        }

        if (dist_sq <= radius_sq) friends_near += 1.0f;
      } else {
        enemies_alive += 1.0f;

        if (dist_sq < nearest_enemy_dist_sq) {
          nearest_enemy_dist_sq = dist_sq;
          nearest_enemy = player;
        }

        if (dist_sq <= radius_sq) enemies_near += 1.0f;
      }
    }

    ctx.blackboard.Set<float>("local_friends", friends_near);
    ctx.blackboard.Set<float>("local_enemies", enemies_near);
    ctx.blackboard.Set<float>("local_advantage", friends_near - enemies_near);
    ctx.blackboard.Set<float>("team_alive", team_alive);
    ctx.blackboard.Set<float>("enemies_alive", enemies_alive);

    // The closest enemy, which is a different question from who we have chosen to shoot at and is
    // published separately for that reason. Target selection scores isolation and weakness heavily,
    // so the enemy worth aiming at is regularly not the one physically on top of us - and in the
    // human corpus that is normal, not a fault: humans shoot past the nearest enemy about as often
    // as the bots do (median shot at the second-nearest, 2-3 tiles further out).
    //
    // What humans do *not* do is let that choice drive their feet. They hold and break range against
    // whoever is closest while shooting at whoever is worth shooting. Movement anchored on the aim
    // target instead is how a bot ends up drifting toward a distant isolated enemy with three others
    // in its lap, which shows up as a radial velocity of -0.8 tiles/sec while outnumbered where
    // humans measure +3.5.
    if (nearest_enemy) {
      ctx.blackboard.Set<Player*>("nearest_enemy", nearest_enemy);
      ctx.blackboard.Set<Vector2f>("nearest_enemy_position", nearest_enemy->position);
      ctx.blackboard.Set<float>("nearest_enemy_distance", sqrtf(nearest_enemy_dist_sq));
    } else {
      ctx.blackboard.Erase("nearest_enemy");
      ctx.blackboard.Erase("nearest_enemy_position");
      ctx.blackboard.Erase("nearest_enemy_distance");
    }

    // Our own isolation, measured exactly the way the victim-prediction data measures it. Real
    // players who died were a median 41 tiles from support two seconds beforehand against a 27 tile
    // baseline, so this doubles as a self-preservation signal, not just a regroup trigger.
    if (nearest_teammate) {
      float support_distance = sqrtf(nearest_teammate_dist_sq);

      ctx.blackboard.Set<Player*>("nearest_teammate", nearest_teammate);
      ctx.blackboard.Set<Vector2f>("nearest_teammate_position", nearest_teammate->position);
      ctx.blackboard.Set<float>("support_distance", support_distance);
    } else {
      // Last one standing. Erase rather than leave a stale teammate pointer behind - a dead or
      // departed Player* would otherwise be read by the spacing and regroup nodes for the rest of
      // the round.
      ctx.blackboard.Erase("nearest_teammate");
      ctx.blackboard.Erase("nearest_teammate_position");
      ctx.blackboard.Set<float>("support_distance", kNoSupportDistance);
    }

    return behavior::ExecuteResult::Success;
  }

  float radius = 25.0f;
};

}  // namespace teamversus
}  // namespace zero

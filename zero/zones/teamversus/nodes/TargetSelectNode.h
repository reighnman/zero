#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/TeamVersus.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

#include <limits>

namespace zero {
namespace teamversus {

// Picks who to fight, by scoring every reachable enemy once instead of chaining "nearest target,
// then override if someone is low, then override if the team is focusing someone" sequences.
//
// The chained-override shape is easy to get wrong in two specific ways that this avoids by
// construction:
//
//  - Every sub-sequence in an override chain except the last has to be Success-decorated or a later
//    override silently never runs, and "last" stops being last the moment someone appends a rule.
//  - Re-deriving aim and acceleration per override means the same node type runs several times per
//    tick against different targets, so any node holding per-tick memory under a fixed blackboard
//    key reads back a sample belonging to a different player.
//
// One node, one pass, one winner, one downstream aim computation.
//
// The score is a weighted sum of the things that actually predict a kill in real matches, in
// descending order of how strongly they predicted it:
//
//  - Isolation. Of all the criteria tested against 168 real deaths, distance-from-own-support was
//    the standout, and it held up when measured two seconds *before* the kill rather than at the
//    moment of it (so it isn't just circular). Median victim: 41 tiles from support, versus a 27
//    tile baseline for living players. Attacking the enemy who is cut off is the single highest
//    value targeting rule available. There is real headroom here too: the nearest enemy has median
//    isolation 31 tiles, while the *most* isolated enemy has median 51 - a plain "attack nearest"
//    rule leaves that on the table every tick.
//
//  - Weakness. Victims died with a median 10% energy, and the low-energy rank of the victim among
//    their team was the most extreme of any criterion. Since SeeEnergy is off in this arena, this
//    is necessarily the heuristic tracker's estimate rather than a fact.
//
//  - Elimination pressure. Killing someone on their last life removes them from the match for good,
//    which permanently shifts every later fight's head-count - and head-count is what the exchange
//    data says decides fights.
//
//  - Focus fire. Around 70% of a team already shares the same nearest enemy at any moment, without
//    any coordination mechanism. Nudging toward whoever our teammates are already engaging
//    reproduces that emergent behavior and concentrates damage instead of spreading it.
//
//  - Proximity, weighted lightly. Distance matters for whether we can hit them at all, but the
//    exchange data says it is not what decides fights, so it must not dominate the other terms.
//
// Sticky targeting is applied last: the current target gets a bonus so we don't oscillate between
// two near-equal candidates every tick, which would leave the aim solver permanently resetting its
// per-target history and the movement controller permanently re-choosing an orbit direction.
struct TargetSelectNode : public behavior::BehaviorNode {
  TargetSelectNode(const char* output_key) : output_key(output_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& pm = game.player_manager;
    auto& energy_tracker = ctx.bot->bot_controller->energy_tracker;
    auto* region_registry = ctx.bot->bot_controller->region_registry.get();

    RosterTracker* roster = nullptr;
    auto opt_teamversus = ctx.blackboard.Value<TeamVersus*>("teamversus");
    if (opt_teamversus && *opt_teamversus) roster = &(*opt_teamversus)->roster;

    PlayerId previous_id = ctx.blackboard.ValueOr<PlayerId>("target_id", kInvalidPlayerId);

    // Which enemy each of our teammates is closest to, so we can reward converging on one of them.
    // Nearest-enemy is used as the proxy for "who they are engaging" because it is what the
    // spontaneous-focus-fire measurement itself used, and because a teammate's actual intent is not
    // observable over the wire.
    Player* best = nullptr;
    float best_score = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* enemy = pm.players + i;

      if (!IsLiveEnemy(game, *self, *enemy)) continue;
      if (region_registry && !region_registry->IsConnected(self->position, enemy->position)) continue;

      float distance = enemy->position.Distance(self->position);
      if (distance > max_range) continue;

      float score = 0.0f;

      // --- isolation ---------------------------------------------------------------------------
      float isolation = GetSupportDistance(game, *enemy);
      if (isolation > isolation_cap) isolation = isolation_cap;
      score += (isolation / isolation_cap) * weight_isolation;

      // --- weakness ----------------------------------------------------------------------------
      float energy_percent = energy_tracker.GetEnergyPercent(*enemy);
      score += (1.0f - energy_percent) * weight_weakness;

      // --- elimination pressure ----------------------------------------------------------------
      if (roster) {
        score += roster->EliminationPressure(enemy->name) * weight_elimination;
      }

      // --- focus fire --------------------------------------------------------------------------
      if (IsTeamFocus(game, *self, *enemy)) {
        score += weight_focus;
      }

      // --- proximity ---------------------------------------------------------------------------
      score += (1.0f - distance / max_range) * weight_proximity;

      // --- stickiness --------------------------------------------------------------------------
      if (enemy->id == previous_id) {
        score += weight_sticky;
      }

      if (score > best_score) {
        best_score = score;
        best = enemy;
      }
    }

    if (!best) {
      // No reachable enemy. Clear the derived keys as well - a stale target_position would
      // otherwise keep the movement and firing branches aiming at a dead player's last location.
      ctx.blackboard.Erase(output_key);
      ctx.blackboard.Erase("target_id");
      ctx.blackboard.Erase("target_position");
      ctx.blackboard.Erase("target_energy");
      ctx.blackboard.Erase("target_energy_percent");
      ctx.blackboard.Erase("target_distance");
      ctx.blackboard.Erase("target_isolation");

      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set<Player*>(output_key, best);
    ctx.blackboard.Set<PlayerId>("target_id", best->id);
    ctx.blackboard.Set<Vector2f>("target_position", best->position);
    ctx.blackboard.Set<float>("target_energy", energy_tracker.GetEnergy(*best));
    ctx.blackboard.Set<float>("target_energy_percent", energy_tracker.GetEnergyPercent(*best));
    ctx.blackboard.Set<float>("target_distance", best->position.Distance(self->position));
    ctx.blackboard.Set<float>("target_isolation", GetSupportDistance(game, *best));

    return behavior::ExecuteResult::Success;
  }

  const char* output_key = nullptr;

  // Beyond this there is no meaningful engagement to be had - the corpus's own bullet hit rate is
  // already down at the false-attribution noise floor well before here.
  float max_range = 70.0f;
  // Isolation past this is not more meaningful than isolation at this, so saturate rather than
  // letting one enemy who wandered across the map dominate the score forever.
  float isolation_cap = 60.0f;

  float weight_isolation = 3.0f;
  float weight_weakness = 3.5f;
  float weight_elimination = 2.0f;
  float weight_focus = 1.5f;
  float weight_proximity = 2.0f;
  float weight_sticky = 1.0f;

 private:
  // True if any living teammate's own nearest enemy is this player.
  static bool IsTeamFocus(Game& game, const Player& self, const Player& candidate) {
    auto& pm = game.player_manager;

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* mate = pm.players + i;

      if (!IsLiveTeammate(game, self, *mate)) continue;

      Player* mate_nearest = nullptr;
      float closest_sq = std::numeric_limits<float>::max();

      for (size_t j = 0; j < pm.player_count; ++j) {
        Player* enemy = pm.players + j;

        if (!IsLiveEnemy(game, self, *enemy)) continue;

        float dist_sq = enemy->position.DistanceSq(mate->position);
        if (dist_sq < closest_sq) {
          closest_sq = dist_sq;
          mate_nearest = enemy;
        }
      }

      if (mate_nearest && mate_nearest->id == candidate.id) return true;
    }

    return false;
  }
};

}  // namespace teamversus
}  // namespace zero

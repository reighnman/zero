#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/nexus/LivesTracker.h>

namespace zero {
namespace nexus {

// Picks the enemy the team should collapse on, by scoring every valid enemy rather than blindly
// taking the nearest (NearestMemoryTargetNode) or the globally weakest (LowestTargetNode).
//
// In a team game the decisive advantage is local numbers, so the target worth attacking is the one
// who is cut off from their own support and already under pressure from ours. Replay analysis of
// 21 real 4v4 league matches (ZeroReplayAnalyzer, ~790k player-snapshots, 274 kills) measured which
// properties actually predicted who died. Ranking each victim among their own live team two
// seconds *before* the kill (0.0 = most extreme on that criterion, 0.5 = pure chance) - the lead
// time matters, because measured at the instant of death the victim trivially has ~0 energy and is
// trivially adjacent to whoever just shot them:
//
//   already closest to the eventual killer : 0.1
//   most enemies already near them         : 0.2
//   lowest energy                          : 0.3
//   most isolated from their teammates     : 0.4
//
// And in absolute terms, two seconds before dying the median victim had *zero* teammates within 25
// tiles (mean 0.3) while carrying 1-2 attackers, and sat 37 tiles from their nearest teammate
// versus a 26-tile baseline for living players. So isolation is real but coarse as a ranking among
// only four players; the sharper signals are "we already have people on them" and "they have
// nobody near them".
//
// The weights below follow that ordering. Also note real teams already converge without being told
// to: 70% of a team shared the same nearest enemy at any given moment, so `focus` here is
// reinforcing an observed habit rather than inventing coordination.
struct IsolatedTargetNode : public behavior::BehaviorNode {
  IsolatedTargetNode(const char* player_key, float swarm_radius = 25.0f)
      : player_key(player_key), swarm_radius(swarm_radius) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& region_registry = *ctx.bot->bot_controller->region_registry;
    auto& energy_tracker = ctx.bot->bot_controller->energy_tracker;

    Player* best = nullptr;
    float best_score = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* candidate = game.player_manager.players + i;

      if (!IsValidTarget(game, *self, *candidate, region_registry)) continue;

      float score = ScoreTarget(game, *self, *candidate, energy_tracker, ctx);

      if (score > best_score) {
        best_score = score;
        best = candidate;
      }
    }

    if (!best) return behavior::ExecuteResult::Failure;

    ctx.blackboard.Set(player_key, best);

    return behavior::ExecuteResult::Success;
  }

  const char* player_key = nullptr;
  float swarm_radius = 25.0f;

 private:
  // Same validity filters LowestTargetNode uses, so target selection can't latch onto someone
  // unreachable, respawning, desynced, or sitting in safe.
  bool IsValidTarget(Game& game, Player& self, Player& candidate, RegionRegistry& region_registry) {
    if (candidate.ship >= 8) return false;
    if (candidate.frequency == self.frequency) return false;
    if (candidate.IsRespawning()) return false;
    if (candidate.position == Vector2f(0, 0)) return false;
    if (!IsSynchronized(game, candidate)) return false;
    if (!region_registry.IsConnected(self.position, candidate.position)) return false;
    if (game.connection.map.GetTileId(candidate.position) == kTileIdSafe) return false;

    return true;
  }

  float ScoreTarget(Game& game, Player& self, Player& candidate, HeuristicEnergyTracker& energy_tracker,
                    behavior::ExecuteContext& ctx) {
    float swarm_radius_sq = swarm_radius * swarm_radius;

    // How far this candidate is from their own nearest living teammate. Larger means they're cut
    // off and can't be traded for if we commit.
    float support_distance = kNoSupportDistance;
    // How many of *our* team are already close enough to this candidate to contribute. This is the
    // focus-fire term - it pulls the whole team onto one target instead of splitting up.
    int focus = 0;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* other = game.player_manager.players + i;

      if (other->id == candidate.id) continue;
      if (other->ship >= 8) continue;
      if (other->IsRespawning()) continue;
      if (other->position == Vector2f(0, 0)) continue;

      float distance = other->position.Distance(candidate.position);

      if (other->frequency == candidate.frequency) {
        if (distance < support_distance) support_distance = distance;
      } else if (distance * distance <= swarm_radius_sq) {
        ++focus;
      }
    }

    float distance_to_self = self.position.Distance(candidate.position);

    float max_energy = (float)game.connection.settings.ShipSettings[candidate.ship].MaximumEnergy;
    float energy_percent = max_energy > 0.0f ? energy_tracker.GetEnergy(candidate) / max_energy : 1.0f;

    // Normalize each term into roughly [0,1] so the weights below are directly comparable.
    float isolation_term = std::min(support_distance, kNoSupportDistance) / kNoSupportDistance;
    float focus_term = std::min((float)focus, kFocusSaturation) / kFocusSaturation;
    float weakness_term = 1.0f - std::min(std::max(energy_percent, 0.0f), 1.0f);
    float proximity_term = 1.0f - std::min(distance_to_self, kProximityFalloff) / kProximityFalloff;

    // How close this opponent is to being knocked out of the match for good. Killing someone on
    // their last life permanently reduces the enemy team's numbers, which is worth more than the
    // same kill on a fresh opponent who simply respawns.
    float elimination_term = 0.0f;
    auto opt_lives = ctx.blackboard.Value<LivesTracker*>("lives_tracker");
    if (opt_lives && *opt_lives) {
      elimination_term = (*opt_lives)->EliminationPressure(candidate.name);
    }

    return kFocusWeight * focus_term + kProximityWeight * proximity_term + kWeaknessWeight * weakness_term +
           kIsolationWeight * isolation_term + kEliminationWeight * elimination_term;
  }

  inline bool IsSynchronized(Game& game, Player& player) {
    if (game.radar.InRadarView(player.position)) {
      return game.player_manager.IsSynchronized(player);
    }

    return true;
  }

  // Weight ordering mirrors the measured predictive strength described above (strongest first),
  // plus the elimination term, which is strategic rather than statistical - the replays can show
  // who gets killed, but not that a knockout is worth more than a respawn.
  static constexpr float kFocusWeight = 0.30f;
  static constexpr float kProximityWeight = 0.25f;
  static constexpr float kWeaknessWeight = 0.18f;
  static constexpr float kIsolationWeight = 0.12f;
  static constexpr float kEliminationWeight = 0.15f;

  // Support distance at/above this is treated as fully isolated. The median living player sits 26
  // tiles from their nearest teammate and future victims sit ~37, so saturating here keeps the
  // term discriminating across the range that actually matters.
  static constexpr float kNoSupportDistance = 45.0f;
  // Two attackers already on a target is enough to call it a focused target.
  static constexpr float kFocusSaturation = 2.0f;
  // Beyond this, closer-vs-farther stops mattering much for target choice.
  static constexpr float kProximityFalloff = 60.0f;
};

}  // namespace nexus
}  // namespace zero

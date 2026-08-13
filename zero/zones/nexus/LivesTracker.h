#pragma once

#include <zero/game/GameEvent.h>

#include <string>
#include <unordered_map>

namespace zero {
namespace nexus {

// Tracks how many lives each player has left in an elimination match.
//
// Nexus team matches (twos/threes/fours) give every player a fixed number of lives - 3 by default,
// but it's a per-match setting, so `starting_lives` is configurable rather than hardcoded. A team
// wins by knocking out every opposing player, which makes an opponent's remaining lives a real
// strategic quantity: an enemy on their last life is worth far more to kill than a fresh one,
// because removing them permanently reduces the other team's numbers for the rest of the match -
// every permanent removal shifts every subsequent fight toward our side.
//
// Keyed by player name rather than player id, because ids are only stable for the current session
// and get recycled as players leave and rejoin between rounds, whereas the name is what identifies
// a person across the whole match.
struct LivesTracker {
  void OnDeath(const char* player_name) {
    if (!player_name || !*player_name) return;
    ++deaths[player_name];
  }

  u32 DeathsOf(const char* player_name) const {
    if (!player_name || !*player_name) return 0;

    auto iter = deaths.find(player_name);
    if (iter == deaths.end()) return 0;

    return iter->second;
  }

  // Lives left before this player is knocked out for good. Saturates at zero rather than wrapping,
  // since a player can keep dying in a non-elimination arena where lives don't apply.
  u32 LivesRemaining(const char* player_name) const {
    u32 died = DeathsOf(player_name);
    return died >= starting_lives ? 0 : starting_lives - died;
  }

  // 0.0 = untouched, 1.0 = one hit away from elimination. Used to weight target selection toward
  // opponents who are closest to being removed from the match entirely.
  float EliminationPressure(const char* player_name) const {
    if (starting_lives == 0) return 0.0f;

    u32 died = DeathsOf(player_name);
    if (died >= starting_lives) return 1.0f;

    return (float)died / (float)starting_lives;
  }

  void Reset() { deaths.clear(); }

  u32 starting_lives = 3;
  std::unordered_map<std::string, u32> deaths;
};

}  // namespace nexus
}  // namespace zero

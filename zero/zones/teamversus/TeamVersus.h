#pragma once

#include <zero/Types.h>
#include <zero/game/Clock.h>

#include <string>
#include <unordered_map>

namespace zero {
namespace teamversus {

// Shared, non-behavior state for the team-versus zone. One instance lives on the zone controller
// and is published to the blackboard under "teamversus" so nodes can read it without the tree
// having to thread a dozen separate keys through.
//
// This deliberately holds only facts the behavior tree cannot derive for itself from the game
// state on a given tick - match lifecycle (which arrives as chat, not as game state) and per-player
// death counts (which are events, not state). Anything derivable from the current tick belongs in a
// node, not here.

// Tracks how many lives each player has left in a knockout match.
//
// Team-versus matches give every player a fixed number of lives (3 by default, configurable via
// TeamVersus:MatchLives). A team loses when every one of its players is knocked out, which makes an
// opponent's remaining lives a genuine strategic quantity rather than a scoreboard detail: killing
// someone on their last life permanently shrinks the enemy team, and every fight after that is
// fought at a numbers advantage. The exchange-ratio data from real matches is dominated by local
// head-count advantage, so a permanent removal is worth far more than an even trade.
//
// Keyed by player name rather than PlayerId: ids are only stable for a session and get recycled as
// players leave and rejoin between rounds, whereas the name identifies the person across the match.
struct RosterTracker {
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
  // since players can keep dying in an arena where lives don't actually apply.
  u32 LivesRemaining(const char* player_name) const {
    u32 died = DeathsOf(player_name);
    return died >= starting_lives ? 0 : starting_lives - died;
  }

  // 0.0 = untouched, 1.0 = one death away from elimination. Used to weight target selection toward
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

// Match lifecycle, driven by arena chat rather than game state.
//
// The zone announces the real start of a match with a "GO!" arena message once its ready check
// finishes. Nothing in the game state marks that moment - a bot is simply moved out of spec some
// unknown amount of time beforehand and is expected to sit still until told to go. So the tree
// gates all offensive behavior on `live` and the controller is the only thing that sets it.
//
// `start_deadline` is a pure safety net: if the "GO!" message is ever missed (arena reconfigured,
// message text changed, packet lost), the match goes live on its own after a generous delay rather
// than leaving the bot inert for the whole round.
struct MatchState {
  bool live = false;
  Tick start_tick = 0;
  Tick start_deadline = 0;

  void EnterSpectate() {
    live = false;
    start_deadline = 0;
  }

  // Called when we're pulled out of spec, before we know when the match actually begins.
  void ArmSafetyNet(Tick now, u32 timeout_ticks) {
    live = false;
    // MAKE_TICK keeps this inside the 31-bit tick space the TICK_* comparison macros assume.
    start_deadline = MAKE_TICK(now + timeout_ticks);
    // A deadline of exactly zero is the "disarmed" sentinel, so nudge off it in the rare wrap case.
    if (start_deadline == 0) start_deadline = 1;
  }

  void Begin(Tick now) {
    live = true;
    start_tick = now;
    start_deadline = 0;
  }

  // Returns true once the match should be treated as running. Checked every tick by the tree.
  bool Update(Tick now) {
    if (!live && start_deadline != 0 && TICK_GTE(now, start_deadline)) {
      Begin(now);
    }

    return live;
  }
};

struct TeamVersus {
  MatchState match;
  RosterTracker roster;

  // Self-queue command, resent periodically while sitting in spectator mode so a bot that was
  // dropped from the queue (arena recycle, match ending while we were mid-request, a queue that
  // forgets us) puts itself back in line instead of idling in spec for the rest of the session.
  //
  // Only ever sent while spectating - once we are in a ship we are already in a match, and
  // requeueing from there would ask to be pulled out of the one we are playing. Setting
  // TeamVersus:QueueCommand to an empty value disables it entirely, for arenas where the zone
  // pulls bots in on its own.
  std::string queue_command = "?next";
  // Ticks are centiseconds, so this is one minute.
  u32 queue_interval_ticks = 6000;
};

}  // namespace teamversus
}  // namespace zero

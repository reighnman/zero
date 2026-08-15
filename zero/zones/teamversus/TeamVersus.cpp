#include <string.h>
#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/game/GameEvent.h>
#include <zero/game/Logger.h>
#include <zero/zones/ZoneController.h>
#include <zero/zones/teamversus/TeamVersus.h>
#include <zero/zones/teamversus/TeamVersusBehavior.h>

namespace zero {
namespace teamversus {

// Zone controller for team knockout matches.
//
// Its whole job is the two things a behavior tree genuinely cannot do for itself: react to chat,
// and react to events. Match start arrives as an arena message and deaths arrive as events; neither
// is visible as game state on any given tick, so both are captured here and published through the
// shared TeamVersus struct rather than being re-derived somewhere in the tree.
struct TeamVersusController : ZoneController, EventHandler<ChatEvent>, EventHandler<PlayerDeathEvent> {
  bool IsZone(Zone zone) override {
    // Clear anything the previous zone left behind. The blackboard survives a zone change, and a
    // dangling TeamVersus* would be read by nodes that have no idea the object is gone.
    bot->execute_ctx.blackboard.Erase("teamversus");
    teamversus = nullptr;

    return zone == Zone::TeamVersus;
  }

  void CreateBehaviors(const char* arena_name) override;

  void HandleEvent(const ChatEvent& event) override;
  void HandleEvent(const PlayerDeathEvent& event) override;

  std::unique_ptr<TeamVersus> teamversus;
};

static TeamVersusController controller;

void TeamVersusController::HandleEvent(const ChatEvent& event) {
  if (!in_zone || !teamversus) return;

  // Only arena messages carry the match lifecycle. Public/private/channel traffic is never looked
  // at, so there is no reason to build strings out of it.
  if (event.type != ChatType::Arena) return;

  std::string message = event.message;

  // The zone announces the real start of the match once its ready check finishes. Until this
  // arrives, the bot sits in whatever ship it was given without engaging - a bot that opens fire
  // during a ready check is both wrong and immediately identifiable.
  if (message.find("GO!") != std::string::npos) {
    Log(LogLevel::Info, "TeamVersus: match start received.");

    teamversus->match.Begin(GetCurrentTick());

    // A new match means everybody is back to a full set of lives. Without this the roster would
    // carry the previous match's deaths forward and immediately treat fresh opponents as being one
    // hit from elimination, which would distort target selection for the entire round.
    teamversus->roster.Reset();
  }
}

void TeamVersusController::HandleEvent(const PlayerDeathEvent& event) {
  if (!in_zone || !teamversus) return;

  teamversus->roster.OnDeath(event.player.name);

  Log(LogLevel::Info, "TeamVersus: %s died (%u deaths, %u lives left).", event.player.name,
      teamversus->roster.DeathsOf(event.player.name), teamversus->roster.LivesRemaining(event.player.name));
}

void TeamVersusController::CreateBehaviors(const char* arena_name) {
  Log(LogLevel::Info, "Registering TeamVersus behaviors.");

  // Enemy energy is not broadcast in a versus arena, so every target-energy decision runs on the
  // heuristic tracker's estimate. Average is the right bias for that: Maximum makes every idle
  // enemy look permanently healthy and suppresses target selection's weakness term entirely, while
  // Initial makes them look weaker than they are and invites dives that don't convert.
  bot->bot_controller->energy_tracker.estimate_type = EnergyHeuristicType::Average;

  teamversus = std::make_unique<TeamVersus>();

  // Lives per player. Three is the usual knockout format, but it is a per-match setting rather than
  // a rule of the game, so it stays configurable.
  auto opt_lives = bot->config->GetInt("TeamVersus", "MatchLives");
  teamversus->roster.starting_lives = opt_lives ? (u32)*opt_lives : 3;
  teamversus->roster.Reset();

  // Self-queue command, resent once a minute for as long as we are sitting in spectator mode.
  // Defaults to "?next"; set TeamVersus:QueueCommand to an empty value to disable queueing in
  // arenas where the zone pulls bots into matches on its own.
  auto opt_queue = bot->config->GetString("TeamVersus", "QueueCommand");
  if (opt_queue) teamversus->queue_command = *opt_queue;

  auto opt_queue_interval = bot->config->GetInt("TeamVersus", "QueueInterval");
  if (opt_queue_interval && *opt_queue_interval > 0) {
    teamversus->queue_interval_ticks = (u32)*opt_queue_interval;
  }

  bot->execute_ctx.blackboard.Set("teamversus", teamversus.get());

  Log(LogLevel::Info, "TeamVersus: %u lives per player, queue command '%s' every %u ticks.",
      teamversus->roster.starting_lives,
      teamversus->queue_command.empty() ? "(disabled)" : teamversus->queue_command.data(),
      teamversus->queue_interval_ticks);

  auto& repo = bot->bot_controller->behaviors;

  repo.Add("teamversus", std::make_unique<TeamVersusBehavior>());

  SetBehavior("teamversus");
}

}  // namespace teamversus
}  // namespace zero

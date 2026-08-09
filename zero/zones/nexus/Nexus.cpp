#include <string.h>
#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/game/GameEvent.h>
#include <zero/game/Logger.h>
#include <zero/zones/ZoneController.h>
#include <zero/zones/nexus/PubOffenseBehavior.h>
#include <zero/zones/nexus/PubCoverBehavior.h>
#include <zero/zones/nexus/TestBehavior.h>
#include <zero/zones/nexus/DuelBehavior.h>
#include <zero/zones/nexus/TwosBehavior.h>
#include <zero/zones/nexus/ThreesBehavior.h>
#include <zero/zones/nexus/FoursBehavior.h>
#include <zero/zones/nexus/TwosBoxBehavior.h>

#include <zero/zones/nexus/Nexus.h>

namespace zero {
namespace nexus {

struct NexusController : ZoneController, EventHandler<ChatEvent> {
   bool IsZone(Zone zone) override {
    bot->execute_ctx.blackboard.Erase("nexus");
    bot->execute_ctx.blackboard.Erase("flag_position");
    nexus = nullptr;
    return zone == Zone::Nexus;
  }

  void CreateBehaviors(const char* arena_name) override;

  void HandleEvent(const ChatEvent& event) override;

  //void CreateFlagroomBitset();

  std::unique_ptr<Nexus> nexus;
};

static NexusController controller;

void NexusController::HandleEvent(const ChatEvent& event) {
  std::string sender = event.sender;
  std::string message = event.message;


  auto& chat_queue = bot->bot_controller->chat_queue;


  if (event.type == ChatType::Team && message.find("SAFE") != std::string::npos &&
      !(message.find("NOT") != std::string::npos)) {

      // SAFE
    Log(LogLevel::Info, "Setting tchat_safe");
    bot->execute_ctx.blackboard.Set("tchat_safe", sender);
    bot->execute_ctx.blackboard.Set<u32>("tchat_safe_timer", GetCurrentTick() + 600);

    // Event::Dispatch(ChatQueueEvent::Public("On my way!"));
  } else if (event.type == ChatType::Team && message.find("SAFE") != std::string::npos &&

      // NOT SAFE
    message.find("NOT") != std::string::npos) {
    Log(LogLevel::Info, "Setting tchat_notsafe");
    bot->execute_ctx.blackboard.Set("tchat_notsafe", sender);
    bot->execute_ctx.blackboard.Set<u32>("tchat_notsafe_timer", GetCurrentTick() + 600);

    // Event::Dispatch(ChatQueueEvent::Public("On my way!"));
  }

  // The match system sends "GO!" over private chat once the ready check finishes and the match
  // actually begins. All of the versus behaviors (Fours, Duel, Twos, TwosBox, Threes) set a
  // "match_startup" timer with a blind guess at how long ready-up will take when they leave spec,
  // then gate ship entry / pre-fire / engaging on it expiring. Forcing that same key to expire the
  // instant we see "GO!" makes them react to the real match start instead of the guess, without
  // needing any changes in the individual behaviors - the blind timer they set on leaving spec
  // becomes just a safety net in case this message is ever missed.
  if ((event.type == ChatType::Private || event.type == ChatType::RemotePrivate) &&
      message.find("GO!") != std::string::npos) {
    Log(LogLevel::Info, "Match start message received, expiring match_startup.");
    bot->execute_ctx.blackboard.Set<u32>("match_startup", GetCurrentTick());
  }
}

void NexusController::CreateBehaviors(const char* arena_name) {
  Log(LogLevel::Info, "Registering Nexus behaviors.");

  bot->bot_controller->energy_tracker.estimate_type = EnergyHeuristicType::Average;

  nexus = std::make_unique<Nexus>();
  bot->execute_ctx.blackboard.Set("nexus", nexus.get());

  auto& repo = bot->bot_controller->behaviors;

  repo.Add("puboffense", std::make_unique<PubOffenseBehavior>());
  repo.Add("pubcover", std::make_unique<PubCoverBehavior>());
  repo.Add("duel", std::make_unique<DuelBehavior>());
  repo.Add("twos", std::make_unique<TwosBehavior>());
  repo.Add("threes", std::make_unique<ThreesBehavior>());
  repo.Add("fours", std::make_unique<FoursBehavior>());
  repo.Add("twosbox", std::make_unique<TwosBoxBehavior>());
  repo.Add("test", std::make_unique<TestBehavior>());
  

  SetBehavior("puboffense");
}

}  // namespace nexus
}  // namespace zero

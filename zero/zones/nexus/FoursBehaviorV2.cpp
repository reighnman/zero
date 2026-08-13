#include "FoursBehaviorV2.h"

#include <zero/zones/nexus/NexusTeamBehaviorV2.h>

namespace zero {
namespace nexus {

// The 4v4 variant of the shared V2 team tree - see NexusTeamBehaviorV2.cpp for the behavior itself
// and the replay analysis behind its tuning. Only the matchmaking queue differs between roster
// sizes.
std::unique_ptr<behavior::BehaviorNode> FoursBehaviorV2::CreateTree(behavior::ExecuteContext& ctx) {
  return CreateNexusTeamTreeV2(ctx, NexusTeamConfigV2::Fours());
}

}  // namespace nexus
}  // namespace zero

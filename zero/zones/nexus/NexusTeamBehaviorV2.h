#pragma once

#include <zero/behavior/BehaviorTree.h>
#include <zero/zones/nexus/NexusTeamConfigV2.h>

#include <memory>

namespace zero {
namespace nexus {

// Builds the shared V2 team-match behavior tree for a given mode.
//
// Twos/Threes/Fours share the same rules, weapons and match flow, so they share one tree - but they
// are played on differently sized arenas with different roster sizes, so every distance and the
// roster-dependent branches come from the mode's NexusTeamConfigV2 rather than being fixed here.
// See that header for what varies, what deliberately does not, and where each number came from.
//
// Keeping the tree in a single translation unit is deliberate. The previous generation of these
// behaviors copy-pasted whole node structs into each behavior .cpp, and when one copy was updated
// and the others weren't, the result was multiple incompatible definitions of the same type across
// translation units - an ODR violation that manifested as a segfault far from its cause. One
// definition, parameterized, avoids reintroducing that class of bug.
//
// `config` only needs to survive the call, not the returned tree: node constructors copy the values
// they need out of it during construction.
std::unique_ptr<behavior::BehaviorNode> CreateNexusTeamTreeV2(behavior::ExecuteContext& ctx,
                                                              const NexusTeamConfigV2& config);

}  // namespace nexus
}  // namespace zero

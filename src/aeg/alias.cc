#include "aeg.h"
#include "cfg/expanded.h"

namespace aeg {

llvm::AliasResult AEG::check_alias(NodeRef ref1, NodeRef ref2) const {
    const Node& node1 = lookup(ref1);
    const Node& node2 = lookup(ref2);
    
    assert(node1.may_access());
    assert(node2.may_access());
    
    if (node1.inst->is_special() || node2.inst->is_special()) {
        return llvm::AliasResult::MustAlias;
    }
    
    const ValueLoc vl1 = get_value_loc(ref1);
    const ValueLoc vl2 = get_value_loc(ref2);
    
    const auto it = alias_rel.find(std::make_pair(vl1, vl2));
    if (it == alias_rel.end()) {
        return llvm::AliasResult::MayAlias;
    } else {
        return it->second;
    }
}

void AEG::add_alias_result(const ValueLoc& vl1, const ValueLoc& vl2, llvm::AliasResult res) {
    if (res != llvm::AliasResult::MayAlias) {
        alias_rel.emplace(std::make_pair(vl1, vl2), res);
        alias_rel.emplace(std::make_pair(vl2, vl1), res);
    }
}

}

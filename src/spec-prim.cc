#include "spec-prim.h"
#include "cfg/cfg.h"

SpeculationInfo::SpeculationInfo(const AEGPO& po): po(po) {}

void SpeculationInfo::add(const SpeculationPrimitive& primitive) {
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (NodeRef src : order) {
        if (const auto *src_I = std::get_if<const llvm::Instruction *>(&po.lookup(src).v)) {
            for (NodeRef dst : po.po.fwd.at(src)) {
                if (const auto *dst_I = std::get_if<const llvm::Instruction *>(&po.lookup(dst).v)) {
                    if (const auto res = primitive.can_tfo(*src_I, *dst_I)) {
                        primitive_tfos.emplace(std::make_tuple(src, dst),
                                               PrimitiveTFOInfo {*res});
                    }
                }
            }
        }
    }
}

SpeculationPrimitive::Result BranchPrimitive::can_tfo(const llvm::Instruction *src, const llvm::Instruction *dst) const {
    if (const auto *B = llvm::dyn_cast<llvm::BranchInst>(src)) {
        if (B->getNumSuccessors() > 1) {
            llvm::errs() << "Branch: " << *B << "\n";
            return {{{.always_speculative = false}}};
        }
    }
    return std::nullopt;
}

SpeculationPrimitive::Result AddrSpecPrimitive::can_tfo(const llvm::Instruction *src, const llvm::Instruction *dst) const {
    if (llvm::isa<llvm::LoadInst>(dst)) {
        return {{{.always_speculative = false}, {.always_speculative = true}}};
    } else {
        return std::nullopt;
    }
}

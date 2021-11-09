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

bool AEG::compatible_types_pointee(const llvm::Type *T1, const llvm::Type *T2) {
    const std::array<const llvm::Type *, 2> Ts = {T1, T2};

    const auto report = [] () -> llvm::raw_ostream& {
        return logv(1);
    };
    
    // types are compatible if they are equal
    if (T1 == T2) {
        report() << "query: equal-yes\n";
        return true;
    }
    
    if (T1->isVoidTy() || T2->isVoidTy()) {
        report() << "query: void-yes\n";
        return true;
    }
    
    for (const llvm::Type *T : Ts) {
        if (const llvm::IntegerType *IT = llvm::dyn_cast<llvm::IntegerType>(T)) {
            if (IT->getBitWidth() == 8) {
                report() << "query: char-yes\n";
                return true;
            }
        }
    }
    
    // check if function types
    if (T1->isFunctionTy() || T2->isFunctionTy()) {
        report() << "query: function-no\n";
        return false;
    }
    
    // if struct, assume compatible
    if (T2->isStructTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::StructType *S1 = llvm::dyn_cast<llvm::StructType>(T1)) {
        report() << "query: struct\n";
        
        if (T2->isStructTy()) {
            report() << "query: struct-unk\n";
            return true;
        }
        // ASSUME: T2 isn't a struct
        
        if (S1->getNumElements() == 0) {
            return false;
        }
        
        const bool res = compatible_types_pointee(S1->getElementType(0), T2);
        report() << "query: struct-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    // if array
    if (T2->isArrayTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T1)) {
        report() << "query: array\n";
        
#if 0
        if (const llvm::ArrayType *AT2 = llvm::dyn_cast<llvm::ArrayType>(T2)) {
            if (AT->getNumElements() != AT2->getNumElements()) {
                return false;
            }
        }
#endif

        const bool res = compatible_types_pointee(AT->getElementType(), T2);
        report() << "query: array-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    // if vector
    if (T1->isVectorTy() || T2->isVectorTy()) {
        report() << "query: vector-unk\n";
        return true;
    }
    
    // if integer
    if (T1->isIntegerTy() && T2->isIntegerTy()) {
        report() << "query: integer\n";
        const llvm::IntegerType *IT1 = llvm::cast<llvm::IntegerType>(T1);
        const llvm::IntegerType *IT2 = llvm::cast<llvm::IntegerType>(T2);
        const bool res = IT1->getBitWidth() == IT2->getBitWidth();
        report() << "query: integer-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    // if pointer
    if (T2->isPointerTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::PointerType *P1 = llvm::dyn_cast<llvm::PointerType>(T1)) {
        report() << "query: pointer\n";
        const bool res = T2->isPointerTy();
        report() << "query: pointer-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    report() << "query: unknown: " << *T1 << ": " << *T2 << "\n";
    return true;
}


bool AEG::compatible_types(const llvm::Type *P1, const llvm::Type *P2) {
    logv(1) << "query: type\n";
    assert(P1->isPointerTy() && P2->isPointerTy());
    return compatible_types_pointee(P1->getPointerElementType(), P2->getPointerElementType());
}

AEG::AddressKind AEG::get_addr_kind(const llvm::Value *V) {
    // check cache
    const auto it = addr_kinds.find(V);
    if (it != addr_kinds.end()) {
        return it->second;
    }
    
    // classify
    AddressKind kind;
    if (llvm::isa<llvm::AllocaInst>(V)) {
        kind = AddressKind::STACK;
    } else if (llvm::isa<llvm::Constant>(V)) {
        kind = AddressKind::CONST;
    } else if (const llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(V)) {
        kind = get_addr_kind(GEP->getPointerOperand());
    } else {
        kind = AddressKind::UNKNOWN;
    }
    
    addr_kinds.emplace(V, kind);
    return kind;
}

}

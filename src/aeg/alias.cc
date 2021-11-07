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
    
    // types are compatible if they are equal
    if (T1 == T2) {
        std::cerr << "query: equal\n";
        return true;
    }
    
    if (T1->isVoidTy() || T2->isVoidTy()) {
        std::cerr << "query: void\n";
        return true;
    }
    
    for (const llvm::Type *T : Ts) {
        if (const llvm::IntegerType *IT = llvm::dyn_cast<llvm::IntegerType>(T)) {
            if (IT->getBitWidth() == 8) {
                std::cerr << "query: char\n";
                return true;
            }
        }
    }
    
    // check if function types
    if (T1->isFunctionTy() || T2->isFunctionTy()) {
        std::cerr << "query: function\n";
        return false;
    }
    
    // if struct, assume compatible
    if (T2->isStructTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::StructType *S1 = llvm::dyn_cast<llvm::StructType>(T1)) {
        std::cerr << "query: struct\n";
        
        if (T2->isStructTy()) {
            std::cerr << "query: struct-struct\n";
            return true;
        }
        // ASSUME: T2 isn't a struct
        
        if (S1->getNumElements() == 0) {
            return false;
        }
        
        const bool res = compatible_types_pointee(S1->getElementType(0), T2);
        std::cerr << "query: struct-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    // if array
    if (T2->isArrayTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T1)) {
        std::cerr << "query: array\n";

        const bool res = compatible_types_pointee(AT->getElementType(), T2);
        std::cerr << "query: array-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    // if vector
    if (T1->isVectorTy() || T2->isVectorTy()) {
        std::cerr << "query: vector\n";
        return true;
    }
    
    // if integer
    if (T1->isIntegerTy() && T2->isIntegerTy()) {
        std::cerr << "query: integer\n";
        const llvm::IntegerType *IT1 = llvm::cast<llvm::IntegerType>(T1);
        const llvm::IntegerType *IT2 = llvm::cast<llvm::IntegerType>(T2);
        return IT1->getBitWidth() == IT2->getBitWidth();
    }
    
    // if pointer
    if (T2->isPointerTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::PointerType *P1 = llvm::dyn_cast<llvm::PointerType>(T1)) {
        std::cerr << "query: pointer\n";
        const bool res = T2->isPointerTy();
        std::cerr << "query: pointer-" << (res ? "yes" : "no") << "\n";
        return res;
    }
    
    llvm::errs() << "query: unknown: " << *T1 << ": " << *T2 << "\n";
    return true;
}


bool AEG::compatible_types(const llvm::Type *P1, const llvm::Type *P2) {
    std::cerr << "query: type\n";
    assert(P1->isPointerTy() && P2->isPointerTy());
    return compatible_types_pointee(P1->getPointerElementType(), P2->getPointerElementType());
}

}

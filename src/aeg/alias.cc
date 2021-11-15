#include "aeg.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"
#include "util/llvm.h"
#include "timer.h"
#include "util/z3.h"
#include "util/llvm.h"
#include "util/output.h"

namespace aeg {

llvm::AliasResult AEG::check_alias(const ValueLoc& a_, const ValueLoc& b_) const {
    const ValueLoc *a = &a_;
    const ValueLoc *b = &b_;
    if (b_ < a_) {
        std::swap(a, b);
    }
    const auto it = alias_rel.find(std::make_pair(*a, *b));
    if (it == alias_rel.end()) {
        return llvm::AliasResult::MayAlias;
    } else {
        return it->second;
    }
}

llvm::AliasResult AEG::check_alias(NodeRef ref1, NodeRef ref2) const {
    const Node& node1 = lookup(ref1);
    const Node& node2 = lookup(ref2);
    
    assert(node1.may_access());
    assert(node2.may_access());
    
    if (node1.inst->is_special() || node2.inst->is_special()) {
        return llvm::AliasResult::MustAlias;
    }
    
    return check_alias(get_value_loc(ref1), get_value_loc(ref2));
}

void AEG::add_alias_result(const ValueLoc& vl1_, const ValueLoc& vl2_, llvm::AliasResult res) {
    if (res != llvm::AliasResult::MayAlias) {
        const ValueLoc *vl1 = &vl1_;
        const ValueLoc *vl2 = &vl2_;
        if (!(*vl1 < *vl2)) {
            std::swap(vl1, vl2);
        }
        alias_rel.emplace(std::make_pair(*vl1, *vl2), res);
    }
}

#define report(args) logv(3, args)
bool AEG::compatible_types_pointee(const llvm::Type *T1, const llvm::Type *T2) {
    const std::array<const llvm::Type *, 2> Ts = {T1, T2};

    // types are compatible if they are equal
    if (T1 == T2) {
        report("query: equal-yes\n");
        return true;
    }
    
    if (T1->isVoidTy() || T2->isVoidTy()) {
        report("query: void-yes\n");
        return true;
    }
    
    for (const llvm::Type *T : Ts) {
        if (const llvm::IntegerType *IT = llvm::dyn_cast<llvm::IntegerType>(T)) {
            if (IT->getBitWidth() == 8) {
                report("query: char-yes\n");
                return true;
            }
        }
    }
    
    // check if function types
    if (T1->isFunctionTy() || T2->isFunctionTy()) {
        report("query: function-no\n");
        return false;
    }
    
    // if struct, assume compatible
    if (T2->isStructTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::StructType *S1 = llvm::dyn_cast<llvm::StructType>(T1)) {
        report("query: struct\n");
        
        if (T2->isStructTy()) {
            report("query: struct-unk\n");
            return true;
        }
        // ASSUME: T2 isn't a struct
        
        if (S1->getNumElements() == 0) {
            return false;
        }
        
        const bool res = compatible_types_pointee(S1->getElementType(0), T2);
        report("query: struct-" << (res ? "yes" : "no") << "\n");
        return res;
    }
    
    // if array
    if (T2->isArrayTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T1)) {
        report("query: array\n");
        const bool res = compatible_types_pointee(AT->getElementType(), T2);
        report("query: array-" << (res ? "yes" : "no") << "\n");
        return res;
    }
    
    // if vector
    if (T1->isVectorTy() || T2->isVectorTy()) {
        report("query: vector-unk\n");
        return true;
    }
    
    // if integer
    if (T1->isIntegerTy() && T2->isIntegerTy()) {
        report("query: integer\n");
        const llvm::IntegerType *IT1 = llvm::cast<llvm::IntegerType>(T1);
        const llvm::IntegerType *IT2 = llvm::cast<llvm::IntegerType>(T2);
        const bool res = IT1->getBitWidth() == IT2->getBitWidth();
        report("query: integer-" << (res ? "yes" : "no") << "\n");
        return res;
    }
    
    // if pointer
    if (T2->isPointerTy()) {
        std::swap(T1, T2);
    }
    if (const llvm::PointerType *P1 = llvm::dyn_cast<llvm::PointerType>(T1)) {
        report("query: pointer\n");
        const bool res = T2->isPointerTy();
        report("query: pointer-" << (res ? "yes" : "no") << "\n");
        return res;
    }
    
    report("query: unknown: " << *T1 << ": " << *T2 << "\n");
    return true;
}


std::optional<llvm::AliasResult> AEG::compute_alias(const AddrInfo& a, const AddrInfo& b, llvm::AliasAnalysis& AA) {
    const AddrInfo *x = &a;
    const AddrInfo *y = &b;
    
    /* check if LLVM's built-in alias analysis is valid */
    if (po.llvm_alias_valid(a.id, b.id)) {
        return AA.alias(a.V, b.V);
    }
    
    if (alias_mode.llvm_only) {
        return std::nullopt;
    }
    
    // EXPERIMENTAL: try inter-procedural alias analysis
    if (!compatible_types(a.V->getType(), b.V->getType())) {
        return llvm::AliasResult::NoAlias;
    }
    
#if 0
    // NOTE: This is already addressed in the AA rule that makes all alloca's distinct.
    /* unless alloca's scope is a prefix of another scope, it can't alias */
    {
        if (!util::prefixeq(y->id.func, x->id.func)) {
            std::swap(x, y);
        }
        if (!util::prefixeq(x->id.func, y->id.func)) {
            if (llvm::isa<llvm::AllocaInst>(x->V)) {
                return llvm::NoAlias;
            }
        }
    }
#endif
    
#if 0
    // NOTE: I also think this is covered elsewhere.
    /* check if address kinds differ */
    {
        const AddressKind k1 = get_addr_kind(a.V);
        const AddressKind k2 = get_addr_kind(b.V);
        if (k1 != AddressKind::UNKNOWN && k2 != AddressKind::UNKNOWN && k1 != k2) {
            return llvm::AliasResult::NoAlias;
        }
    }
#endif
    
#if 0
    // NOTE: This is covered elsewhere.
    {
        if (llvm::isa<llvm::Argument>(x->V)) {
            std::swap(x, y);
        }
        if (llvm::isa<llvm::Argument>(x->V) && llvm::isa<llvm::AllocaInst>(y->V)) {
            return llvm::AliasResult::NoAlias;
        }
    }
#endif
    
    {
        /*
         * If the types of the alloca and the getelementptr base aren't equal, then we know that the getelementptr result can't alias.
         */
        if (llvm::isa<llvm::AllocaInst>(y->V)) {
            std::swap(x, y);
        }
        if (const llvm::AllocaInst *AI = llvm::dyn_cast<llvm::AllocaInst>(x->V)) {
            const llvm::Type *T1 = AI->getType()->getPointerElementType();
            if (const llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(y->V)) {
                if (!llvm::getelementptr_can_zero(GEP)) {
                    return llvm::AliasResult::NoAlias;
                }
                
                const llvm::Type *T2 = GEP->getPointerOperand()->getType()->getPointerElementType();
                if (T1 != T2) {
                    return llvm::AliasResult::NoAlias;
                }
            }
            
            const llvm::Type *T2 = y->V->getType()->getPointerElementType();
            if (!T1->isStructTy() && T2->isStructTy()) {
                return llvm::AliasResult::NoAlias;
            }
        }
    }
    
    {
        logv(3, "alias-fail: " << *a.V << " -- " << *b.V << "\n");
    }
    
    /* check whether pointers point to different address spaces */
    
    return std::nullopt;
}


bool AEG::compatible_types(const llvm::Type *P1, const llvm::Type *P2) {
    report("query: type\n");
    assert(P1->isPointerTy() && P2->isPointerTy());
    return compatible_types_pointee(P1->getPointerElementType(), P2->getPointerElementType());
}
#undef report

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

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
    Timer timer;
    using ID = CFG::ID;
    
    using AddrInfoVec = std::vector<AddrInfo>;
    AddrInfoVec addrs;
    std::unordered_set<ValueLoc> seen;
    
    for (NodeRef i : node_range()) {
        const Node& node = lookup(i);
        if (node.addr_def) {
            const ID& id = *po.lookup(i).id;
            const llvm::Value *V = dynamic_cast<const RegularInst&>(*node.inst).I;
            const AddrInfo addr = {
                .id = id,
                .V = V,
                .e = *node.addr_def,
                .ref = i
            };
            addrs.push_back(addr);
            [[maybe_unused]] const auto res = seen.emplace(addr.vl());
            
#if 0
            assert(res.second);
#endif
            // TODO: ignore collisions for now, since they're introduced during the CFG expansion step.
        }
    }
    
    // check for arguments
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        const CFG::Node& po_node = po.lookup(i);
        if (const auto *inst = dynamic_cast<const RegularInst *>(node.inst.get())) {
            for (const llvm::Value *V : inst->addr_refs) {
                // check if it's missing from the po node's refs
                if (po_node.refs.contains(V)) { continue; }
                
                if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
                    /* NOTE: It's ok if these are actually inlined arguments -- they are coming from constants that weren't bound during CFG::resolve_refs(). This is a bug that needs to be fixed. (Just makes the analysis less precise.) */

                    const ID id {po_node.id->func, {}};
                    if (seen.emplace(ValueLoc(id, V)).second) {
                        const auto it = std::find_if(node.addr_refs.begin(), node.addr_refs.end(),
                                                     [&] (const auto& p) {
                            return p.first == V;
                        });
                        assert(it != node.addr_refs.end());
                        addrs.push_back({.id = id, .V = V, .e = it->second, .ref = std::nullopt});
                    }
                } else if (llvm::isa<llvm::Constant>(V)) {
                    assert(V->getType()->isPointerTy());
                    const ID id {{}, {}};
                    const ValueLoc vl {id, V};
                    if (seen.emplace(vl).second) {
                        const auto it = std::find_if(node.addr_refs.begin(), node.addr_refs.end(), [&] (const auto& p) {
                            return p.first == V;
                        });
                        assert(it != node.addr_refs.end());
                        addrs.push_back({.id = id, .V = V, .e = it->second, .ref = std::nullopt});
                    }
                }
            }
        }
    }
    
    std::cerr << addrs.size() << " addrs\n";
    
    // optimization parameters
#if 0
    alias_rel.reserve(addrs.size() * addrs.size());
#endif
    
    
    
    unsigned nos = 0, musts = 0, mays = 0;
    const auto add_aa = [&] (const AddrInfo& a, const AddrInfo& b, llvm::AliasResult result, const char *desc) {
        std::optional<z3::expr> cond;
        switch (result) {
            case llvm::AliasResult::NoAlias:
                ++nos;
                cond = a.e != b.e;
                break;
                
            case llvm::AliasResult::MayAlias:
                ++mays;
                break;
                
            case llvm::AliasResult::MustAlias:
                ++musts;
                cond = a.e == b.e;
                break;
                
            default:
                std::abort();
        }
        
        if (cond) {
            constraints(*cond, util::to_string("AA:", desc));
        }
        
        add_alias_result(a.vl(), b.vl(), result);
    };

    
    /* AA: all AllocaInst, GlobalObject, BlockAddress values have distinct addresses */
    {
        std::cerr << __FUNCTION__ << ": ensuring allocas, globals, blocks have distinct addresses...\n";
        Timer timer;
        z3::expr_vector v {context.context};
        for (const AddrInfo& addr : addrs) {
            if (llvm::isa<llvm::AllocaInst, llvm::GlobalValue, llvm::BlockAddress>(addr.V)) {
                v.push_back(addr.e);
            }
        }
        constraints(z3::distinct2(v), "alloca-addrs-distinct");
    }
    
    /* AA: all pairs of AllocaInsts and Arguments cannot alias */
    {
        std::cerr << __FUNCTION__ << ": allocas and arguments cannot alias...\n";
        Timer timer;
        std::vector<z3::expr> allocas; // this is also used elsewhere
        std::vector<z3::expr> args;
        for (const AddrInfo& addr : addrs) {
            if (llvm::isa<llvm::AllocaInst>(addr.V)) {
                allocas.push_back(addr.e);
            } else if (llvm::isa<llvm::Argument>(addr.V)) {
                args.push_back(addr.e);
            }
        }
        logv(1, "(alloca, arg) pairs: " << allocas.size() * args.size() << "\n");

        for (const z3::expr& alloca : allocas) {
            for (const z3::expr& arg : args) {
                constraints(alloca != arg, "alloca-argument-distinct");
            }
        }
    }
    
    /* AA: apply LLVM's built-in alias analysis if possible
     * Restrictions: both VLs must have the same call stack and loops must nest.
     * We can make this efficient by sorting all addresses into different buckets by function callstack.
     */
    {
        logv(1, __FUNCTION__ << ": applying LLVM's built-in alias analysis...");
        std::size_t count = 0;
        std::map<std::vector<CFG::FuncID>, std::vector<AddrInfoVec::const_iterator>> groups;
        for (auto it = addrs.begin(); it != addrs.end(); ++it) {
            const AddrInfo& addr = *it;
            groups[addr.id.func].push_back(it);
        }
        
        for (const auto& p : groups) {
            const auto& group = p.second;
            util::for_each_unordered_pair(group, [&] (AddrInfoVec::const_iterator it1, AddrInfoVec::const_iterator it2) {
                if (!util::prefixeq_bi(it1->id.loop, it2->id.loop)) { return; }
                add_aa(*it1, *it2, AA.alias(it1->V, it2->V), "llvm");
                ++count;
            });
        }
        
        logv(1, "processed " << count << " pairs\n");
    }
    
    /* AA: allocas cannot alias with getelementptrs */
    {
        logv(1, __FUNCTION__ << ": allocas and gelementptrs...\n");
        using Vec = std::vector<AddrInfoVec::const_iterator>;
        Vec allocas, geps_nonzero;
        std::unordered_map<const llvm::Type *, std::pair<Vec, Vec>> types;
        std::size_t num_type_allocas = 0;
        std::size_t num_type_geps = 0;
        for (auto it = addrs.begin(); it != addrs.end(); ++it) {
            if (llvm::isa<llvm::AllocaInst>(it->V)) {
                allocas.push_back(it);
                types[it->V->getType()->getPointerElementType()].first.push_back(it);
                ++num_type_allocas;
            } else if (const llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(it->V)) {
                if (!llvm::getelementptr_can_zero(GEP)) {
                    geps_nonzero.push_back(it);
                }
                types[GEP->getPointerOperandType()->getPointerElementType()].second.push_back(it);
                ++num_type_geps;
            }
        }
        
        /* process pairs in allocas x geps_nonzero */
        logv_(1, allocas.size() * geps_nonzero.size() << " pairs (1/2)\n");
#if 0
        for (const auto& alloca : allocas) {
            for (const auto& gep_nonzero : geps_nonzero) {
                add_aa(*alloca, *gep_nonzero, llvm::AliasResult::NoAlias, "alloca-gep-nonzero");
            }
        }
#elif 0
        {
            z3::expr_vector allocas_ = z3::transform(context.context, allocas, [] (const auto& x) { return x->e; });
            z3::expr_vector geps_nonzero_ = z3::transform(context.context, geps_nonzero, [] (const auto& x) { return x->e; });
            constraints(z3::no_intersect("alloca-geps-nonzero", context.context.int_sort(), allocas_, geps_nonzero_), "alloca-gep-nonzero");
        }
#elif 0
        {
            z3::expr_vector allocas_ = z3::transform(context.context, allocas, [] (const auto& x) { return x->e; });
            for (const auto& gep : geps_nonzero) {
                allocas_.push_back(gep->e);
                constraints(z3::distinct(allocas_), "alloca-gep-nonzero");
                allocas_.pop_back();
            }
        }
#endif
        
#if 0
        /* process different-type alloca, gep pairs */
        {
            std::size_t sum = std::transform_reduce(types.begin(), types.end(), static_cast<std::size_t>(0), std::plus<std::size_t>(), [&] (const auto& p) -> std::size_t {
                return p.second.first.size() * (num_type_geps - p.second.first.size());
            });
            logv_(1, sum << " pairs (2/2)\n");
            
            /* Well, the only ones that may alias are same-type. So map each GEP type to different integer.
             * NO.
             *
             * Map each Alloca type to different integer. This is OK since we know they never alias.
             * Then each GEP type
             *
             */
            
            for (const auto& gep_type : types) {
                const auto& geps = gep_type.second.second;
                z3::expr_vector allocas(context.context);
                for (const auto& alloca_type : types) {
                    if (alloca_type.first == gep_type.first) { continue; }
                    for (const auto& alloca : alloca_type.second.first) {
                        allocas.push_back(alloca->e);
                    }
                    for (const auto& gep : geps) {
                        allocas.push_back(gep->e);
                        constraints(z3::distinct2(allocas), "alloca-gep-type-mismatch");
                        allocas.pop_back();
                    }
                }
            }
            
        }
#endif
        
        
        /* Alloca isn't struct, GEP is struct */
        {
            z3::expr_vector allocas(context.context);
            z3::expr_vector geps(context.context);
            for (const AddrInfo& addr : addrs) {
                if (llvm::isa<llvm::AllocaInst>(addr.V) && !addr.V->getType()->getPointerElementType()->isStructTy()) {
                    allocas.push_back(addr.e);
                } else if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(addr.V)) {
                    if (GEP->getPointerOperandType()->getPointerElementType()->isStructTy()) {
                        geps.push_back(addr.e);
                    }
                }
            }
            
            logv(1, "alloca struct, gep not struct " << geps.size() << "\n");
            
            for (const z3::expr& gep : geps) {
                allocas.push_back(gep);
                constraints(z3::distinct2(allocas), "alloca-struct-not-gep");
                allocas.pop_back();
            }
        }
        
        
    }
    
    
#if 0
    // add constraints
    
    using ValueLocSet = std::unordered_set<ValueLoc>;
    ValueLocSet skip_vls; // skip because already saw 'must alias'
    
    const auto is_arch = [&] (const AddrInfo& x) -> z3::expr {
        if (x.ref) {
            return lookup(*x.ref).arch;
        } else {
            return context.TRUE;
        }
    };
    
    Progress progress {addrs.size()};
    for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
        ++progress;
        const ValueLoc vl1 = it1->vl();
        if (skip_vls.contains(vl1)) { continue; }
        for (auto it2 = std::next(it1); it2 != addrs.end(); ++it2) {
            const ValueLoc vl2 = it2->vl();

            if (check_alias(vl1, vl2) != llvm::AliasResult::MayAlias) { continue; }
            
            if (const auto alias_res = compute_alias(*it1, *it2, AA)) {
                if (skip_vls.contains(vl2)) { continue; }

                std::optional<z3::expr> cond;
                switch (*alias_res) {
                    case llvm::AliasResult::NoAlias: {
                        cond = it1->e != it2->e;
                        ++nos;
                        break;
                    }
                        
                    case llvm::AliasResult::MayAlias: {
                        ++mays;
                        break;
                    }
                        
                    case llvm::AliasResult::MustAlias: {
                        skip_vls.insert(vl2);
                        cond = it1->e == it2->e;
                        ++musts;
                        break;
                    }
                        
                    default: std::abort();
                }
                
                if (cond) {
                    if (!alias_mode.transient) {
                        cond = z3::implies(is_arch(*it1) && is_arch(*it2), *cond);
                    }
                    constraints(*cond, "alias-analysis");
                }
                add_alias_result(vl1, vl2, *alias_res);
                
            }
        }
    }
#endif
    
    std::cerr << "NoAlias: " << nos << "\n";
    std::cerr << "MustAlias: " << musts << "\n";
    std::cerr << "MayAlias: " << addrs.size() * addrs.size() - nos - musts << "\n";
}

}

#include "aeg.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"
#include "util/llvm.h"
#include "util/timer.h"
#include "util/z3.h"
#include "util/llvm.h"
#include "mon/proto.h"
#include "mon/client.h"
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


llvm::AliasResult AEG::compute_alias(const AddrInfo& a, const AddrInfo& b) const {
    /* check if we already found result */
    if (const auto result = check_alias(a.vl(), b.vl())) {
        if (result != llvm::MayAlias) {
            return result;
        }
    }
    
    /* check if LLVM's built-in alias analysis is valid */
    if (po.llvm_alias_valid(a.id, b.id)) {
        return AA.alias(a.V, b.V);
    }
    
    if (a.vl() == b.vl()) {
        return llvm::MustAlias;
    }
    
    
    /* AA: all AllocaInst, GlobalObject, BlockAddress values have distinct addresses */
    {
        const auto is = [] (const llvm::Value *V) -> bool {
            return llvm::isa<llvm::AllocaInst, llvm::GlobalObject, llvm::BlockAddress>(V);
        };
        if (is(a.V) && is(b.V)) {
            return llvm::AliasResult::NoAlias;
        }
    }
    
    /* AA: all pairs of AllocaInsts and Arguments cannot alias */
    {
        const auto is = [] (const AddrInfo& a, const AddrInfo& b) -> bool {
            return llvm::isa<llvm::AllocaInst>(a.V) && llvm::isa<llvm::Argument>(b.V);
        };
        if (is(a, b) || is(b, a)) {
            return llvm::AliasResult::NoAlias;
        }
    }
    
    /* Allocas and nonzero GEPs cannot alias */
    {
        const auto is = [] (const AddrInfo& a, const AddrInfo& b) -> bool {
            if (llvm::isa<llvm::AllocaInst>(a.V)) {
                if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(b.V)) {
                    if (!llvm::getelementptr_can_zero(GEP)) {
                        return true;
                    }
                }
            }
            return false;
        };
        if (is(a, b) || is(b, a)) {
            return llvm::AliasResult::NoAlias;
        }
    }
    
    /* Allocas and GEPs where alloca isn't struct and GEP is struct cannot alias */
    {
        const auto is = [] (const AddrInfo& a, const AddrInfo& b) -> bool {
            // check if alloca
            if (llvm::isa<llvm::AllocaInst>(a.V)) {
                // check if doesn't contain struct
                if (!llvm::contains_struct(a.V->getType()->getPointerElementType())) {
                    // check if GEP is struct
                    if (b.V->getType()->getPointerElementType()->isStructTy()) {
                        return true;
                    }
                }
            }
            return false;
        };
        if (is(a, b) || is(b, a)) {
            return llvm::AliasResult::NoAlias;
        }
    }
    
#if 0
    /* GEPs on AllocaInst, GlobalObjects, BlockAddresses values have distinct addresses */
    {
        const auto is = [] (const AddrInfo& a, const AddrInfo& b) -> bool {
            if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(a.V)) {
                const auto *ptr_op = GEP->getPointerOperand();
                if ()
            }
            return false;
        };
    }
#endif
    
    
    return llvm::AliasResult::MayAlias;
}


llvm::AliasResult AEG::compute_alias(NodeRef a, NodeRef b) const {
    const auto special = [&] (NodeRef ref) -> bool {
        return ref == entry || exits.contains(ref);
    };
    if (special(a) || special(b)) {
        return llvm::AliasResult::MayAlias;
    }
    
    // TODO: find out why some loookups fail
    const auto it1 = vl2addr.find(get_value_loc(a));
    const auto it2 = vl2addr.find(get_value_loc(b));
    if (it1 == vl2addr.end() || it2 == vl2addr.end()) {
        return llvm::AliasResult::MayAlias;
    } else {
        return compute_alias(it1->second, it2->second);
    }
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
    

    // map vls to addrs
    for (const AddrInfo& addr : addrs) {
        vl2addr.emplace(addr.vl(), addr);
    }
    
    unsigned filtered = 0;

    
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
            bool bad = false;
#ifndef NDEBUG
            z3::solver solver(context.context);
            z3::expr_vector vec(context.context);
            vec.push_back(*cond);
            bad = (solver.check(vec) != z3::sat);
#else
            cond = cond->simplify();
            bad = cond->is_false();
#endif
            if (bad) {
                std::cerr << "AA error: impossible AA assertion\n";
                std::cerr << "condition: " << *cond << "\n";
                std::cerr << "values:\n";
                llvm::errs() << *a.V << "\n";
                llvm::errs() << *b.V << "\n";
                std::abort();
            }
        }
        
        if (cond) {
            if (cond->is_true()) {
                ++filtered;
            } else {
                constraints(*cond, util::to_string("AA:", desc));
            }
        }
        
        add_alias_result(a.vl(), b.vl(), result);
    };

    {
        std::unordered_set<const llvm::AllocaInst *> AIs;
        for (const AddrInfo& addr : addrs) {
            if (const auto *AI = llvm::dyn_cast<llvm::AllocaInst>(addr.V)) {
                AIs.insert(AI);
            }
        }
        logv(1, __FUNCTION__ << ": " << AIs.size() << " AllocaInsts\n");
    }
    
    

#if 0
    // NOTE: This clause is causing some to timeout.
    /* AA: all AllocaInst, GlobalObject, BlockAddress values have distinct addresses */
    {
        logv(1, __FUNCTION__ << ": ensuring allocas, globals, blocks have distinct addresses...");
        std::vector<z3::expr_vector> vs;
        for (const AddrInfo& addr : addrs) {
            if (llvm::isa<llvm::AllocaInst, llvm::GlobalValue, llvm::BlockAddress>(addr.V)) {
                if (vs.empty() || vs.back().size() >= distinct_limit) {
                    vs.emplace_back(context.context);
                }
                vs.back().push_back(addr.e);
            }
        }
        for (const z3::expr_vector& v : vs) {
            constraints(z3::distinct2(v), "alloca-addrs-distinct");
        }
        std::size_t size = std::transform_reduce(vs.begin(), vs.end(), 0, std::plus<std::size_t>(), [] (const z3::expr_vector& v) -> std::size_t {
            return v.size();
        });
        logv_(1, size << " (" << vs.size() << " groups)\n");
        
        mon::Message msg;
        client.send_property(po.function_name(), "aa-distinct", util::to_string(size));
    }
    
#endif
    
    
#if 0
    /* AA: GEP of distinct Alloca bases have different values */
    {
        logv(1, __FUNCTION__ << ": GEPs with distinct alloca bases...");
        std::vector<std::pair<const llvm::AllocaInst *, AddrInfoVec::const_iterator>> map; // map from alloca base to GEP
        for (auto it = addrs.begin(); it != addrs.end(); ++it) {
            const auto& addr = *it;
            if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(addr.V)) {
                const auto *ptr_op = GEP->getPointerOperand();
                if (const auto *AI = llvm::dyn_cast<llvm::AllocaInst>(ptr_op)) {
                    map.emplace_back(AI, it);
                }
            }
        }
        
        unsigned i = 0;
        util::for_each_unordered_pair(map, [&] (const auto& p1, const auto& p2) {
            if (p1.first != p2.first) {
                add_aa(*p1.second, *p2.second, llvm::AliasResult::NoAlias, "geps-with-alloca-pointer-operands");
                ++i;
            }
        });
        
        logv_(1, i << " items\n");
    }
    
    
    /* AA: GEP w/ Alloca base, GlobalObject, BlockAddress have distinct addresses */
    
    
#endif
    
    
    
#if 1
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
#endif
    
#if 1
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
                // check if we're in the same context
                if (!util::prefixeq_bi(it1->id.loop, it2->id.loop)) { return; }
                
                // verify that at least one pointer is written to
                if (llvm::pointer_is_read_only(it1->V) && llvm::pointer_is_read_only(it2->V)) { return; }

                // apply alias result
                const auto alias_result = AA.alias(it1->V, it2->V);
                if (alias_result == llvm::AliasResult::MayAlias) { return; }
                add_aa(*it1, *it2, alias_result, "llvm");
                ++count;
            });
        }
        
        logv(1, "processed " << count << " pairs\n");
    }
#endif
    
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
        
        // DEBUG: print out how many GEPs have alloca's as indices
        {
            unsigned i = 0;
            for (const AddrInfo& addr : addrs) {
                if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(addr.V)) {
                    if (llvm::isa<llvm::AllocaInst>(GEP->getPointerOperand())) {
                        if (!llvm::getelementptr_const_offset(GEP)) {
                            ++i;
                        }
                    }
                }
            }
            logv(1, __FUNCTION__ << ": " << i << " nonconstant GEPs have alloca's as indices\n");
        }
      
        
        
        
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
            
            logv(1, "alloca struct, gep not struct " << allocas.size() * geps.size() << "\n");
            
#if 0
            for (const z3::expr& gep : geps) {
                allocas.push_back(gep);
                constraints(z3::distinct2(allocas), "alloca-struct-not-gep");
                allocas.pop_back();
            }
#endif
        }
        
        
        /* INFO */
        logv(1, __FUNCTION__ << ": filtered " << filtered << " tautological constraints\n");
        
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

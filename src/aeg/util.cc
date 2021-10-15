#include "aeg.h"
#include "cfg/expanded.h"

AEG::Mems AEG::get_mems(z3::expr Node::*pred) {
    z3::context& ctx = context.context;
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    Mems mems;
    z3::expr mem = z3::const_array(ctx.int_sort(), ctx.int_val(static_cast<unsigned>(entry)));
    for (const NodeRef ref : order) {
        mems.emplace(ref, mem);
        if (ref == entry) { continue; }
        const Node& node = lookup(ref);
        if (!node.may_write()) { continue; }
        mem = z3::conditional_store(mem, node.get_memory_address(), ctx.int_val(static_cast<unsigned>(ref)), node.*pred);
    }
    return mems;
}

/*
 * Note that transient execution always starts after a node with multiple children; this is an invariant.
 * As a first pass, we can do an arch mem pass.
 *
 * When two paths join, we can then merge the two paths using the reverse-postorder interleaving approach.
 * When they fork, we only add memory
 *
 * Exec Memory.
 * Need to discard results from transient execution windows.
 * Solution:
 * When collecting from preds, condition on using particular predecessor's mem is also:
 *   - not the case that pred is trans and cur is arch.
 */

AEG::Mems AEG::get_exec_mems() {
    z3::context& ctx = context.context;
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));

    const z3::expr init_mem = z3::const_array(ctx.int_sort(), ctx.int_val(static_cast<unsigned>(entry)));
    Mems ins;
    Mems outs = {{entry, init_mem}};
    for (const NodeRef cur : order) {
        if (cur == entry) { continue; }
        const Node& cur_node = lookup(cur);
        
        const auto tfos = get_nodes(Direction::IN, cur, Edge::TFO);
        assert(!tfos.empty());
        
        z3::expr mem = std::accumulate(tfos.begin(), tfos.end(), init_mem, [&] (const z3::expr& acc, const auto& tfo) -> z3::expr {
            const NodeRef pred = tfo.first;
            return z3::ite(tfo.second, outs.at(pred), acc);
        });
        
        ins.emplace(cur, mem);
        
        if (cur_node.may_write()) {
            // TODO: this might be sped up by not making these stores conditional, since the above logic filters out unexecuted nodes.
            mem = z3::conditional_store(mem, cur_node.get_memory_address(), ctx.int_val(static_cast<unsigned>(cur)), cur_node.exec());
        }
        
        outs.emplace(cur, mem);
    }
    
    return ins;
}

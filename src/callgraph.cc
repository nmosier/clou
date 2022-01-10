#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Function.h>
#include <llvm/ADT/SCCIterator.h>

#include "callgraph.h"


#if 0


class TestPass: public llvm::ModulePass {
public:
    static inline char ID = 0;
    
    std::vector<llvm::Function *> order;
    
    TestPass(): llvm::ModulePass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
#if 1
        AU.setPreservesAll();
        AU.addRequired<FunctionOrderingPass>();
#endif
    }
    
    virtual bool runOnModule(llvm::Module& M) override {
#if 1
        const auto& function_ordering = getAnalysis<FunctionOrderingPass>();
        assert(!function_ordering.order.empty());
        for (const auto& F : function_ordering.order) {
            std::cerr << F->getName().str() << "\n";
        }
        std::cerr << "HERE\n";
#endif
        return false;
    }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override {}
};

llvm::RegisterPass<FunctionOrderingPass> X {
    "function-ordering", "Function Ordering Pass", false, false
};

llvm::RegisterPass<TestPass> Y {
    "test", "Test Pass", false, true,
};

#endif

// TEST 2
class Test2Pass: public llvm::CallGraphSCCPass {
public:
    static inline char ID = 0;
    
    Test2Pass(): llvm::CallGraphSCCPass(ID) {}
    
    virtual bool runOnSCC(llvm::CallGraphSCC& F) override {
        return false;
    }
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
        AU.setPreservesAll();
    }
};

class Test3Pass: public llvm::ModulePass {
public:
    static inline char ID = 0;
    
    Test3Pass(): llvm::ModulePass(ID) {}
    
    virtual bool runOnModule(llvm::Module& M) override {
        llvm::CallGraph& CG = getAnalysis<llvm::CallGraphWrapperPass>().getCallGraph();

        auto scc_it = llvm::scc_begin(&CG);
        while (!scc_it.isAtEnd()) {
            const auto& scc = *scc_it;
            std::cerr << "scc: ";
            for (llvm::CallGraphNode *scc_node : scc) {
                if (llvm::Function *F = scc_node->getFunction()) {
                    if (!F->isDeclaration()) {
                        std::cerr << " " << F->getName().str();
                    }
                }
            }
            std::cerr << "\n";
            ++scc_it;
        }
        return false;
    }
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
        AU.setPreservesAll();
        AU.addRequired<llvm::CallGraphWrapperPass>();
    }
};

llvm::RegisterPass<FunctionOrderingPass> X {
    "function-ordering", "Function Ordering Pass",
};

llvm::RegisterPass<Test2Pass> A_ {
    "test2", "Test 2 Pass",
};

llvm::RegisterPass<Test3Pass> A {
    "test3", "Test 3 Pass",
};

static void RegisterTest3Pass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new Test3Pass());
}

static llvm::RegisterStandardPasses RegisterTest3Pass_1 {
    llvm::PassManagerBuilder::EP_ModuleOptimizerEarly,
    RegisterTest3Pass
};

static llvm::RegisterStandardPasses RegisterTest3Pass_2 {
    llvm::PassManagerBuilder::EP_EnabledOnOptLevel0,
    RegisterTest3Pass
};

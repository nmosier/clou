#pragma once

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/Analysis/LoopPass.h>
#include <llvm/IR/IntrinsicInst.h>
#include <llvm/IR/Operator.h>
#include <llvm/IR/Intrinsics.h>

#include <string>
#include <unordered_map>

using Annotations = std::unordered_map<const llvm::Value *, std::string>;

struct AnnotationPass final: public llvm::ModulePass {
    static inline char ID = 0;
    
    AnnotationPass(): llvm::ModulePass(ID) {}
    
    virtual bool runOnModule(llvm::Module& M) override;
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
        AU.setPreservesAll();
    }
    
    Annotations results;
    const Annotations& getResults() const { return results; }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override;
};

#pragma once

#include <string>
#include <ostream>
#include <optional>
#include <mutex>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/IR/Module.h>

namespace llvm {

template <typename T>
std::string to_string(const T& x) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << x;
    return s;
}

bool getelementptr_can_zero(const llvm::GetElementPtrInst *GEP);
std::optional<int> getelementptr_const_offset(const llvm::GetElementPtrInst *GEP);
std::optional<int> getelementptr_min_offset(const llvm::GetElementPtrInst *GEP);
std::optional<int> getelementptr_max_offset(const llvm::GetElementPtrInst *GEP);

bool contains_struct(const llvm::Type *T);

bool pointer_is_read_only(const llvm::Value *P);

  extern std::mutex errs_mutex;
struct locked_raw_ostream {
    std::unique_lock<std::mutex> lock;
    llvm::raw_ostream& os;
    
    locked_raw_ostream(llvm::raw_ostream& os, std::mutex& mutex): lock(mutex), os(os) {}
    
    operator llvm::raw_ostream&() { return os; }
};

llvm::raw_ostream& print_full_debug_info(llvm::raw_ostream& os, const llvm::DebugLoc& DL);

}

template <typename T>
std::ostream& llvm_to_cxx_os(std::ostream& os, const T& x) {
    return os << llvm::to_string(x);
}

inline std::ostream& operator<<(std::ostream& os, const llvm::Instruction& I) {
   return llvm_to_cxx_os(os, I);
}


namespace llvm {

unsigned get_min_loop_iterations(const llvm::Loop *L);

/** Parse clang annotations. Output iterator must accept pairs of llvm::Value and std::string. */
template <class OutputIt>
OutputIt parse_annotations(const llvm::Module& M, OutputIt out) {

    // Global Annotations
    if (const llvm::GlobalVariable *G = M.getNamedGlobal("llvm.global.annotations")) {
        assert(G->getNumOperands() == 1);
        const llvm::Value *V = G->getOperand(0);
        const llvm::ConstantArray *CA = llvm::cast<llvm::ConstantArray>(V);
        for (const llvm::Value *V : CA->operands()) {
            const llvm::ConstantStruct *CS = llvm::cast<llvm::ConstantStruct>(V);
            assert(CS->getNumOperands() == 5);
            const llvm::Value *annotated = CS->getOperand(0)->getOperand(0);
            const llvm::GlobalVariable *GV_str = llvm::cast<llvm::GlobalVariable>(CS->getOperand(1)->getOperand(0));
            const llvm::ConstantDataArray *A_str = llvm::cast<llvm::ConstantDataArray>(GV_str->getOperand(0));
            const std::string str = A_str->getAsCString().str();
            *out++ = std::make_pair(annotated, str);
        }
    }
    
    return out;
}

}

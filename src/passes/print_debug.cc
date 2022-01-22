#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>

#include <string>
#include <fstream>

static std::string get_line(const std::string& path, unsigned line, unsigned col = 0) {
    std::ifstream ifs {path};
    std::string res;
    for (unsigned i = 0; i < line && std::getline(ifs, res); ++i) {}
    if (res.size() >= col) {
        res = res.substr(col);
    }
    return res;
}

struct PrintDebugPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    PrintDebugPass(): llvm::FunctionPass(ID) {}
    
    virtual bool runOnFunction(llvm::Function& F) override {
        llvm::raw_ostream& os = llvm::errs();
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                const llvm::DebugLoc& loc = I.getDebugLoc();
                os << I;
                if (loc) {
                    std::string s;
                    llvm::raw_string_ostream ss (s);
                    loc.print(ss);
                    
                    char *s_ = ::strdup(s.c_str());
                    char *tok = s_;
                    const char *filename_ = ::strsep(&tok, ":");
                    const std::string filename = filename_;
                    std::free(s_);
                    
                    os << ": " << s << ": " << get_line(filename, loc.getLine(), loc.getCol() - 1);
                }
                os << "\n";
            }
        }
        
        return false;
    }
};


namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new PrintDebugPass());
}

llvm::RegisterStandardPasses registerCFP {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};

llvm::RegisterPass<PrintDebugPass> X {
    "count_functions", "Count Functions Pass",
};

}

#pragma once

/* inst.h -- get information about instructions
 */

#include <unordered_set>

#include <llvm/IR/Instructions.h>

#include "util/xmacro.h"
#include "cfg/cfg.h"


struct Inst {
#define INST2_KIND_X(X) \
X(ENTRY) \
X(EXIT) \
X(LOAD) \
X(STORE) \
X(FENCE) \
X(CALL) \
X(OTHER)
    
    XM_ENUM_DEF(Kind, INST2_KIND_X);
    
    virtual Kind kind() const noexcept = 0;
    virtual bool is_special() const noexcept { return false; }
    virtual bool is_entry() const noexcept { return false; }
    virtual bool is_exit() const noexcept { return false; }
    virtual bool is_memory() const noexcept { return false; }
    virtual bool is_fence() const noexcept { return false; }
    
    virtual Option may_read() const noexcept { return Option::NO; }
    virtual Option may_write() const noexcept { return Option::NO; }
    virtual Option may_xsread() const noexcept { return Option::NO; }
    virtual Option may_xswrite() const noexcept { return Option::NO; }
    
    virtual const llvm::Instruction *get_inst() const = 0;

    static const char *kind_tostr(Kind kind) {
        XM_ENUM_TOSTR(INST2_KIND_X, kind, "(invalid)");
    }
    
    static Inst *Create(Entry);
    static Inst *Create(Exit);
    static Inst *Create(const llvm::Instruction *I);
    static Inst *Create(const CFG::Node::Call& call);
    
    virtual void print(std::ostream& os) const;
    virtual ~Inst() {}
};

inline std::ostream& operator<<(std::ostream& os, Inst::Kind kind) {
    return os << Inst::kind_tostr(kind);
}

struct SpecialInst: Inst {
    virtual bool is_special() const noexcept override { return true; }
    
    virtual const llvm::Instruction *get_inst() const override { return nullptr; }
};

struct EntryInst: SpecialInst {
    virtual Kind kind() const noexcept override { return ENTRY; }
    
    virtual bool is_entry() const noexcept override { return true; }
    
    virtual Option may_write() const noexcept override { return Option::MUST; }
    virtual Option may_xswrite() const noexcept override { return Option::MUST; }
};

struct ExitInst: SpecialInst {
    virtual Kind kind() const noexcept override { return EXIT; }
    
    virtual bool is_exit() const noexcept override { return true; }
    
    virtual Option may_read() const noexcept override { return Option::MUST; }
    virtual Option may_xsread() const noexcept override { return Option::MUST; }
};

struct RegularInst: Inst {
    const llvm::Value *addr_def = nullptr;
    std::vector<const llvm::Value *> addr_refs;
    const llvm::Instruction *I;
    
    RegularInst(const llvm::Instruction *I);
    
    virtual void print(std::ostream& os) const override {
        Inst::print(os);
        std::string s;
        llvm::raw_string_ostream ss(s);
        ss << " " << *I;
        os << s;
    }
    
    virtual const llvm::Instruction *get_inst() const override { return I; }
};

struct FenceInst: RegularInst {
    virtual Kind kind() const noexcept override { return FENCE; }
    virtual bool is_fence() const noexcept override { return true; }
    
    FenceInst(const llvm::Instruction *I): RegularInst(I) {}
};

struct MemoryInst: RegularInst {
    virtual bool is_memory() const noexcept override { return true; }
    
    virtual const llvm::Value *get_memory_operand() const = 0;
    
    MemoryInst(const llvm::Instruction *I): RegularInst(I) {}
};

struct LoadInst: MemoryInst {
    virtual Kind kind() const noexcept override { return LOAD; }
    
    virtual Option may_read() const noexcept override { return Option::MUST; }
    virtual Option may_xsread() const noexcept override { return Option::MUST; }
    virtual Option may_xswrite() const noexcept override { return Option::MAY; }
    
    virtual const llvm::Value *get_memory_operand() const override {
        return I->getOperand(0);
    }
    
    LoadInst(const llvm::Instruction *I): MemoryInst(I) {}
};

struct StoreInst: MemoryInst {
    virtual Kind kind() const noexcept override { return STORE; }
    
    virtual Option may_write() const noexcept override { return Option::MUST; }
    virtual Option may_xsread() const noexcept override { return Option::MUST; }
    virtual Option may_xswrite() const noexcept override { return Option::MUST; } // TODO: relax this to 'MAY' once we do silent stores
    
    virtual const llvm::Value *get_memory_operand() const override {
        if (const auto *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
            return SI->getPointerOperand();
        } else {
            std::abort();
        }
    }
    
    const llvm::Value *get_value_operand() const {
        if (const auto *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
            return SI->getValueOperand();
        } else {
            std::abort();
        }
    }
    
    StoreInst(const llvm::Instruction *I): MemoryInst(I) {}
};

struct CallInst: MemoryInst {
    const llvm::Value *arg;
    
    virtual Kind kind() const noexcept override { return CALL; }
    
    virtual Option may_read() const noexcept override { return Option::MAY; }
    virtual Option may_write() const noexcept override { return Option::MAY; }
    virtual Option may_xsread() const noexcept override { return Option::MAY; }
    virtual Option may_xswrite() const noexcept override { return Option::MAY; }
    
    // TODO: this might need more fine-grained control over adding assertions, since write -> xswrite, etc.
    
    virtual const llvm::Value *get_memory_operand() const override {
        return arg;
    }
    
    CallInst(const llvm::Instruction *I, const llvm::Value *arg);
    
};

struct OtherInst: RegularInst {
    virtual Kind kind() const noexcept override { return OTHER; }
    
    OtherInst(const llvm::Instruction *I): RegularInst(I) {}
};


inline std::ostream& operator<<(std::ostream& os, const Inst& inst) {
    inst.print(os);
    return os;
}

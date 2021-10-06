#pragma once

/* inst.h -- get information about instructions
 */

#include <unordered_set>

#include <llvm/IR/Instructions.h>

#include "xmacro.h"
#include "lcm.h"

#if 0
struct Inst {
    
#define INST_KIND_X(X, ...)                  \
X(ENTRY, ##__VA_ARGS__)                                     \
X(EXIT,  ##__VA_ARGS__)                                      \
X(READ,  ##__VA_ARGS__)                                      \
X(WRITE, ##__VA_ARGS__)                                     \
X(FENCE, ##__VA_ARGS__)                                     \
X(CALL,  ##__VA_ARGS__)       \
X(OTHER, ##__VA_ARGS__)
    
#define INST_SET_read(X, ...) \
X(READ, ##__VA_ARGS__) \
X(EXIT, ##__VA_ARGS__)
    
#define INST_SET_write(X, ...) \
X(WRITE, ##__VA_ARGS__) \
X(ENTRY, ##__VA_ARGS__)
    
#define INST_SET_special(X, ...) \
X(ENTRY, ##__VA_ARGS__) \
X(EXIT, ##__VA_ARGS__)
    
#define INST_SET_entry(X, ...) X(ENTRY, ##__VA_ARGS__)
#define INST_SET_exit(X, ...) X(EXIT, ##__VA_ARGS__)

#define INST_SET_rw_inst(X, ...) \
X(READ, ##__VA_ARGS__) \
X(WRITE, ##__VA_ARGS__)
    
#define INST_SETS(X, ...) \
X(read, ##__VA_ARGS__) \
X(write, ##__VA_ARGS__) \
X(special, ##__VA_ARGS__) \
X(entry, ##__VA_ARGS__) \
X(exit, ##__VA_ARGS__) \
X(rw_inst, ##__VA_ARGS__)
    
#define INST_SET_DEF_ELT(name) Inst::Kind::name,
#define INST_SET_DEF(name, ...) \
static inline const std::unordered_set<Inst::Kind> name##_set = { \
INST_SET_##name(INST_SET_DEF_ELT) \
};
    
#define INST_SET_IS(name) \
bool is_##name() const { \
return name##_set.find(kind) != name##_set.end(); \
}
    
   XM_ENUM_CLASS_DEF(Kind, INST_KIND_X);
    
   INST_SETS(INST_SET_DEF)
   INST_SETS(INST_SET_IS)

   const llvm::Instruction *I = nullptr;
   Kind kind;
   const llvm::Value *addr_def = nullptr;
   std::vector<const llvm::Value *> addr_refs;

   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind); }
   void set_kind();
   void set_addr();
   
   void set_base();
   void set(const llvm::Instruction *I_) { I = I_; set_kind(); set_addr(); }
   void set(Entry) { kind = Kind::ENTRY; }
   void set(Exit) { kind = Kind::EXIT; }
   
   template <typename Arg>
   explicit Inst(const Arg& arg) { set(arg); }
    
    template <typename Derived>
    bool isa() const {
        static_assert(std::is_base_of<llvm::Instruction, Derived>());
        return llvm::dyn_cast<Derived>(I) != nullptr;
    }
};

std::ostream& operator<<(std::ostream& os, Inst::Kind kind);
std::ostream& operator<<(std::ostream& os, const Inst& inst);
#endif


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
    
    virtual Kind kind() const = 0;
    virtual bool is_special() const { return false; }
    virtual bool is_entry() const { return false; }
    virtual bool is_exit() const { return false; }
    virtual bool is_memory() const { return false; }
    
    virtual Option may_read() const { return Option::NO; }
    virtual Option may_write() const { return Option::NO; }
    virtual Option may_xsread() const { return Option::NO; }
    virtual Option may_xswrite() const { return Option::NO; }
    
    virtual const llvm::Instruction *get_inst() const = 0;

    static const char *kind_tostr(Kind kind) {
        XM_ENUM_TOSTR(INST2_KIND_X, kind, "(invalid)");
    }
    
    static Inst *Create(Entry);
    static Inst *Create(Exit);
    static Inst *Create(const llvm::Instruction *I);
    
    virtual void print(std::ostream& os) const {
        os << kind();
    }
    
    virtual ~Inst() {}
};

inline std::ostream& operator<<(std::ostream& os, Inst::Kind kind) {
    return os << Inst::kind_tostr(kind);
}

struct SpecialInst: Inst {
    virtual bool is_special() const override { return true; }
    
    virtual const llvm::Instruction *get_inst() const override { return nullptr; }
};

struct EntryInst: SpecialInst {
    virtual Kind kind() const override { return ENTRY; }
    
    virtual bool is_entry() const override { return true; }
    
    virtual Option may_write() const override { return Option::MUST; }
    virtual Option may_xswrite() const override { return Option::MUST; }
};

struct ExitInst: SpecialInst {
    virtual Kind kind() const override { return EXIT; }
    
    virtual bool is_exit() const override { return true; }
    
    virtual Option may_read() const override { return Option::MUST; }
    virtual Option may_xsread() const override { return Option::MUST; }
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
    virtual Kind kind() const override { return FENCE; }
    
    FenceInst(const llvm::Instruction *I): RegularInst(I) {}
};

struct MemoryInst: RegularInst {
    virtual bool is_memory() const override { return true; }
    
    virtual const llvm::Value *get_memory_operand() const = 0;
    
    MemoryInst(const llvm::Instruction *I): RegularInst(I) {}
};

struct LoadInst: MemoryInst {
    virtual Kind kind() const override { return LOAD; }
    
    virtual Option may_read() const override { return Option::MUST; }
    virtual Option may_xsread() const override { return Option::MUST; }
    virtual Option may_xswrite() const override { return Option::MAY; }
    
    virtual const llvm::Value *get_memory_operand() const override {
        return I->getOperand(0);
    }
    
    LoadInst(const llvm::Instruction *I): MemoryInst(I) {}
};

struct StoreInst: MemoryInst {
    virtual Kind kind() const override { return STORE; }
    
    virtual Option may_write() const override { return Option::MUST; }
    virtual Option may_xsread() const override { return Option::MUST; }
    virtual Option may_xswrite() const override { return Option::MUST; } // TODO: relax this to 'MAY' once we do silent stores
    
    virtual const llvm::Value *get_memory_operand() const override {
        return I->getOperand(1);
    }
    
    StoreInst(const llvm::Instruction *I): MemoryInst(I) {}
};

struct CallInst: MemoryInst {
    virtual Kind kind() const override { return CALL; }
    
    virtual Option may_read() const override { return Option::MAY; }
    virtual Option may_write() const override { return Option::MAY; }
    virtual Option may_xsread() const override { return Option::MAY; }
    virtual Option may_xswrite() const override { return Option::MAY; }
    // TODO: this might need more fine-grained control over adding assertions, since write -> xswrite, etc.
    
    CallInst(const llvm::Instruction *I): MemoryInst(I) { std::abort(); }
};

struct OtherInst: RegularInst {
    virtual Kind kind() const override { return OTHER; }
    
    OtherInst(const llvm::Instruction *I): RegularInst(I) {}
};


inline std::ostream& operator<<(std::ostream& os, const Inst& inst) {
    inst.print(os);
    return os;
}

#include <fstream>
#include <sstream>
#include <unordered_set>

#include <llvm/IR/DataLayout.h>
#include <llvm/IR/Operator.h>
#include <llvm/IR/IntrinsicInst.h>

#include "llvm.h"

namespace llvm {

bool getelementptr_can_zero(const llvm::GetElementPtrInst *GEP) {
    for (const llvm::Value *V : GEP->indices()) {
        if (const llvm::ConstantInt *CI = llvm::dyn_cast<llvm::ConstantInt>(V)) {
            if (CI->getValue().getLimitedValue() != 0) {
                return false;
            }
        }
    }
    return true;
}

// TODO: unify these 3 GEP offset functions.
std::optional<int> getelementptr_const_offset(const llvm::GetElementPtrInst *GEP) {
    const llvm::Module *M = GEP->getParent()->getParent()->getParent();
    llvm::DataLayout layout {M};
    
    llvm::Type *T = GEP->getPointerOperandType();
    std::size_t offset = 0;
    for (const llvm::Value *V : GEP->indices()) {
        if (const llvm::ConstantInt *CI = llvm::dyn_cast<llvm::ConstantInt>(V)) {
            const uint64_t index = CI->getSExtValue();
            
            if (const llvm::StructType *ST = llvm::dyn_cast<llvm::StructType>(T)) {
                for (uint64_t i = 0; i < index; ++i) {
                    offset += layout.getTypeSizeInBits(ST->getElementType(i));
                }
                T = ST->getElementType(index);
            } else if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
                T = AT->getElementType();
                offset += layout.getTypeSizeInBits(T) * index;
            } else if (const llvm::PointerType *PT = llvm::dyn_cast<llvm::PointerType>(T)) {
                T = PT->getElementType();
                offset += layout.getTypeSizeInBits(T) * index;
            } else {
                std::abort();
            }
        } else {
            return std::nullopt;
        }
    }
    
    return offset / 8;
}

std::optional<int> getelementptr_min_offset(const llvm::GetElementPtrInst *GEP) {
    const llvm::Module *M = GEP->getParent()->getParent()->getParent();
    llvm::DataLayout DL {M};
    llvm::Type *T = GEP->getPointerOperandType();
    std::size_t offset = 0;
    for (const llvm::Value *V : GEP->indices()) {
        if (const llvm::ConstantInt *CI = llvm::dyn_cast<llvm::ConstantInt>(V)) {
            
            const uint64_t index = CI->getSExtValue();
            
            if (const llvm::StructType *ST = llvm::dyn_cast<llvm::StructType>(T)) {
                for (uint64_t i = 0; i < index; ++i) {
                    offset += DL.getTypeSizeInBits(ST->getElementType(i));
                }
                T = ST->getElementType(index);
            } else if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
                T = AT->getElementType();
                offset += DL.getTypeSizeInBits(T) * index;
            } else if (const llvm::PointerType *PT = llvm::dyn_cast<llvm::PointerType>(T)) {
                T = PT->getElementType();
                offset += DL.getTypeSizeInBits(T) * index;
            } else {
                std::abort();
            }
            
        } else {
            
            if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
                
                // assume minimum index is 0
                T = AT->getElementType();
                offset += 0;
                
            } else {
                return std::nullopt;
            }
            
        }
    }
    
    return offset / 8;
}


std::optional<int> getelementptr_max_offset(const llvm::GetElementPtrInst *GEP) {
    const llvm::Module *M = GEP->getParent()->getParent()->getParent();
    llvm::DataLayout DL {M};
    llvm::Type *T = GEP->getPointerOperandType();
    std::size_t offset = 0;
    for (const llvm::Value *V : GEP->indices()) {
        if (const llvm::ConstantInt *CI = llvm::dyn_cast<llvm::ConstantInt>(V)) {
            
            const uint64_t index = CI->getSExtValue();
            
            if (const llvm::StructType *ST = llvm::dyn_cast<llvm::StructType>(T)) {
                for (uint64_t i = 0; i < index; ++i) {
                    offset += DL.getTypeSizeInBits(ST->getElementType(i));
                }
                T = ST->getElementType(index);
            } else if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
                T = AT->getElementType();
                offset += DL.getTypeSizeInBits(T) * index;
            } else if (const llvm::PointerType *PT = llvm::dyn_cast<llvm::PointerType>(T)) {
                T = PT->getElementType();
                offset += DL.getTypeSizeInBits(T) * index;
            } else {
                std::abort();
            }
            
        } else {
            
            if (const llvm::ArrayType *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
                
                // assume index can't go out of bounds
                T = AT->getElementType();
                const uint64_t nelems = AT->getNumElements();
                if (nelems > 0) {
                    offset += nelems * DL.getTypeSizeInBits(T);
                    return offset / 8;
                } else {
                    return std::nullopt;
                }
                
                
            } else {
                return std::nullopt;
            }
            
        }
    }
    
    return offset / 8;
}


bool contains_struct(const llvm::Type *T) {
    if (const auto *AT = llvm::dyn_cast<llvm::ArrayType>(T)) {
        return contains_struct(AT->getElementType());
    } else if (const auto *FT = llvm::dyn_cast<llvm::FunctionType>(T)) {
        return false;
    } else if (const auto *IT = llvm::dyn_cast<llvm::IntegerType>(T)) {
        return false;
    } else if (const auto *PT = llvm::dyn_cast<llvm::PointerType>(T)) {
        return false;
    } else if (const auto *ST = llvm::dyn_cast<llvm::StructType>(T)) {
        return true;
    } else if (const auto *VT = llvm::dyn_cast<llvm::VectorType>(T)) {
        return contains_struct(VT->getElementType());
    } else {
        std::abort();
    }
}


bool pointer_is_read_only(const llvm::Value *P) {
    // TODO: To make this safer, we shouldn't even look at function calls (unfortunately).
    
    if (llvm::isa<llvm::Argument, llvm::Instruction, llvm::GlobalValue, llvm::Operator>(P)) {
        for (const llvm::User *U : P->users()) {
            if (const auto *I = llvm::dyn_cast<llvm::Instruction>(U)) {
                if (I->mayWriteToMemory()) {
                    if (const auto *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
                        if (SI->getPointerOperand() == P) {
                            return false;
                        }
                    } else if (const auto *CI = llvm::dyn_cast<llvm::CallInst>(I)) {
                        // don't care if P == CI->getCalledOperand()
                        for (unsigned i = 0; i < CI->getNumArgOperands(); ++i) {
                            const auto *A = CI->getArgOperand(i);
                            if (P == A && !CI->doesNotAccessMemory(i) && !CI->onlyReadsMemory(i)) {
                                return false;
                            }
                        }
                    } else if (const auto *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
                        if (LI->isVolatile()) {
                            return false;
                        } else {
                            llvm::errs() << __FUNCTION__ << ": non-volatile LoadInst may write to memory: " << *I << "\n";
                            std::abort();
                        }
                    } else {
                        llvm::errs() << __FUNCTION__ << ": unrecognized instruction that may write to memory: " << *I << "\n";
                        std::abort();
                    }
                }
            }
        }
        
        return true;
    }
    
    if (llvm::isa<llvm::ConstantPointerNull, llvm::BasicBlock>(P)) {
        return true;
    }

    llvm::errs() << __FUNCTION__ << ": unrecognized value: " << *P << "\n";
    std::abort();
}


// TODO: delete
locked_raw_ostream cerr() {
    static std::mutex mutex;
    return locked_raw_ostream(llvm::errs(), mutex);
}

std::mutex errs_mutex;



llvm::raw_ostream& print_full_debug_info(llvm::raw_ostream& os, const llvm::DebugLoc& DL) {
    
    std::string s;
    llvm::raw_string_ostream ss {s};
    DL.print(ss);
    
    char *s_ = ::strdup(s.c_str());
    char *tok = s_;
    const char *path_ = ::strsep(&tok, ":");
    assert(path_ != nullptr);
    
    std::stringstream path;
    if (const char *src_dir = std::getenv("SRC")) {
        path << src_dir << "/";
    }
    path << path_;
    std::free(s_);
    
    std::ifstream ifs {path.str()};
    os << path.str() << ":" << DL.getLine() << ":\n";
    std::string line;
    for (unsigned i = 0; i < DL.getLine() && ifs; ++i) {
        std::getline(ifs, line);
    }
    
    if (!ifs) {
        os << "(no debug info)\n";
    } else {
        os << line << "\n";
        os << std::string(DL.getCol() - 1, ' ') << '^' << "\n";
    }
        
    return os;
}


unsigned get_min_loop_iterations(const llvm::Loop *L) {
    std::unordered_set<const llvm::BasicBlock *> blocks;
    std::copy(L->block_begin(), L->block_end(), std::inserter(blocks, blocks.end()));
    for (const llvm::Loop *subloop : *L) {
        for (const llvm::BasicBlock *subblock : subloop->blocks()) {
            blocks.erase(subblock);
        }
    }
    
    for (const llvm::BasicBlock *B : blocks) {
        for (const llvm::Instruction& I : *B) {
            if (const llvm::IntrinsicInst *II = llvm::dyn_cast<llvm::IntrinsicInst>(&I)) {
                if (II->getIntrinsicID() == llvm::Intrinsic::var_annotation) {
                    const llvm::Value *V = llvm::cast<llvm::GEPOperator>(II->getArgOperand(1));
                    const llvm::GlobalVariable *GV = llvm::cast<llvm::GlobalVariable>(V);
                    const llvm::ConstantDataArray *CDA = llvm::cast<llvm::ConstantDataArray>(GV->getInitializer());
                    const std::string s = CDA->getAsCString().str();
                    
                    std::string_view sv = s;
                    const auto pos = sv.find('=');
                    if (pos != std::string_view::npos) {
                        std::string_view key = sv.substr(0, pos);
                        std::string_view value = sv.substr(pos + 1);
                        if (key == "loop.min") {
                            return std::stoul(std::string(value));
                        }
                    }
                }
            }
        }
    }
    
    return 0;
}

}

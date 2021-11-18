#pragma once

#include "leakage.h"

namespace lkg {

struct SpectreV4_Leakage {
    NodeRef transmitter;
    NodeRef bypassed_store;
    NodeRef sourced_store;
    NodeRef load;
    
    Leakage leakage() const {
        return Leakage {
            .vec = {sourced_store, bypassed_store, load, transmitter},
            .transmitter = transmitter,
        };
    }
    
    NodeRefVec vec() const {
        return {sourced_store, bypassed_store, load, transmitter};
    }
    
    NodeRef get_transmitter() const { return transmitter; }
};

class SpectreV4_Detector: public Detector {
public:
    SpectreV4_Detector(aeg::AEG& aeg, Solver& solver): Detector(aeg, solver) {}
private:
    SpectreV4_Leakage leak;
    
    virtual void run_() override final;
    void run_load(NodeRef access, CheckMode mode); /*!< binds speculative load */
    void run_bypassed_store(CheckMode mode); /*!< binds bypassed store */
    void run_sourced_store(CheckMode mode); /*!< binds sourced stores */
    virtual std::string name() const override final { return "SpectreV4"; }
};

}

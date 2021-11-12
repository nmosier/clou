#pragma once

#include "leakage.h"

namespace lkg {

struct SpectreV4_Leakage: Leakage<SpectreV4_Leakage> {
    NodeRef transmitter;
    NodeRef bypassed_store;
    NodeRef sourced_store;
    NodeRef load;
    
    NodeRefVec vec() const {
        return {sourced_store, bypassed_store, load, transmitter};
    }
    
    NodeRef get_transmitter() const { return transmitter; }
};

class SpectreV4_Detector: public Detector_<SpectreV4_Leakage> {
public:
    SpectreV4_Detector(aeg::AEG& aeg, z3::solver& solver): Detector_(aeg, solver) {}
private:
    SpectreV4_Leakage leak;
    
    virtual void run_() override final;
    void run_load(NodeRef access); /*!< binds speculative load */
    void run_bypassed_store(); /*!< binds bypassed store */
    void run_sourced_store(); /*!< binds sourced stores */
    virtual std::string name() const override final { return "SpectreV4"; }
};

}

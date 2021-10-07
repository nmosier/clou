#pragma once

#include "cfg/cfg.h"

class CFG_Unrolled;

class CFG_Calls: public CFG {
public:
    CFG_Calls(const unsigned num_specs): CFG(num_specs) {} // TODO: this shouldn't be necessary
    
    void construct(const CFG& in);
};

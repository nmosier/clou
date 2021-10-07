#pragma once

#include "cfg/cfg.h"

class AEGPO_Unrolled;

class CFG_Calls: public AEGPO {
public:
    CFG_Calls(const unsigned num_specs): AEGPO(num_specs) {} // TODO: this shouldn't be necessary
    
    void construct(const AEGPO& in);
};

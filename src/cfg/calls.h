#pragma once

#include "cfg/cfg.h"

class CFG_Unrolled;

class CFG_Calls: public CFG {
public:
    void construct(const CFG& in);
};

#include "rel.h"

BinaryInstRel::Nodes BinaryInstRel::nodes() const {
   Nodes nodes;
   nodes.insert(begin(), end()); 
}

#pragma once

#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "util/container.h"

template <typename Key>
class natural_set;

using NodeRef = std::size_t;
using NodeRefVec = std::vector<NodeRef>;
#if 1
using NodeRefSet = std::unordered_set<NodeRef>;
#else
using NodeRefSet = util::natural_set<NodeRef>;
#endif
using NodeRefRel = std::unordered_map<NodeRef, NodeRefSet>;
using NodeRefMap = NodeRefRel;

NodeRefMap& operator+=(NodeRefMap& a, const NodeRefMap& b);

using NodeRefBitset = util::natural_set<NodeRef>;
using NodeRefBitmap = std::unordered_map<NodeRef, NodeRefBitset>;
using NodeRefPair = std::pair<NodeRef, NodeRef>;

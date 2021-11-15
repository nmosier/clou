#pragma once

#include <vector>
#include <unordered_set>
#include <optional>
#include <unordered_map>

#include "util/container.h"

template <typename Key>
class natural_set;

using NodeRef = std::size_t;
using NodeRefVec = std::vector<NodeRef>;
using NodeRefSet = std::unordered_set<NodeRef>;
using NodeRefRel = std::unordered_map<NodeRef, NodeRefSet>;
using NodeRefMap = NodeRefRel;

NodeRefMap& operator+=(NodeRefMap& a, const NodeRefMap& b);

using NodeRefBitset = util::natural_set<NodeRef>;
using NodeRefBitmap = std::unordered_map<NodeRef, NodeRefBitset>;
using NodeRefPair = std::pair<NodeRef, NodeRef>;

using NodeRefOpt = std::optional<NodeRef>;

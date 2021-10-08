#include <vector>
#include <unordered_set>
#include <unordered_map>

using NodeRef = std::size_t;
using NodeRefVec = std::vector<NodeRef>;
using NodeRefSet = std::unordered_set<NodeRef>;
using NodeRefRel = std::unordered_map<NodeRef, NodeRefSet>;
using NodeRefMap = NodeRefRel;

NodeRefMap& operator+=(NodeRefMap& a, const NodeRefMap& b);

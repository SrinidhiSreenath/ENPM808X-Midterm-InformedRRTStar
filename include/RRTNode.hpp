#include <memory>
#include <vector>

class RRTNode {
 public:
  RRTNode::RRTNode() {}
  RRTNode::~RRTNode() {}

  std::vector<double> state;
  std::shared_ptr<RRTNode> parent;
  double costToCome;

  void setState(const double &x, const double &y);
  void setParent(const std::shared_ptr<RRTNode> &parentObject);
};
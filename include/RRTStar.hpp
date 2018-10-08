#include "include/RRT.hpp"

class RRTStar : public RRT {
 private:
  const double rewireRange_ = 2.0;
  std::vector<std::shared_ptr<RRTNode>> rewireNodes_;

 public:
  void rewireRRTree(RRTNode &rewireNode,
                    const std::shared_ptr<RRTNode> &rewiredParent);
};
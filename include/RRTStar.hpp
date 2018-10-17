#include "RRT.hpp"

class RRTStar : public RRT {
 private:
  std::pair<size_t, double> plannerParams_ = getPlannerParameters();
  size_t maxIterations_ = plannerParams_.first;
  std::vector<RRTNode> RRTStarTree = getRRTree();
  const double rewireRange_ = 2.0 * plannerParams_.second;
  std::vector<std::shared_ptr<RRTNode>> rewireNodes_;
  std::shared_ptr<RRTNode> goalNodePtr;

 public:
  RRTStar();

  ~RRTStar();

  std::pair<size_t, double> getPlannerParams();

  void setStartAndGoal(const std::vector<double> &start,
                       const std::vector<double> &goal);

  std::shared_ptr<RRTNode> findClosestTreeNode(
      const std::vector<double> &randomNode);

  std::shared_ptr<RRTNode> rewireRRTree(
      const std::vector<double> &newNode,
      std::shared_ptr<RRTNode> &currentParent);

  void appendRRTree(const std::vector<double> &validNode,
                    const std::shared_ptr<RRTNode> &validNodeParent);

  std::vector<std::pair<double, double>> getPlannerPath();

  std::shared_ptr<RRTNode> getGoalNodePtr();

  std::vector<RRTNode> getRRTStarTree();

  void runPlanner();

  void resetPlanner();
};

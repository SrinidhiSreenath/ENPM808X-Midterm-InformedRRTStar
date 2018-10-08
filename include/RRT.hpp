#include "include/RRTNode.hpp"
#include "include/map.hpp"

class RRT {
 private:
  const size_t maxIterations_ = 1000;
  const double steerParameter_ = 1.0;

  std::vector<double> startNode_;
  std::vector<double> goalNode_;
  std::vector<std::pair<double, double>> plannerPath_;

  Map map_;

 public:
  RRT::RRT() {}
  RRT::~RRT() {}

  std::vector<RRTNode> RRTree;

  void setMap(const std::vector<std::pair<double, double>> &boundary,
              const std::vector<std::vector<double>> &obstacles);
  void setStartAndGoal(const std::vector<double> &start,
                       const std::vector<double> &goal);
  std::vector<double> generateRandomNode();
  double getEuclideanDistance(const std::vector<double> &node,
                              const RRTNode &treeNode);
  RRTNode findClosestTreeNode(const std::vector<double> &randomNode);
  std::vector<double> generateNewNode(const std::vector<double> &randomNode,
                                      const RRTNode &treeNode);
  bool isValidNode(std::vector<double> &newNode, const RRTNode &treeNode);
  void appendRRTree(std::vector<double> &validNode,
                    const std::shared_ptr<RRTNode> &validNodeParent);
  bool isGoalReached(std::vector<double> &newNode);
  std::vector<std::pair<double, double>> getPlannerPath();
  void runPlanner();
  void resetPlanner();
};
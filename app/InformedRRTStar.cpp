// Class header file
#include "InformedRRTStar.hpp"

// CPP Headers
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

InformedRRTStar::InformedRRTStar() {
  std::cout << "Initializing Informed RRTStar planner!" << std::endl;
}

InformedRRTStar::~InformedRRTStar() {
  std::cout << "Finished Informed RRTStar plan!" << std::endl;
}

std::vector<double> InformedRRTStar::generateHeuristicNode() {
  std::vector<double> heuristicNode;

  int heuristic = std::sqrt((cmax_ * cmax_ - 0.75 * cmin_ * cmin_));
  heuristicNode.push_back(startNode_[0] + rand() % heuristic);
  heuristicNode.push_back(startNode_[1] + rand() % heuristic);

  return heuristicNode;
}

void InformedRRTStar::runPlanner() {
  auto randomNode = generateRandomNode();
  bool reachedGoal = false;
  std::pair<double, double> goalNodePt =
      std::make_pair(goalNode_[0], goalNode_[1]);
  size_t iterations = 0;
  bool check = true;

  while (iterations < maxIterations_) {
    if (!reachedGoal) {
      iterations++;
      auto closestTreeNode = findClosestTreeNode(randomNode);
      auto newNode = generateNewNode(randomNode, closestTreeNode);

      auto treeNodePts = closestTreeNode->getState();
      std::pair<double, double> treeNode =
          std::make_pair(treeNodePts[0], treeNodePts[1]);
      std::pair<double, double> newNodePt =
          std::make_pair(newNode[0], newNode[1]);

      if (map_.isValidNode(treeNode, newNodePt)) {
        appendRRTree(newNode, closestTreeNode);
      }
      randomNode = generateRandomNode();
      std::vector<double> leafNode = RRTStarTree.back().getState();
      std::pair<double, double> leafNodePt =
          std::make_pair(leafNode[0], leafNode[1]);

      reachedGoal =
          isGoalReached(leafNode) && map_.isValidNode(leafNodePt, goalNodePt);
    } else {
      if (check) {
        std::shared_ptr<RRTNode> goalParentPtr =
            std::make_shared<RRTNode>(RRTStarTree.back());
        appendRRTree(goalNode_, goalParentPtr);
        check = false;
        std::cout << "Goal Found!" << std::endl;
        cmin_ = getEuclideanDistance(startNode_, goalNode_);
      }

      iterations++;
      auto closestTreeNode = findClosestTreeNode(randomNode);
      auto newNode = generateNewNode(randomNode, closestTreeNode);

      auto treeNodePts = closestTreeNode->getState();
      std::pair<double, double> treeNode =
          std::make_pair(treeNodePts[0], treeNodePts[1]);
      std::pair<double, double> newNodePt =
          std::make_pair(newNode[0], newNode[1]);

      auto rewiredParent = rewireRRTree(newNode, closestTreeNode);

      if (map_.isValidNode(treeNode, newNodePt)) {
        appendRRTree(newNode, rewiredParent);
      }
      auto waypoints = getPlannerPath();
      cmax_ = 0.0;
      for (size_t iter = 0; iter < waypoints.size() - 1; iter++) {
        cmax_ +=
            std::hypot((waypoints[iter + 1].first - waypoints[iter].first),
                       (waypoints[iter + 1].second - waypoints[iter].second));
      }
      randomNode = generateHeuristicNode();
    }
  }

  if (!reachedGoal && iterations >= maxIterations_) {
    resetPlanner();
    std::cout << "RRTStar Planner could not find a path" << std::endl;
  }
}

void InformedRRTStar::resetPlanner() {
  RRTStarTree.clear();
  startNode_.clear();
  goalNode_.clear();
  map_.resetMap();
}

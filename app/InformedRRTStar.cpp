/****************************************************************************
 * MIT License
 * Copyright (c) 2018 Srinidhi Sreenath
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

/**
 *  @file    InformedRRTStar.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/16/2018
 *  @version 1.0
 *
 *  @brief RRT STar Class definition for the planner
 *
 *  @section DESCRIPTION
 *
 *  Source file for class Informed RRT Star. The class is used to implement the
 * faster convergence of the optimized version of the Rapidly-exploring Random
 * Tree algorithm  (i-RRT*) to find a path from a start point to a given
 * goal point in a pre-defined environment. Once a path is found from RRT
 * algorithm, the i-RRT* samples nodes only in the heuristic region around the
 * path to minimize the cost to come i.e cost to arrive at a given node from
 * start node with a path in the tree.
 *
 */

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
  //  heuristic region is the region around start and goal node defined by an
  //  ellipse engulfing the path. Here I have approximated a bit by taking the
  //  square engulfing the ellipse.
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
      // Instead of sampling a random node in the environment, the sampling
      // takes place in the heuristic region. This accelerates the convergence
      // of the algorithm.
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

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
 *  @file    RRTStar.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/15/2018
 *  @version 1.0
 *
 *  @brief RRT STar Class definition for the planner
 *
 *  @section DESCRIPTION
 *
 *  Source file for class RRT Star. The class is used to implement the optimized
 *  version of the Rapidly-exploring Random Tree algorithm  (RRT*) to find a
 *  path from a start point to a given goal point in a pre-defined environment.
 *  Once a path is found from RRT algorithm, the RRT* samples nodes and
 *  minimizes the cost to come i.e cost to arrive at a given node from start
 *  node with a path in the tree. The algorithm runs for the max number of
 *  iterations and does not terminate as soon as the goal is found.
 *
 */
// Class header file
#include "RRTStar.hpp"

// CPP Headers
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

RRTStar::RRTStar() {
  std::cout << "Initializing RRTStar planner!" << std::endl;
}

RRTStar::~RRTStar() { std::cout << "Finished RRTStar plan!" << std::endl; }

std::pair<size_t, double> RRTStar::getPlannerParams() { return plannerParams_; }

void RRTStar::setStartAndGoal(const std::vector<double> &start,
                              const std::vector<double> &goal) {
  startNode_ = start;
  goalNode_ = goal;

  //  Create an RRTNode object for the start point
  RRTNode startNode;
  startNode.setState(startNode_[0], startNode_[1]);

  // Append the start node to the empty tree. This makes the start node the root
  // of the tree
  RRTStarTree.push_back(startNode);
}

std::shared_ptr<RRTNode> RRTStar::findClosestTreeNode(
    const std::vector<double> &randomNode) {
  double minDist = std::numeric_limits<double>::max();
  std::shared_ptr<RRTNode> closestNodePtr;
  // Search the entire tree to find the closest node to the random
  // node
  for (auto &treeNode : RRTStarTree) {
    auto nodeState = treeNode.getState();
    double dist = getEuclideanDistance(randomNode, nodeState);

    if (dist < minDist) {
      closestNodePtr = std::make_shared<RRTNode>(treeNode);
      minDist = dist;
    }
  }
  return closestNodePtr;
}

std::shared_ptr<RRTNode> RRTStar::rewireRRTree(
    const std::vector<double> &newNode,
    std::shared_ptr<RRTNode> &currentParent) {
  rewireNodes_.clear();
  std::shared_ptr<RRTNode> rewireNodePtr;

  for (auto &treeNode : RRTStarTree) {
    auto nodeState = treeNode.getState();
    double dist = getEuclideanDistance(newNode, nodeState);

    //  check for tree nodes within the rewire range of the current node
    if (dist < rewireRange_) {
      rewireNodePtr = std::make_shared<RRTNode>(treeNode);
      rewireNodes_.push_back(rewireNodePtr);
    }
  }

  auto parentCost = currentParent->getCostToCome();
  double dist = getEuclideanDistance(newNode, currentParent->getState());
  double currentNodeCost = parentCost + dist;

  double newCost = std::numeric_limits<double>::max();

  for (const auto &rnd : rewireNodes_) {
    double rewireParentCost = rnd->getCostToCome();
    dist = getEuclideanDistance(newNode, currentParent->getState());
    newCost = rewireParentCost + dist;

    //  If another node has a better cost to come for the node, update the
    //  parent of the current node
    if (newCost < currentNodeCost) {
      currentParent = rnd;
    }
  }
  return currentParent;
}

void RRTStar::appendRRTree(const std::vector<double> &validNode,
                           const std::shared_ptr<RRTNode> &validNodeParent) {
  // create a RRT node object for the new valid node to be added to the tree
  RRTNode newNode;
  // set the state of the new node
  newNode.setState(validNode[0], validNode[1]);
  // set the parent node of the current node
  newNode.setParent(validNodeParent);
  // set the cost of the node
  auto parentCost = validNodeParent->getCostToCome();
  double dist = getEuclideanDistance(validNode, validNodeParent->getState());
  double currentNodeCost = parentCost + dist;
  newNode.setCostToCome(currentNodeCost);

  //  set the goal node pointer
  if (validNode[0] == goalNode_[0] && validNode[1] == goalNode_[1]) {
    goalNodePtr = std::make_shared<RRTNode>(newNode);
  }

  // append the current tree with the new node
  RRTStarTree.push_back(newNode);
}

std::vector<std::pair<double, double>> RRTStar::getPlannerPath() {
  std::vector<std::pair<double, double>> plannedPath;
  // last element of the RRTree should be the goal node.
  auto node = *goalNodePtr;
  auto nodePoints = node.getState();
  plannedPath.push_back(std::make_pair(nodePoints[0], nodePoints[1]));
  auto nodeParent = node.getParent();

  while (nodePoints[0] != startNode_[0] && nodePoints[1] != startNode_[1] &&
         nodeParent != nullptr) {
    nodePoints = nodeParent->getState();
    plannedPath.push_back(std::make_pair(nodePoints[0], nodePoints[1]));
    nodeParent = nodeParent->getParent();
  }

  std::reverse(plannedPath.begin(), plannedPath.end());
  return plannedPath;
}

std::shared_ptr<RRTNode> RRTStar::getGoalNodePtr() { return goalNodePtr; }

std::vector<RRTNode> RRTStar::getRRTStarTree() { return RRTStarTree; }

void RRTStar::runPlanner() {
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
      //  goal is found
      if (check) {
        std::shared_ptr<RRTNode> goalParentPtr =
            std::make_shared<RRTNode>(RRTStarTree.back());
        appendRRTree(goalNode_, goalParentPtr);
        check = false;
        std::cout << "Goal Found!" << std::endl;
      }
      //  following optimization code runs until the iterations cap is met
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
      randomNode = generateRandomNode();
    }
  }

  if (!reachedGoal && iterations >= maxIterations_) {
    resetPlanner();
    std::cout << "RRTStar Planner could not find a path" << std::endl;
  }
}

void RRTStar::resetPlanner() {
  RRTStarTree.clear();
  startNode_.clear();
  goalNode_.clear();
  map_.resetMap();
}

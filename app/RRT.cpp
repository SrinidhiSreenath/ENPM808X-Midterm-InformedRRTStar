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
 *  @file    RRT.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/14/2018
 *  @version 1.0
 *
 *  @brief RRT Class definition for the planner
 *
 *  @section DESCRIPTION
 *
 *  Source file for class RRT. The class is used to implement the
 *  Rapidly-exploring Random Tree algorithm to find a path from a start point to
 *  a given goal point in a pre-defined environment. The algorithm starts by
 *  sampling random nodes in the environment and expanding a tree in the
 *  environment with the start node as root node. The tree expansion takes place
 *  until the goal is reached or for a set number of iterations. Once the goal
 *  is found, a path is found by traversing upwards in the tree from goal node
 *  until the root node i.e start node.
 *
 */
// Class header file
#include "RRT.hpp"

// CPP Headers
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

RRT::RRT() { std::cout << "Initializing RRT planner!" << std::endl; }

RRT::~RRT() { std::cout << "Finished RRT plan!" << std::endl; }

std::pair<size_t, double> RRT::getPlannerParameters() {
  std::pair<size_t, double> plannerParams =
      std::make_pair(maxIterations_, driveParameter_);
  return plannerParams;
}

void RRT::setMap(const std::vector<std::pair<double, double>> &boundary,
                 const std::vector<std::vector<double>> &obstacles) {
  // Set boundary of map
  map_.setWorkspaceBoundary(boundary);

  // Append the map with obstacles
  for (const auto &obs : obstacles) {
    map_.addObstacle(obs);
  }
}

void RRT::setStartAndGoal(const std::vector<double> &start,
                          const std::vector<double> &goal) {
  startNode_ = start;
  goalNode_ = goal;

  //  Create an RRTNode object for the start point
  RRTNode startNode;
  startNode.setState(startNode_[0], startNode_[1]);

  // Append the start node to the empty tree. This makes the start node the root
  // of the tree
  RRTree.push_back(startNode);
}

std::vector<double> RRT::generateRandomNode() {
  std::vector<double> randPoint;
  // Generate random x and y values within the boundary of the
  // workspace
  int range = std::round((map_.boundaryXlimits[1] - map_.boundaryXlimits[0]));
  randPoint.push_back(map_.boundaryXlimits[0] + rand() % range);
  randPoint.push_back(map_.boundaryYlimits[0] + rand() % range);

  return randPoint;
}

double RRT::getEuclideanDistance(const std::vector<double> &node,
                                 const std::vector<double> &treeNode) {
  return std::hypot(treeNode[0] - node[0], treeNode[1] - node[1]);
}

std::shared_ptr<RRTNode> RRT::findClosestTreeNode(
    const std::vector<double> &randomNode) {
  double minDist = std::numeric_limits<double>::max();
  std::shared_ptr<RRTNode> closestNodePtr;
  // Search the entire tree to find the closest node to the random
  // node
  for (auto &treeNode : RRTree) {
    auto nodeState = treeNode.getState();
    double dist = getEuclideanDistance(randomNode, nodeState);

    if (dist < minDist) {
      closestNodePtr = std::make_shared<RRTNode>(treeNode);
    }
  }
  return closestNodePtr;
}

std::vector<double> RRT::generateNewNode(
    const std::vector<double> &randomNode,
    const std::shared_ptr<RRTNode> &closestTreeNode) {
  // get the tree node state
  auto treeNodePoints = closestTreeNode->getState();
  // define vector from tree node point to random node point.
  std::vector<double> v = {randomNode[0] - treeNodePoints[0],
                           randomNode[1] - treeNodePoints[1]};
  // normalize the vector
  auto magnitude = getEuclideanDistance(randomNode, treeNodePoints);
  std::vector<double> u = {v[0] / magnitude, v[1] / magnitude};

  // define newNode at a steering distance from tree node.
  std::vector<double> newNode = {treeNodePoints[0] + driveParameter_ * u[0],
                                 treeNodePoints[1] + driveParameter_ * u[1]};

  return newNode;
}

void RRT::appendRRTree(const std::vector<double> &validNode,
                       const std::shared_ptr<RRTNode> &validNodeParent) {
  // create a RRT node object for the new valid node to be added to the tree
  RRTNode newNode;
  // set the state of the new node
  newNode.setState(validNode[0], validNode[1]);
  // set the parent node of the current node
  newNode.setParent(validNodeParent);

  // append the current tree with the new node
  RRTree.push_back(newNode);
}

bool RRT::isGoalReached(const std::vector<double> &newNode) {
  auto dist = getEuclideanDistance(newNode, goalNode_);
  if (dist <= driveParameter_) {
    return true;
  }
  return false;
}

std::vector<std::pair<double, double>> RRT::getPlannerPath() {
  std::vector<std::pair<double, double>> plannedPath;
  // last element of the RRTree should be the goal node.
  auto node = RRTree.back();
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

std::vector<RRTNode> RRT::getRRTree() { return RRTree; }

void RRT::runPlanner() {
  auto randomNode = generateRandomNode();
  bool reachedGoal = false;
  std::pair<double, double> goalNodePt =
      std::make_pair(goalNode_[0], goalNode_[1]);
  size_t iterations = 0;

  while (!reachedGoal && iterations < maxIterations_) {
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
    std::vector<double> leafNode = RRTree.back().getState();
    std::pair<double, double> leafNodePt =
        std::make_pair(leafNode[0], leafNode[1]);

    reachedGoal =
        isGoalReached(leafNode) && map_.isValidNode(leafNodePt, goalNodePt);
  }
  // Append goal point to the tree
  if (iterations < maxIterations_) {
    std::shared_ptr<RRTNode> goalParentPtr =
        std::make_shared<RRTNode>(RRTree.back());
    appendRRTree(goalNode_, goalParentPtr);
  } else {
    resetPlanner();
    std::cout << "RRT Planner could not find a path" << std::endl;
  }
}

void RRT::resetPlanner() {
  RRTree.clear();
  startNode_.clear();
  goalNode_.clear();
  plannerPath_.clear();
  map_.resetMap();
}

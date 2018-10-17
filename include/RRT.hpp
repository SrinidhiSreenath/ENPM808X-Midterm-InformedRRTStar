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
 *  @file    RRT.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/11/2018
 *  @version 1.0
 *
 *  @brief RRT class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class RRT which implements the Rapidly-exploring Random Tree
 *  algorithm to find a path given a map.
 *
 */

#ifndef INCLUDE_RRT_HPP_
#define INCLUDE_RRT_HPP_

#include "RRTNode.hpp"
#include "map.hpp"

/**
 *  @brief Class RRT
 *  The following class RRT implements the Rapidly-exploring random tree to find
 *  a path from start to goal in an environment
 */
class RRT {
 private:
  const size_t maxIterations_ = 1000;  ///< parameter that defines maximum
                                       ///< iterations the algorithm should run
  const double driveParameter_ =
      3.0;  ///< a parameter of the RRT algorithm that defines the maximum
            ///< distance the random tree grows by one node a time

  std::vector<std::pair<double, double>>
      plannerPath_;  ///< a vector of pair points [x,y] describing the path the
                     ///< algorithm finds
  std::vector<RRTNode>
      RRTree;  ///< vector of object of class RRTNode used to store the tree
               ///< datastructure the algorithm generates

 public:
  std::vector<double>
      startNode_;  ///< variable to hold the starting node for the algorithm
  std::vector<double> goalNode_;  ///< variable to hold the goal node where the
                                  ///< algorithm terminates
  Map map_;  ///< object of class Map to hold the information regarding the
             ///< environment
  /**
   *   @brief  Default constructor for RRT. Prints basic information regarding
   *           planner.
   *
   *   @param  none
   *   @return void
   */
  RRT();

  /**
   *   @brief  Destructor for RRT
   *
   *   @param  none
   *   @return void
   */
  ~RRT();

  /**
   *   @brief  function to return planner parameters
   *
   *   @param none
   *   @return pair of values (unsigned int, double) defining the max iterations
   *           and drive paramater respectively
   */
  std::pair<size_t, double> getPlannerParameters();

  /**
   *   @brief  setter function to define the environment
   *
   *   @param  boundary is a vector of paired points [x,y] defining the
   * boundary vertices. obstacles is a vector of obstacle container which in
   * turn is a vector of double points [x1,y1,x2,y2...xN,yN] defining the
   *           boundary vertices of an obstacle in the environment.
   *   @return void
   */
  void setMap(const std::vector<std::pair<double, double>> &boundary,
              const std::vector<std::vector<double>> &obstacles);

  /**
   *   @brief  setter function to set start and goal point for the RRT algorithm
   *
   *   @param  start defines the start point as a vector [x,y] of double
   *           elements
   *           goal defines the goal point as a vector [x,y] of double elements
   *   @return void
   */
  virtual void setStartAndGoal(const std::vector<double> &start,
                               const std::vector<double> &goal);

  /**
   *   @brief  samples a random point [x,y] in the map environment
   *
   *   @param  none
   *   @return vector [x,y] as double elements
   */
  std::vector<double> generateRandomNode();

  /**
   *   @brief  calculates the euclidean distance between a point
   *           and a RRTree node
   *
   *   @param  node is a random point in the environment as a vector of double
   *           points
   *           treeNode is a vector of double type representing a node in the
   *           RRTree
   *   @return the euclidean distance between the random node and the tree node
   *           as double element
   */
  double getEuclideanDistance(const std::vector<double> &node,
                              const std::vector<double> &treeNode);

  /**
   *   @brief  finds the closest node in the tree to a given randomly sampled
   *           node in the environment
   *
   *   @param  randomNode is the randomly sampled node in the environment as a
   *           vector of double elements
   *   @return A smart pointer (shared_ptr) pointing to the object of the
   *           closest tree node
   */
  virtual std::shared_ptr<RRTNode> findClosestTreeNode(
      const std::vector<double> &randomNode);

  /**
   *   @brief  generates a new node that is reachable from the closest tree
   *           node to the randomly sampled node, at a drive/steer distance
   *
   *   @param  randomNode is the randomly sampled node in the environment as a
   *           vector of double elements [x,y]
   *           closestTreeNode is a pointer to the object of class RRT
   *           representing the closest node in the tree to the random node
   *   @return a vector of double elements representing the new node [x,y]
   *           drivable from the tree node
   */
  std::vector<double> generateNewNode(
      const std::vector<double> &randomNode,
      const std::shared_ptr<RRTNode> &closestTreeNode);

  /**
   *   @brief  appends the RRTree in the workspace with the new valid node
   *
   *   @param  validNode is the newly generated node at a drivable distance to
   *           the current tree in the environment as a vector of double
   *           elements [x,y]
   *           validNodeParent is the pointer to the object of class RRT
   *           pointing to the parent of the validNode
   *   @return void
   */
  virtual void appendRRTree(const std::vector<double> &validNode,
                            const std::shared_ptr<RRTNode> &validNodeParent);

  /**
   *   @brief  checks if the goal node is within drivable distance from the
   *           current tree in the environment
   *
   *   @param  newNode is the latest node added to the RRTree as a vector of
   *           double elements [x,y]
   *   @return true if goal is within drivable range, false otherwise
   */
  bool isGoalReached(const std::vector<double> &newNode);

  /**
   *   @brief  returns a path found by the RRT algorithm as a set of waypoints
   *
   *   @param  none
   *   @return vector of pair of double elements [x,y] representing the set of
   *           waypoints
   */
  virtual std::vector<std::pair<double, double>> getPlannerPath();

  /**
   *   @brief  returns the tree resulting from the RRT algorithm
   *
   *   @param  none
   *   @return vector of nodes i.e objects of RRTNode class
   */
  std::vector<RRTNode> getRRTree();

  /**
   *   @brief  starts the execution of the RRT planning algorithm
   *
   *   @param  none
   *   @return void
   */
  virtual void runPlanner();

  /**
   *   @brief  resets the planner by clearing all member varibles of RRT class
   *           planner object
   *
   *   @param  none
   *   @return void
   */
  virtual void resetPlanner();
};
#endif  //  INCLUDE_RRT_HPP_

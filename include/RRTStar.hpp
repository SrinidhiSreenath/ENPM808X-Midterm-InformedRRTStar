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
 *  @file    RRTStar.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/14/2018
 *  @version 1.0
 *
 *  @brief RRT Star class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class RRT Star which implements the Optimized
 *  Rapidly-exploring Random Tree algorithm to find a path given a map.
 *
 */

#ifndef INCLUDE_RRTSTAR_HPP_
#define INCLUDE_RRTSTAR_HPP_

// Parent class header file
#include "RRT.hpp"

/**
 *  @brief Class RRT, derived from class RRT
 *  The following class RRT Star optimizes the path found by RRT algorithm using
 *  cost to come to rewire the path and the tree nodes as well.
 */
class RRTStar : public RRT {
 protected:
  const double rewireRange_ =
      2.0 * driveParameter_;  ///<  Range around a node to check for
                              ///<  possible new parent to rewire to

  std::shared_ptr<RRTNode> goalNodePtr;  ///<  Pointer to the object of class
                                         ///<  RRTNode denoting the goal node

  bool reachedGoal = false;  ///<  Boolean variable to check if goal is reached

 public:
  /**
   *   @brief  Default constructor for RRT Star. Prints basic information
   * regarding planner.
   *
   *   @param  none
   *   @return void
   */
  RRTStar();

  /**
   *   @brief  Destructor for RRT Star
   *
   *   @param  none
   *   @return void
   */
  ~RRTStar();

  /**
   *   @brief  finds out if a better parent exists based on cummulative cost to
   *           come (cost to arrive a the node from start node) for a given node
   *
   *   @param  newNode is the latest node added to the tree as a
   *           vector of double elements
   *           currentParent is the pointer to parent object of newNode
   *   @return A smart pointer (shared_ptr) pointing to the object of the
   *           parent node which minimizes the cost to come for the newNode
   */
  std::shared_ptr<RRTNode> rewireRRTree(
      const std::vector<double> &newNode,
      std::shared_ptr<RRTNode> &currentParent);

  /**
   *   @brief  appends the RRTStarTree in the workspace with the new valid node
   *
   *   @param  validNode is the newly generated node at a drivable distance to
   *           the current tree in the environment as a vector of double
   *           elements [x,y]
   *           validNodeParent is the pointer to the object of class RRT
   *           pointing to the parent of the validNode
   *   @return void
   */
  void appendRRTree(const std::vector<double> &validNode,
                    const std::shared_ptr<RRTNode> &validNodeParent);

  /**
   *   @brief  returns a path found by the RRT Star algorithm as a set of
   *           waypoints
   *
   *   @param  none
   *   @return vector of pair of double elements [x,y] representing the set of
   *           waypoints
   */
  std::vector<std::pair<double, double>> getPlannerPath();

  /**
   *   @brief  returns a pointer to the object of goal node
   *
   *   @param  none
   *   @return a pointer pointing to the object of class RRTNode containing the
   *           goal node
   */
  std::shared_ptr<RRTNode> getGoalNodePtr();

  /**
   *   @brief  starts the execution of the RRT Star planning algorithm
   *
   *   @param  none
   *   @return void
   */
  virtual void runPlanner();

  /**
   *   @brief  resets the planner by clearing all member varibles of RRT
   *           Star class planner object
   *
   *   @param  none
   *   @return void
   */
  virtual void resetPlanner();
};
#endif  // INCLUDE_RRTSTAR_HPP_

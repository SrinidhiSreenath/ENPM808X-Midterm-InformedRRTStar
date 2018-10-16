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
 *  @file    RRTNode.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/10/2018
 *  @version 1.0
 *
 *  @brief RRTNode class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class RRTNode.
 *
 */

#ifndef INCLUDE_RRTNODE_HPP_
#define INCLUDE_RRTNODE_HPP_

// CPP Headers
#include <memory>
#include <vector>

/**
 *  @brief Class RRTNode
 *  The following class RRTNode is used to hold information of a node in the
 *  tree formed by RRT algorithm.
 */
class RRTNode {
 private:
  std::vector<double>
      state;  ///< variable to hold the state of the node i.e [x,y] coordinates
  std::shared_ptr<RRTNode> parent =
      nullptr;  ///< pointer to the parent node object. A null pointer by
                ///< default
  double costToCome = 0.0;  ///< cost variable describing the cost to arrive at
                            ///< this node from the start node
 public:
  /**
   *   @brief  Default constructor for RRTNode
   *
   *   @param  none
   *   @return void
   */
  RRTNode();

  /**
   *   @brief  Destructor for RRTNode
   *
   *   @param  none
   *   @return void
   */
  ~RRTNode();

  /**
   *   @brief  setter function to set state [x,y] of the node
   *
   *   @param  x and y are cartesian coordinates in double type
   *   @return void
   */
  void setState(const double &x, const double &y);

  /**
   *   @brief  function to return state [x,y] of the node
   *
   *   @param  none
   *   @return vector of double type points containing state [x,y]
   */
  std::vector<double> getState();

  /**
   *   @brief  setter function to set parent node of the current node
   *
   *   @param parentObject is the pointer to the object of the parent node
   *   @return void
   */
  void setParent(const std::shared_ptr<RRTNode> &parentObject);

  /**
   *   @brief  function to return current node's parent node object pointer
   *
   *   @param none
   *   @return a shared_ptr to the object of the parent node class
   */
  std::shared_ptr<RRTNode> getParent();

  /**
   *   @brief  setter function to set cost to come for the node
   *
   *   @param  cost to travel to the current node from start node as double
   *   @return void
   */
  void setCostToCome(const double &cost);

  /**
   *   @brief  function to return cost to come of the node
   *
   *   @param  none
   *   @return costToCome of the node as double
   */
  double getCostToCome();
};

#endif  // INCLUDE_RRTNODE_HPP_

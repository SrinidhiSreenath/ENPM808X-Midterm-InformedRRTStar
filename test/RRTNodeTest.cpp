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
 *  @file    RRTNodeTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/10/2018
 *  @version 1.0
 *
 *  @brief Unit tests for class RRTNode
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test member functions of class RRTNode.
 *
 */

#include <gtest/gtest.h>

// Class header file
#include "RRTNode.hpp"

/**
 *@brief Test case to check setting state of RRTNode

 *@param none
 *@return none
 */
TEST(RRTNodeSetStateTest, testSetStateOfRRTNode) {
  RRTNode newNode;
  newNode.setState(2.73, 2.68);
  auto state = newNode.getState();

  ASSERT_EQ(state[0], 2.73);
  ASSERT_EQ(state[1], 2.68);
}

/**
 *@brief Test case to check setting parent node for current node

 *@param none
 *@return none
 */
TEST(RRTNodeSetParentNodeTest, testSetParentOfCurrentRRTNode) {
  std::vector<RRTNode> RRTree;
  RRTNode newNode;
  RRTNode parentNode;

  RRTree.push_back(newNode);

  std::vector<double> parentState{3, 4};
  parentNode.setState(parentState[0], parentState[1]);
  RRTree.push_back(parentNode);

  std::shared_ptr<RRTNode> parentNodePtr = std::make_shared<RRTNode>(RRTree[1]);
  RRTree[0].setParent(parentNodePtr);
  ASSERT_EQ(RRTree[0].getParent(), parentNodePtr);

  auto parentPtr = RRTree[0].getParent();
  ASSERT_EQ(parentPtr->getState(), parentState);
}

/**
 *@brief Test case to check changing parent node for current node

 *@param none
 *@return none
 */
TEST(RRTNodeChangeParentNodeTest, testSetNewParentForCurrentRRTNode) {
  std::vector<RRTNode> RRTree;
  RRTNode newNode;
  RRTNode parentNode;
  RRTNode newParentNode;

  RRTree.push_back(newNode);

  std::vector<double> parentState{8.96, 7.26};
  parentNode.setState(parentState[0], parentState[1]);
  RRTree.push_back(parentNode);

  std::vector<double> newParentState{5.45, 6.69};
  newParentNode.setState(newParentState[0], newParentState[1]);
  RRTree.push_back(newParentNode);

  std::shared_ptr<RRTNode> parentNodePtr = std::make_shared<RRTNode>(RRTree[1]);
  RRTree[0].setParent(parentNodePtr);
  ASSERT_EQ(RRTree[0].getParent(), parentNodePtr);
  auto parentPtr = RRTree[0].getParent();
  ASSERT_EQ(parentPtr->getState(), parentState);

  // Change the parent of the current node
  std::shared_ptr<RRTNode> newParentNodePtr =
      std::make_shared<RRTNode>(RRTree[2]);
  RRTree[0].setParent(newParentNodePtr);
  ASSERT_EQ(RRTree[0].getParent(), newParentNodePtr);
  auto newParentPtr = RRTree[0].getParent();
  ASSERT_EQ(newParentPtr->getState(), newParentState);
}

/**
 *@brief Test case to check setting cost to come for current node

 *@param none
 *@return none
 */
TEST(RRTNodeSetCostToCome, testCostToComeForCurrentNode) {
  RRTNode newNode;
  newNode.setCostToCome(23.69);

  ASSERT_EQ(newNode.getCostToCome(), 23.69);
}

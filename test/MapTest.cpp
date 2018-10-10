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
 *  @file    MapTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/8/2018
 *  @version 1.0
 *
 *  @brief Unit tests for class Map
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test member functions of class Map.
 *
 */

#include <gtest/gtest.h>

// Class header file
#include "map.hpp"

Map sampleMap;  ///< class object

std::vector<std::pair<double, double>>
    boundary;  ///< variable to hold boundary vertices
std::vector<std::vector<double>>
    obstacles;  ///< variable to hold obstacles and their vertices

/**
 *@brief Test case to check setting workspace boundary

 *@param none
 *@return none
 */
TEST(MapInitializationTest1, testSetMapWorkspaceBoundary) {
  // Define boundary vertices of workspace
  boundary.push_back(std::make_pair(0.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 100.0));
  boundary.push_back(std::make_pair(0.0, 100.0));

  sampleMap.setWorkspaceBoundary(boundary);

  ASSERT_EQ(sampleMap.workspaceBoundary, boundary);
}

/**
 *@brief Test case to check setting obstacles

 *@param none
 *@return none
 */
TEST(MapInitializationTest2, testAddObsatclesToMap) {
  // Define custom set of obstacles
  obstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  obstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  obstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  obstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});
  obstacles.push_back({25.0, 25.0, 35.0, 25.0, 35.0, 40.0, 25.0, 40.0});
  obstacles.push_back({65.0, 25.0, 75.0, 25.0, 75.0, 40.0, 65.0, 40.0});
  obstacles.push_back({40.0, 40.0, 60.0, 40.0, 60.0, 50.0, 40.0, 50.0});

  for (const auto &obs : obstacles) {
    sampleMap.addObstacle(obs);
  }

  for (size_t it = 0; it < sampleMap.obstacleList.size(); it++) {
    ASSERT_EQ(sampleMap.obstacleList[it], obstacles[it]);
  }
}

/**
 *@brief Test case to check isOutofMap member function output based on input
         node

 *@param none
 *@return none
 */
TEST(MapValidNodeTest1, testValidNodeBasedOnBoundary) {
  boundary.push_back(std::make_pair(0.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 100.0));
  boundary.push_back(std::make_pair(0.0, 100.0));

  sampleMap.setWorkspaceBoundary(boundary);

  std::pair<double, double> nodeInsideBoundary{
      99.9, 99.9};  ///< sample node within boundary
  std::pair<double, double> nodeOutsideBoundary{
      100.1, 100.1};  ///< sample node outside boundary

  ASSERT_EQ(sampleMap.isOutofMap(nodeInsideBoundary), false);
  ASSERT_EQ(sampleMap.isOutofMap(nodeOutsideBoundary), true);
}

/**
 *@brief Test case to check isValidNode member function output based on input
         node

 *@param none
 *@return none
 */
TEST(MapValidNodeTest2, testValidConnectionToTreeNode) {
  boundary.push_back(std::make_pair(0.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 100.0));
  boundary.push_back(std::make_pair(0.0, 100.0));

  sampleMap.setWorkspaceBoundary(boundary);

  obstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  obstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  obstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  obstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});
  obstacles.push_back({25.0, 25.0, 35.0, 25.0, 35.0, 40.0, 25.0, 40.0});
  obstacles.push_back({65.0, 25.0, 75.0, 25.0, 75.0, 40.0, 65.0, 40.0});
  obstacles.push_back({40.0, 40.0, 60.0, 40.0, 60.0, 50.0, 40.0, 50.0});

  for (const auto &obs : obstacles) {
    sampleMap.addObstacle(obs);
  }
  // valid node in free space
  std::pair<double, double> treeNode1{10.0, 50.0};
  std::pair<double, double> newNode1{20.0, 70.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode1, newNode1), true);

  // invalid node with new node out of boundary
  std::pair<double, double> treeNode2{70.0, 90.0};
  std::pair<double, double> newNode2{85.0, 110.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode2, newNode2), false);

  // invalid node with connection between new node and tree node colliding with
  // obstacle space
  std::pair<double, double> treeNode3{45.0, 35.0};
  std::pair<double, double> newNode3{55.0, 55.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode3, newNode3), false);

  // invalid node with new node inside an obstacle
  std::pair<double, double> treeNode4{85.0, 10.0};
  std::pair<double, double> newNode4{75.0, 7.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode4, newNode4), false);

  // invalid node with connection between new node and tree node colliding with
  // obstacle space
  std::pair<double, double> treeNode5{12.0, 88.0};
  std::pair<double, double> newNode5{27.0, 95.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode5, newNode5), false);

  // invalid node with new node on egde of an obstacle
  std::pair<double, double> treeNode6{83.0, 65.0};
  std::pair<double, double> newNode6{93.0, 69.0};
  ASSERT_EQ(sampleMap.isValidNode(treeNode6, newNode6), false);
}

/**
 *@brief Test case to check resetting map

 *@param none
 *@return none
 */
TEST(MapResetTest, testMapResetFunction) {
  boundary.push_back(std::make_pair(0.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 0.0));
  boundary.push_back(std::make_pair(100.0, 100.0));
  boundary.push_back(std::make_pair(0.0, 100.0));

  sampleMap.setWorkspaceBoundary(boundary);

  obstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  obstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  obstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  obstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});
  obstacles.push_back({25.0, 25.0, 35.0, 25.0, 35.0, 40.0, 25.0, 40.0});
  obstacles.push_back({65.0, 25.0, 75.0, 25.0, 75.0, 40.0, 65.0, 40.0});
  obstacles.push_back({40.0, 40.0, 60.0, 40.0, 60.0, 50.0, 40.0, 50.0});

  for (const auto &obs : obstacles) {
    sampleMap.addObstacle(obs);
  }

  sampleMap.resetMap();

  size_t size = 0;  ///< unsigned variable to compare sizes

  ASSERT_EQ(sampleMap.obstacleList.size(), size);
  ASSERT_EQ(sampleMap.workspaceBoundary.size(), size);
}
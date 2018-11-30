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
 *  @file    MockRRTTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    11/29/2018
 *  @version 1.0
 *
 *  @brief Unit tests for class RRT mocking Map class
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test the class members of RRT planner.
 *
 */
// Google Test Headers
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// CPP Headers
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

// Class header files
#include "RRT.hpp"
#include "RRTNode.hpp"
#include "map.hpp"

using ::testing::_;
using ::testing::AtLeast;

class MockMap : public Map {
 public:
  MOCK_METHOD0(resetMap, void());
  MOCK_METHOD1(setWorkspaceBoundary,
               void(const std::vector<std::pair<double, double>> &boundary));
  MOCK_METHOD1(addObstacle, void(const std::vector<double> &obstacle));
};

class MockMapTest : public ::testing::Test {
 public:
  MockMap myMap;              ///< Initialize mock map class
  RRT testPlan = RRT(myMap);  ///< Initialize RRT planner

  std::vector<std::pair<double, double>>
      testBoundary;  ///< variable to hold boundary vertices
  std::vector<std::vector<double>>
      testObstacles;  ///< variable to hold obstacles and their vertices

  void SetUp() {
    // Define the environment
    testBoundary.push_back(std::make_pair(0.0, 0.0));
    testBoundary.push_back(std::make_pair(100.0, 0.0));
    testBoundary.push_back(std::make_pair(100.0, 100.0));
    testBoundary.push_back(std::make_pair(0.0, 100.0));

    testObstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
    testObstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
    testObstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
    testObstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});
  }

  void TearDown() {
    // Clear the boundary and obstacle
    testBoundary.clear();
    testObstacles.clear();
  }
};

TEST_F(MockMapTest, testSettingEnvironment) {
  // Expect the environment to be set up by making calls to the mocked class
  EXPECT_CALL(myMap, setWorkspaceBoundary(testBoundary)).Times(AtLeast(1));

  // Four obstacles are sent, hence the addObstacle method is expected to be
  // called four times.
  EXPECT_CALL(myMap, addObstacle(_)).Times(AtLeast(testObstacles.size()));

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);
}

TEST_F(MockMapTest, testResettingPlanner) {
  // Expect the environment to be set up by making calls to the mocked class
  EXPECT_CALL(myMap, setWorkspaceBoundary(testBoundary)).Times(AtLeast(1));

  // Four obstacles are sent, hence the addObstacle method is expected to be
  // called four times.
  EXPECT_CALL(myMap, addObstacle(_)).Times(AtLeast(testObstacles.size()));

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);

  // Expect the call for map to be reset
  EXPECT_CALL(myMap, resetMap()).Times(AtLeast(1));

  // Reset the planner
  testPlan.resetPlanner();
}

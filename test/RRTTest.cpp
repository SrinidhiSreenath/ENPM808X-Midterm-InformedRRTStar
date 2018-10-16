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
 *  @file    RRTTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/13/2018
 *  @version 1.0
 *
 *  @brief Unit tests for class RRT
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test the class members of RRT planner.
 *
 */

#include <gtest/gtest.h>
#include <cmath>

// Class header file
#include "RRT.hpp"

RRT testPlan;  ///< Initialize Test planner

std::vector<std::pair<double, double>>
    testBoundary;  ///< variable to hold boundary vertices
std::vector<std::vector<double>>
    testObstacles;  ///< variable to hold obstacles and their vertices

/**
 *@brief Test case to check the RRT planner. The test case checks whether the
         path found is correct based on the following three conditions:

         1. The last waypoint of the path should be the goal node. (i.e the goal
            node is a leaf node in the RRTree)
         2. Tracing the parent of the goal node in the tree, the path should end
            at the start node i.e root node
         3. The distance between each waypoint should be within drivable range
            of the planner

 *@param none
 *@return none
 */
TEST(RRTValidPathTest, testValidityOfPathGivenByTheRRTPlanner) {
  // Define the environment
  testBoundary.push_back(std::make_pair(0.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 100.0));
  testBoundary.push_back(std::make_pair(0.0, 100.0));

  testObstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  testObstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  testObstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  testObstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);

  // Set start and goal point
  std::vector<double> start = {1.0, 1.0};
  std::vector<double> goal = {50.0, 51.0};

  testPlan.setStartAndGoal(start, goal);

  // Execute the planner
  testPlan.runPlanner();

  // Get the planner path
  auto waypoints = testPlan.getPlannerPath();

  // Get the tree generated by the planner
  auto tree = testPlan.getRRTree();

  // 1. The last waypoint of the path should be the goal node. (i.e the goalnode
  // is a leaf node in the RRTree)
  std::vector<double> planGoal = {waypoints.back().first,
                                  waypoints.back().second};
  ASSERT_EQ(goal, planGoal);

  auto treeLeaf = tree.back().getState();
  ASSERT_EQ(planGoal, treeLeaf);

  // 2. Tracing the parent of the goal node in the tree, the path should end at
  // the start node i.e root node
  auto RRTree = testPlan.getRRTree();
  auto parent = RRTree.back().getParent();
  std::vector<double> rootNode;
  while (parent != nullptr) {
    rootNode = parent->getState();
    parent = parent->getParent();
  }
  ASSERT_EQ(start, rootNode);

  // 3. The distance between each waypoint should be within drivable range of
  // the planner
  double driveParam = testPlan.getPlannerParameters().second;
  for (size_t it = 0; it < waypoints.size() - 1; it++) {
    auto child = waypoints[it];
    auto parent = waypoints[it + 1];

    auto dist =
        std::hypot(parent.first - child.first, parent.second - child.second);

    ASSERT_LE(dist, driveParam + 0.0001);
  }
}

TEST(RRTPlannerResetTest, testRRTResetFunction) {
  // Define the environment
  testBoundary.push_back(std::make_pair(0.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 100.0));
  testBoundary.push_back(std::make_pair(0.0, 100.0));

  testObstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  testObstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  testObstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  testObstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);

  // Set start and goal point
  std::vector<double> start = {1.0, 1.0};
  std::vector<double> goal = {50.0, 51.0};

  testPlan.setStartAndGoal(start, goal);

  // Execute the planner
  testPlan.runPlanner();

  // Reset the planner
  testPlan.resetPlanner();

  // Assert segmentation fault when trying to access the RRTree or the planned
  // path
  ASSERT_EXIT((testPlan.getRRTree(), exit(0)), ::testing::ExitedWithCode(0),
              ".*");
  ASSERT_EXIT((testPlan.getPlannerPath(), exit(0)),
              ::testing::KilledBySignal(SIGSEGV), ".*");
}

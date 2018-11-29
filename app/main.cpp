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
 *  @file    main.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/16/2018
 *  @version 1.0
 *
 *  @brief main source file to implement the planner
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement the Informed RRT* planner. A custom environment is
 *  defined and the planner is run. The final path is visualised using the
 *  matplotlibcpp library
 *
 *
 */
#include <cmath>
#include <iostream>

#include "InformedRRTStar.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

/**
 *   @brief  function that plots the planned path and the environment
 *
 *   @param  boundary is the boundary of the environment as vector of paired
 *           double points
 *           obstacles is the vector of vector containing the obstacle
 *           coordinates each as [x1,y1,x2,y2,...xN,yN]
 *           waypoints are the points along the path as a vector of paired
 *           double elements
 *   @return void
 */
void plotPlan(const std::vector<std::pair<double, double>> &boundary,
              const std::vector<std::vector<double>> &obstacles,
              const std::vector<std::pair<double, double>> &waypoints,
              std::vector<RRTNode> &tree) {
  double maxX = std::numeric_limits<double>::min();
  double minX = std::numeric_limits<double>::max();
  double maxY = std::numeric_limits<double>::min();
  double minY = std::numeric_limits<double>::max();

  for (const auto &bound : boundary) {
    if (bound.first > maxX) {
      maxX = bound.first;
    }
    if (bound.first < minX) {
      minX = bound.first;
    }
    if (bound.second > maxY) {
      maxY = bound.second;
    }
    if (bound.second < minY) {
      minY = bound.second;
    }
  }
  plt::title("Informed RRT Star Path Planner");

  plt::ylim(minY, maxY);
  plt::xlim(minX, maxX);

  std::map<std::string, std::string> keywords;
  keywords["alpha"] = "0.5";
  keywords["color"] = "green";
  keywords["hatch"] = "-";

  for (const auto &obs : obstacles) {
    std::vector<double> x, y;
    for (size_t iter = 0; iter < obs.size(); iter += 2) {
      x.push_back(obs[iter]);
      y.push_back(obs[iter + 1]);
    }
    x.push_back(obs[0]);
    y.push_back(obs[1]);
    plt::plot(x, y, "g");

    std::vector<double> fillx = {obs[0], obs[2]};
    std::vector<double> filly1 = {obs[1], obs[7]};
    std::vector<double> filly2 = {obs[3], obs[5]};

    plt::fill_between(fillx, filly1, filly2, keywords);
  }

  for (auto &node : tree) {
    std::vector<double> x;
    std::vector<double> y;

    x.push_back(node.getState()[0]);
    y.push_back(node.getState()[1]);

    plt::plot(x, y, "b-o");

    if (node.getParent() != nullptr) {
      auto parent = node.getParent();
      x.push_back(parent->getState()[0]);
      y.push_back(parent->getState()[1]);

      plt::plot(x, y, "c");
    }
  }

  std::vector<double> wayPointX, wayPointY;

  for (const auto &pt : waypoints) {
    wayPointX.push_back(pt.first);
    wayPointY.push_back(pt.second);
    plt::plot(wayPointX, wayPointY, "r");
    plt::pause(0.001);
  }

  plt::show();
}

/**
 *   @brief  main function implementing the planner
 *
 *   @param  none
 *   @return integer 0 indication successful execution
 */
int main() {
  RRTStar testPlan;  ///< Initialize Test planner

  std::vector<std::pair<double, double>>
      testBoundary;  ///< variable to hold boundary vertices
  std::vector<std::vector<double>>
      testObstacles;  ///< variable to hold obstacles and their vertices

  testBoundary.push_back(std::make_pair(0.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 0.0));
  testBoundary.push_back(std::make_pair(100.0, 100.0));
  testBoundary.push_back(std::make_pair(0.0, 100.0));

  testObstacles.push_back({0.0, 80.0, 10.0, 80.0, 10.0, 90.0, 0.0, 90.0});
  testObstacles.push_back({0.0, 90.0, 25.0, 90.0, 25.0, 100.0, 0.0, 100.0});
  testObstacles.push_back({20.0, 0.0, 80.0, 0.0, 80.0, 15.0, 20.0, 15.0});
  testObstacles.push_back({93.0, 40.0, 100.0, 40.0, 100.0, 90.0, 93.0, 90.0});
  testObstacles.push_back({25.0, 25.0, 35.0, 25.0, 35.0, 40.0, 25.0, 40.0});
  testObstacles.push_back({65.0, 25.0, 75.0, 25.0, 75.0, 40.0, 65.0, 40.0});
  testObstacles.push_back({40.0, 40.0, 60.0, 40.0, 60.0, 50.0, 40.0, 50.0});

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);

  // Set start and goal point
  std::vector<double> start = {1.0, 1.0};
  std::vector<double> goal = {50.0, 55.0};

  testPlan.setStartAndGoal(start, goal);

  // Execute the planner
  testPlan.runPlanner();

  // Get the tree
  auto tree = testPlan.getRRTStarTree();

  // Get the planner path
  auto waypoints = testPlan.getPlannerPath();

  // Visualize the environment and the path
  plotPlan(testBoundary, testObstacles, waypoints, tree);

  return 0;
}

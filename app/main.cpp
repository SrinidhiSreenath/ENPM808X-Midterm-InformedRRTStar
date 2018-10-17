#include <cmath>
#include <iostream>

#include "InformedRRTStar.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void plotPlan(const std::vector<std::pair<double, double>> &boundary,
              const std::vector<std::vector<double>> &obstacles,
              const std::vector<std::pair<double, double>> &waypoints) {
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
      maxY = bound.first;
    }
    if (bound.second < minY) {
      minY = bound.first;
    }
  }
  plt::title("Informed RRT Star Path Planner");

  plt::ylim(minY, maxY);
  plt::xlim(minX, maxX);

  std::vector<double> x, y;

  for (const auto &obs : obstacles) {
    x.clear();
    y.clear();
    for (size_t iter = 0; iter < obs.size(); iter += 2) {
      x.push_back(obs[iter]);
      y.push_back(obs[iter + 1]);
    }
    x.push_back(obs[0]);
    y.push_back(obs[1]);
    plt::plot(x, y, "r");
  }

  std::vector<double> wayPointX, wayPointY;

  for (const auto &pt : waypoints) {
    wayPointX.push_back(pt.first);
    wayPointY.push_back(pt.second);
    plt::plot(wayPointX, wayPointY, "b");
    plt::pause(0.001);
  }

  plt::show();
}

int main() {
  InformedRRTStar testPlan;  ///< Initialize Test planner

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
  // testObstacles.push_back({25.0, 25.0, 35.0, 25.0, 35.0, 40.0, 25.0, 40.0});
  // testObstacles.push_back({65.0, 25.0, 75.0, 25.0, 75.0, 40.0, 65.0, 40.0});
  // testObstacles.push_back({40.0, 40.0, 60.0, 40.0, 60.0, 50.0, 40.0, 50.0});

  // Set the map for the planner
  testPlan.setMap(testBoundary, testObstacles);

  // Set start and goal point
  std::vector<double> start = {1.0, 1.0};
  std::vector<double> goal = {50.0, 55.0};

  testPlan.setStartAndGoal(start, goal);

  // Execute the planner
  testPlan.runPlanner();

  // Get the planner path
  auto waypoints = testPlan.getPlannerPath();

  // for (const auto &pt : waypoints) {
  //   std::cout << pt.first << " , " << pt.second << std::endl;
  // }

  plotPlan(testBoundary, testObstacles, waypoints);

  return 0;
}

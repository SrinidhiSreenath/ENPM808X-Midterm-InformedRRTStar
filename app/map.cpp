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
 *  @file    map.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/13/2018
 *  @version 1.0
 *
 *  @brief Map class definition
 *
 *  @section DESCRIPTION
 *
 *  Source file for class Map. The class supports defining a Map to support path
 *  planning algorithms. The class stores the map information and has the
 *  functionality to check if a given node in the workspace can be added to
 *  existing tree by checking possible collision with the obstacle space.
 *
 */
// Class header file
#include "map.hpp"

// CPP Headers
#include <algorithm>
#include <iostream>
#include <limits>

Map::Map() {}

Map::~Map() {}

void Map::setWorkspaceBoundary(
    const std::vector<std::pair<double, double>> &boundary) {
  workspaceBoundary = boundary;

  double maxX = std::numeric_limits<double>::min();
  double minX = std::numeric_limits<double>::max();
  double maxY = std::numeric_limits<double>::min();
  double minY = std::numeric_limits<double>::max();

  for (const auto &bound : workspaceBoundary) {
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

  boundaryXlimits = {minX, maxX};
  boundaryYlimits = {minY, maxY};
}

void Map::addObstacle(const std::vector<double> &obstacle) {
  obstacleList.push_back(obstacle);
}

bool Map::onSegment(const std::pair<double, double> &firstPoint,
                    const std::pair<double, double> &secondPoint,
                    const std::pair<double, double> &thirdPoint) {
  if (secondPoint.first <= std::max(firstPoint.first, thirdPoint.first) &&
      secondPoint.first >= std::min(firstPoint.first, thirdPoint.first) &&
      secondPoint.second <= std::max(firstPoint.second, thirdPoint.second) &&
      secondPoint.second >= std::min(firstPoint.second, thirdPoint.second)) {
    return true;
  } else {
    return false;
  }
}

int Map::getOrientation(const std::pair<double, double> &firstPoint,
                        const std::pair<double, double> &secondPoint,
                        const std::pair<double, double> &thirdPoint) {
  int orientation = (secondPoint.second - firstPoint.second) *
                        (thirdPoint.first - secondPoint.first) -
                    (secondPoint.first - firstPoint.first) *
                        (thirdPoint.second - secondPoint.second);

  if (orientation == 0) {
    return 0;
  } else {
    return (orientation > 0) ? 1 : 2;
  }
}

bool Map::isIntersect(const std::pair<double, double> &treeNode,
                      const std::pair<double, double> &newNode,
                      const std::pair<double, double> &firstVertex,
                      const std::pair<double, double> &secondVertex) {
  int firstOrient = getOrientation(treeNode, newNode, firstVertex);
  int secondOrient = getOrientation(treeNode, newNode, secondVertex);
  int thirdOrient = getOrientation(firstVertex, secondVertex, treeNode);
  int fourthOrient = getOrientation(firstVertex, secondVertex, newNode);

  // General case
  if (firstOrient != secondOrient && thirdOrient != fourthOrient) return true;

  // Special Cases
  // treeNode, newNode and firstVertex are colinear and firstVertex lies on
  // segment connecting new node to tree
  if (firstOrient == 0 && onSegment(treeNode, firstVertex, newNode))
    return true;

  // treeNode, newNode and secondVertex are colinear and secondVertex lies on
  // segment connecting new node to tree
  if (secondOrient == 0 && onSegment(treeNode, secondVertex, newNode))
    return true;

  // firstVertex, secondVertex and treeNode are colinear and treeNode lies on
  // obstacle edge
  if (thirdOrient == 0 && onSegment(firstVertex, treeNode, secondVertex))
    return true;

  // firstVertex, secondVertex and newNode are colinear and newNode lies on
  // obstacle edge
  if (fourthOrient == 0 && onSegment(firstVertex, newNode, secondVertex))
    return true;

  return false;  // Doesn't fall in any of the above cases
}

bool Map::isOutofMap(const std::pair<double, double> &node) {
  if (node.first > boundaryXlimits[1] || node.first < boundaryXlimits[0] ||
      node.second > boundaryYlimits[1] || node.second < boundaryYlimits[0]) {
    return true;
  } else {
    return false;
  }
}

bool Map::isValidNode(const std::pair<double, double> &treeNode,
                      const std::pair<double, double> &newNode) {
  if (isOutofMap(newNode)) {
    return false;
  }
  for (const auto &obstacle : obstacleList) {
    for (size_t i = 0; i < obstacle.size(); i += 2) {
      std::pair<double, double> firstVertex, secondVertex;
      if (i + 2 < obstacle.size()) {
        firstVertex = std::make_pair(obstacle[i], obstacle[i + 1]);
        secondVertex = std::make_pair(obstacle[i + 2], obstacle[i + 3]);
      } else {
        firstVertex = std::make_pair(obstacle[i], obstacle[i + 1]);
        secondVertex = std::make_pair(obstacle[0], obstacle[1]);
      }

      if (isIntersect(treeNode, newNode, firstVertex, secondVertex)) {
        return false;
      }
    }
  }
  return true;
}

void Map::resetMap() {
  obstacleList.clear();
  workspaceBoundary.clear();
  boundaryXlimits.clear();
  boundaryYlimits.clear();
}

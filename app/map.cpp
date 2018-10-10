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
 *  @file    map.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/8/2018
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
  return;
}

void Map::addObstacle(const std::vector<double> &obstacle) { return; }

bool Map::onSegment(const std::pair<double, double> &firstPoint,
                    const std::pair<double, double> &secondPoint,
                    const std::pair<double, double> &thirdPoint) {
  return 0;
}

int Map::getOrientation(const std::pair<double, double> &firstPoint,
                        const std::pair<double, double> &secondPoint,
                        const std::pair<double, double> &thirdPoint) {
  return 0;
}

bool Map::isIntersect(const std::pair<double, double> &treeNode,
                      const std::pair<double, double> &newNode,
                      const std::pair<double, double> &firstVertex,
                      const std::pair<double, double> &secondVertex) {
  return 0;
}

bool Map::isOutofMap(const std::pair<double, double> &node) { return 0; }

bool Map::isValidNode(const std::pair<double, double> &treeNode,
                      const std::pair<double, double> &newNode) {
  return 0;
}

void Map::resetMap() { return; }
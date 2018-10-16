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
 *  @date    10/7/2018
 *  @version 1.0
 *
 *  @brief Map class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Map.
 *
 */

#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

// CPP Headers
#include <utility>
#include <vector>

/**
 *  @brief Class Map
 *  The following class Map aids in storing the layout of the environment
 */
class Map {
 public:
  std::vector<std::vector<double>>
      obstacleList;  ///< Variable to hold list of obstacles. Each obstacle is
                     ///< defined as a vector of points in the order [x1, y1,
                     ///< x2, y2 ... xN, yN]
  std::vector<std::pair<double, double>>
      workspaceBoundary;  ///< Variable to hold workspace boundary as a vector
  ///< of paired points [x,y]
  std::vector<double> boundaryXlimits;  ///< Variable to hold min and max limits
                                        ///< of the workspace boundary on x axis
  std::vector<double> boundaryYlimits;  ///< Variable to hold min and max limits
                                        ///< of the workspace boundary on y axis

  /**
   *   @brief  Default constructor for Map
   *
   *   @param  none
   *   @return void
   */
  Map();

  /**
   *   @brief  Destructor for Map
   *
   *   @param  none
   *   @return void
   */
  ~Map();

  /**
   *   @brief  setter function to set workspace boundary
   *
   *   @param  boundary is a vector of paired points [x,y] defining the boundary
   *           vertices
   *   @return void
   */
  void setWorkspaceBoundary(
      const std::vector<std::pair<double, double>> &boundary);

  /**
   *   @brief  function to add obstacle to a Map
   *
   *   @param  obstacle is a vector of double points [x1,y1,x2,y2...xN,yN]
   *           defining the boundary vertices of an obstacle
   *   @return void
   */
  void addObstacle(const std::vector<double> &obstacle);

  /**
   *   @brief  checks if the second point lies on the line segment connecting
   *           first point and third point.
   *
   *   @param  firstPoint, secondPoint and thirdPoint are each pair of double
   *           [x,y] points in workspace
   *   @return true if second point lies on the line segment, false if it
   *           doesn't
   */
  bool onSegment(const std::pair<double, double> &firstPoint,
                 const std::pair<double, double> &secondPoint,
                 const std::pair<double, double> &thirdPoint);

  /**
   *   @brief  determines the oreintation i.e clockwise, anti-clockwise or
   *           collinearity of three ordered points.
   *
   *   @param  firstPoint, secondPoint and thirdPoint are each pair of
   *           double [x,y] points in workspace
   *   @return 0 if collinear, 1 if clockwise, 2 if anti-clockwise orientation
   *           as integer
   */
  int getOrientation(const std::pair<double, double> &firstPoint,
                     const std::pair<double, double> &secondPoint,
                     const std::pair<double, double> &thirdPoint);

  /**
   *   @brief  determines if the line segments of tree node and new node, and
   *           first vertex and second vertex intersect
   *
   *   @param  treeNode is the node in the tree as pair of double points [x,y]
   *           newNode is the node in the workspace as pair of double points
   *           [x,y]
   *           firstVertex, secondVertex are the vertices of an edge of an
   *           obstacle as pair of double points [x,y] each
   *   @return true if the line segments intersect, false if they don't
   */
  bool isIntersect(const std::pair<double, double> &treeNode,
                   const std::pair<double, double> &newNode,
                   const std::pair<double, double> &firstVertex,
                   const std::pair<double, double> &secondVertex);

  /**
   *   @brief  determines if the given node is out of map bounds
   *
   *   @param  node is the point [x,y] in workspace as pair of double
   *   @return true if node is out of boundary, false otherwise
   */
  bool isOutofMap(const std::pair<double, double> &node);

  /**
   *   @brief  determines if the given node can be added to the existing tree in
   *           the workspace without any conflicts
   *
   *   @param  treeNode is the node in the tree as pair of double points [x,y]
   *           newNode is the node in the workspace as pair of double points
   *           [x,y]
   *   @return true if the treeNode can be added to the tree, false otherwise
   */
  bool isValidNode(const std::pair<double, double> &treeNode,
                   const std::pair<double, double> &newNode);

  /**
   *   @brief  function to reset the map by clearing the member variables
   *
   *   @param  none
   *   @return void
   */
  void resetMap();
};

#endif  // INCLUDE_MAP_HPP_

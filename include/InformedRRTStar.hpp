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
 *  @file    RRTStar.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/15/2018
 *  @version 1.0
 *
 *  @brief Informed RRT Star class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Informed RRT Star which implements the Fastly
 *  converging Optimized Rapidly-exploring Random Tree algorithm to find a path
 *  given a map.
 *
 */
#ifndef INCLUDE_INFORMEDRRTSTAR_HPP_
#define INCLUDE_INFORMEDRRTSTAR_HPP_

//  Parent class header file
#include "RRTStar.hpp"

/**
 *  @brief Class Informed RRT Star, derived from class RRT Star
 *  The following class RRT Star converges faster to find the optimized path
 * found by RRT Star algorithm using a heuristic model.
 */
class InformedRRTStar : public RRTStar {
 private:
  double cmin_ =
      0.0;  ///<  heuristic parameter. Direct distance from start to goal node
  double cmax_ = 0.0;  ///<  heuristic parameter. Cost to come value for the
                       ///<  goal node i.e distance along the path found

 public:
  /**
   *   @brief  Default constructor for Informed RRT Star. Prints basic
   * information regarding planner.
   *
   *   @param  none
   *   @return void
   */
  InformedRRTStar();

  /**
   *   @brief  Destructor for Informed RRT Star
   *
   *   @param  none
   *   @return void
   */
  ~InformedRRTStar();

  /**
   *   @brief  samples a random point [x,y] in the heuristic region
   *
   *   @param  none
   *   @return vector [x,y] as double elements
   */
  std::vector<double> generateHeuristicNode();

  /**
   *   @brief  starts the execution of the Informed RRT Star planning algorithm
   *
   *   @param  none
   *   @return void
   */
  void runPlanner();

  /**
   *   @brief  resets the planner by clearing all member varibles of Informed
   *           RRT Star class planner object
   *
   *   @param  none
   *   @return void
   */
  void resetPlanner();
};
#endif  //  INCLUDE_INFORMEDRRTSTAR_HPP_

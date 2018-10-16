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
 *  @file    RRTNode.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    10/13/2018
 *  @version 1.0
 *
 *  @brief RRTNode class definition
 *
 *  @section DESCRIPTION
 *
 *  Source file for class RRTNode. The class is used to hold information
 *  regarding a node in the workspace. A node consists of state i.e cartesian
 *  coordinates [x,y] and its parent node in the tree.
 *
 */

#include "RRTNode.hpp"

RRTNode::RRTNode() {}

RRTNode::~RRTNode() {}

void RRTNode::setState(const double &x, const double &y) {
  state.push_back(x);
  state.push_back(y);
}

std::vector<double> RRTNode::getState() { return state; }

void RRTNode::setParent(const std::shared_ptr<RRTNode> &parentObject) {
  parent = parentObject;
}

std::shared_ptr<RRTNode> RRTNode::getParent() { return parent; }

void RRTNode::setCostToCome(const double &cost) { costToCome = cost; }

double RRTNode::getCostToCome() { return costToCome; }

# Informed RRT* Algorithm - C++ Implementation
[![Build Status](https://travis-ci.com/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar.svg?branch=GMock_Extra_Credit) [![Coverage Status](https://coveralls.io/repos/github/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar/badge.svg?branch=GMock_Extra_Credit)](https://coveralls.io/github/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar?branch=GMock_Extra_Credit)[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar/blob/master/LICENSE)
---

## Overview

This repository consists of the C++ implementation of the Informed RRT* algorithm for autonomous navigation. Robotic vacuum
cleaners are becoming a household thing in our modern world and has a huge market potential. One of the main components of
the robotic vacuum cleaners is it's ability to plan a path from a given start to end point in an environment. This software
is developed for a fictional cleaning robot Xoomba for ACME Robotics, a fictional organization.

Sampling-based algorithms represent the configuration space with a roadmap of sampled configurations. These algorithms work 
well for high-dimensional configuration spaces, because unlike combinatorial algorithms, their running time is not 
(explicitly) exponentially dependent on the dimension of configurations.

Rapidly-Exploring Random Trees (RRTs) is a planning technique for single-query problems which uses an incremental tree 
expansion from randomly drawn samples to grow towards previously unsearchable areas in the map space. Optimal RRTS (RRT*), 
an improvement on RRTs, extend the problem to finding the optimal solution in the planning space that is computed 
asymptotically. Informed RRT* improves the convergence speed of RRT* by introducing a heuristic, similar to the way in which 
A* improves upon Dijkstra’s algorithm.

## GMock Tests
In the original project, Informed RRT Star planner was developed. The main project consisted of 5 classes -
1. Class Map to define the environment
2. Class RRTNode to describe a node in the environment.
3. Class RRT which implements the RRT planner. Map and RRTNode are a part of this class.
4. RRTStar planner derived from RRT class.
5. InformedRRTSTar planner derived from RRTStar planner.

As RRT class uses Map class, Map class is mocked and then the RRT class is tested. To mock the class Map, all of its methods are made virtual as well as the destructor. A new test MockRRTTest is developed in which a mock of Map class and a few of its' methods is created. This is subsequently tested with the initialization of RRT class which recieves environment information i.e boundary and obstacle information and creates an instance of class Map to store the information. 

In the gmock test, RRT class is initialized with the mocked map class, which is derived from the original Map class, and the mocked map class initialization is tested. 
 
To instructions to run the unit tests are given in the Install section.

## Result

The following image is the result of the RRT algorithm on execution.
![alt text](https://github.com/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar/blob/GMock_Extra_Credit/results/RRTplan.png)

## Solo Iterative Process (SIP)
Solo Iterative Process (SIP) is used in the development of the project. Test Driven Development appoach is used to comply with the short development cycle. The planning and development of the project is done in four sprints. 

[Product backlog, Iteration backlogs, Work log and Sprint Schedule](https://docs.google.com/spreadsheets/d/1fvrJKm83capWolsztqe_W-Q733m2EkzF_6xD1WWORhc/edit?usp=sharing).

## License
```
MIT License

Copyright (c) 2018 Srinidhi Sreenath

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
## Dependencies

1. [matplotlib-cpp](https://github.com/lava/matplotlib-cpp), a simple C++ plotting library, resemble the plotting API used by Matlab, is used to facilitate visualization.

2. Google Test framework for unit testing - gtest and gmock.

### Installation
matplotlib-cpp works by wrapping the popular python plotting library matplotlib. (matplotlib.org) This means you have to have a working python installation, including development headers. On Ubuntu:
```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
```
The C++-part of the library consists of the single header file matplotlibcpp.h which can be placed anywhere. Place this in the include directory of the project.

### CMake
Since the build system is CMake, the following is added to the CMakeLists.txt:
```
find_package(PythonLibs 2.7)
target_include_directories(myproject PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(myproject ${PYTHON_LIBRARIES})
```
where myproject is the project to be built.

## Standard install via command-line

Note: The following branch only implements the RRT planner as GMock test is written for Map class and tested with RRT class which uses the Map class.

```
git clone https://github.com/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar.git --branch GMock_Extra_Credit
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Doxygen Documentation
Doxygen Documentation generation steps:
```
cd <path to repository>
mkdir Doxygen
cd Doxygen
doxygen -g <config_file_name>
```
Open configuration file and update the following:
```
PROJECT_NAME = 'ENPM808X-Midterm-InformedRRTStar'
INPUT = ../InformedRRTStar ../include ../test
```
Run and generate the documents by executing the following:
```
doxygen <config_file_name>
```

## Working with Eclipse IDE ##

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/SrinidhiSreenath/ENPM808X-Midterm-InformedRRTStar.git
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of ENPM808X-Midterm-InformedRRTStar

```
cd ~/workspace
mkdir -p InformedRRTStar-eclipse
cd InformedRRTStar-eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../ENPM808X-Midterm-InformedRRTStar/
```

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> 
Select "InformedRRTStar-eclipse" directory created previously as root directory -> Finish

# Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.


## Build

To build the project, in Eclipse, unfold InformedRRTStar-eclipse project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.

## Run

1. In Eclipse, right click on the InformedRRTStar-eclipse in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. shell-app, cpp-test for unit testing)


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the boilerplate-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. shell-app).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

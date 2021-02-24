# Search Planner Simulator
This project showcases a "Search Planner" subsystem I developed in MATLAB for my third year individual design project at university. The Search Planner provides offline searching capabilities to a mobile robot via a complete coverage paradigm. This is achieved by performing Boustrophedon Cellular Decomposition on a complete map to divide it into obstacle free regions, and then planning rectilinear boustrophedon paths within these cells (this effectively reduces area coverage to a cell order planning problem).

The search path is communicated via an ordered set of waypoints. To test the effectiveness of the Search Planner, a mobile search robot simulator was developed using MATLAB's Mobile Robotics Simulation Toolbox. The overall project is best demonstrated via the simulations and details on how to set them up and run the are provided below.

## Prerequisites
MATLAB is required to run this project. I recommend using the latest version, MATLAB 2020b - the project has not been tested thoroughly on other versions of MATLAB.

The following toolboxes are dependencies, and so must be installed via MATLAB's add-on manager:
* Mobile Robotics Simulation Toolbox (https://uk.mathworks.com/matlabcentral/fileexchange/66586-mobile-robotics-simulation-toolbox)
* Statistics and Machine Learning Toolbox (https://uk.mathworks.com/products/statistics.html)

Please be aware that the aforementioned toolboxes will in turn have their own toolbox dependencies. Links have been provided for further information.

Simulations are planar and thus from my experience are not overly computationally complex. As a reference, I developed this project and ran simulations on a 2013 Macbook Air with an Intel Core i5 processor and 8GB of RAM.

## Setup
Clone the repository into a directory of your choice that is accessible to MATLAB. Open the directory in MATLAB's file explorer and the Search Planner Simulator should be ready to use.

## Running the Search Planner Simulator
The two scripts, "benchmark_tests" and "basic_functionality_test.m" showcase the project.

benchmark_test.m is the main script I used for benchmark testing and data collection. Please start with this script. When run, the user is prompted to select one of eight occupancy maps via the command window. After selection, three windows are displayed:
1. The planned search path trajectory within the selected map (animation)
2. The segmentation of the selected map into obstacle-free cells using Boustrophedon Cell Decomposition (static)
3. A simulation of a mobile robot following the generated search path (animation)

Search path progress is communicated via the command window. Once the simulation is finished (when the robot reaches the last waypoint), search performance metrics are stored in the "SearchTestSuite.m" object.

basic_functionality_test.m was developed to validate the search simulator and collection of search performance metrics. When run, a simulation window will display a mobile robot in a simple environment. The robot will perform a 'figure 8' trajectory to test basic movement, collide into an obstacle twice to test collision detection, and then head towards an OPI to test object detection.

## Project Reflection
As this was completed for a university project with a limited timeframe, focus was placed on robustness first. The project still requires search performance optimisation to reduce travel distance and search duration, and a number of methods are readily apparent (the cell ordering algorithm in particular can be improved). In fact, for my final year university project, I am trying to develop a new bio-inspired search and rescue algorithm that provides online performance and hopefully offers promising qualities to this interesting field.


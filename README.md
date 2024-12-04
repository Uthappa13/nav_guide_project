# Project NavGuide

[![CICD Workflow status](https://github.com/Uthappa13/nav_guide_project/actions/workflows/test.yml/badge.svg)](https://github.com/Uthappa13/nav_guide_project/actions/workflows/test.yml) [![codecov](https://codecov.io/gh/Uthappa13/nav_guide_project/branch/main/graph/badge.svg)](https://codecov.io/gh/Uthappa13/nav_guide_project) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Phase-3
### Project Summary
We are designing a swarm navigation robot system for warehouse applications at Acme Robotics, aimed at improving safety and efficiency in material handling operations. This system is vital in a warehouse environment where there are multiple tasks that need to be accomplished in parallel. The goal of the multi-robot system will be to navigate the warehouse environment successfully and reach their destinations.

### Personnel Info
The authors of this project are Rishie Raj and Uthappa Madettira, both graduate students in Robotics at the University of Maryland, College Park.

### AIP Workflow
This project was developed using the Agile Development Process (AIP) along with pair programming (with a driver and navigator), with a focus on test-driven development (TDD). The product backlog details have been linked below for Iteration-3. The link to the sprint meeting discussions for this sprint have also been linked below.

 - [Product Backlog](https://docs.google.com/spreadsheets/d/1E_nRD0vp5bYWbiwfghffEHXf7bQaf3OA0RdkEj6O_uE/edit?usp=drive_link)
 - [Sprint Meeting Plan and Review](https://drive.google.com/file/d/1K6vTpOrr-nsGCVGlvDfw2RAoYpGxLTmL/view?usp=sharing)

The latest (Phase 3) developed UML class and activity diagrams can be found in the UML/revised directory. The earlier devised UML diagrams as a part of Phase 0 are available in the UML/initial directory.

### Building the Code

In order to build, run the following codes once you are in the working directory
```bash
colcon build 
source install/setup.bash
```

### Building for Unit and Integration Tests

Run these commands to build for running the unit tests and integration tests.
```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

### Running the Demo

To run the demo example of four Turtlebot 3 Waffle Pi, run the commands below.
```bash
source install/setup.bash
ros2 launch my_controller gazebo.launch.py 
```
In another terminal, run the following to start the solver node and simulation.
```bash
source install/setup.bash
ros2 run my_controller robot_commander
```


### Running the Unit and Integration Tests

Execute the following commands to run the previosuly built unit and integration tests.
```bash
source install/setup.bash
colcon test
```


### Getting the Test Coverage Report for `my_controller`:

To get the test coverage for the ROS 2 package, run the commands listed below.
``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller/test_coverage/index.html
```

### Getting the Test Coverage Report for `my_model`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/my_model/test_coverage/index.html
# Project NavGuide

[![CICD Workflow status](https://github.com/Uthappa13/nav_guide_project/actions/workflows/test.yml/badge.svg)](https://github.com/Uthappa13/nav_guide_project/actions/workflows/test.yml) [![codecov](https://codecov.io/gh/Uthappa13/nav_guide_project/branch/main/graph/badge.svg)](https://codecov.io/gh/Uthappa13/nav_guide_project) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Phase-1
### Project Summary
We are designing a swarm navigation robot system for warehouse applications at Acme Robotics, aimed at improving safety and efficiency in material handling operations. This system is vital in a warehouse environment where there are multiple tasks that need to be accomplished in parallel. The goal of the multi-robot system will be to navigate the warehouse environment successfully and reach their destinations.

### Personnel Info
The authors of this project are Rishie Raj and Uthappa Madettira, both graduate students in Robotics at the University of Maryland, College Park.

### AIP Workflow
This project was developed using the Agile Development Process (AIP) along with pair programming (with a driver and navigator), with a focus on test-driven development (TDD). The product backlog details have been linked below for Iteration-1. The link to the sprint meeting discussions for this sprint have also been linked below.

 - [Product Backlog](https://docs.google.com/spreadsheets/d/1E_nRD0vp5bYWbiwfghffEHXf7bQaf3OA0RdkEj6O_uE/edit?usp=drive_link)
 - [Sprint Meeting Plan and Review](https://drive.google.com/file/d/1Y9RJL7VUDx96JyT7fjYdGnY3CoYmM8Pr/view?usp=drive_link)

The latest (Phase 1) developed UML class and activity diagrams can be found in the UML/revised directory. The earlier devised UML diagrams as a part of Phase 0 are available in the UML/initial directory.

### Building the Code

As we have only built the `my_model` package right now, in order to build the run the package, run the following codes once you are in the working directory
```bash
# Move to the package directory
  cd src/my_model/
# Create all the make files:
  cmake -S ./ -B build/
# Compile and build the code to the 'build' directory from scratch:
  cmake --build build/ --clean-first
# Clean the 'build' directory:
  cmake --build build/ --target clean
# Remove the 'build' directory to rebuild the project if necessary:
  rm -rf build/
```

## Build unit test and code coverage

```bash
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D COVERAGE=ON -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html
```

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.

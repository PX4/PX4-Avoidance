#!/bin/bash
catkin clean -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Coverage -DCATKIN_ENABLE_TESTING=True
catkin build avoidance local_planner global_planner safe_landing_planner
catkin build avoidance --no-deps --catkin-make-args avoidance-test_coverage
catkin build local_planner --no-deps --catkin-make-args local_planner-test_coverage
catkin build global_planner --no-deps --catkin-make-args global_planner-test_coverage
catkin build safe_landing_planner --no-deps --catkin-make-args safe_landing_planner-test_coverage
# disable branch coverage, since it isn't useful until the line coverage is better
# lcov -a ~/catkin_ws/build/avoidance/coverage.info -a  ~/catkin_ws/build/local_planner/coverage.info -a  ~/catkin_ws/build/global_planner/coverage.info -a  ~/catkin_ws/build/safe_landing_planner/coverage.info -o repo_total.info --rc lcov_branch_coverage=1
lcov -a ~/catkin_ws/build/avoidance/coverage.info -a  ~/catkin_ws/build/local_planner/coverage.info -a  ~/catkin_ws/build/global_planner/coverage.info -a  ~/catkin_ws/build/safe_landing_planner/coverage.info -o repo_total.info
sed -i "s/$(pwd | sed 's/\//\\\//g')\///g" repo_total.info

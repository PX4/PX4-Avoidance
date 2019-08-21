#!/bin/bash

# Generate the existing state machine diagrams again
python generate_flow_diagram.py ../avoidance/test/test_usm.cpp
python generate_flow_diagram.py ../safe_landing_planner/src/nodes/waypoint_generator.cpp
python generate_flow_diagram.py ../local_planner/src/nodes/waypoint_generator.cpp

# Print the diff with the remote branch (empty if no diff)
git --no-pager diff -U0 --color

# Check if there are changes, and failed
if ! git diff-index --quiet HEAD --; then echo "State machine diagram generation not up to date, run tools/check_state_machine_diagrams.sh"; exit 1; fi

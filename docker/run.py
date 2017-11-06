#!/bin/python3

import argparse
import subprocess

# Define arguments
parser = argparse.ArgumentParser()
parser.add_argument("planner", help="the planner to start (local | global)")
flavour = parser.add_mutually_exclusive_group()
flavour.add_argument("--sim", help="run in gazebo", action="store_true")
flavour.add_argument("--prod-debug", help="deploy in debug mode, allowing an external machine to connect to the ROS network through VPN (useful for plotting topics in rviz while running on the drone)", action="store_true")
flavour.add_argument("--prod-release", help="deploy in release mode", action="store_true")
args = parser.parse_args()

# Sanity check
if args.planner != "local" and args.planner != "global":
    print("Planner has to be \"local\" or \"global\"!")
    exit(1)

# Start module in desired mode
if args.prod_release:
    print("Deploying {}_planner in RELEASE mode".format(args.planner))
    subprocess.call("docker-compose -f ./components/components.yml build mavros", shell=True)
    subprocess.call("./{}_planner/{}-planner-prod/{}-planner-prod-release/run.sh".format(args.planner, args.planner, args.planner))
elif args.prod_debug:
    print("Deploying {}_planner in DEBUG mode".format(args.planner))
    subprocess.call("docker-compose -f ./components/components.yml build mavros", shell=True)
    subprocess.call("./{}_planner/{}-planner-prod/{}-planner-prod-debug/run.sh".format(args.planner, args.planner, args.planner))
else:
    print("Running simulation for planner {}".format(args.planner))
    subprocess.call("docker-compose -f ./components/components.yml build mavros sitl-avoidance-server", shell=True)
    subprocess.call("./{}_planner/{}-planner-dev/run.sh".format(args.planner, args.planner))

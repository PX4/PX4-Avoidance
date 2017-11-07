#!/bin/python3

import argparse
import os
import shutil
import subprocess

def main(args):
    checkArgs(args)
    runGlobalPrerequisites()
    runContainers()

def createArgsParser():
    parser = argparse.ArgumentParser()
    parser.add_argument("planner", help="the planner to start (local | global)")

    flavour = parser.add_mutually_exclusive_group()
    flavour.add_argument("--sim", help="run in gazebo", action="store_true")
    flavour.add_argument("--prod-debug", help="deploy in debug mode, allowing an external machine to connect to the ROS network through VPN (useful for plotting topics in rviz while running on the drone)", action="store_true")
    flavour.add_argument("--prod-release", help="deploy in release mode", action="store_true")

    return parser.parse_args()

def checkArgs(args):
    if args.planner != "local" and args.planner != "global":
        print("Planner has to be \"local\" or \"global\"!")
        exit(1)

def runGlobalPrerequisites():
    while isDockerComposeNotInstalled():
        installDockerCompose()

    subprocess.call("docker-compose -f ./components/components.yml build mavros", shell=True)

def isDockerComposeNotInstalled():
    return shutil.which("docker-compose") == ''

def installDockerCompose():
    print("You need to install docker-compose in order to run this script. From another terminal, follow these instructions:")

    if os.path.exists("/usr/local/bin"):
        installPath = "/usr/local/bin"
    else:
        installPath = "/usr/bin"

    sysname = os.uname().sysname
    machine = os.uname().machine
    remotePath = "https://github.com/docker/compose/releases/download/1.17.0/docker-compose-{}-{}".format(sysname, machine)
    curlCommand = "sudo curl -L {} -o {}/docker-compose".format(remotePath, installPath)

    print()
    print("    1. Run: $ {}".format(curlCommand))
    print("    2. Run: $ sudo chmod +x {}/docker-compose".format(installPath))
    print()

    input("When finished, press any key to continue...\n")

def runContainers():
    if args.prod_release:
        print("Deploying {}_planner in RELEASE mode".format(args.planner))
        subprocess.call("./{}_planner/{}-planner-prod/{}-planner-prod-release/run.sh".format(args.planner, args.planner, args.planner))
    elif args.prod_debug:
        print("Deploying {}_planner in DEBUG mode".format(args.planner))
        subprocess.call("./{}_planner/{}-planner-prod/{}-planner-prod-debug/run.sh".format(args.planner, args.planner, args.planner))
    else:
        print("Running simulation for planner {}".format(args.planner))
        subprocess.call("docker-compose -f ./components/components.yml build sitl-avoidance-server", shell=True)
        subprocess.call("./{}_planner/{}-planner-dev/run.sh".format(args.planner, args.planner))

if __name__ == '__main__':
    args = createArgsParser()
    main(args)

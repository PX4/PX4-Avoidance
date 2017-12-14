#!/bin/python3

import argparse
import os
import shutil
import subprocess

def main(args):
    checkArgs(args)
    runGlobalPrerequisites()
    workingDir = findWorkingDir(args)

    if args.run:
        buildContainers(args, workingDir)
        runContainers(args, workingDir)
    elif args.stop:
        stopContainers(workingDir)
    elif args.build:
        buildContainers(args, workingDir)
    elif args.rebuild:
        rebuildContainers(args, workingDir)
    else:
        runContainers(args, workingDir)

def createArgsParser():
    parser = argparse.ArgumentParser()
    parser.add_argument("planner", help="the planner to start (local | global)")

    addRunModes(parser)
    addRunCommands(parser)

    return parser.parse_args()

def addRunModes(parser):
    runMode = parser.add_mutually_exclusive_group()
    runMode.add_argument("--sim", help="run in gazebo", action="store_true")
    runMode.add_argument("--prod-debug", help="deploy in debug mode, allowing an external machine to connect to the ROS network through VPN (useful for plotting topics in rviz while running on the drone)", action="store_true")
    runMode.add_argument("--prod-release", help="deploy in release mode", action="store_true")

def addRunCommands(parser):
    runCommands = parser.add_mutually_exclusive_group()
    runCommands.add_argument("--run", help="build and run the containers", action="store_true")
    runCommands.add_argument("--run-only", help="run the containers (this is the default)", action="store_true")
    runCommands.add_argument("--stop", help="stop the containers, if running", action="store_true")
    runCommands.add_argument("--build", help="build the containers if necessary (internally running `docker-compose build`)", action="store_true")
    runCommands.add_argument("--rebuild", help="rebuild the containers from scratch (internally running `docker-compose build --no-cache`)", action="store_true")

def checkArgs(args):
    if args.planner != "local" and args.planner != "global":
        print("Planner has to be \"local\" or \"global\"!")
        exit(1)

def runGlobalPrerequisites():
    while isDockerComposeNotInstalled():
        installDockerCompose()

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

def findWorkingDir(args):
    if args.prod_release:
        return "./{}_planner/{}-planner-prod/{}-planner-prod-release".format(args.planner, args.planner, args.planner)
    elif args.prod_debug:
        return "./{}_planner/{}-planner-prod/{}-planner-prod-debug".format(args.planner, args.planner, args.planner)
    else:
        return "./{}_planner/{}-planner-dev".format(args.planner, args.planner)

def runContainers(args, workingDir):
    if args.prod_release:
        print("Deploying {}_planner in RELEASE mode".format(args.planner))
    elif args.prod_debug:
        print("Deploying {}_planner in DEBUG mode".format(args.planner))
    else:
        print("Running simulation for planner {}".format(args.planner))

    subprocess.call("{}/run.sh".format(workingDir), shell=True)

def stopContainers(workingDir):
    subprocess.call("docker-compose -f {}/docker-compose.yml down".format(workingDir), shell=True)

def buildContainers(args, workingDir):
    subprocess.call("docker-compose -f ./components/components.yml build mavros", shell=True)
    subprocess.call("docker-compose -f {}/docker-compose.yml build".format(workingDir), shell=True)

    if not args.prod_release and not args.prod_debug:
        subprocess.call("docker-compose -f ./components/components.yml build sitl-avoidance-server", shell=True)

def rebuildContainers(args, workingDir):
    subprocess.call("docker-compose -f ./components/components.yml build --no-cache mavros", shell=True)
    subprocess.call("docker-compose -f {}/docker-compose.yml build --no-cache".format(workingDir), shell=True)

    if not args.prod_release and not args.prod_debug:
        subprocess.call("docker-compose -f ./components/components.yml build --no-cache sitl-avoidance-server", shell=True)

if __name__ == '__main__':
    args = createArgsParser()
    main(args)

#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Analysis') {

      parallel {

        stage('Catkin build on ROS workspace') {
          agent {
            docker {
              image 'px4io/px4-dev-ros-melodic:2019-10-04'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'ls -l'
            sh '''#!/bin/bash -l
              echo $0;
              mkdir -p catkin_ws/src;
              cd catkin_ws;
              git -C ${WORKSPACE}/catkin_ws/src/Firmware submodule update --init --recursive --force Tools/sitl_gazebo;
              git clone --recursive ${WORKSPACE}/catkin_ws/src/Firmware/Tools/sitl_gazebo src/mavlink_sitl_gazebo;
              git -C ${WORKSPACE}/catkin_ws/src/Firmware fetch --tags;
              source /opt/ros/melodic/setup.bash;
              catkin init;
              wstool init src src/avoidance/dependencies.rosinstall
              wstool update -t src
              catkin build avoidance -j$(nproc) -l$(nproc);
            '''
          }
          post {
            always {
              sh 'rm -rf catkin_ws'
            }
            failure {
              archiveArtifacts(allowEmptyArchive: false, artifacts: '.ros/**/*.xml, .ros/**/*.log')
            }
          }
          options {
            checkoutToSubdirectory('catkin_ws/src/avoidance')
          }
        }
      } // prallel
    } // stage analysis
  } // stages

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
    GIT_AUTHOR_EMAIL = "bot@px4.io"
    GIT_AUTHOR_NAME = "PX4BuildBot"
    GIT_COMMITTER_EMAIL = "bot@px4.io"
    GIT_COMMITTER_NAME = "PX4BuildBot"
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '20', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
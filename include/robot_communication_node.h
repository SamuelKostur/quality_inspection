#include <ros/ros.h>
#include <quality_inspection/MovRobToScanPosAction.h>
#include <actionlib/server/simple_action_server.h>
#include <stdlib.h>
#include <tinyxml.h>
#include <stdio.h>
#include <string.h>
#include <csignal>

// Socket
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

typedef actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> Server;

class RobotCommunicationNode{
    public: 
    ros::NodeHandle n;
    actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> as;
    int sockfd_server;
    int sockfd_client;

    RobotCommunicationNode();
    void nodeExitFunc();
    int sendRobotCommand(double robot_pose_command [6]);
    int readRobotPose(quality_inspection::MovRobToScanPosResult *robotPos);
    void executeCB(const quality_inspection::MovRobToScanPosGoalConstPtr& goal);
    int initSocket();
};

void sigintCB(int signum);
std::function<void()> exitFunc;
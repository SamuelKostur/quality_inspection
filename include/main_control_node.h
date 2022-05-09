#include <ros/ros.h>
#include <quality_inspection/MovRobToScanPosAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/SetTransformationMatrix.h>

#include <thread>
#include <std_srvs/SetBool.h>
#include <my_semaphore.h>
#include <auxiliaries.h>
#include <csignal>

#define CONNECT_ROBOT 1

class MainControlNode{
    public:
    ros::NodeHandle n;
    ros::ServiceServer comm_server_main_processScan;
    Semaphore sem;

    #if CONNECT_ROBOT
        actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> robotClient;
    #endif

    ros::ServiceClient cameraClient;
    phoxi_camera::GetFrame scan;

    //vector of robot scan poses {x, y, z, A, B, C} {mm, mm, mm, deg, deg, deg}
    std::vector< std::vector<double>> robotScanPoses;    
    std::vector<double> robotHomePose;

    std::atomic<bool> exitFlag{false};

    MainControlNode();
    void nodeExitFunc();
    bool semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    quality_inspection::MovRobToScanPosGoal setGoal(std::vector<double> goalArray);
    int movRobToScanPos(std::vector<double> pose);
    int loadRobotScanPoses();
    int loadRobotHomePose();
    int triggerScan();
    int scanWholePart();
    void mainEndlessLoop();
};

void sigintCB(int signum);
std::function<void()> exitFunc;
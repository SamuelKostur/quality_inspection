#include <ros/ros.h>
#include <quality_inspection/MovRobToScanPosAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/SetTransformationMatrix.h>

#include <thread>
#include <std_srvs/SetBool.h>
#include<Semaphore.h>

#define CONNECT_ROBOT 1

using std::cout;
using std::vector;
using std::endl;

class MainControl{
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
    vector<vector<double>> robotScanPoses = {{780.80, 247.40, 204.88, 132.95, 3.4, 179.05},
                                            {749.80, -19.13, 240.26, 132.82, 4.95, 177.37},
                                            {203.01, -339.43, 219.94, 51.00, 4.86, -179.83},
                                            {294.49, 126.36, 598.63, 79.11, 61.83, 97.29},
                                            {345.37, 437.17, 598.85, 77.80, 62.26, 96.90},
                                            {-58.01, 611.61, 222.38, -38.61, 8.83, -170.97},
                                            {-85.37, 375.77, 212.30, -45.46, 6.59, -179.71}};
    
    vector<double> robotHomePose = {218.80, 393.40, 704.72, 113.37, 46.55, 55.06};

    #if CONNECT_ROBOT
      MainControl(): robotClient("movRobToScanPos", true), sem(0,1){        
        cout << "Waiting for the availability of the action server handling communication with robot." << endl;
        robotClient.waitForServer();
        cout << "connected to action server" << endl;
        
        cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
        scan.request.in = -1;

        comm_server_main_processScan = n.advertiseService("comm_main_processScan", &MainControl::semServCB, this);

        std::thread endlessThread (&MainControl::mainEndlessLoop, this);
        ros::spin();
        endlessThread.join();

      }
    #else     
      MainControl(){       
        cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
        scan.request.in = -1;

        mainEndlessLoop();
      }
    #endif

      bool semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        std::cout << "called" << std::endl;
        sem.release();
        return true;
      }

    quality_inspection::MovRobToScanPosGoal setGoal(vector<double> goalArray){
      quality_inspection::MovRobToScanPosGoal goal;
      goal.x = goalArray[0];
      goal.y = goalArray[1];
      goal.z = goalArray[2];
      goal.A = goalArray[3];
      goal.B = goalArray[4];
      goal.C = goalArray[5];
      return goal;
    }

    int movRobToScanPos(vector<double> pose){
      #if CONNECT_ROBOT
        quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
        robotClient.sendGoal(setGoal(pose));
        robotClient.waitForResult(ros::Duration(30.0)); //maximum time to move robot to desired position
        if (robotClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          finalRobPos = robotClient.getResult();
          cout << "robot is in position " << "x: " << finalRobPos->x  << ", "
                                          << "y: " << finalRobPos->y  << ", "
                                          << "z: " << finalRobPos->z  << ", "
                                          << "A: " << finalRobPos->A  << ", "
                                          << "B: " << finalRobPos->B  << ", "
                                          << "C: " << finalRobPos->C  << ", "
                                          << endl;
          return 0;
        }
        else{
          cout <<"error during robot positioning" << endl;
          return -1;
        }
      #else
        return 0;
      #endif
    }

    int triggerScan(){
      if(cameraClient.call(scan)){
        ROS_INFO("%d\n",(bool)scan.response.success);
        return 0;
      }
      else{
        ROS_ERROR("Failed to call service getFrame");
        return -1;
      }
    }

    int scanWholePart(){
      // function handle scanning of the one whole part
      for(auto& pose: robotScanPoses){
        if(movRobToScanPos(pose) != 0) return -1;
        if(triggerScan() != 0) return -1;
      }
      if (movRobToScanPos(robotHomePose) != 0) return -1;
      return 0;
    }

    void mainEndlessLoop(){
      if(movRobToScanPos(robotHomePose) != 0) return;
      while(1){
        sem.acquire();
        getchar();
        if(scanWholePart() != 0) return;
      }
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "main");
  MainControl mainControl;
  return 0;
}
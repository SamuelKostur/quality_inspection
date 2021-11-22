  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <stdlib.h>
  #include <phoxi_camera/GetFrame.h>
  
  typedef actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> Client;
  using namespace std;
  class MainControl{
    public:
      ros::NodeHandle n;
      actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> client;

      ros::ServiceClient cameraClient;
      phoxi_camera::GetFrame scan;

      //vector of robot poses {x, y, z, A, B, C}
      vector<vector<double>> robotPoses = {{493.54, 474.27, 408.72, -166.89, 33.72, 166.38},
                                           {604.56, 318.81, 365.03, 149.99, 28.06, 164.57},
                                           {469.91, -128.03, 408.67, 89.15, 23.61, 157.28},
                                           {344.85, -169.52, 378.81, 72.53, 23.56, 163.9},
                                           
                                           {272.59, -110.18, 512.98, 56.28, 34.47, 157.81},
                                           {300.44, 24.16, 568.90, 94.24, 39.68, 98.02},
                                           {325.92, 260.59, 613.00, 95.72, 61.34, 96.42},
                                           {329.21, 495.81, 572.70, 140.49, 68.32, 130.36},
                                           {323.73, 624.93, 494.75, -144.16, 61.44, 174.62},

                                           {15.38, 535.33, 484.33, -73.99, 41.96, 161.67},
                                           {-133.16, 437.75, 336.15, -45.23, 30.92, 176.66},
                                           {27.57, 358.33, 569.22, -44.55, 41.93, 145.90},
                                           {194.15, 393.29, 569.19, -87.52, 42.39, 131.79}};

      MainControl(): client("movRobToScanPos", true){
        cout << "Waiting for the availability of the action server handling communication with robot." << endl;
        client.waitForServer();
        cout << "connected to action server" << endl;
        
        cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
        scan.request.in = -1;
      }

      void executeMainCycle(){
        quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
        for(auto& pose: robotPoses){
          getchar();
          client.sendGoal(setGoal(pose));
          client.waitForResult(ros::Duration(30.0)); //maximum time to move robot to desired position
          if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
              finalRobPos = client.getResult();
              cout << "robot is in position " << "x: " << finalRobPos->x  << ", "
                                              << "y: " << finalRobPos->y  << ", "
                                              << "z: " << finalRobPos->z  << ", "
                                              << "A: " << finalRobPos->A  << ", "
                                              << "B: " << finalRobPos->B  << ", "
                                              << "C: " << finalRobPos->C  << ", "
                                              << endl;
          }
          else{
              cout <<"error during robot positioning" << endl;
              return;
          }

          if(cameraClient.call(scan)){
            ROS_INFO("%d\n",(bool)scan.response.success);

          }
          else{
            ROS_ERROR("Failed to call service getFrame");
            return;
          }

        }
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
  };

  int main(int argc, char** argv){
    ros::init(argc, argv, "main");

    MainControl mainControl;
    mainControl.executeMainCycle();
    return 0;
  }
  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <stdlib.h>
  #include <phoxi_camera/GetFrame.h>
  #include <phoxi_camera/SetTransformationMatrix.h>
  
  typedef actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> Client;
  using namespace std;
  class MainControl{
    public:
      ros::NodeHandle n;
      actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> client;

      ros::ServiceClient cameraClient;
      phoxi_camera::GetFrame scan;

      //vector of robot poses {x, y, z, A, B, C} 
      //TODO home robot pose somewhere above the part, to avoid colision when moving from last to first pose
      vector<vector<double>> robotPoses = {{780.80, 247.40, 204.88, 132.95, 3.4, 179.05},
                                             {749.80, -19.13, 240.26, 132.82, 4.95, 177.37},
                                             {203.01, -339.43, 219.94, 51.00, 4.86, -179.83},
                                             {294.49, 126.36, 598.63, 79.11, 61.83, 97.29},
                                             {345.37, 437.17, 598.85, 77.80, 62.26, 96.90},
                                             {-58.01, 611.61, 222.38, -38.61, 8.83, -170.97},
                                             {-85.37, 375.77, 212.30, -45.46, 6.59, -179.71}};

      MainControl(): client("movRobToScanPos", true){
        cout << "Waiting for the availability of the action server handling communication with robot." << endl;
        client.waitForServer();
        cout << "connected to action server" << endl;
        
        cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
        scan.request.in = -1;
      }

      //  MainControl(){
      //   cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
      //   scan.request.in = -1;
      // }

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
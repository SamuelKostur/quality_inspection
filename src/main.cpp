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
      vector<vector<double>> robotPoses = {{113.47, 473.59, 256.68, -62.08, 20.18, 179.67},
                                           {418.90, 245.69, 662.63, -93.26, 60.94, 149.65}};

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
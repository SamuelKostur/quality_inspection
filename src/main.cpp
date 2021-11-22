  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <stdlib.h>
  
  typedef actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> Client;
  using namespace std;
  class MainControl{
    public:
      actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> client;
      //vector of robot poses {x, y, z, A, B, C}
      vector<vector<double>> robotPoses = {{493.54, 474.27, 408.72, -166.89, 33.72, 166.38},
                                           {604.56, 318.81, 365.03, 149.99, 28.06, 164.57}};

      MainControl(): client("movRobToScanPos", true){
        cout << "Waiting for the availability of the action server handling communication with robot." << endl;
        client.waitForServer();
        cout << "connected to action server" << endl;
      }

      void executeMainCycle(){
        quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
        for(auto& pose: robotPoses){
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
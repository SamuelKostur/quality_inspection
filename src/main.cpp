  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <stdlib.h>
  
  typedef actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> Client;
  using namespace std;
  class MainControl{
    public:
      actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> client;
      //vector of robot poses (in this case desired angle on idnividual axis) {A1, A2, A3, A4, A5, A6}
      vector<vector<double>> robotPoses = {{-34.37, -51.82, 97.55, 28.77, 56.92, 90.25},
                                           {-36.40, -70.34, 92.41, 28.77, 56.92, 90.25}};

      MainControl(): client("movRobToScanPos", true){
        cout << "Waiting for the availability of the action server handling communication with robot." << endl;
        client.waitForServer();
        cout << "connected to action server" << endl;
      }

      void executeMainCycle(){
        quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
        for(auto& pose: robotPoses){
          client.sendGoal(setGoal(pose));
          client.waitForResult(ros::Duration(15.0)); //maximum time to move robot to desired position
          if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
              finalRobPos = client.getResult();
              cout << "robot is in position " << "x: " << finalRobPos->x << endl;
          }
          else{
              cout <<"error during robot positioning" << endl;
              return;
          }
        }
      }
    
      quality_inspection::MovRobToScanPosGoal setGoal(vector<double> goalArray){
        quality_inspection::MovRobToScanPosGoal goal;
        goal.A1 = goalArray[0];
        goal.A2 = goalArray[1];
        goal.A3 = goalArray[2];
        goal.A4 = goalArray[3];
        goal.A5 = goalArray[4];
        goal.A6 = goalArray[5];
        return goal;
      }
  };

  int main(int argc, char** argv){
    ros::init(argc, argv, "main");

    MainControl mainControl;
    mainControl.executeMainCycle();

    //printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
  }
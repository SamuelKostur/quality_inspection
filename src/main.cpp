  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <stdlib.h>
  
  typedef actionlib::SimpleActionClient<quality_inspection::MovRobToScanPosAction> Client;
  
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "main");
    Client client("movRobToScanPos", true); // true -> don't need ros::spin()
    client.waitForServer();
    quality_inspection::MovRobToScanPosGoal goal;
    quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
    goal.A1 = -34.37;
    goal.A2 = -51.82;
    goal.A3 = 97.55;
    goal.A4 = 28.77;
    goal.A5 = 56.92;
    goal.A6 = 90.25;
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        finalRobPos = client.getResult();
        std::cout << "robot is in position " << "x: " << finalRobPos->x << std::endl;
    }

    sleep(5);
    goal.A1 = -36.40;
    goal.A2 = -70.34;
    goal.A3 = 92.41;
    goal.A4 = 28.77;
    goal.A5 = 56.92;
    goal.A6 = 90.25;
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        finalRobPos = client.getResult();
        std::cout << "robot is in position " << "x: " << finalRobPos->x << std::endl;
    }

    //printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
  }
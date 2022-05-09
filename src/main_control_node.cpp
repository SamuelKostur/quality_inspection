#include <main_control_node.h>

#if CONNECT_ROBOT
  MainControlNode::MainControlNode(): robotClient("movRobToScanPos", true), sem(0,1){  
    exitFunc = boost::bind(&MainControlNode::nodeExitFunc, this);

    if(loadRobotScanPoses() != 0){
      return;
    }
    if(loadRobotHomePose() != 0){
      return;
    }    

    std::cout << "Waiting for the availability of the action server handling communication with robot." << std::endl;
    robotClient.waitForServer();
    std::cout << "connected to action server" << std::endl;
    
    cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
    scan.request.in = -1;

    comm_server_main_processScan = n.advertiseService("comm_main_processScan", &MainControlNode::semServCB, this);

    std::thread endlessThread (&MainControlNode::mainEndlessLoop, this);
    ros::spin();
    endlessThread.join();

  }
#else     
  MainControl::MainControl(){       
    cameraClient = n.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
    scan.request.in = -1;

    mainEndlessLoop();
  }
#endif

void MainControlNode::nodeExitFunc(){
  std::cout << "exiting node started" << std::endl; 
  exitFlag.store(true);
  sem.release();
}

bool MainControlNode::semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std::cout << "called" << std::endl;
  sem.release();
  return true;
}

quality_inspection::MovRobToScanPosGoal MainControlNode::setGoal(std::vector<double> goalArray){
  quality_inspection::MovRobToScanPosGoal goal;
  goal.x = goalArray[0];
  goal.y = goalArray[1];
  goal.z = goalArray[2];
  goal.A = goalArray[3];
  goal.B = goalArray[4];
  goal.C = goalArray[5];
  return goal;
}

int MainControlNode::movRobToScanPos(std::vector<double> pose){
  #if CONNECT_ROBOT
    quality_inspection::MovRobToScanPosResultConstPtr finalRobPos;
    robotClient.sendGoal(setGoal(pose));
    robotClient.waitForResult(ros::Duration(30.0)); //maximum time to move robot to desired position
    if (robotClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      finalRobPos = robotClient.getResult();
      std::cout << "robot is in position " << "x: " << finalRobPos->x  << ", "
                                      << "y: " << finalRobPos->y  << ", "
                                      << "z: " << finalRobPos->z  << ", "
                                      << "A: " << finalRobPos->A  << ", "
                                      << "B: " << finalRobPos->B  << ", "
                                      << "C: " << finalRobPos->C  << ", "
                                      << std::endl;
      return 0;
    }
    else{
      std::cout <<"error during robot positioning" << std::endl;
      return -1;
    }
  #else
    return 0;
  #endif
}

int MainControlNode::loadRobotScanPoses(){
  //get path to file containing robot scanning positions
  std::string str1;
  if( !n.getParam("/input_data_folder", str1)){
    ROS_ERROR("Failed to get parameter from server.");
    return -1;
  }
  std::string str2;
  if( !n.getParam("/robot_scanning_poses", str2)){
    ROS_ERROR("Failed to get parameter from server.");
    return -1;
  }
  std::string path = str1 + str2;
  int numCols = 6; // robot pose is defined as {x, y, z, A, B, C}
  if(loadTxtTo2DVector(path, robotScanPoses, numCols, -1) != 0){
    return -1;
  }

  return 0;
}

int MainControlNode::loadRobotHomePose(){
  if( !n.getParam("/robot_home_pose",robotHomePose)){
    ROS_ERROR("Failed to get parameter from server.");
    return -1;
  }

  return 0;
}

int MainControlNode::triggerScan(){
  if(cameraClient.call(scan)){
    ROS_INFO("%d\n",(bool)scan.response.success);
    return 0;
  }
  else{
    ROS_ERROR("Failed to call service getFrame");
    return -1;
  }
}

int MainControlNode::scanWholePart(){
  // function handle scanning of the one whole part
  for(auto& pose: robotScanPoses){
    if(movRobToScanPos(pose) != 0) return -1;
    if(triggerScan() != 0) return -1;
  }
  if (movRobToScanPos(robotHomePose) != 0) return -1;
  return 0;
}

void MainControlNode::mainEndlessLoop(){
  if(movRobToScanPos(robotHomePose) != 0) return;
  while(1){
    sem.acquire();
    if(exitFlag.load()){
      std::cout << "exiting mainEndlessLoop" << std::endl;
      return;
    }
    getchar();
    if(scanWholePart() != 0) return;
  }
}

void sigintCB(int signum){
  static int sigintRaised = 0;
  if(!sigintRaised){
    sigintRaised = 1;
    //if there is a serious problem while executing and we need to 
    //properly exit programm use raise(SIGINT); followed by return; from currect thread (function)
    exitFunc(); //this handles shutting down socket from ScanningSimulationNode
    ros::shutdown();
    std::cout << "ros is shutting down" << std::endl;
    while (ros::ok()){}
    std::cout << "ros finished shutting down" << std::endl;
    //exit() is called somewhere from ros::shutdown(), i tried it with atexit
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "main_control");
  std::signal(SIGINT, sigintCB); //should be after ros::init()
  MainControlNode mainControl;
  return 0;
}
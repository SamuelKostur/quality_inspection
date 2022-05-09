#include <scanning_simulation_node.h>

ScanningSimulationNode::ScanningSimulationNode():sem(0,1){
    exitFunc = boost::bind(&ScanningSimulationNode::nodeExitFunc, this);
    
    comm_server_main_processScan = n.advertiseService("comm_main_processScan", &ScanningSimulationNode::semServCB, this);
    if(loadSimulationInputDataPath() != 0){
        return;
    }
    if(loadRobScanPosesCount() != 0){
        return;
    }
    pointCloudPub = n.advertise<sensor_msgs::PointCloud2>("/phoxi_camera/pointcloud", 1);
    texturePub = n.advertise<sensor_msgs::Image>( "/phoxi_camera/texture", 1);
    std::cout << "service ready" << std::endl;

    std::thread endlessThread (&ScanningSimulationNode::mainEndlessLoop, this);
    ros::spin();
    endlessThread.join();
}

void ScanningSimulationNode::nodeExitFunc(){
  std::cout << "exiting node started" << std::endl; 
  exitFlag.store(true);
  sem.release();
}

void ScanningSimulationNode::mainEndlessLoop(){
    while(true){
        sem.acquire();
        if(exitFlag.load()){
            std::cout << "exiting mainEndlessLoop" << std::endl;
            return;
        }
        std::cout << "beginning to simulate scanning of the part..." << std::endl;
        for(int i = 0; i < numRobScanPoses; i++){
            if (simulateScanning() != 0 ){
                return;
            }
            //sleep(20);                    
            //getchar();   
            std::cout << i << std::endl;                 
        }
        std::cout << "waiting to complete scan processing..." << std::endl;   
}
}

bool ScanningSimulationNode::semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::cout << "called" << std::endl;
    sem.release();
    return true;
}

int ScanningSimulationNode::loadSimulationInputDataPath(){
    if( !n.getParam("/simulation_input_data_folder", simulationInputDataPath)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    return 0;
}

int ScanningSimulationNode::loadRobScanPosesCount(){
    if( !n.getParam("/simulation_robot_scanning_poses_count", numRobScanPoses)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }

    return 0;
}

void ScanningSimulationNode::BreakNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud){
    // apply contra transformation from photoneo ros pkg normals (where each is divided by 1000)
    // to ensure normal vector size 1 (sqrt(nx^2+ny^2+nz^2))
    for(int row = 0; row < pointCloud->height; row++){
        for(int col = 0; col < pointCloud->width; col++){
            pointCloud->at(col, row).normal_x /= 1000;
            pointCloud->at(col, row).normal_y /= 1000;
            pointCloud->at(col, row).normal_z /= 1000;
        }
    }
}

void ScanningSimulationNode::publishData(pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud, cv::Mat img){
    ros::Time timeNow = ros::Time::now();
    std_msgs::Header header;
    header.stamp = timeNow;

    sensor_msgs::PointCloud2 pointCloudMsg; 
    pcl::toROSMsg(*pointCloud, pointCloudMsg);
    pointCloudMsg.header = header;
    pointCloudPub.publish(pointCloudMsg);
    
    img.convertTo(img, CV_32FC1);
    sensor_msgs::ImagePtr textureMsg  = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img).toImageMsg();          
    texturePub.publish(textureMsg);
}

int ScanningSimulationNode::simulateScanning (){
    static int currIdxRobPose = 0;
    //load point cloud
    MyPointCloud::Ptr pointCloud = pcl::make_shared<MyPointCloud>();
    std::string PCDpath = simulationInputDataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointNormal> (PCDpath, *pointCloud) == -1){
        PCL_ERROR(std::string("Couldn't read file ").append(PCDpath).append("\n").c_str());
        return -1;
    }
    BreakNormals(pointCloud);

    //load img
    cv::Mat img;
    // 0 is flag  cv::IMREAD_GRAYSCALE, careful because default is  bgr
    std::string imgPath = simulationInputDataPath+ "img" + std::to_string(currIdxRobPose) + ".bmp";
    // 0 is flag  cv::IMREAD_GRAYSCALE, careful because default is  bgr
    img = cv::imread(imgPath, 0);
    if(img.data == NULL){
        ROS_ERROR("%s", std::string("Couldn't read file ").append(imgPath).append("\n").c_str());
        return -1;
    }

    publishData(pointCloud, img);
    currIdxRobPose++;
    currIdxRobPose = (currIdxRobPose < numRobScanPoses) ? currIdxRobPose : 0;
    return 0;
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
    ros::init(argc, argv, "scanning_simulation", ros::init_options::NoSigintHandler); 
    std::signal(SIGINT, sigintCB); //should be after ros::init()
    ScanningSimulationNode testPub;    
    return 0;
}
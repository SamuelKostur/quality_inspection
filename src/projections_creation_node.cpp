#include <projections_creation_node.h>

ProjectionsCreationNode::ProjectionsCreationNode(){
    create2DprojectionsServer = n.advertiseService("/quality_inspection/create2Dprojections", &ProjectionsCreationNode::create2DprojectionsCB, this);
    
    if(loadOutputDataPath() != 0){
        sleep(1);
        return;
    }
    
    if(loadRivetPositions() != 0){
        sleep(1);
        return;
    }
    cloud = pcl::make_shared<MyPointCloud>();
    partID = 0;

    ros::spin();
}

bool ProjectionsCreationNode::create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res){
    auto t1 = std::chrono::high_resolution_clock::now();
    pcl::fromROSMsg(req.pc2, *cloud);

    createProjections();    
    auto t2 = std::chrono::high_resolution_clock::now();
    auto durationProj= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Duration Projections: " << durationProj.count()/1000000.0 << std::endl;

    pcl::io::savePCDFileBinary (currScanningDataFolder() + "completePointCloudProj.pcd", *cloud);
    cloud->clear();
    partID++;
    return true;
}

int ProjectionsCreationNode::loadRivetPositions(){
    //get path to file containing robot scanning positions
    std::string str1;
    if( !n.getParam("/input_data_folder", str1)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string str2;
    if( !n.getParam("/rivet_positions", str2)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string path = str1 + str2;
    int numCols = 3; //rivet positions is defined as {X,Y,Z}
    if(loadTxtTo2DVector(path, rivetLocations, 3, -1) != 0){
        return -1;
    }

    return 0;
}

int ProjectionsCreationNode::loadInitialPartID(){
    int temp_partID;
    if( !n.getParam("/initial_part_ID", temp_partID)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }

    partID = u_int64_t(temp_partID);
    return 0;
}

// data paths
int ProjectionsCreationNode::loadOutputDataPath(){
    if( !n.getParam("/output_data_folder", outputDataPath)){
        ROS_ERROR("Failed to get parameter from server.");        
        return -1;
    }
    boost::filesystem::create_directories(outputDataPath);
    return 0;
}

std::string ProjectionsCreationNode::currPartFolder(){
    std::string path = outputDataPath + "part_ID_" + std::to_string(partID) + "/";    
    boost::filesystem::create_directories(path);
    return path;
}

std::string ProjectionsCreationNode::currRivetFolder(unsigned int rivetID){
    std::string path = currPartFolder() + "rivets/" + "rivet_ID_" + std::to_string(rivetID) + "/";
    boost::filesystem::create_directories(path);
    return path;
}

std::string ProjectionsCreationNode::currScanningDataFolder(){
    std::string path = currPartFolder() + "scanning_data/";
    boost::filesystem::create_directories(path);
    return path;
}

void ProjectionsCreationNode::saveRivetCenterDistances(std::vector<std::future<std::array<float,2>>>& threadVec){
    std::ofstream file;
    file.open(currPartFolder() + "rivetDeviations.txt");
    if(!file){
        std::cout<<"File not created...";
        return;
    }
    float avgRealDist = 0;
    float avgPixelDist = 0;    
    file << "rivet_ID [int]; " << "pixel deviation [float]; " << "real deviation (mm) [float]" << "\n";    
    for (int i = 0; i < threadVec.size(); i++){  
        std::array<float,2> rivCenterDist = threadVec[i].get();    
        file <<  i << "\t" << rivCenterDist[0] << "\t" << rivCenterDist[1] * 1000 << "\n";
        avgPixelDist += rivCenterDist[0];
        avgRealDist += rivCenterDist[1] * 1000;
    }
    avgPixelDist /= threadVec.size();
    avgRealDist /= threadVec.size();
    file << "Average pixel deviation:" << avgPixelDist << "; Average real deviation (mm):" << avgRealDist << "\n";
    file.close();
}

void ProjectionsCreationNode::createProjections(){
    //1.64s processing for 30 rivets
    unsigned int numRivets = rivetLocations.size();
    std::vector<std::future<std::array<float,2>>> threadVec;
    threadVec.resize(numRivets);
    for(int rivetID = 0; rivetID < numRivets; rivetID++){
        MyPointCloud::Ptr rivetCloud (new MyPointCloud);
        Projections projections;
        projections.extractRivetPointCloud(cloud, *rivetCloud, rivetLocations[rivetID]);
        auto t = std::async(std::launch::async, &ProjectionsCreationNode::adjustPCcreateProj, this, rivetCloud, projections, rivetLocations[rivetID], rivetID);
        threadVec.at(rivetID) = std::move(t);	
    }

    for (auto& t : threadVec){
        if(t.wait_for(std::chrono::seconds(15)) != std::future_status::ready){
            std::cout << "maximum time of creating 2D projections reached" << std::endl;            
            return;
        }
    }
    saveRivetCenterDistances(threadVec);
}

std::array<float,2> ProjectionsCreationNode::adjustPCcreateProj(MyPointCloud::Ptr rivetCloudPtr, Projections projections, std::vector<float> rivetLocation, int rivetID){
    //MyPointCloud::Ptr rivetCloudPtr  = rivetCloud.makeShared();
    projections.adjustRivetPointCloud(rivetCloudPtr, rivetLocation);
    std::array<float,2>  rivCenterDist = projections.create2DProjectionsImages(rivetCloudPtr, currRivetFolder(rivetID));
    return rivCenterDist;
}

// void ProjectionsCreationNode::createProjections(){
//     //4.46s processing for 30 rivets
//     unsigned int numRivets = sizeof(rivetLocations) / sizeof(rivetLocations[0]);
//     for(int rivetID = 0; rivetID < numRivets; rivetID++){
//         pcl::PointCloud<pcl::PointXYZINormal>::Ptr rivetCloud (new pcl::PointCloud<pcl::PointXYZINormal>);
//         projections.extractRivetPointCloud(cloud, *rivetCloud, rivetLocations[rivetID]);
//         projections.adjustRivetPointCloud(rivetCloud, rivetLocations[rivetID]);
//         projections.create2DProjectionsImages(rivetCloud, currRivetFolder(rivetID));
//     }
//     partID++;
// }

///////////////////////////////////main//////////////////////////////////////////
int main(int argc, char** argv){    
    ros::init(argc, argv, "projections_creation");   
    ProjectionsCreationNode projectionsNode;    
    return 0;
}
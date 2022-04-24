#include <projectionsNode.h>

ProjectionsNode::ProjectionsNode(){
    create2DprojectionsServer = n.advertiseService("/quality_inspection/create2Dprojections", &ProjectionsNode::create2DprojectionsCB, this);
    
    createDataPath();
    loadRivetPositions();
    partID = 0;

    ros::spin();
}

bool ProjectionsNode::create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res){
    auto t1 = std::chrono::high_resolution_clock::now();
    cloud = pcl::make_shared<MyPointCloud>();
    pcl::fromROSMsg(req.pc2, *cloud);

    createProjections();    
    auto t2 = std::chrono::high_resolution_clock::now();
    auto durationProj= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Duration Projections: " << durationProj.count()/1000000.0 << std::endl;

    pcl::io::savePCDFileBinary (currScanningDataFolder() + "completePointCloudProj.pcd", *cloud);

    return true;
}

void ProjectionsNode::loadRivetPositions(){
    std::string path = ros::package::getPath("quality_inspection"); 
    path = path.substr(0, path.find_last_of("/\\"));    
    path = path.substr(0, path.find_last_of("/\\"));       
    path = path + "/cadModels/2layerData/rivetPositions.txt";
    std::ifstream file;
    file.open(path);
    if(!file){
        std::cout<<"Cannot open file with rivet positions...";
        return;
    }
    std::string line;
    while(std::getline(file, line)){
        rivetLocations.push_back(std::vector<float>(3, 0.0));
        std::stringstream lineStream(line);
        lineStream >> rivetLocations.back().at(0); //X rivet position
        lineStream >> rivetLocations.back().at(1); //Y rivet position
        lineStream >> rivetLocations.back().at(2); //Z rivet position
    }
    file.close();
    // for(auto& pos : rivetLocations){
    //     std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    // }
}


// data paths
void ProjectionsNode::createDataPath(){
    dataPath = ros::package::getPath("quality_inspection"); 
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
    dataPath = dataPath + "/data_quality_inspection/";
    boost::filesystem::create_directories(dataPath);
}

std::string ProjectionsNode::currPartFolder(){
    std::string path = dataPath + "part_ID_" + std::to_string(partID) + "/";    
    boost::filesystem::create_directories(path);
    return path;
}

std::string ProjectionsNode::currRivetFolder(unsigned int rivetID){
    std::string path = currPartFolder() + "rivets/" + "rivet_ID_" + std::to_string(rivetID) + "/";
    boost::filesystem::create_directories(path);
    return path;
}

std::string ProjectionsNode::currScanningDataFolder(){
    std::string path = currPartFolder() + "scanning_data/";
    boost::filesystem::create_directories(path);
    return path;
}

void ProjectionsNode::saveRivetCenterDistances(std::vector<std::future<std::array<float,2>>>& threadVec){
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

void ProjectionsNode::createProjections(){
    //1.64s processing for 30 rivets
    unsigned int numRivets = rivetLocations.size();
    std::vector<std::future<std::array<float,2>>> threadVec;
    threadVec.resize(numRivets);
    for(int rivetID = 0; rivetID < numRivets; rivetID++){
        MyPointCloud rivetCloud;
        projections.extractRivetPointCloud(cloud, rivetCloud, rivetLocations[rivetID]);
        auto t = std::async(std::launch::async, &ProjectionsNode::adjustPCcreateProj, this, rivetCloud, projections, rivetLocations[rivetID], rivetID);
        threadVec.at(rivetID) = std::move(t);	
    }

    for (auto& t : threadVec){
        if(t.wait_for(std::chrono::seconds(15)) != std::future_status::ready){
            std::cout << "maximum time of creating 2D projections reached" << std::endl;            
            return;
        }
    }
    saveRivetCenterDistances(threadVec);

    partID++;
}

std::array<float,2> ProjectionsNode::adjustPCcreateProj(MyPointCloud rivetCloud, Projections projections, std::vector<float> rivetLocation, int rivetID){
    MyPointCloud::Ptr rivetCloudPtr  = rivetCloud.makeShared();
    projections.adjustRivetPointCloud(rivetCloudPtr, rivetLocation);
    std::array<float,2>  rivCenterDist = projections.create2DProjectionsImages(rivetCloudPtr, currRivetFolder(rivetID));
    return rivCenterDist;
}

// void ProjectionsNode::createProjections(){
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
    ros::init(argc, argv, "projectionsNode");   
    ProjectionsNode projectionsNode;    
    return 0;
}
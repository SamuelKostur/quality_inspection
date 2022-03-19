#include <projectionsNode.h>

ProjectionsNode::ProjectionsNode(){
    create2DprojectionsServer = n.advertiseService("/quality_inspection/create2Dprojections", &ProjectionsNode::create2DprojectionsCB, this);
    
    createDataPath();

    partID = 0;

    ros::spin();
}

bool ProjectionsNode::create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res){
    cloud = pcl::make_shared<MyPointCloud>();
    pcl::fromROSMsg(req.pc2, *cloud);

    pcl::io::savePCDFileBinary (currScanningDataFolder() + "completePointCloudProj.pcd", *cloud);

    auto t1 = std::chrono::high_resolution_clock::now(); 
    createProjections();    
    auto t2 = std::chrono::high_resolution_clock::now();
    auto durationProj= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Duration Projections: " << durationProj.count()/1000000.0 << std::endl;

    res.message = "beem";
    return true;
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

void ProjectionsNode::createProjections(){
    //1.64s processing for 30 rivets
    unsigned int numRivets = sizeof(rivetLocations) / sizeof(rivetLocations[0]);
    std::vector<std::future<void>> threadVec;
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

    partID++;
}

void ProjectionsNode::adjustPCcreateProj(MyPointCloud rivetCloud, Projections projections, float* rivetLocation, int rivetID){
    MyPointCloud::Ptr rivetCloudPtr  = rivetCloud.makeShared();
    projections.adjustRivetPointCloud(rivetCloudPtr, rivetLocation);
    projections.create2DProjectionsImages(rivetCloudPtr, currRivetFolder(rivetID));
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
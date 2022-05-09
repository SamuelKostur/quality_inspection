#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <quality_inspection/create2Dprojections.h>
#include <ros/package.h>
#include <projections_creation.h>
#include <chrono>
#include <vector>
#include <future>
#include <array>
#include <auxiliaries.h>


typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
};

class ProjectionsCreationNode{
    public:
      ProjectionsCreationNode();

    private:
    ros::NodeHandle n;
    ros::ServiceServer create2DprojectionsServer;

    std::string outputDataPath;
    MyPointCloud::Ptr cloud;

    //Projections projections;

    u_int64_t partID;

    std::vector<std::vector<float>> rivetLocations;

    int loadOutputDataPath();
    std::string currPartFolder();
    std::string currRivetFolder(unsigned int rivetID);
    std::string currScanningDataFolder();
    bool create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res);
    void createProjections();
    std::array<float,2> adjustPCcreateProj(MyPointCloud::Ptr rivetCloud, Projections projections, std::vector<float> rivetLocation, int rivetID);
    int loadRivetPositions();
    int loadInitialPartID();
    void saveRivetCenterDistances(std::vector<std::future<std::array<float,2>>>& threadVec);
};
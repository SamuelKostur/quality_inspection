#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <quality_inspection/create2Dprojections.h>
#include <ros/package.h>
#include <projections.h>
#include <chrono>
#include <vector>
#include <future>
#include <array>


typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
};

class ProjectionsNode{
    public:
      ProjectionsNode();

    private:
    ros::NodeHandle n;
    ros::ServiceServer create2DprojectionsServer;

    std::string dataPath;
    MyPointCloud::Ptr cloud;

    Projections projections;

    unsigned long partID;

    //x, y, z location of the center of a rivet in meters
    // float rivetLocations [3][3] ={{0.060848f, -0.152035f, -0.036782},
    //                               {0.063324f, 0.139139f, -0.035323f},
    //                               {-0.065320f, 0.145271f, 0.005731f}};
    std::vector<std::vector<float>> rivetLocations;

    void createDataPath();
    std::string currPartFolder();
    std::string currRivetFolder(unsigned int rivetID);
    std::string currScanningDataFolder();
    bool create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res);
    void createProjections();
    std::array<float,2> adjustPCcreateProj(MyPointCloud rivetCloud, Projections projections, std::vector<float> rivetLocation, int rivetID);
    void loadRivetPositions();
    void saveRivetCenterDistances(std::vector<std::future<std::array<float,2>>>& threadVec);
};
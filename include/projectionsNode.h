#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <quality_inspection/create2Dprojections.h>
#include <ros/package.h>
#include <projections.h>
#include <chrono>
#include <vector>
#include <future>


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
    float rivetLocations [2][3] ={{0.077743f, 0.117416f, -0.033278f},
                                  {0.0751653f, -0.175274f, -0.0346544f}};

    void createDataPath();
    std::string currPartFolder();
    std::string currRivetFolder(unsigned int rivetID);
    std::string currScanningDataFolder();
    bool create2DprojectionsCB(quality_inspection::create2Dprojections::Request &req, quality_inspection::create2Dprojections::Response &res);
    void createProjections();
    void adjustPCcreateProj(MyPointCloud rivetCloud, Projections projections, float* rivetLocation, int rivetID);
};
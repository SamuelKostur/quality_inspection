#pragma once

#include <ros/ros.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <future>
#include <thread>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <csignal>
#include <chrono>
#include <newTransform.hpp>
#include <mutex>
#include <condition_variable>
#include<std_srvs/SetBool.h>
#include <chrono>
#include <Semaphore.h>

//point cloud registration
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/random_sample.h>

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
  using Indices = std::vector<int>;
};

typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

#define SAVE_PARTIAL_DATA 0

struct CPCindices{
    MyPointCloud CPC;
    std::vector<int> PCPindices;
};

class ProcessScan{
    public:
        ProcessScan();

    private:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub;
        ros::Subscriber textureSub;
        
        ros::ServiceClient comm_client_main_processScan;

        std::string dataPath;

        std::promise<int> exitPromise;
        std::shared_future<int> exitFuture;

        std::vector<std::promise<MyPointCloud>> promiseVec;
    	std::vector<std::future<MyPointCloud>> futureVec;
    	std::mutex mtx;

        MyPointCloud::Ptr completePointCloud  = MyPointCloud::Ptr (new MyPointCloud);
        std::vector<int> PPCindices;  

        Semaphore semMT2_MT3;
        std::promise<CPCindices> promCPCindices;
        std::future<CPCindices> futCPCindices;

        pcl::PointCloud<pcl::PointNormal>::Ptr CADcloud  = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        //vector of robot scan poses {x, y, z, A, B, C} {mm, mm, mm, deg, deg, deg}
        std::vector<std::vector<double>> robotScanPoses = {{809.80, 286.87, 309.01, 143.67, 15.68, 179.30}, //{780.80, 247.40, 204.88, 132.95, 3.4, 179.05},
                                             {749.80, -19.13, 240.26, 132.82, 4.95, 177.37},
                                             {203.01, -339.43, 219.94, 51.00, 4.86, -179.83},
                                             {294.49, 126.36, 598.63, 79.11, 61.83, 97.29},
                                             {345.37, 437.17, 598.85, 77.80, 62.26, 96.90},
                                             {-58.01, 611.61, 222.38, -38.61, 8.83, -170.97},
                                             {-85.37, 375.77, 212.30, -45.46, 6.59, -179.71}};
        
        int numRobPoses;

        void procScanExitFunc();
        void createDataPath();       
        void combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture);
        void processPointCloud(std::future<MyPointCloud> future, int currIdxRobPose);
        void fillFutPromVec();
        void endlessLoop();
        void mainThread3();
};

void extractUnmeasuredPoints(MyPointCloud& pointCloud);
void robotPoseToMetersAndRadians(std::vector<double> &robotScanPoses);
void transformPointCloudFromTCPtoRobot(std::vector<double> robotPose, MyPointCloud& pointCloud);
cv::Mat createTextureImage(const sensor_msgs::Image::ConstPtr& originalTexture);
void addTextureToPointCloud(MyPointCloud& pointCloud, cv::Mat& img);
void correctNormals(MyPointCloud& pointCloud);
void sigintCB(int signum);

template<typename T>
void resetPromFut(std::promise<T>& prom, std::future<T>& fut);
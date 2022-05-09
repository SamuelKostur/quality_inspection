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
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <memory>

#include <my_transform.hpp>
#include <std_srvs/SetBool.h>

#include <my_semaphore.h>

#include <pcl/filters/normal_space.h>

#include <atomic>

typedef pcl::PointNormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
  using Indices = std::vector<int>;
};

class ScanningSimulationNode{
    public:
        ros::NodeHandle n;
        ros::Publisher pointCloudPub;
        ros::Publisher texturePub;
        ros::ServiceServer comm_server_main_processScan;
        std::string simulationInputDataPath;
        Semaphore sem;
        int numRobScanPoses;
        std::atomic<bool> exitFlag{false};

        ScanningSimulationNode();
        void nodeExitFunc();
        void mainEndlessLoop();
        bool semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        int loadSimulationInputDataPath();
        int loadRobScanPosesCount();
        void BreakNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud);
        void publishData(pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud, cv::Mat img);
        int simulateScanning ();
};

void sigintCB(int signum);
std::function<void()> exitFunc;
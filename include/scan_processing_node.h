#pragma once

#include <scan_processing.h>

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
  using Indices = std::vector<int>;
};

typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

#define SAVE_PARTIAL_DATA 1

struct CPCindices{
    MyPointCloud CPC;
    std::vector<int> PCPindices;
};

class ScanProcessingNode{
    public:
        ScanProcessingNode();

    private:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub, textureSub;        
        ros::ServiceClient client_srvs_main_scanProcessing, client_srvs_create2Dprojections;

        std::string partialOutputDataPath;

        std::promise<int> exitPromise;
        std::shared_future<int> exitFuture;

        std::vector<std::promise<MyPointCloud>> prom_vec_MT1_MT2;
    	std::vector<std::future<MyPointCloud>> fut_vec_MT1_MT2;

        MyPointCloud::Ptr completePointCloud  = MyPointCloud::Ptr (new MyPointCloud);
        std::vector<int> PPCindices;  

        Semaphore sem_MT2_MT3;
        std::promise<CPCindices> prom_MT2_MT3;
        std::future<CPCindices> fut_MT2_MT3;

        pcl::PointCloud<pcl::PointNormal>::Ptr CADcloud  = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        //vector of robot scan poses {x, y, z, A, B, C} {mm, mm, mm, deg, deg, deg}
        std::vector<std::vector<double>> robotScanPoses;        
        Eigen::Affine3f initialTransform;
        float tableZcoord;

        int numRobScanPoses;

        void procScanExitFunc();
        int loadPartialoutputDataPath();
        int loadCADmodel();
        int loadRobotScanPoses();
        int loadExpInitTransform();
        int loadTableZcoord();
        void combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture);
        MyPointCloud processPointCloud(std::future<MyPointCloud> future, int currIdxRobPose);
        void mainThread2Loop();
        void mainThread3Loop();
};

void sigintCB(int signum);
std::function<void()> exitFunc;

template<typename T>
void resetPromFut(std::promise<T>& prom, std::future<T>& fut);

std::vector<int>  createPPCIndices(int PPCstartIdx,int PPCendIdx);
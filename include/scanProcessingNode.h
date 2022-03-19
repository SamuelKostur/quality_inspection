#pragma once

#include <scanProcessing.h>

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

class ScanProcessingNode{
    public:
        ScanProcessingNode();

    private:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub, textureSub;        
        ros::ServiceClient client_srvs_main_scanProcessing, client_srvs_create2Dprojections;

        std::string dataPath;

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
        std::vector<std::vector<double>> robotScanPoses = {{809.80, 286.87, 309.01, 143.67, 15.68, 179.30}, //{780.80, 247.40, 204.88, 132.95, 3.4, 179.05},
                                             {749.80, -19.13, 240.26, 132.82, 4.95, 177.37},
                                             {203.01, -339.43, 219.94, 51.00, 4.86, -179.83},
                                             {294.49, 126.36, 598.63, 79.11, 61.83, 97.29},
                                             {345.37, 437.17, 598.85, 77.80, 62.26, 96.90},
                                             {-58.01, 611.61, 222.38, -38.61, 8.83, -170.97},
                                             {-85.37, 375.77, 212.30, -45.46, 6.59, -179.71}};
        
        const Eigen::Matrix4f initialAllign = (Eigen::Matrix4f() << 0.939394, -0.341999, 0.0239785, -0.210154,
                                                                    0.341857,  0.9397, 0.00991899, -0.408845,
                                                                    -0.0259249, -0.00112061, 0.999325, -0.128573,
                                                                    0, 0, 0, 1).finished();

        int numRobPoses;

        void procScanExitFunc();
        void createDataPath();       
        void combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture);
        MyPointCloud processPointCloud(std::future<MyPointCloud> future, int currIdxRobPose);
        void mainThread2Loop();
        void mainThread3Loop();
};

void sigintCB(int signum);

template<typename T>
void resetPromFut(std::promise<T>& prom, std::future<T>& fut);

std::vector<int>  createPPCIndices(int PPCstartIdx,int PPCendIdx);
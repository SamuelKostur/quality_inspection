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

#include <newTransform.hpp>
#include <std_srvs/SetBool.h>

#include<Semaphore.h>

#include <pcl/filters/normal_space.h>

typedef pcl::PointNormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

namespace pcl{
  using boost::shared_ptr;
  using boost::make_shared;
  using Indices = std::vector<int>;
};

Semaphore sem(0,1);

class TestPub{
    public:
        ros::NodeHandle n;
        ros::Publisher pointCloudPub;
        ros::Publisher texturePub;
        ros::ServiceServer comm_server_main_processScan;
        std::string dataPath;
        
        int numRobPoses = 7;
 
        TestPub(){
            comm_server_main_processScan = n.advertiseService("comm_main_processScan", &TestPub::semServCB, this);
            createDataPath();
            pointCloudPub = n.advertise<sensor_msgs::PointCloud2>("/phoxi_camera/pointcloud", 1);
            texturePub = n.advertise<sensor_msgs::Image>( "/phoxi_camera/texture", 1);
            std::cout << "service ready" << std::endl;

            std::thread endlessThread (&TestPub::mainEndlessLoop, this);
            normalSubSampling( );
            ros::spin();
            endlessThread.join();
        }

        void mainEndlessLoop(){
            while(true){
                sem.acquire();
                std::cout << "beginning to scan next part..." << std::endl;
                for(int i = 0; i < 7; i++){
                    simulateScanning();
                    //sleep(20);                    
                    //getchar();   
                    std::cout << i << std::endl;                 
                }
                std::cout << "waiting to complete scan processing..." << std::endl;   
        }
      }

        bool semServCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
            std::cout << "called" << std::endl;
            sem.release();
            return true;
        }

        void createDataPath(){
            dataPath = ros::package::getPath("quality_inspection"); 
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
            dataPath = dataPath + "/InspectionFilesSim/";
            boost::filesystem::create_directories(dataPath);
        }  

        void BreakNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud){
            // apply contra transformation from photoneo ros pkg normals (where each is divided by 1000)
            // to ensure normal vector size 1 (sqrt(nx^2+ny^2+nz^2))
            for(int row = 0; row < pointCloud->height; row++){
                for(int col = 0; col < pointCloud->width; col++){
                    pointCloud->at(col, row).normal_x /= 1000;
                    pointCloud->at(col, row).normal_y /= 1000;
                    pointCloud->at(col, row).normal_z /= 1000;
                }
            }
        }

        void publishData(pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud, cv::Mat img){
            ros::Time timeNow = ros::Time::now();
            std_msgs::Header header;
            header.stamp = timeNow;
   
            sensor_msgs::PointCloud2 pointCloudMsg; 
            pcl::toROSMsg(*pointCloud, pointCloudMsg);
            pointCloudMsg.header = header;
            pointCloudPub.publish(pointCloudMsg);
            
            img.convertTo(img, CV_32FC1);
            sensor_msgs::ImagePtr textureMsg  = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img).toImageMsg();          
            texturePub.publish(textureMsg);
        }

        void testTransform(pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud){ 
            Eigen::Matrix4f transMatrix;
            transMatrix.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1            
            transMatrix.block<3,3>(0,0) << 0.945861, -0.323869,-0.021337,
                                           0.324402,  0.945447, 0.0298916,
                                           0.010492, -0.0351951, 0.999325;
            transMatrix.block<3,1>(0,3) << -0.213803, -0.406436, -0.129558;
            
            std::cout  << transMatrix.matrix() << std::endl;
            auto tstart = std::chrono::high_resolution_clock::now();
            newPcl::transformPointCloudWithNormals(*pointCloud, *pointCloud, transMatrix); 
            auto tend = std::chrono::high_resolution_clock::now();
            auto durationTransform = std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart);
            std::cout << "duration Transform: " << durationTransform.count()/1000000.0 << std::endl;
        }

        void testConcatenatePointCLouds(){
            pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud0 (new pcl::PointCloud<pcl::PointNormal>);
            pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> pointCloud1 (new pcl::PointCloud<pcl::PointNormal>);
            pcl::io::loadPCDFile<pcl::PointNormal> (dataPath + "pointCloud_original" + std::to_string(0) + ".pcd", *pointCloud0);
            pcl::io::loadPCDFile<pcl::PointNormal> (dataPath + "pointCloud_original" + std::to_string(1) + ".pcd", *pointCloud1);
            pointCloud0->operator+=(*pointCloud1);
            pointCloud1->at(1).x = 123;
            std::cout << pointCloud0->at(1).x << " " << pointCloud1->at(1).x;
        }

        
        void normalSubSampling( ){
            std::cout << "sampling" << std::endl;
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudIn= pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::PointCloud<pcl::PointNormal>  cloudOut;
            pcl::Indices ind;
            pcl::io::loadPCDFile<pcl::PointNormal> (dataPath + "pointCloudCAD"+ ".pcd", *cloudIn);

            std::cout << cloudIn->size() << std::endl;

            pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> subSamp;
            subSamp.setSample(50000);
            subSamp.setInputCloud(cloudIn);            
            subSamp.setNormals(cloudIn);
            subSamp.setSeed(0);
            subSamp.setBins(100, 100, 100);
            subSamp.filter(cloudOut);
            std::cout << cloudOut.size() << std::endl;
        }

        void simulateScanning (){
            static int currIdxRobPose = 0;
            
            MyPointCloud::Ptr pointCloud = pcl::make_shared<MyPointCloud>();
            cv::Mat img;
            pcl::io::loadPCDFile<pcl::PointNormal> (dataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);
            img = cv::imread(dataPath + "img" + std::to_string(currIdxRobPose) + ".bmp", 0);// 0 je flag  cv::IMREAD_GRAYSCALE, POZOR LEBO DEFAULT JE BGR
            BreakNormals(pointCloud);
            publishData(pointCloud, img);

            currIdxRobPose++;
            currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;
        }
};

int main(int argc, char** argv){    
    ros::init(argc, argv, "tesdataPublisherInPC", ros::init_options::NoSigintHandler); 
    TestPub testPub;    
    return 0;
}
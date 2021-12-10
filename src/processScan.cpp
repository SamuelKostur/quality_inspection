#include <ros/ros.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include<pcl/filters/extract_indices.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <future>
#include <thread>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/package.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

#define SAVE_PARTIAL_DATA 1

using namespace std;
class ProcessScan{
    public:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub;
        ros::Subscriber textureSub;    
        std::vector<std::promise<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> promiseVec;
    	std::vector<std::future<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> futureVec;
    	std::mutex mtx;
        std::string dataPath;

        MyPointCloud::Ptr completePointCloud  = MyPointCloud::Ptr (new MyPointCloud);

        //vector of robot scan poses {x, y, z, A, B, C} {mm, mm, mm, deg, deg, deg}
        vector<vector<double>> robotScanPoses = {{780.80, 247.40, 204.88, 132.95, 3.4, 179.05},
                                             {749.80, -19.13, 240.26, 132.82, 4.95, 177.37},
                                             {203.01, -339.43, 219.94, 51.00, 4.86, -179.83},
                                             {294.49, 126.36, 598.63, 79.11, 61.83, 97.29},
                                             {345.37, 437.17, 598.85, 77.80, 62.26, 96.90},
                                             {-58.01, 611.61, 222.38, -38.61, 8.83, -170.97},
                                             {-85.37, 375.77, 212.30, -45.46, 6.59, -179.71}};
        
        int numRobPoses;
 
        ProcessScan(){
            createDataPath();
            cout << dataPath << endl;
            numRobPoses = robotScanPoses.size(); //this has to be first

            fillFutPromVec(); //call here to ensure that futures are ready before data callback is processed

            std::thread endlessThread (&ProcessScan::endlessLoop, this);

            message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(n, "/phoxi_camera/pointcloud", 1);
            message_filters::Subscriber<sensor_msgs::Image> textureSub(n, "phoxi_camera/texture", 1);
            message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pointCloudSub, textureSub, 10);
            sync.registerCallback(boost::bind(&ProcessScan::combCB, this, _1, _2));

            ros::spin();
            endlessThread.join(); // or use std:;thread::detach
        }

        void createDataPath(){
            dataPath = ros::package::getPath("quality_inspection"); 
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
            dataPath = dataPath + "/InspectionFiles/";
            boost::filesystem::create_directories(dataPath);
        }

        void extractUnmeasuredPoints(MyPointCloud::Ptr pointCloud){
            //Unmeasured points (pixels) caused by shadows are given the default coordinates (0,0,0).
            pcl::PointIndices::Ptr unmeasured (new pcl::PointIndices);
            for(int i = 0; i < pointCloud->size(); i++){
                if((pointCloud->points[i].x == 0)&&(pointCloud->points[i].y == 0)&&(pointCloud->points[i].z == 0)){
                    unmeasured->indices.push_back(i);
                }                
            }

            pcl::ExtractIndices<MyPoint> extraction;
            extraction.setInputCloud(pointCloud);
            extraction.setIndices(unmeasured);
            extraction.setNegative(true);
            extraction.filter(*pointCloud);
        }

        void robotPoseToMetersAndRadians(vector<double> &robotScanPoses){
            // KUKA robots use the so called XYZ-ABC format. 
            // XYZ is the position in millimeters. 
            // ABC are angles in degrees, with A rotating around z axis, B rotating around y axis and C rotating around x axis. 
            // The rotation convention is z-y′-x′′ (i.e. x-y-z) and computed by rz(A)ry(B)rx(C).
            robotScanPoses[0] = robotScanPoses[0] / 1000.0;
            robotScanPoses[1] = robotScanPoses[1] / 1000.0;
            robotScanPoses[2] = robotScanPoses[2] / 1000.0;
            robotScanPoses[3] = robotScanPoses[3] / 180.0 * M_PI;
            robotScanPoses[4] = robotScanPoses[4] / 180.0 * M_PI;
            robotScanPoses[5] = robotScanPoses[5] / 180.0 * M_PI;
        }

        void transformPointCloudFromTCPtoRobot(vector<double> robotPose, MyPointCloud::Ptr pointCloud){
            robotPoseToMetersAndRadians(robotPose);
            
            Eigen::Matrix3f R;
            R = Eigen::AngleAxisf(robotPose[3], Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(robotPose[4], Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(robotPose[5], Eigen::Vector3f::UnitX());
            
            Eigen::Vector3f T;
            T = Eigen::Vector3f(robotPose[0], robotPose[1], robotPose[2]);
 
            Eigen::Matrix4f transMatrix;
            transMatrix.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            transMatrix.block<3,3>(0,0) = R;
            transMatrix.block<3,1>(0,3) = T;
            pcl::transformPointCloud(*pointCloud, *pointCloud, transMatrix);            
        }

        cv::Mat createTextureImage(const sensor_msgs::Image::ConstPtr& originalTexture){
            cv::Mat img (originalTexture->height, originalTexture->width, CV_32FC1,
                                  (void*)(&originalTexture->data[0]));
            // before normalization and conversion is intensity scale probably 0-4095 
            // i measured min 4 and max 4094
            // but it probably somehow corresponds to LED power which is 0-4095
            cv::normalize(img, img, 0, 255, CV_MINMAX);
            img.convertTo(img, CV_8UC1);
            return img.clone();
        }

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr addTextureToPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud, cv::Mat& img){
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointCloudTexture (new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::copyPointCloud(*pointCloud, *pointCloudTexture);

            for(int rowIdx = 0; rowIdx < img.rows; rowIdx++){
                const uint8_t* imgRow = img.ptr<uint8_t>(rowIdx);
                for(int colIdx = 0; colIdx < img.cols; colIdx++){
                    pointCloudTexture->at(colIdx, rowIdx).intensity = imgRow[colIdx];
                }
            }
            return pointCloudTexture;
        }

        void combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture){
            static int currIdxRobPose = 0;
            cout << "combining partial point cloud with intensity " << currIdxRobPose << "..." << endl;
            pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud (new pcl::PointCloud<pcl::PointNormal>);
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);        
            cv::Mat img = createTextureImage(originalTexture);
            #if SAVE_PARTIAL_DATA
                cv::imwrite(dataPath + "img" + std::to_string(currIdxRobPose) + ".bmp", img);
            #endif
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointCloudNormI= addTextureToPointCloud(pointCloud, img);

            promiseVec[currIdxRobPose].set_value(pointCloudNormI);
            currIdxRobPose++;
            currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;
        }

        void processPointCloud(std::future<MyPointCloud::Ptr> future, int currIdxRobPose){
            cout << "waiting for partial point cloud" << currIdxRobPose << "..." << endl;
        	MyPointCloud::Ptr pointCloud = future.get();
            cout << "processing partial point cloud" << currIdxRobPose << "..." << endl;
            #if SAVE_PARTIAL_DATA
                pcl::io::savePCDFileASCII (dataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);
            #endif
            extractUnmeasuredPoints(pointCloud);  //prepisat typ co ide do funkcie alebo spravit template
            transformPointCloudFromTCPtoRobot(robotScanPoses[currIdxRobPose], pointCloud);
            #if SAVE_PARTIAL_DATA
                pcl::io::savePCDFileASCII (dataPath + "pointCloud" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);
            #endif
            mtx.lock();
            completePointCloud->operator+=(*pointCloud);
            mtx.unlock();
        }

        void fillFutPromVec(){
            futureVec.reserve(numRobPoses);
            promiseVec.reserve(numRobPoses);
            for(int i  = 0; i < numRobPoses; i++){
        		std::promise<MyPointCloud::Ptr> promise;
                futureVec.emplace(futureVec.begin() + i, promise.get_future());
        		promiseVec.emplace(promiseVec.begin() + i, std::move(promise));
        	}
        }

        void endlessLoop(){
        	std::vector<future<void>> threadVec;
            threadVec.reserve(8);
        	while(1){
        		//tu vytvorit 8 threadov na spracovanie point cloudu ktory je ulozeny v premennej ci uz globalnej alebo vo future,
        		//a nasledne pred pridavanim do celkoveho point cloudu este pridat mutex aby viacere thready naraz nepristupovali k tej istej premennej
        		//processing
        		for (int i = 0; i < numRobPoses; i++){
                    auto t = std::async(std::launch::async, &ProcessScan::processPointCloud, this, std::move(futureVec[i]), i);
        			threadVec.emplace(threadVec.begin() + i, std::move(t));     	
        		}
                
        		for (auto& t : threadVec){
                    if(t.wait_for(std::chrono::seconds(600)) != future_status::ready){
                        cout << "timeout over, problem procesing data" << endl;
                        return;
                    }			
        		}
                fillFutPromVec();
                
                pcl::io::savePCDFileASCII (dataPath + "completePointCloud.pcd", *completePointCloud);
                completePointCloud->clear();
                cout << "all processing done..." << endl; 
        		//tu uz mam spojeny completePointCLoud, mozem icp na model a extrahovat rivety
        		//na kazdy rivet jedno vlakno
        	}
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "processScanNode");    
    ProcessScan processScan;
    //processScan.endlessLoop();
    return 0;
}
#include <ros/ros.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include<pcl/filters/extract_indices.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

typedef pcl::PointNormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

#define FRAME_WIDTH 2064
#define FRAME_HEIGHT 1544

using namespace std;
class ProcessScan{
    public:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub;
        ros::Subscriber textureSub;        

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
            // pointCloudSub = n.subscribe<sensor_msgs::PointCloud2> ("/phoxi_camera/pointcloud", 1, &ProcessScan::pointCloudCB, this);
            // textureSub = n.subscribe<sensor_msgs::Image> ("/texture", 1, &ProcessScan::textureCB, this);

            message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(n, "/phoxi_camera/pointcloud", 1);
            message_filters::Subscriber<sensor_msgs::Image> textureSub(n, "/texture", 1);
            message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pointCloudSub, textureSub, 10);
            sync.registerCallback(boost::bind(&ProcessScan::combCB, _1, _2));

            numRobPoses = robotScanPoses.size();

            ros::spin();
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

        std::vector<std::vector<uint8_t>> createTextureImage(const sensor_msgs::Image::ConstPtr& originalTexture){
            //texture comes after point cloud
            std::vector<std::vector<uint8_t>> img(FRAME_HEIGHT, vector<uint8_t> (FRAME_WIDTH, 0));
            for(int row = 0; row < FRAME_HEIGHT; row++){
                for(int col = 0; col < FRAME_WIDTH; col++){
                    img[row][col] = originalTexture->data[(FRAME_WIDTH * row) + col];
                }
            }
            return img;
        }

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr addTextureToPointCloud(MyPointCloud::Ptr pointCloud, std::vector<std::vector<uint8_t>>& img){
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointCloudTexture (new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::copyPointCloud(*pointCloud, *pointCloudTexture);
            for (int row = 0; row < img.size(); row++){
                for (int col = 0; col < img[0].size(); col++){
                    pointCloudTexture->at(row, col).intensity = img[row][col];
                }
            }
            return pointCloudTexture;
        }

        void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud){
            cout << "processing started..." << endl;
            static int currIdxRobPose = 0;
            MyPointCloud::Ptr pointCloud (new MyPointCloud);
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);
            cout << "saving original point cloud..." << endl;
            pcl::io::savePCDFileASCII ("pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);
            cout << "extracting unmeasured points..." << endl;
            extractUnmeasuredPoints(pointCloud);  
            cout << "transforming point cloud from TCP to robot coord space..." << endl;   
            transformPointCloudFromTCPtoRobot(robotScanPoses[currIdxRobPose], pointCloud);
            completePointCloud->operator+=(*pointCloud);
            cout << "saving transformed point cloud..." << endl;   
            pcl::io::savePCDFileASCII ("pointCloud" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);

            currIdxRobPose++;
            if(currIdxRobPose == numRobPoses){
                cout << "saving complete point cloud..." << endl; 
                pcl::io::savePCDFileASCII ("completePointCloud.pcd", *completePointCloud);
                completePointCloud->clear();
            }
            currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;
            cout << "processing done..." << endl; 
        }

        void processPointCloud(){
            
        }

        void combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture){
            MyPointCloud::Ptr pointCloud (new MyPointCloud);
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);            
            std::vector<std::vector<uint8_t>> img = createTextureImage(originalTexture);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointCloudNormI= addTextureToPointCloud(pointCloud, img);


        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "processScanNode");
    ProcessScan processScan;
    return 0;
}
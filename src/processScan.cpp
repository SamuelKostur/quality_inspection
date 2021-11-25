#include <ros/ros.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include<pcl/filters/extract_indices.h>

typedef pcl::PointNormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;
using namespace std;
class ProcessScan{
    public:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub;

        MyPointCloud::Ptr completePointCloud  = MyPointCloud::Ptr (new MyPointCloud);

        //vector of robot poses {x, y, z, A, B, C}
        vector<vector<double>> robotPoses = {{113.47, 473.59, 256.68, -62.08, 20.18, 179.67},
                                             {418.90, 245.69, 662.63, -93.26, 60.94, 149.65}};
        
        int numRobPoses;
 
        ProcessScan(){
            pointCloudSub = n.subscribe<sensor_msgs::PointCloud2> ("/phoxi_camera/pointcloud", 1, &ProcessScan::callback, this);   
            numRobPoses = robotPoses.size();
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

        void robotPoseToMetersAndRadians(vector<double> &robotPoses){
            // KUKA robots use the so called XYZ-ABC format. 
            // XYZ is the position in millimeters. 
            // ABC are angles in degrees, with A rotating around z axis, B rotating around y axis and C rotating around x axis. 
            // The rotation convention is z-y′-x′′ (i.e. x-y-z) and computed by rz(A)ry(B)rx(C).

            robotPoses[0] = robotPoses[0] / 1000.0;
            robotPoses[1] = robotPoses[1] / 1000.0;
            robotPoses[2] = robotPoses[2] / 1000.0;

            robotPoses[3] = robotPoses[3] / 180.0 * M_PI;
            robotPoses[4] = robotPoses[4] / 180.0 * M_PI;
            robotPoses[5] = robotPoses[5] / 180.0 * M_PI;
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

        void callback(const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud){
            static int currIdxRobPose = 0;
            MyPointCloud::Ptr pointCloud (new MyPointCloud);
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);
            pcl::io::savePCDFileASCII ("pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);
            extractUnmeasuredPoints(pointCloud);           
            transformPointCloudFromTCPtoRobot(robotPoses[currIdxRobPose], pointCloud);
            completePointCloud->operator+=(*pointCloud);
            pcl::io::savePCDFileASCII ("pointCloud" + std::to_string(currIdxRobPose) + ".pcd", *pointCloud);

            currIdxRobPose++;
            if(currIdxRobPose == numRobPoses){
                pcl::io::savePCDFileASCII ("completePointCloud.pcd", *completePointCloud);
            }
            currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "processScanNode");
    ProcessScan processScan;
    ros::spin();
    return 0;
}
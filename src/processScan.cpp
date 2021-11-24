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

        //vector of robot poses {x, y, z, A, B, C}
        vector<vector<double>> robotPoses = {{493.54, 474.27, 408.72, -166.89, 33.72, 166.38},
                                             {604.56, 318.81, 365.03, 149.99, 28.06, 164.57},
                                             {469.91, -128.03, 408.67, 89.15, 23.61, 157.28},
                                             {344.85, -169.52, 378.81, 72.53, 23.56, 163.9},
                                             
                                             {272.59, -110.18, 512.98, 56.28, 34.47, 157.81},
                                             {300.44, 24.16, 568.90, 94.24, 39.68, 98.02},
                                             {325.92, 260.59, 613.00, 95.72, 61.34, 96.42},
                                             {329.21, 495.81, 572.70, 140.49, 68.32, 130.36},
                                             {323.73, 624.93, 494.75, -144.16, 61.44, 174.62},
 
                                             {15.38, 535.33, 484.33, -73.99, 41.96, 161.67},
                                             {-133.16, 437.75, 336.15, -45.23, 30.92, 176.66},
                                             {27.57, 358.33, 569.22, -44.55, 41.93, 145.90},
                                             {194.15, 393.29, 569.19, -87.52, 42.39, 131.79}};
 
        ProcessScan(){
            pointCloudSub = n.subscribe<sensor_msgs::PointCloud2> ("/phoxi_camera/pointcloud", 1, &ProcessScan::callback, this);
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

            Eigen::Affine3f transMatrix;
            transMatrix = (Eigen::Affine3f)Eigen::Translation3f(robotPose[0], robotPose[1], robotPose[2]);

            Eigen::Matrix3f rotMatrix;
            rotMatrix = Eigen::AngleAxisf(robotPose[3], Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(robotPose[4], Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(robotPose[5], Eigen::Vector3f::UnitZ());
            transMatrix.rotate(rotMatrix);
            
            pcl::transformPointCloud(*pointCloud, *pointCloud, transMatrix);            
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud){
            static int idx = 0;
            MyPointCloud::Ptr pointCloud (new MyPointCloud);  
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);
            extractUnmeasuredPoints(pointCloud);            
            transformPointCloudFromTCPtoRobot(robotPoses[idx], pointCloud);
            pcl::io::savePCDFileASCII ("pointCloud" + std::to_string(idx) + ".pcd", *pointCloud);
            idx++;
            printf("tu \n");
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "processScanNode");
    ProcessScan processScan;
    ros::spin();
    return 0;
}
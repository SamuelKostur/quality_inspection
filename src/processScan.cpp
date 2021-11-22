#include <ros/ros.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>

//scanning parameters can be defined in phoxi_camera/config/phoxi_camera.yaml
//coord space, transformacna matica, scanner id, and other scaning parameters
//in order to use phoxi ros interface: rosrun phoxi_camera phoxi_camera 
class ProcessScan{
    public:
        ros::NodeHandle n;
        ros::Subscriber pointCloudSub;

        ProcessScan(){
            pointCloudSub = n.subscribe<sensor_msgs::PointCloud2> ("/phoxi_camera/pointcloud", 1, &ProcessScan::callback, this);
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud){
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);  
            pcl::fromROSMsg(*originalPointCloud,*pointCloud);
            pcl::io::savePCDFileASCII ("pointCloud.pcd", *pointCloud);
        }


};

int main(int argc, char** argv){
    ros::init(argc, argv, "processScanNode");
    ProcessScan processScan;
    return 0;
}
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

#include <quality_inspection/create2Dprojections.h>

#include <pcl/filters/normal_space.h>

namespace pcl{
  using boost::shared_ptr;
};

typedef pcl::PointXYZINormal MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

void extractUnmeasuredPoints(MyPointCloud& pointCloud);
void robotPoseToMetersAndRadians(std::vector<double> &robotScanPoses);
void transformPointCloudFromTCPtoRobot(std::vector<double> robotPose, MyPointCloud& pointCloud);
cv::Mat createTextureImage(const sensor_msgs::Image::ConstPtr& originalTexture);
void addTextureToPointCloud(MyPointCloud& pointCloud, cv::Mat& img);
void correctNormals(MyPointCloud& pointCloud);
void extractTable(MyPointCloud& pointCloud);
void extractDistantPoints(pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, MyPointCloud& regCloud);

/////////////////////////templates////////////////////////////////
template<typename T>
Eigen::Affine3f transfToOrigin(pcl::shared_ptr<pcl::PointCloud<T>> cloud){
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Affine3f translation = (Eigen::Affine3f)Eigen::Translation3f(
                                -centroid[0], -centroid[1], -centroid[2]);

  newPcl::transformPointCloudWithNormals(*cloud, *cloud, translation);
  return translation;
}

template<typename T>
void randomSubSampling(boost::shared_ptr<pcl::PointCloud<T>> cloudIn, pcl::PointCloud<T>& cloudOut, unsigned int numSamples, unsigned int seed){
  pcl::RandomSample<T> subSamp;
  subSamp.setSample( std::min(numSamples, (unsigned int) cloudIn->size()));
  subSamp.setInputCloud(cloudIn);
  subSamp.setSeed(seed);
  subSamp.filter(cloudOut);
}

template<typename T>
void normalSubSampling(boost::shared_ptr<pcl::PointCloud<T>> cloudIn, pcl::PointCloud<T>& cloudOut, unsigned int numSamples, unsigned int seed){
  pcl::NormalSpaceSampling<T, T> subSamp;
  subSamp.setSample( std::min(numSamples, (unsigned int) cloudIn->size()));
  subSamp.setInputCloud(cloudIn);
  subSamp.setNormals(cloudIn);
  subSamp.setSeed(seed);
  subSamp.setBins(100, 100, 100);
  subSamp.filter(cloudOut);
}

template<typename T1>
Eigen::Affine3f doICP(pcl::shared_ptr<pcl::PointCloud<T1>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, 
          unsigned int minRandomSamples, unsigned int maxRandomSamples, int ICPiterNum, float maxMSE, float maxCorDist){
  //conversion because ICP requaires same Point type of both clouds, and since i need normals i need at least PointNormal
  pcl::PointCloud<pcl::PointNormal>::Ptr regCloudPNormal (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*regCloud, *regCloudPNormal);  

  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setMaxCorrespondenceDistance (maxCorDist);
  icp.setMaximumIterations (ICPiterNum);
  icp.setTransformationEpsilon(maxMSE); //temporary as maxMSE

  pcl::PointCloud<pcl::PointNormal>::Ptr tempRegCloud (new pcl::PointCloud<pcl::PointNormal>);
  auto tsubsample = std::chrono::high_resolution_clock::now();
  normalSubSampling(regCloudPNormal, *tempRegCloud, maxRandomSamples, 10);
  auto t1 = std::chrono::high_resolution_clock::now();
  icp.setInputSource(tempRegCloud);
  icp.setInputTarget(refCloud);
  icp.align(*tempRegCloud);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto durationSubsampling = std::chrono::duration_cast<std::chrono::microseconds>(t1 - tsubsample);
  std::cout << "duration ICP: " << durationICP.count()/1000000.0 <<  "duration subsampling: " << durationSubsampling.count()/1000000.0 <<std::endl;
  newPcl::transformPointCloudWithNormals(*regCloud, *regCloud, icp.getFinalTransformation());
  return (Eigen::Affine3f) icp.getFinalTransformation();
}

// template<typename T1>
// Eigen::Affine3f doICP(pcl::shared_ptr<pcl::PointCloud<T1>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, 
//           int minRandomSamples, int maxRandomSamples, int ICPiterNum, float maxMSE, float maxCorDist){
//   //conversion because ICP requaires same Point type of both clouds, and since i need normals i need at least PointNormal
//   pcl::PointCloud<pcl::PointNormal>::Ptr regCloudPNormal (new pcl::PointCloud<pcl::PointNormal>);
//   pcl::copyPointCloud(*regCloud, *regCloudPNormal);  

//   pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
//   icp.setMaxCorrespondenceDistance (maxCorDist);
//   icp.setMaximumIterations (1);
//   //icp.setEuclideanFitnessEpsilon (6e-5);

//   Eigen::Affine3f totalTransform(Eigen::Affine3f::Identity());
//   pcl::RandomSample<pcl::PointNormal> r;
//   //r.setSample(randomSamples);
//   int sampInc = (maxRandomSamples-minRandomSamples)/(ICPiterNum-1);
//   pcl::PointCloud<pcl::PointNormal>::Ptr tempRegCloud (new pcl::PointCloud<pcl::PointNormal>);
//   for (int i = 0; i < ICPiterNum ; i++ ){
//     auto t1 = std::chrono::high_resolution_clock::now(); 
//     if(maxRandomSamples < regCloud->size()){
//       r.setSample((int)(minRandomSamples + (sampInc * i)));
//       r.setInputCloud(regCloudPNormal);
//       r.setSeed(i*10);
//       r.filter(*tempRegCloud);
//       icp.setInputSource(tempRegCloud);
//     }
//     else{
//       icp.setInputSource(regCloudPNormal);
//     }

//     icp.setInputTarget(refCloud);

//     auto t2 = std::chrono::high_resolution_clock::now();
//     icp.align(*tempRegCloud);
//     auto t3 = std::chrono::high_resolution_clock::now(); 
//     // dont need to use transformPointCloudWithNormals because pcl::IterativeClosestPointWithNormals by default calculates point to plane distance
//     // based on point from regCloud and plane from refCloud, so transforming regCloudNormals doesnt do anything just takes longer
//     // but it can be set to plane to plane so its worth to try in the future
//     newPcl::transformPointCloud(*regCloudPNormal, *regCloudPNormal, icp.getFinalTransformation());
//     totalTransform *= icp.getFinalTransformation();
//     auto t4 = std::chrono::high_resolution_clock::now();

//     auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
//     auto durationBEFORE = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
//     auto durationAFTER = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
//     std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
//     std::cout << "duration ICP: " << durationICP.count()/1000000.0 <<
//     "duration OTHER: " << durationBEFORE.count()/1000000.0 <<
//     "duration AFTER: " << durationAFTER.count()/1000000.0 << std::endl;
//     std::cout << "iteration" << i << " error " << icp.getFitnessScore() << std::endl;

//     if(icp.getFitnessScore() <  maxMSE){
//         std::cout << "icp converged with MSE" << icp.getFitnessScore() << std::endl;
//         break;
//     }
//   }
//   newPcl::transformPointCloudWithNormals(*regCloud, *regCloud, totalTransform);

//   // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   // icp.getFitnessScore() << std::endl;
//   // std::cout << icp.getFinalTransformation() << std::endl;

//   return totalTransform;
// }

template<typename T>
Eigen::Affine3f alignPointClouds(pcl::shared_ptr<pcl::PointCloud<T>> regCloud, 
pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, int minRandomSamples, int maxRandomSamples, int ICPiterNum, float maxMSE, float maxCorDist){
  Eigen::Affine3f totalTransf(Eigen::Affine3f::Identity());
  auto t1 = std::chrono::high_resolution_clock::now();
  totalTransf =  transfToOrigin(regCloud);
  auto t2 = std::chrono::high_resolution_clock::now();
  totalTransf = doICP(regCloud, refCloud, minRandomSamples, maxRandomSamples, ICPiterNum, maxMSE, maxCorDist) * totalTransf;
  auto t3 = std::chrono::high_resolution_clock::now();
  
  auto durationCentroid = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  std::cout << "total ICP duration: " << durationICP.count()/1000000.0 << 
  "total centroid transf duration: " << durationCentroid.count()/1000000.0 << 
  std::endl << std::endl << std::endl;
  return totalTransf;
}

template<typename T>
int loadPointCloud(pcl::shared_ptr<pcl::PointCloud<T>> cloud, std::string name){
    //load point cloud from pcd file
    if (pcl::io::loadPCDFile<T> (name, *cloud) == -1) //* load the file
    {
      PCL_ERROR (std::string("Couldn't read file").append(name).append("\n").c_str());
      return (-1);
    }
    std::cout << "Loaded "
      << cloud->width * cloud->height
      << " data points from" +  name + "with the following fields: "
      << std::endl;
    return 0;
}

template<typename T1, typename T2>
std::vector<float> calculateDistances(pcl::shared_ptr<pcl::PointCloud<T1>> refCloud, pcl::PointCloud<T2>& cloud){
  //in this case ref cloud can be also registered cloud, when distances are calculated for every point of CAD model
  std::vector<float> distances;
  distances.resize(refCloud->size());

  pcl::search::KdTree<pcl::PointNormal> kdtree;
  kdtree.setInputCloud(refCloud);
  pcl::PointIndices neighbourIndices;
  neighbourIndices.indices.resize(1);
  std::vector<float> neighbourSqrDistances;
  neighbourSqrDistances.resize(1);
  
  // //point to point
  // for (int i = 0; i < cloud.size(); i++){
  //   kdtree.nearestKSearchT(cloud[i], 1, neighbourIndices.indices, neighbourSqrDistances);
  //   distances[i] = std::sqrt(neighbourSqrDistances[0]);
  // }

  //point to plane
  for (int i = 0; i < cloud.size(); i++){
    kdtree.nearestKSearchT(cloud[i], 1, neighbourIndices.indices, neighbourSqrDistances);
    T1 neighbour = refCloud->at(neighbourIndices.indices[0]);
    //project distance vector on normal vector
    float dotProduct = ((cloud.at(i).x - neighbour.x) * neighbour.normal_x +
                        (cloud.at(i).y - neighbour.y) * neighbour.normal_y +
                        (cloud.at(i).z - neighbour.z) * neighbour.normal_z);
    distances[i] = std::abs(dotProduct); //dont need to divide by length of normal since normal is normalized
  }

  return distances;
}
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
#include <my_transform.hpp>
#include <mutex>
#include <condition_variable>
#include <std_srvs/SetBool.h>
#include <chrono>
#include <my_semaphore.h>

//point cloud registration
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/random_sample.h>

#include <quality_inspection/create2Dprojections.h>

#include <pcl/filters/normal_space.h>
#include <pcl/filters/voxel_grid.h>

#include <auxiliaries.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <math.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

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
void extractTable(MyPointCloud& pointCloud, float tableZcoord);
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
          unsigned int maxRandomSamples, int ICPiterNum, float maxTE, float maxCorDist){
  //conversion because ICP requaires same Point type of both clouds, and since i need normals i need at least PointNormal
  pcl::PointCloud<pcl::PointNormal>::Ptr regCloudPNormal (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*regCloud, *regCloudPNormal);  
  
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setMaxCorrespondenceDistance (maxCorDist);
  icp.setMaximumIterations (ICPiterNum);
  icp.setTransformationEpsilon(maxTE); //temporary as maxMSE 

  pcl::PointCloud<pcl::PointNormal>::Ptr tempRegCloud (new pcl::PointCloud<pcl::PointNormal>);
  auto tsubsample = std::chrono::high_resolution_clock::now();
  normalSubSampling(regCloudPNormal, *tempRegCloud, maxRandomSamples, 10);
  auto t1 = std::chrono::high_resolution_clock::now();

  //add rejection of correspondences based on surface normals
  // pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr crsn (new pcl::registration::CorrespondenceRejectorSurfaceNormal);
  // crsn->setThreshold(cos(M_PI/18.f)); //20 degrees
  // crsn->initializeDataContainer<pcl::PointNormal, pcl::PointNormal>();
  // crsn->setInputSource<pcl::PointNormal>(tempRegCloud);
  // crsn->setInputTarget<pcl::PointNormal>(refCloud);
  // crsn->setInputNormals<pcl::PointNormal,pcl::PointNormal>(tempRegCloud);
  // crsn->setTargetNormals<pcl::PointNormal,pcl::PointNormal>(refCloud);
  // icp.addCorrespondenceRejector(crsn);
  // std::vector<pcl::registration::CorrespondenceRejector::Ptr> rejVec = icp.getCorrespondenceRejectors();
  // for(auto &rej : rejVec){
  //   std::cout << rej->getClassName() << std::endl;
  // }
  // one to oen correspondence rejector
  //pcl::registration::CorrespondenceRejectorOneToOne::Ptr croto (new pcl::registration::CorrespondenceRejectorOneToOne);
  // croto->setSourcePoints<pcl::PointNormal>(tempRegCloud);
  // croto->setTargetPoints<pcl::PointNormal>(refCloud);
  //icp.addCorrespondenceRejector(croto);

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

template<typename T>
void fineRegPPCtoCAD(pcl::shared_ptr<pcl::PointCloud<T>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud){
    doICP(regCloud, refCloud, 25000, 30, 1e-10, 0.003);
}

template<typename T>
Eigen::Affine3f doFPFH(pcl::shared_ptr<pcl::PointCloud<T>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, std::string path){
 pcl::PointCloud<pcl::PointNormal>::Ptr regCloudPNormal (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*regCloud, *regCloudPNormal);  

  auto t1 = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointNormal>::Ptr tempRefCloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::VoxelGrid<pcl::PointNormal> sor;
  sor.setInputCloud (refCloud);
  sor.setLeafSize (0.015f, 0.015f, 0.015f);
  sor.filter (*tempRefCloud);
  pcl::io::savePCDFileBinary (path + "sampledRef.pcd", *tempRefCloud);

  auto t2 = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointNormal>::Ptr tempRegCloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::VoxelGrid<pcl::PointNormal> sor2;
  sor2.setInputCloud (regCloudPNormal);
  sor2.setLeafSize (0.015f, 0.015f, 0.015f);
  sor2.filter (*tempRegCloud);
  pcl::io::savePCDFileBinary (path + "sampledReg.pcd", *tempRegCloud);

  auto t3 = std::chrono::high_resolution_clock::now();
  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(tempRefCloud);
  fpfh.setInputNormals (tempRefCloud);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  fpfh.setSearchMethod (tree);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_ref (new pcl::PointCloud<pcl::FPFHSignature33> ());
  fpfh.setRadiusSearch (0.05);
  fpfh.compute (*fpfhs_ref);

  auto t4 = std::chrono::high_resolution_clock::now();

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh2;
  fpfh2.setInputCloud(tempRegCloud);
  fpfh2.setInputNormals (tempRegCloud);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  fpfh.setSearchMethod (tree2);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_reg (new pcl::PointCloud<pcl::FPFHSignature33> ());
  fpfh2.setRadiusSearch (0.05);
  fpfh2.compute (*fpfhs_reg);

  auto t5 = std::chrono::high_resolution_clock::now();
  
  pcl::SampleConsensusInitialAlignment<pcl::PointNormal,pcl::PointNormal, pcl::FPFHSignature33> sac_ia_;
  sac_ia_.setNumberOfSamples(3);  
  sac_ia_.setCorrespondenceRandomness(1);
  sac_ia_.setMinSampleDistance (0.03);
  sac_ia_.setMaxCorrespondenceDistance (1);
  sac_ia_.setMaximumIterations(1000);
  sac_ia_.setInputTarget (tempRefCloud);
  sac_ia_.setTargetFeatures(fpfhs_ref);
  sac_ia_.setInputSource (tempRegCloud);
  sac_ia_.setSourceFeatures (fpfhs_reg);
  sac_ia_.align (*tempRegCloud);
  Eigen::Matrix4f tfm;
  tfm =sac_ia_.getFinalTransformation();

  auto t6 = std::chrono::high_resolution_clock::now();

  auto durationSubsampleRef= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto durationSubsampleReg = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  auto durationFPFHref = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
  auto durationFPFHreg = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4);
  auto durationSACIA = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5);
  std::cout << "Subsampling ref : " << durationSubsampleRef.count()/1000000.0 << 
  " Subsampling reg : " << durationSubsampleReg.count()/1000000.0 << 
  " FPFH ref : " << durationFPFHref.count()/1000000.0 << 
  " FPFH reg : " << durationFPFHreg.count()/1000000.0 << 
  " SAC_IA: " << durationSACIA.count()/1000000.0 << 
  std::endl;

  newPcl::transformPointCloudWithNormals(*regCloud, *regCloud, tfm);
  std::cout << tfm.matrix() << std::endl;
  return (Eigen::Affine3f) tfm;
}

template<typename T>
Eigen::Affine3f coarseCPCtoCAD(pcl::shared_ptr<pcl::PointCloud<T>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, std::string path){  
  //use this when there is no good initial guess for transformation of the arriving parts, because they can be differently oriented and located
  Eigen::Affine3f totalTransf(Eigen::Affine3f::Identity());
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = std::chrono::high_resolution_clock::now();
  totalTransf = doFPFH(regCloud, refCloud, path);
  pcl::io::savePCDFileBinary (path + "completePointCloudSACIA.pcd", *regCloud);
  totalTransf = doICP(regCloud, refCloud, 100000, 12, 1e-5, 0.1) * totalTransf;
  auto t3 = std::chrono::high_resolution_clock::now();
  
  auto durationCentroid = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  std::cout << "total ICP duration: " << durationICP.count()/1000000.0 << 
  "total centroid transf duration: " << durationCentroid.count()/1000000.0 << 
  std::endl << std::endl << std::endl;
  return totalTransf;
}

template<typename T>
Eigen::Affine3f coarseCPCtoCAD(pcl::shared_ptr<pcl::PointCloud<T>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, 
                               Eigen::Affine3f initialGuess, std::string path){  
//use this when there is good initial guess for transformation of the arriving parts, because they are alike oriented and located
  Eigen::Affine3f totalTransf(Eigen::Affine3f::Identity());
  auto t1 = std::chrono::high_resolution_clock::now();
  totalTransf = totalTransf * initialGuess;
  newPcl::transformPointCloudWithNormals(*regCloud, *regCloud, initialGuess);
  //totalTransf =  transfToOrigin(regCloud);
  auto t2 = std::chrono::high_resolution_clock::now();
  pcl::io::savePCDFileBinary (path + "completePointCloudSACIA.pcd", *regCloud);
  totalTransf = doICP(regCloud, refCloud, 100000, 12, 1e-5, 0.1) * totalTransf;
  auto t3 = std::chrono::high_resolution_clock::now();
  
  auto durationCentroid = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  std::cout << "total ICP duration: " << durationICP.count()/1000000.0 << 
  "total centroid transf duration: " << durationCentroid.count()/1000000.0 << 
  std::endl << std::endl << std::endl;
  return totalTransf;
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
#pragma once

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <newTransform.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>

namespace pcl{
  using boost::shared_ptr;
  using Indices = std::vector<int>;
};


class Projections{ 
  public:
  int imgWidth = 85;
  int imgHeight = 85;
  float imgRange = 0.02;
  float pixelSize = imgRange / (float) imgWidth;

    template<typename T>
    void extractRivetPointCloud(pcl::shared_ptr<pcl::PointCloud<T>> originalCloud, pcl::PointCloud<T>& rivetCloud, float *rivetLocation) {
      //this will extract part of the point cloud inside of cube with side length 2*offset meters and center in rivetLocation
      float offset = 0.01; //in meters
      pcl::CropBox<T> filter;
      filter.setNegative(false);
      filter.setMin(Eigen::Vector4f(rivetLocation[0] - offset, rivetLocation[1] - offset, rivetLocation[2] - offset, 1.0)); //minX, minY, minZ, normalization(right bottom element homMat[4][4] of homogenous matrix)
      filter.setMax(Eigen::Vector4f(rivetLocation[0] + offset,  rivetLocation[1] + offset, rivetLocation[2] + offset, 1.0)); //maX, maxY, maxZ, normalization
      filter.setInputCloud(originalCloud);
      filter.filter(rivetCloud);
    }

    template<typename T>
    void rotateRivetPointCloudToXYplane(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //rotate rivet point cloud so it will lay in the XY plane
      //problem is i cant be sure whether rotated point cloud doesnt have reversed Z coordinate
      //i dont understand everything step i am doing here
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<T> seg;
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.001);
      seg.setInputCloud(rivetCloud);
      seg.segment(*inliers, *coefficients);

      Eigen::Matrix<float,1,3> rivetPlaneNormal;
      rivetPlaneNormal[0] = coefficients->values[0];
      rivetPlaneNormal[1] = coefficients->values[1];
      rivetPlaneNormal[2] = coefficients->values[2];

      Eigen::Matrix<float,1,3> xyPlaneNormal;
      xyPlaneNormal[0] = 0.0f;
      xyPlaneNormal[1] = 0.0f;
      xyPlaneNormal[2] = 0.1f;

      Eigen::Matrix<float,1,3> rotationVector = xyPlaneNormal.cross(rivetPlaneNormal);
      float theta = -atan2(rotationVector.norm(), xyPlaneNormal.dot(rivetPlaneNormal));

      Eigen::Affine3f transToXYPlane = Eigen::Affine3f::Identity();
      transToXYPlane.rotate(Eigen::AngleAxisf (theta, rotationVector.normalized()));
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, transToXYPlane);
    }

    template<typename T>
    void cutRelevantRivetCloudArea(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //crop rivet point cloud so that only relevant circular area around Rivet remains
      float relevantRadius = 0.01; //a bit bigger then offset
      pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
      pcl::ExtractIndices<T> extraction;

      for(int i = 0; i < rivetCloud->points.size(); i++){
        if ( sqrt( pow(rivetCloud->points[i].x,2.0) + pow(rivetCloud->points[i].y,2.0)) >= relevantRadius){
          outliers->indices.push_back(i);
        }
      }
      extraction.setInputCloud(rivetCloud);
      extraction.setIndices(outliers);
      extraction.setNegative(true);
      extraction.filter(*rivetCloud);
    }

    template<typename T>
    void adjustRivetPointCloud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, float *rivetLocation){
      //translate rivet point cloud so the beginning of the coordinate system will be in the rivet location
      Eigen::Affine3f translation= (Eigen::Affine3f)Eigen::Translation3f(-rivetLocation[0], -rivetLocation[1], -rivetLocation[2]);
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, translation);
      
      //rotate rivet point cloud so it will lay in the XY plane
      rotateRivetPointCloudToXYplane(rivetCloud);
      
      //crop rivet point cloud so that only relevant circular area around Rivet remains
      cutRelevantRivetCloudArea(rivetCloud);
      
      //translate cropped rivet point cloud so every point has a positive XY coordinate
      float offset = 0.01;
      Eigen::Affine3f translationToPositiveXY= (Eigen::Affine3f)Eigen::Translation3f(+offset, +offset, -0);
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, translationToPositiveXY);      
    }

    //this is defined as inline since it is member function
    //so it can be defined across multiple translational units
    void writeRGBTensorToBitmap(std::vector<std::vector<std::vector<int>>> RGBtensor,std::string fileName){
      //write 3D vector (3*widht*height RGBtensor) to bitmap image, i dont understand everything what is happening here
      int w = RGBtensor[0][0].size();
      int h = RGBtensor[0].size();

      //char filePath[100];
      //strcpy(filePath, "Images/");
      //strcat(filePath, fileName);
      
      const char* filePath = fileName.c_str();

      FILE *f;
      unsigned char *img = NULL;
      int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

      img = (unsigned char *)malloc(3*w*h);
      memset(img,0,3*w*h);
      int x;
      int y;
      int r;
      int g;
      int b;

      for(int i=0; i<w; i++)
      {
          for(int j=0; j<h; j++)
          {
              x=i; y=(h-1)-j;
              r = RGBtensor[0][i][j];
              g = RGBtensor[1][i][j];
              b = RGBtensor[2][i][j];
              if (r > 255) r=255;
              if (g > 255) g=255;
              if (b > 255) b=255;
              img[(x+y*w)*3+2] = (unsigned char)(r);
              img[(x+y*w)*3+1] = (unsigned char)(g);
              img[(x+y*w)*3+0] = (unsigned char)(b);
          }
      }

      unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
      unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
      unsigned char bmppad[3] = {0,0,0};

      bmpfileheader[ 2] = (unsigned char)(filesize    );
      bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
      bmpfileheader[ 4] = (unsigned char)(filesize>>16);
      bmpfileheader[ 5] = (unsigned char)(filesize>>24);

      bmpinfoheader[ 4] = (unsigned char)(       w    );
      bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
      bmpinfoheader[ 6] = (unsigned char)(       w>>16);
      bmpinfoheader[ 7] = (unsigned char)(       w>>24);
      bmpinfoheader[ 8] = (unsigned char)(       h    );
      bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
      bmpinfoheader[10] = (unsigned char)(       h>>16);
      bmpinfoheader[11] = (unsigned char)(       h>>24);

      f = fopen(filePath, "wb");
      fwrite(bmpfileheader,1,14,f);
      fwrite(bmpinfoheader,1,40,f);
      for(int i=0; i<h; i++)
      {
          fwrite(img+(w*(h-i-1)*3),3,w,f);
          fwrite(bmppad,1,(4-(w*3)%4)%4,f);
      }

      free(img);
      fclose(f);
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> create2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection (without further features)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 255)));
      
      for (int i = 0; i < rivetCloud->size(); i++){
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
      }
      return RGBtensor;
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> createDepth2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection  based on depth (Depth image)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me (so depth is towards me)
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 0)));

      float avgDepth = 0.0f;
      for(int i = 0; i < rivetCloud->points.size(); i++){
        avgDepth += rivetCloud->points[i].z;
      } 
      avgDepth /= rivetCloud->points.size();

      float depthRangeOffset = 0.0003; // in meters, allong z axis, represents positive or negative offset from average depth value(or some other defined value), the whole accepted range is then depthRangeOffset*2
      int colorIntensity;
      
      for (int i = 0; i < rivetCloud->size(); i++){
        colorIntensity = (int) (((std::max(std::min((rivetCloud->points[i].z - avgDepth),depthRangeOffset),-depthRangeOffset) + depthRangeOffset) / (depthRangeOffset*2)) * 255.0);
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 255 - colorIntensity;
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = colorIntensity;
        //B clorIntensity because virtual Z axis in image plane is towards me, so closer object to me (z axis has higher value) is more blue it is and more distant object from me is, more red it is
      }
      return RGBtensor;
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> createDepthDiff2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection  based on depth (Depth image)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and ZPointCloudConstPtr = typename PointCloud::ConstPtr axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 0)));

      //set all Z coordinate values to 0 in order to execute search of closest neighbours in xy plane regardless of their Z coordinate
      pcl::PointCloud<T> xyRivetCLoud = *rivetCloud;
      for(int i = 0; i < rivetCloud->size(); i++){
        xyRivetCLoud.points[i].z = 0;
      }
      
      std::vector<float> depthDiffs(rivetCloud->size(), 0);
      pcl::search::KdTree<T> kdtree;
      float radius = 0.0008; //0.0017
      unsigned int maxNumNeighbours = 0; // If set to 0 or to a number higher than the number of points in the input cloud, all neighbors in radius will be returned. 
      pcl::Indices neighboursIndices;
      std::vector<float> neighbourSqrDistances;
      kdtree.setInputCloud(xyRivetCLoud.makeShared());
      for (int indRiv = 0; indRiv < rivetCloud->size(); indRiv++){
          kdtree.radiusSearch(indRiv, radius, neighboursIndices, neighbourSqrDistances, maxNumNeighbours);
          for (int indNbr = 0; indNbr < neighboursIndices.size(); indNbr++){
            depthDiffs[indRiv] += abs(rivetCloud->points[indRiv].z - rivetCloud->points[neighboursIndices[indNbr]].z);
          }
          if(neighboursIndices.size() > 0){
            depthDiffs[indRiv] /= neighboursIndices.size();
          }
      }

      //edge detection with this cycle and even smaller radius
      // for (int i = 0; i < depthDiffs.size(); i++){
      //   if(depthDiffs[i] < 0.0002){
      //     depthDiffs[i] = 0;
      //   }
      // }

      float depthDiffRange = 0.00011;
      int colorIntensity;

      for (int i = 0; i < rivetCloud->size(); i++){
        colorIntensity = std::min(depthDiffs[i]/depthDiffRange, 1.0f) * 255.0;
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = colorIntensity;
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = colorIntensity;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = colorIntensity;
        //the higher the difference, the bigger the color instensity
      }
      return RGBtensor;
    }

    //this is defined as inline since it is member function
    //so it can be defined across multiple translational units
    std::vector<int> HSVtoRGB(float H, float S,float V){
      if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
          std::cout<<"The givem HSV values are not in valid range"<<std::endl;
          return {0, 0, 0};
      }
      float s = S/100;
      float v = V/100;
      float C = s*v;
      float X = C*(1-abs(fmod(H/60.0, 2)-1));
      float m = v-C;
      float r,g,b;
      if(H >= 0 && H < 60){
          r = C,g = X,b = 0;
      }
      else if(H >= 60 && H < 120){
          r = X,g = C,b = 0;
      }
      else if(H >= 120 && H < 180){
          r = 0,g = C,b = X;
      }
      else if(H >= 180 && H < 240){
          r = 0,g = X,b = C;
      }
      else if(H >= 240 && H < 300){
          r = X,g = 0,b = C;
      }
      else{
          r = C,g = 0,b = X;
      }
      int R = (r+m)*255;
      int G = (g+m)*255;
      int B = (b+m)*255;
      return {R, G, B};
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> createNormals2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      //creates two 3*width*height RGBtensors from point cloud to represent 2D projection based on Normals and Curvature
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 255)));
      
      float xIdx, yIdx, xNormal, yNormal, ang;
      float PI = 3.1416;
      std::vector<int> rgbVec;
      //calculate RGBtensor based on Normals
      for (int i = 0; i < rivetCloud->size(); i++){        
        xIdx = floor(rivetCloud->points[i].x / pixelSize);
        yIdx = floor(rivetCloud->points[i].y / pixelSize);
        xNormal = rivetCloudNormals->points[i].normal_x;
        yNormal = rivetCloudNormals->points[i].normal_y;
        ang = atan2(yNormal, xNormal)/PI*180;
        rgbVec = HSVtoRGB(ang > 0 ? ang : (360 + ang), std::min(100.0, 200.0 * abs((xNormal + yNormal)/1)), 100);
        RGBtensor[0][xIdx][yIdx] = rgbVec[0];
        RGBtensor[1][xIdx][yIdx] = rgbVec[1];
        RGBtensor[2][xIdx][yIdx] = rgbVec[2];
        //normal vector size is 1 (sqrt(nx^2+ny^2+nz^2)) and most of it (usually about 0.999) belongs to Z axis since i am in XY plane,
        // because of that HUE is describing vector inclination in XY plane projection based on nx and ny (0-360 degrees, 0is red, 180 is cyan)
        // and SATURATION decsribes the (nx + ny) portion of the vector size, the higher the (nx + ny) portion is the higher the saturation (more color)
      }
      return RGBtensor;
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> createCurvature2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      //creates two 3*width*height RGBtensors from point cloud to represent 2D projection based on Normals and Curvature
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 255)));
      
      // for(int i = 0; i < rivetCloudNormals->size(); i++){
      //   std::cout << rivetCloudNormals->points[i].curvature << std::endl;
      // }
      float curvRelRange = 0.02; // curvature relevant range, experimentaly determined from 1 rivetCloud in such way that
      //c values coresponding to individual points ranged from 0 to 0.007, so i added a bit reserve)
      //Curvature is calculated c = lambda0 / (lambda0 + lamda1 + lambda2), where lambdax is eigenvalue of Covariance matrix. 
      //Covariance matrix is positive semidefinite symetrical matrix, what implies that its eigenvalues are nonnegative.
      //Since eigenvalues are nonnegative, value of c is in interval <0;1>

      //calculate RGBtensor based on Curvature
      for (int i = 0; i < rivetCloud->size(); i++){
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = (int) std::min(255.0, (rivetCloudNormals->points[i].curvature /curvRelRange * 255.0));
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        //R color intensity increases with increasing curvature, min() added in order to evoid error because its possible that some sample will have bigger curvature then defined range
      }
      return RGBtensor;
    }
    
    template<typename T>
    std::vector<std::vector<std::vector<int>>> createRSD2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector< std::vector<int> >(imgWidth, std::vector<int>(imgHeight, 255)));

      // RSD estimation object.
      pcl::RSDEstimation<T, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
      rsd.setInputCloud(rivetCloud);

      rsd.setInputNormals(rivetCloudNormals);
      pcl::shared_ptr<pcl::search::KdTree<T>> tree (new pcl::search::KdTree<T> ());
      rsd.setSearchMethod(tree);
      // Search radius, to look for neighbors. Note: the value given here has to be
      // larger than the radius used to estimate the normals.
      rsd.setRadiusSearch(0.0008);
      // Plane radius. Any radius larger than this is considered infinite (a plane).
      rsd.setPlaneRadius(0.008);
      rsd.setSaveHistograms(false);

      // Object for storing the RSD descriptors for each point.
      pcl::shared_ptr<pcl::PointCloud<pcl::PrincipalRadiiRSD>> descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
      rsd.compute(*descriptors);
      // for(int i = 0; i < descriptors->size(); i++){
      //   std::cout << descriptors->points[i].r_max  << "  " << descriptors->points[i].r_min << std::endl;
      // }

      //calculate RGBtensor based on RSD radius
      for (int i = 0; i < rivetCloud->size(); i++){
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = (int) (255.0 - std::min(255.0, (descriptors->points[i].r_min/rsd.getPlaneRadius() * 255.0)));
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = 0;
        //R color intensity increases with increasing curvature
      }
      return RGBtensor;
    }

    template<typename T>
    pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      pcl::NormalEstimation<T, pcl::Normal> normalEstimation;
      normalEstimation.setInputCloud(rivetCloud);

      pcl::shared_ptr<pcl::search::KdTree<T>> tree (new pcl::search::KdTree<T> ());
      normalEstimation.setSearchMethod (tree);

      pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius:
      normalEstimation.setRadiusSearch (0.0008); //0.0017

      // set coordinates of viewpoint to which normals will be consistently oriented in order to prevent their flipping
      // x y doesnt really matter because i am in XY plane and therefore it wont invoke flipping, what matters is Z coordinate of viewpoint
      // since in 2d projection is Z axis towards me, viewpoint z coordinate can be for example 0.1
      normalEstimation.setViewPoint(0.0, 0.0, 0.1);

      // Compute the normals and curvature, compute() returns [nx,ny.nz,curvature]
      normalEstimation.compute(*rivetCloudNormals); //rivetCLoudNormals has the same size like rivetCloud (i checked it)

      return rivetCloudNormals;
    }

    template<typename T>
    std::vector<std::vector<std::vector<int>>> createIntensity2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection (without further features)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      std::vector<std::vector<std::vector<int>>> RGBtensor (3, std::vector<std::vector<int>>(imgWidth, std::vector<int>(imgHeight, 255)));
      
      for (int i = 0; i < rivetCloud->size(); i++){
        RGBtensor[0][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = rivetCloud->points[i].intensity;
        RGBtensor[1][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = rivetCloud->points[i].intensity;
        RGBtensor[2][floor(rivetCloud->points[i].x / pixelSize)][floor(rivetCloud->points[i].y / pixelSize)] = rivetCloud->points[i].intensity;
      }
      return RGBtensor;
    }

    template<typename T>
    void create2DProjectionsImages(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, std::string dataPath){
      // Eigen::Vector4f minimums, maximums;
      // pcl::getMinMax3D(*rivetCloud,minimums, maximums);
      // std::cout << minimums << std::endl;
      // std::cout << maximums << std::endl;
      // float x_range = maximums.x() - minimums.x(); //just used for testing to to ensure that decided range is right
      // float y_range = maximums.y() - minimums.x(); //just used for testing ensure that decided range is right

      std::vector<std::vector<std::vector<int>>> RGBtensor = create2DprojectionFromPointCLoud(rivetCloud);
      writeRGBTensorToBitmap(RGBtensor, dataPath + "2Dprojection.bmp");
      std::vector<std::vector<std::vector<int>>> RGBtensorDepth = createDepth2DprojectionFromPointCLoud(rivetCloud);
      writeRGBTensorToBitmap(RGBtensorDepth, dataPath + "Depth2Dprojection.bmp");
      std::vector<std::vector<std::vector<int>>> RGBtensorDepthDiff = createDepthDiff2DprojectionFromPointCLoud(rivetCloud);
      writeRGBTensorToBitmap(RGBtensorDepthDiff, dataPath + "DepthDiff2Dprojection.bmp");
      std::vector<std::vector<std::vector<int>>> RGBtensorIntensity = createIntensity2DprojectionFromPointCLoud(rivetCloud);
      writeRGBTensorToBitmap(RGBtensorIntensity, dataPath + "Intensity2Dprojection.bmp");

      pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals = calculateNormals(rivetCloud);
      // use existing normals instead of calculating new ones, problem is phoxi scanner doesnt return curvature
      // pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals (new pcl::PointCloud<pcl::Normal>);
      // pcl::copyPointCloud(*rivetCloud, *rivetCloudNormals); 
      std::vector<std::vector<std::vector<int>>> RGBtensorNormals = createNormals2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeRGBTensorToBitmap(RGBtensorNormals, dataPath + "Normals2Dprojection.bmp");
      
      std::vector<std::vector<std::vector<int>>> RGBtensorCurvature = createCurvature2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeRGBTensorToBitmap(RGBtensorCurvature, dataPath + "Curvature2Dprojection.bmp");
      writeRGBTensorToBitmap(RGBtensorCurvature, dataPath + "Curvature2Dprojection.bmp");
      std::vector<std::vector<std::vector<int>>> RGBtensorRSD = createRSD2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeRGBTensorToBitmap(RGBtensorRSD, dataPath + "RSD2Dprojection.bmp");      
    }
};
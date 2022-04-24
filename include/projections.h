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

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>

#include <array>

namespace pcl{
  using boost::shared_ptr;
  using Indices = std::vector<int>;
};


class ColorSetter{
  public:
    std::vector<std::vector<int>> writeCheckMatrix;
    cv::Mat& RGBimgCV;
    int imgHeight;
    int imgWidth;

   
    ColorSetter(cv::Mat& RGBimgCV_):
                writeCheckMatrix(RGBimgCV_.rows, std::vector<int>(RGBimgCV_.cols, 0)), 
                RGBimgCV(RGBimgCV_),
                imgHeight(RGBimgCV_.rows),
                imgWidth(RGBimgCV_.cols)
                {}
    
    void setPixelColor(int row, int col, int RColor,int GColor, int BColor){
      if((col < imgWidth)&&(col>=0) && (row < imgHeight)&&(row>=0)){            
        cv::Vec3b& color = RGBimgCV.at<cv::Vec3b>(row, col);
        
        if(writeCheckMatrix.at(row).at(col) == 0){
          setColor(color, RColor, GColor, BColor);
        }
        // else{
        //   int avgBColor = calcAvgCol(writeCheckMatrix.at(row).at(col), color[0], BColor);
        //   int avgGColor = calcAvgCol(writeCheckMatrix.at(row).at(col), color[1], GColor);
        //   int avgRColor = calcAvgCol(writeCheckMatrix.at(row).at(col), color[2], RColor);
        //   setColor(color, avgRColor, avgGColor, avgBColor);
        // }
        // writeCheckMatrix.at(row).at(col) +=1;
      }
    }

  private:
    void setColor(cv::Vec3b& color, int RColor,int GColor, int BColor){
      color[0] = BColor;
      color[1] = GColor;
      color[2] = RColor;
    }

    int calcAvgCol(int curMesPointsInPixel, int curColor, int newColor){
      return (curColor*curMesPointsInPixel + newColor)/(curMesPointsInPixel + 1);
    }
};

class Projections{ 
  public:
    //f.e. if equal to 1.1, 10 % more area will be extracted
    //padding to compensate for inaccurate position  because of wrong allingment or production inaccuracy
    float posInaccuracyPaddingMultiplier = 1.2;
    float realAreaDiameter = 0.0151;
    float realAreaRadius = realAreaDiameter/2;
    float extractedAreaRadius = realAreaRadius * posInaccuracyPaddingMultiplier;

    int imgWidth = 73;
    int imgHeight = 73;
    float imgRange = extractedAreaRadius * 2;
    float pixelSize = imgRange / (float) imgWidth;

    template<typename T>
    void extractRivetPointCloud(pcl::shared_ptr<pcl::PointCloud<T>> originalCloud, pcl::PointCloud<T>& rivetCloud, std::vector<float> rivetLocation) {
      //this will extract part of the point cloud inside of cube with side length 2*offset meters and center in rivetLocation  
      float paddingMultiplier = sqrt(2); // to ensure that there wont be empty pixels because of rotation 
      float offset = extractedAreaRadius * paddingMultiplier;//* padding; //in meters
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

      Eigen::Matrix<float,1,3> rotationVector = rivetPlaneNormal.cross(xyPlaneNormal);
      float theta = atan2(rotationVector.norm(), rivetPlaneNormal.dot(xyPlaneNormal));

      Eigen::Affine3f transToXYPlane = Eigen::Affine3f::Identity();
      transToXYPlane.rotate(Eigen::AngleAxisf (theta, rotationVector.normalized()));
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, transToXYPlane);
    }

    template<typename T>
    void flipRivetCloud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      // ensure that rotated point cloud doesnt have reversed Z coordinate
      float elevatedRadius = 0.006739 / 2; //radius of the area which is elevated (center of the rivet)
      float avgDepthRivet = 0;
      int numRivetPoints = 0;
      float avgDepthElevated = 0;
      int numElevatedPoints = 0;
      for(int i = 0; i < rivetCloud->points.size(); i++){
        if ( sqrt( pow(rivetCloud->points[i].x,2.0) + pow(rivetCloud->points[i].y,2.0)) <= realAreaRadius){
          avgDepthRivet += rivetCloud->points[i].z;
          numRivetPoints++;
          if ( sqrt( pow(rivetCloud->points[i].x,2.0) + pow(rivetCloud->points[i].y,2.0)) <= elevatedRadius){
            avgDepthElevated += rivetCloud->points[i].z;
            numElevatedPoints++;
          }  
        }    
      }
      avgDepthRivet /= numRivetPoints;
      avgDepthElevated /= numElevatedPoints;
      if(avgDepthElevated < avgDepthRivet){
        for(int i = 0; i < rivetCloud->points.size(); i++){
          rivetCloud->points[i].z *= -1; 
        }
      }
    }

    template<typename T>
    void cutRelevantRivetCloudArea(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //crop rivet point cloud so that only relevant circular area around Rivet remains
      float relevantRadius = extractedAreaRadius; //a bit bigger then offset
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
    void adjustRivetPointCloud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, std::vector<float> rivetLocation){
      //translate rivet point cloud so the beginning of the coordinate system will be in the rivet location
      Eigen::Affine3f translation= (Eigen::Affine3f)Eigen::Translation3f(-rivetLocation[0], -rivetLocation[1], -rivetLocation[2]);
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, translation);
      
      //rotate rivet point cloud so it will lay in the XY plane
      rotateRivetPointCloudToXYplane(rivetCloud);
      
      //ensure that rotated point cloud doesnt have reversed Z coordinate
      flipRivetCloud(rivetCloud);

      //crop rivet point cloud so that only relevant circular area around Rivet remains
      //cutRelevantRivetCloudArea(rivetCloud);
      
      //translate cropped rivet point cloud so every point has a positive XY coordinate
      float offset = extractedAreaRadius;
      Eigen::Affine3f translationToPositiveXY= (Eigen::Affine3f)Eigen::Translation3f(+offset, +offset, -0);
      newPcl::transformPointCloudWithNormals(*rivetCloud, *rivetCloud, translationToPositiveXY);          
    }

    // template<typename T>
    // void setPixelColor(pcl::PointCloud<T>& rivetCloud, int rivetCloudIdx, cv::Mat& RGBimgCV,
    //                       int RColor,int GColor, int BColor){
    //   int col = floor(rivetCloud.points[rivetCloudIdx].x / pixelSize);
    //   //Reverse ordering of rows, in order to tranform from point cloud coordinate system with origin in bottom left corner
    //   //to img coordinate szstem with origin in top left corner. X axis has same direction in both cases.
    //   int row = imgHeight - 1 - floor(rivetCloud.points[rivetCloudIdx].y / pixelSize);
    //   // col = std::max(std::min(col, imgWidth - 1),0);
    //   // row = imgHeight - 1 - std::max(std::min(row, imgHeight - 1),0);

    //   if((col < imgWidth)&&(col>=0) && (row < imgHeight)&&(row>=0)){
    //     cv::Vec3b & color = RGBimgCV.at<cv::Vec3b>(row, col);
    //     color[0] = BColor;
    //     color[1] = GColor;
    //     color[2] = RColor;
    //   }
    // }

    template<typename T>
    void setPixelColorFromPoint(pcl::PointCloud<T>& rivetCloud, int rivetCloudIdx, ColorSetter& colorSetter,
                          int RColor,int GColor, int BColor){
      int col = floor(rivetCloud.points[rivetCloudIdx].x / pixelSize);
      //Reverse ordering of rows, in order to tranform from point cloud coordinate system with origin in bottom left corner
      //to img coordinate szstem with origin in top left corner. X axis has same direction in both cases.
      int row = imgHeight - 1 - floor(rivetCloud.points[rivetCloudIdx].y / pixelSize);

      colorSetter.setPixelColor(row, col, RColor, GColor, BColor);
    }

    template<typename T>
    cv::Mat create2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection (without further features)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(255));
      
      // for (int idx = 0; idx < rivetCloud->size(); idx++){
      //   setPixelColor(*rivetCloud, idx, RGBimgCV,
      //                 0, //R color
      //                 0, //G color
      //                 0);//B color
      // }

      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      0, //R color
                      0, //G color
                      0);//B color
      }

      return RGBimgCV.clone();
    }

    template<typename T>
    cv::Mat createDepth2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection  based on depth (Depth image)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me (so depth is towards me)
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(0));

      float avgDepthRivet = 0.0f;
      int numRivetPoints = 0;
      for(int i = 0; i < rivetCloud->points.size(); i++){
        if ( sqrt( pow(rivetCloud->points[i].x - extractedAreaRadius,2.0) + pow(rivetCloud->points[i].y - extractedAreaRadius,2.0)) <= realAreaRadius){
          avgDepthRivet += rivetCloud->points[i].z;
          numRivetPoints++;
        }
      } 
      avgDepthRivet /= numRivetPoints;

      float depthRangeOffset = 0.0001; // in meters, allong z axis, represents positive or negative offset from average depth value(or some other defined value), the whole accepted range is then depthRangeOffset*2
      int colorIntensity; 

      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        colorIntensity = (int) (((std::max(std::min((rivetCloud->points[idx].z - avgDepthRivet),depthRangeOffset),-depthRangeOffset) + depthRangeOffset) / (depthRangeOffset*2)) * 255.0);
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      0, //R color
                      0, //G color
                      colorIntensity);//B color
      }//B clorIntensity because virtual Z axis in image plane is towards me, 
      //so closer object to me (z axis has higher value) is more blue it is and more distant object from me is, more red it is

      return RGBimgCV.clone();
    }

    template<typename T>
    cv::Mat createDepthDiff2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection  based on depth (Depth image)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and ZPointCloudConstPtr = typename PointCloud::ConstPtr axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(0));

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

      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        colorIntensity = std::min(depthDiffs[idx]/depthDiffRange, 1.0f) * 255.0;
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      colorIntensity, //R color
                      colorIntensity, //G color
                      colorIntensity);//B color
      }//the higher the difference, the bigger the color instensity

      return RGBimgCV.clone();
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
    cv::Mat createNormals2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      //creates two 3*width*height RGBtensors from point cloud to represent 2D projection based on Normals and Curvature
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(255));
      
      float xIdx, yIdx, xNormal, yNormal, zNormal, ang;
      float PI = 3.1416;
      std::vector<int> rgbVec;
      float normalsRange = 0.03;
      //calculate RGBtensor based on Normals
      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        xNormal = rivetCloudNormals->points[idx].normal_x;
        yNormal = rivetCloudNormals->points[idx].normal_y;
        zNormal = rivetCloudNormals->points[idx].normal_z;
        ang = atan2(yNormal, xNormal)/PI*180;
        rgbVec = HSVtoRGB(ang > 0 ? ang : (360 + ang), std::min(100.0, 100.0 * (1 - abs(zNormal)) / normalsRange), 100.0);
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      rgbVec[0], //R color
                      rgbVec[1], //G color
                      rgbVec[2]);//B color
      }//normal vector size is 1 (sqrt(nx^2+ny^2+nz^2)) and most of it (usually about 0.999) belongs to Z axis since i am in XY plane,
      // because of that HUE is describing vector inclination in XY plane projection based on nx and ny (0-360 degrees, 0is red, 180 is cyan)
      // and SATURATION decsribes the (nx + ny) portion of the vector size, the higher the (nx + ny) portion is the higher the saturation (more color)

      return RGBimgCV.clone();
    }

    template<typename T>
    cv::Mat createCurvature2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      //creates two 3*width*height RGBtensors from point cloud to represent 2D projection based on Normals and Curvature
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(255));
      
      // for(int i = 0; i < rivetCloudNormals->size(); i++){
      //   std::cout << rivetCloudNormals->points[i].curvature << std::endl;
      // }
      float curvRelRange = 0.02; // curvature relevant range, experimentaly determined from 1 rivetCloud in such way that
      //c values coresponding to individual points ranged from 0 to 0.007, so i added a bit reserve)
      //Curvature is calculated c = lambda0 / (lambda0 + lamda1 + lambda2), where lambdax is eigenvalue of Covariance matrix. 
      //Covariance matrix is positive semidefinite symetrical matrix, what implies that its eigenvalues are nonnegative.
      //Since eigenvalues are nonnegative, value of c is in interval <0;1>

      //calculate RGBtensor based on Curvature
      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      (int) std::min(255.0, (rivetCloudNormals->points[idx].curvature /curvRelRange * 255.0)), //R color
                      0, //G color
                      0);//B color
      }//R color intensity increases with increasing curvature, 
      //min() added in order to evoid error because its possible that some sample will have bigger curvature then defined range

      return RGBimgCV.clone();
    }
    
    template<typename T>
    cv::Mat createRSD2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals){
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(255));

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
      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      (int) (255.0 - std::min(255.0, (descriptors->points[idx].r_min/rsd.getPlaneRadius() * 255.0))), //R color
                      0, //G color
                      0);//B color
      }//R color intensity increases with increasing curvature

      return RGBimgCV.clone();
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
    cv::Mat createIntensity2DprojectionFromPointCLoud(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud){
      //creates 3*width*height RGBtensor from point cloud to represent 2D projection (without further features)
      //this way  [0,0] in the picture is in botoom left part, x axis goes from left to right, y axis from bottom to top (exactly like whe using graphs graph), z axis is towards me
      //problem is that because of rotation matrix to XY plane found by ransac i cant be sure whether point cloud and Z axis is not reversed
      cv::Mat RGBimgCV(imgHeight, imgWidth, CV_8UC3, cv::Scalar::all(255));
      
      ColorSetter colorSetter(RGBimgCV);
      for (int idx = 0; idx < rivetCloud->size(); idx++){
        setPixelColorFromPoint(*rivetCloud, idx, colorSetter,
                      rivetCloud->points[idx].intensity, //R color
                      rivetCloud->points[idx].intensity, //G color
                      rivetCloud->points[idx].intensity);//B color
      }

      return RGBimgCV.clone();
    }

    void writeFiltImg(std::string dataPath, std::string name, cv::Mat &img){
      cv::imwrite(dataPath + name + "nonFilt.bmp", img);
      //cv::cvtColor(img, img, CV_BGR2HSV);
      cv::medianBlur(img, img, 3);
      //cv::cvtColor(img, img, CV_HSV2BGR);
      cv::imwrite(dataPath + name + ".bmp", img);
    }

    cv::Vec3f findRivetCenter(cv::Mat &imgBGR, std::string path){
      cv::Mat imgGRAY;
      
      cv::cvtColor(imgBGR, imgGRAY, cv::COLOR_BGR2GRAY);
      //cv::normalize(imgGRAY, imgGRAY,0, 255, CV_MINMAX);
      cv::GaussianBlur(imgGRAY, imgGRAY, cv::Size(3,3),0,0);
      //cv::medianBlur(imgGRAY, imgGRAY, 3);
      cv::imwrite(path+"gray.bmp", imgGRAY);
      std::vector<cv::Vec3f> circles;
      
      cv::HoughCircles(imgGRAY, circles, cv::HOUGH_GRADIENT, 0.6, 100.0,
                       70.0, 10.0, 13, 17);

      if(!circles.empty()){
        cv::Vec3f circ = circles[0];
        cv::circle(imgBGR, cv::Point(circ[0], circ[1]), circ[2], 
                  cv::Scalar(0,255,0), 0);
        imgBGR.at<cv::Vec3b>(circ[1], circ[0]) = {0,255,0};
        return circles[0];
      }
      else{
        return {0, 0};
      }
    }

    std::array<float,2> calcRivetCenterDeviation(int measuredCenterRow, int measuredCenterColumn){
      //returns 2 float array: 1) pixel distance of the centers, 2) real distance of the centers in m
      //supposing row and collumn range is 0 - (size-1), so origin is in [0,0]
      int imgCenterRow = int(imgHeight/2);
      int imgCenterColumn = int(imgWidth/2);
      float pixelDeviation = sqrt(pow(imgCenterRow - measuredCenterRow, 2) + pow(imgCenterColumn - measuredCenterColumn, 2));
      float realDistDeviation = pixelDeviation * pixelSize;
      return {pixelDeviation, realDistDeviation};
    }

    void calcEmptyPixels(cv::Mat& imageNonFilt, cv::Mat& image, std::string path){
      std::size_t pos = path.find("rivet_ID");
      std::string str = path.substr (pos);

      int numEmptyPixels = 0;  
      int numEmptyPixelsNF = 0;
      cv::Vec3b color, colorNF;
      for (int row = 0; row < image.rows; row++){
        for(int column = 0; column < image.cols; column++){
          color = image.at<cv::Vec3b>(row,column);
          colorNF = imageNonFilt.at<cv::Vec3b>(row,column);

          if(color[0] == 255 && color[1] == 255 && color[2] == 255) numEmptyPixels++;
          if(colorNF[0] == 255 && colorNF[1] == 255 && colorNF[2] == 255) numEmptyPixelsNF++;
        }
      }

      std::cout << str << " " << numEmptyPixels << " " << numEmptyPixelsNF << std::endl;
    }    
    
    template<typename T>
    std::array<float,2> create2DProjectionsImages(pcl::shared_ptr<pcl::PointCloud<T>> rivetCloud, std::string dataPath){
      pcl::io::savePCDFileBinary (dataPath + "rivetCloud.pcd", *rivetCloud); 

      cv::Mat RGBimgCV = create2DprojectionFromPointCLoud(rivetCloud);
      cv::Mat RGBimgCVnonFilt = RGBimgCV.clone();
      writeFiltImg(dataPath, "2Dprojection", RGBimgCV);

      cv::Mat RGBimgCVDepth = createDepth2DprojectionFromPointCLoud(rivetCloud);
      cv::Mat RGBimgCVDepthnonFilt = RGBimgCVDepth.clone();
      writeFiltImg(dataPath, "Depth2Dprojection", RGBimgCVDepth);

      cv::Mat RGBimgCVDepthDiff = createDepthDiff2DprojectionFromPointCLoud(rivetCloud);
      writeFiltImg(dataPath, "DepthDiff2Dprojection", RGBimgCVDepthDiff);
      cv::Mat RGBimgCVIntensity = createIntensity2DprojectionFromPointCLoud(rivetCloud);
      writeFiltImg(dataPath, "Intensity2Dprojection", RGBimgCVIntensity);

      pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals = calculateNormals(rivetCloud);
      // use existing normals instead of calculating new ones, problem is phoxi scanner doesnt return curvature
      // pcl::PointCloud<pcl::Normal>::Ptr rivetCloudNormals (new pcl::PointCloud<pcl::Normal>);
      // pcl::copyPointCloud(*rivetCloud, *rivetCloudNormals); 
      cv::Mat RGBimgCVNormals = createNormals2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeFiltImg(dataPath, "Normals2Dprojection", RGBimgCVNormals);      
      cv::Mat RGBimgCVCurvature = createCurvature2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeFiltImg(dataPath, "Curvature2Dprojection", RGBimgCVCurvature);
      cv::Mat RGBimgCVRSD = createRSD2DprojectionFromPointCLoud(rivetCloud, rivetCloudNormals);
      writeFiltImg(dataPath, "RSD2Dprojection", RGBimgCVRSD);
      
      // find rivet center and determine its deviation from model
      cv::Vec3f circle = findRivetCenter(RGBimgCVDepthnonFilt, dataPath);      
      cv::imwrite(dataPath + "RivetCenter" + ".bmp", RGBimgCVDepthnonFilt);
      std::array<float,2> rivCenterDist  = calcRivetCenterDeviation(circle[1], circle[0]);

      calcEmptyPixels(RGBimgCVnonFilt, RGBimgCV, dataPath);
      
      return rivCenterDist;
    }
};
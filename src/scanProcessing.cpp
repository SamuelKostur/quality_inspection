#include <scanProcessing.h>

void extractUnmeasuredPoints(MyPointCloud& pointCloud){
    //Unmeasured points (pixels) caused by shadows are given the default coordinates (0,0,0).
    pcl::PointIndices::Ptr unmeasured (new pcl::PointIndices);
    for(int i = 0; i < pointCloud.size(); i++){
        if((pointCloud.points[i].x == 0)&&(pointCloud.points[i].y == 0)&&(pointCloud.points[i].z == 0)){
            unmeasured->indices.push_back(i);
        }                
    }

    pcl::ExtractIndices<MyPoint> extraction;
    extraction.setInputCloud(pointCloud.makeShared());
    extraction.setIndices(unmeasured);
    extraction.setNegative(true);
    extraction.filter(pointCloud);
}

void robotPoseToMetersAndRadians(std::vector<double> &robotScanPoses){
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

void transformPointCloudFromTCPtoRobot(std::vector<double> robotPose, MyPointCloud& pointCloud){
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
    newPcl::transformPointCloudWithNormals(pointCloud, pointCloud, transMatrix);            
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

void addTextureToPointCloud(MyPointCloud& pointCloud, cv::Mat& img){

    for(int rowIdx = 0; rowIdx < img.rows; rowIdx++){
        const uint8_t* imgRow = img.ptr<uint8_t>(rowIdx);
        for(int colIdx = 0; colIdx < img.cols; colIdx++){
            pointCloud.at(colIdx, rowIdx).intensity = imgRow[colIdx]; 
            //and yes pcl::point cloud has indexing with column first
        }
    }
}

void correctNormals(MyPointCloud& pointCloud){
    // apply contra transformation from photoneo ros pkg normals (where each is divided by 1000)
    // to ensure normal vector size 1 (sqrt(nx^2+ny^2+nz^2))
    for(int row = 0; row < pointCloud.height; row++){
        for(int col = 0; col < pointCloud.width; col++){
            pointCloud.at(col, row).normal_x *= 1000;
            pointCloud.at(col, row).normal_y *= 1000;
            pointCloud.at(col, row).normal_z *= 1000;
        }
    }
}

void extractTable(MyPointCloud& pointCloud){
    float tableZ = 0.042035;
    pcl::PointIndices::Ptr tableIdx (new pcl::PointIndices);
    for(int i = 0; i < pointCloud.size() ; i++){
        if(pointCloud[i].z < tableZ){
            tableIdx->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<MyPoint> extraction;
    extraction.setInputCloud(pointCloud.makeShared());
    extraction.setIndices(tableIdx);
    extraction.setNegative(true);
    extraction.filter(pointCloud);
}

void extractDistantPoints(pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, MyPointCloud& regCloud){
    float outlierDistTreshold = 0.0025; //distance in centimeters
    float outSqrDistThresh = outlierDistTreshold * outlierDistTreshold;
    pcl::PointIndices::Ptr outlierIdxs (new pcl::PointIndices);

    pcl::search::KdTree<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(refCloud);
    pcl::PointIndices neighbourIndices;
    neighbourIndices.indices.resize(1);
    std::vector<float> neighbourSqrDistances;
    neighbourSqrDistances.resize(1);

    for (int i = 0; i < regCloud.size(); i++){
        kdtree.nearestKSearchT(regCloud[i], 1, neighbourIndices.indices, neighbourSqrDistances);
        // if(neighbourSqrDistances[0] > outSqrDistThresh){
        //     outlierIdxs->indices.push_back(i);
        // }
        pcl::PointNormal neighbour = refCloud->at(neighbourIndices.indices[0]);
        float dotProduct = ((regCloud.at(i).x - neighbour.x) * neighbour.normal_x +
                        (regCloud.at(i).y - neighbour.y) * neighbour.normal_y +
                        (regCloud.at(i).z - neighbour.z) * neighbour.normal_z);
        if(std::abs(dotProduct) > outlierDistTreshold){
            outlierIdxs->indices.push_back(i);
        }
       
    }
    std::cout << outlierIdxs->indices.size() << std::endl;
    pcl::ExtractIndices<MyPoint> extraction;
    extraction.setInputCloud(regCloud.makeShared());
    extraction.setIndices(outlierIdxs);
    extraction.setNegative(true);
    extraction.filter(regCloud);
}
#include <processScan.h>

std::function<void()> exitFunc;

/////////////////////immediate postprocessing////////////////////////////////
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

/////////////////////coarse and fine allignment////////////////////////////////
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

template<typename T>
Eigen::Affine3f transfToOrigin(pcl::shared_ptr<pcl::PointCloud<T>> cloud){
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Affine3f translation = (Eigen::Affine3f)Eigen::Translation3f(
                                -centroid[0], -centroid[1], -centroid[2]);

  newPcl::transformPointCloudWithNormals(*cloud, *cloud, translation);
  return translation;
}

template<typename T1>
Eigen::Affine3f doICP(pcl::shared_ptr<pcl::PointCloud<T1>> regCloud, pcl::PointCloud<pcl::PointNormal>::Ptr refCloud, 
          int minRandomSamples, int maxRandomSamples, int ICPiterNum, float maxMSE, float maxCorDist){
  //conversion because ICP requaires same Point type of both clouds, and since i need normals i need at least PointNormal
  pcl::PointCloud<pcl::PointNormal>::Ptr regCloudPNormal (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*regCloud, *regCloudPNormal);  

  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setMaxCorrespondenceDistance (maxCorDist);
  icp.setMaximumIterations (1);
  //icp.setEuclideanFitnessEpsilon (6e-5);
   	
  Eigen::Affine3f totalTransform(Eigen::Affine3f::Identity());
  pcl::RandomSample<pcl::PointNormal> r;
  //r.setSample(randomSamples);
  int sampInc = (maxRandomSamples-minRandomSamples)/(ICPiterNum-1);
  pcl::PointCloud<pcl::PointNormal>::Ptr tempRegCloud (new pcl::PointCloud<pcl::PointNormal>);
  for (int i = 0; i < ICPiterNum ; i++ ){
    auto t1 = std::chrono::high_resolution_clock::now(); 
    if(maxRandomSamples < regCloud->size()){
      r.setSample((int)(minRandomSamples + (sampInc * i)));
      r.setInputCloud(regCloudPNormal);
      r.setSeed(i*10);
      r.filter(*tempRegCloud);
      icp.setInputSource(tempRegCloud);
    }
    else{
      icp.setInputSource(regCloudPNormal);
    }

    icp.setInputTarget(refCloud);

    auto t2 = std::chrono::high_resolution_clock::now();
    icp.align(*tempRegCloud);
    auto t3 = std::chrono::high_resolution_clock::now(); 
    // dont need to use transformPointCloudWithNormals because pcl::IterativeClosestPointWithNormals by default calculates point to plane distance
    // based on point from regCloud and plane from refCloud, so transforming regCloudNormals doesnt do anything just takes longer
    // but it can be set to plane to plane so its worth to try in the future
    newPcl::transformPointCloud(*regCloudPNormal, *regCloudPNormal, icp.getFinalTransformation());
    totalTransform *= icp.getFinalTransformation();
    auto t4 = std::chrono::high_resolution_clock::now();

    auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
    auto durationBEFORE = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    auto durationAFTER = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
    std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "duration ICP: " << durationICP.count()/1000000.0 <<
    "duration OTHER: " << durationBEFORE.count()/1000000.0 <<
    "duration AFTER: " << durationAFTER.count()/1000000.0 << std::endl;
    std::cout << "iteration" << i << " error " << icp.getFitnessScore() << std::endl;

    if(icp.getFitnessScore() <  maxMSE){
        std::cout << "icp converged with MSE" << icp.getFitnessScore() << std::endl;
        break;
    }
  }
  newPcl::transformPointCloudWithNormals(*regCloud, *regCloud, totalTransform);

  // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  // icp.getFitnessScore() << std::endl;
  // std::cout << icp.getFinalTransformation() << std::endl;

  return totalTransform;
}

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

/////////////////////member functions////////////////////////////////
ProcessScan::ProcessScan(): semMT2_MT3(1, 1) {
    exitFunc = boost::bind(&ProcessScan::procScanExitFunc, this);
    exitFuture = exitPromise.get_future();

    numRobPoses = robotScanPoses.size(); //this of course has to be before all references to this variable

    futureVec.resize(numRobPoses);
    promiseVec.resize(numRobPoses);
    fillFutPromVec(); //call here to ensure that futures are ready before data callback is processed

    resetPromFut(promCPCindices, futCPCindices);
    
    createDataPath();
    std::cout << dataPath << std::endl;
    loadPointCloud(CADcloud, dataPath + "pointCloudCAD.pcd");

    std::thread endlessThread (&ProcessScan::endlessLoop, this);
    std::thread mainThread3 (&ProcessScan::mainThread3, this);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(n, "/phoxi_camera/pointcloud", numRobPoses);
    message_filters::Subscriber<sensor_msgs::Image> textureSub(n, "phoxi_camera/texture", numRobPoses);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pointCloudSub, textureSub, numRobPoses);
    sync.registerCallback(boost::bind(&ProcessScan::combCB, this, _1, _2));

    comm_client_main_processScan = n.serviceClient<std_srvs::SetBool>("comm_main_processScan");
    std::cout << " service ready.." << std::endl;
    std_srvs::SetBool empty;
    comm_client_main_processScan.call(empty);
    std::cout << " service called.." << std::endl;
    
    PPCindices.resize(numRobPoses+1);
    PPCindices.emplace_back(0);

    ros::spin();
    endlessThread.join();
    mainThread3.join();
}

void ProcessScan::procScanExitFunc(){
    std::cout << "exiting threads started" << std::endl;
    exitPromise.set_value(1);
    if(!promiseVec.empty()){
        for(int i  = 0; i < promiseVec.size(); i++){
            try{
                promiseVec[i].set_value(MyPointCloud());
            }
            catch(const std::future_error& e)
            {
                //catch error which happens when sigint was evoked while promise was already set 
            }
        }
    }
   
    try{
        promCPCindices.set_value({MyPointCloud(), std::vector<int>()});
    }
    catch(const std::future_error& e)
    {
        //catch error which happens when sigint was evoked while promise was already set 
    }

}

void ProcessScan::createDataPath(){
    dataPath = ros::package::getPath("quality_inspection"); 
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
    dataPath = dataPath + "/InspectionFiles/";
    boost::filesystem::create_directories(dataPath);
}

void ProcessScan::combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture){
    static int currIdxRobPose = 0;
    std::cout << "combining partial point cloud with intensity " << currIdxRobPose << "..." << std::endl;
    //I have to do this in 2 steps, otherweise i get warning because originalPointCloud
    //dooesnt have intensity
    MyPointCloud pointCloud;
    { //I use scope her to destroy pointCloudNormal
    pcl::PointCloud<pcl::PointNormal> pointCloudNormal;
    pcl::fromROSMsg(*originalPointCloud,pointCloudNormal); 
    pcl::copyPointCloud(pointCloudNormal, pointCloud);
    correctNormals(pointCloud);
    }

    cv::Mat img = createTextureImage(originalTexture);
    #if SAVE_PARTIAL_DATA
        cv::imwrite(dataPath + "img" + std::to_string(currIdxRobPose) + ".bmp", img);
    #endif
    addTextureToPointCloud(pointCloud, img);
    
    //ros::shutdown(); would eventually finish this callback, 
    //however i want to prevent setting promise second time
    if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        std::cout << "exiting callback " << currIdxRobPose << std::endl;
        return;
    }
    promiseVec[currIdxRobPose].set_value(pointCloud);
    currIdxRobPose++;
    currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;
}

void ProcessScan::processPointCloud(std::future<MyPointCloud> future, int currIdxRobPose){
    std::cout << "waiting for partial point cloud" << currIdxRobPose << "..." << std::endl;
    MyPointCloud pointCloud = future.get();
    std::cout << "started processing point cloud" << currIdxRobPose << "..." << std::endl;
    if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        std::cout << "exiting thread " << currIdxRobPose << std::endl;
        return;
    }

    #if SAVE_PARTIAL_DATA
        pcl::io::savePCDFileASCII (dataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif
    extractUnmeasuredPoints(pointCloud);  //prepisat typ co ide do funkcie alebo spravit template
    transformPointCloudFromTCPtoRobot(robotScanPoses[currIdxRobPose], pointCloud);
    #if SAVE_PARTIAL_DATA
        pcl::io::savePCDFileASCII (dataPath + "pointCloud" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif

    mtx.lock();
    completePointCloud->operator+=(pointCloud);
    PPCindices[currIdxRobPose+1] = completePointCloud->size();
    std::cout << "size" << PPCindices[currIdxRobPose] << std::endl;
    mtx.unlock();
    std::cout << "processing point cloud" << currIdxRobPose << "done..." << std::endl;
}

void ProcessScan::fillFutPromVec(){
    for(int i  = 0; i < numRobPoses; i++){
        // std::promise<MyPointCloud> promise;
        // futureVec.at(i) = promise.get_future();
        // promiseVec.at(i) = std::move(promise);
        resetPromFut(promiseVec.at(i), futureVec.at(i));
    }
}

template<typename T>
void resetPromFut(std::promise<T>& prom, std::future<T>& fut){
    std::promise<T> promise;
    fut = promise.get_future();
    prom = std::move(promise);
}

void ProcessScan::endlessLoop(){
    std::vector<std::future<void>> threadVec;
    threadVec.resize(numRobPoses);
    while(1){
        //tu vytvorit 8 threadov na spracovanie point cloudu ktory je ulozeny v premennej ci uz globalnej alebo vo future,
        //a nasledne pred pridavanim do celkoveho point cloudu este pridat mutex aby viacere thready naraz nepristupovali k tej istej premennej
        //processing
        for (int i = 0; i < numRobPoses; i++){
            auto t = std::async(std::launch::async, &ProcessScan::processPointCloud, this, std::move(futureVec[i]), i);
            threadVec.at(i) = std::move(t);	
        }
        
        for (auto& t : threadVec){
            if(t.wait_for(std::chrono::seconds(240)) != std::future_status::ready){
                std::cout << "maximum time of scanning one part reached" << std::endl;
                raise(SIGINT);
                return;
            }
        }

        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting endlessLoop thread" << std::endl;
            return;
        }

        fillFutPromVec();
        std::cout << futureVec.size() << " " << promiseVec.size() << " " << threadVec.size() << std::endl;
        //tu servicom odblokujem main node - vyvolam service a v mein node sa stane binSem.release()
        //binSem.acquire() / na zaciatku z 1 do 0
        //vyhodnocovacie moduly zavolaju service a potom sa v callbacku spravy binSem.release();
        //pcl::io::savePCDFileASCII (dataPath + "completePointCloud.pcd", *completePointCloud);
        semMT2_MT3.acquire();

        CPCindices data;
        data.CPC = *completePointCloud;
        data.PCPindices = PPCindices;
        promCPCindices.set_value(data); 


        completePointCloud->clear();
        std::cout << "all processing done..." << std::endl;
        sleep(5);
        std_srvs::SetBool empty;        
        comm_client_main_processScan.call(empty);
        //tu uz mam spojeny completePointCLoud, mozem icp na model a extrahovat rivety
        //na kazdy rivet jedno vlakno
    }    
}

void fineRegPPCtoCAD(pcl::PointCloud<pcl::PointNormal>::Ptr CADcloud, MyPointCloud::Ptr PPC){
    doICP(PPC, CADcloud, 500000, 800000, 10, 1e-7, 0.01);
}

std::vector<int>  createPPCIndices(int PPCstartIdx,int PPCendIdx){
    std::vector<int> indices;
    indices.reserve(PPCendIdx);
    for(int i = PPCstartIdx; i < PPCendIdx; i++){
        indices.emplace_back(i);
    }
    return indices;
}

void ProcessScan::mainThread3(){
    std::vector<std::future<void>> threadVec;
    threadVec.resize(numRobPoses);
    while(true){
        CPCindices data = futCPCindices.get();
        MyPointCloud::Ptr CPC_MT3 = data.CPC.makeShared();
        resetPromFut(promCPCindices, futCPCindices); //clear memory occupied by promise asap
        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting main thread 3" << std::endl;
            return;
        }
        
        std::cout << "coarse allignment..." << std::endl;
        //coarse allignment CPC to CAD
        Eigen::Affine3f totalTransform =  alignPointClouds(CPC_MT3, CADcloud, 8000, 50000, 15, 1e-4, 0.05);
        pcl::io::savePCDFileASCII (dataPath + "completePointCloudCoarseAlligned.pcd", *CPC_MT3);

        //fine allignment PPC to CAD
        std::cout << "fine allignment..." << std::endl;
        std::vector<MyPointCloud::Ptr> PPCvec;
        PPCvec.reserve(numRobPoses);
        for (int i = 0; i < numRobPoses; i++){
            PPCvec.emplace_back(MyPointCloud(*CPC_MT3,createPPCIndices(data.PCPindices[i] , data.PCPindices[i+1])).makeShared());
            auto t = std::async(std::launch::async, fineRegPPCtoCAD, CADcloud, PPCvec.at(i));
            threadVec.at(i) = std::move(t);	
        }

        std::cout << "clearing previous CPC..." << std::endl;
        CPC_MT3->clear();

        for (int i = 0; i < numRobPoses; i++){
            if(threadVec.at(i).wait_for(std::chrono::seconds(240)) != std::future_status::ready){
                std::cout << "maximum time of scanning one part reached" << std::endl;
                raise(SIGINT);
                return;
            }
            CPC_MT3->operator+=(*PPCvec.at(i));
        }

        pcl::io::savePCDFileASCII (dataPath + "completePointCloudFineAlligned.pcd", *CPC_MT3);

        semMT2_MT3.release();
        //this has to be last to prevent creating new promise and future before exiting
        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting main thread 3" << std::endl;
            return;
        }
        std::cout << "processing in MT3 done" << std::endl;
    }
}

// void mainThread3(){
//     future.get();
//     //procesing ICP
//     semEXTRACT.acquire()
//     EXTRACT_MT3.publish();
//     promiseMT3.at() = new;
//     semMT2_MT3.release();
// }

// void comm_process_extract(){
//     semExtract.release();
// }

///////////////////////////////////proper closing///////////////////////////////////
void sigintCB(int signum){
    static int sigintRaised = 0;
    if(!sigintRaised){
        sigintRaised = 1;
        //if there is a serious problem while executing and we need to 
        //properly exit programm use raise(SIGINT); followed by return; from currect thread (function)
        exitFunc(); //this handles shutting down threads in ProcessScan class
        ros::shutdown();
        std::cout << "ros is shutting down" << std::endl;
        while (ros::ok()){}
        std::cout << "ros finished shutting down" << std::endl;
        //exit() is called somewhere from ros::shutdown(), i tried it with atexit
        //however atexit happens after all threads made from main thread are joined
    }
}
///////////////////////////////////main//////////////////////////////////////////
int main(int argc, char** argv){    
    ros::init(argc, argv, "processScanNode", ros::init_options::NoSigintHandler);  
    std::signal(SIGINT, sigintCB); //should be after ros::init()    
    ProcessScan processScan;
    
    return 0;
}


// class completeData{
//     public:
//         MyPointCloud::Ptr CPC;
//         std::vector<pcl::PointIndices::Ptr> PPCindices;
//         int numOfPPCs = 0;
        
//         void addPointCloud(MyPointCloud::Ptr newPointCloud){
//             CPC->operator+=(*newPointCloud);
//             PPCindices.reserve(newPointCloud->size());
//             for(int i = 0; i < newPointCloud->size(); i++){
//                 PPCindices[numOfPPCs]->indices.push_back(i);
//             }
//             numOfPPCs++;
//         }

//         void extractPoints(pcl::PointIndices::Ptr indices){
//             pcl::ExtractIndices<MyPoint> extraction;
//             extraction.setInputCloud(CPC);
//             extraction.setIndices(indices);
//             extraction.setNegative(true);
//             extraction.filter(*CPC);

//         }

//         void getPartialPointCloud(MyPointCloud::Ptr PPC, int PPCnumber){
//             if(PPCnumber < numOfPPCs){
//                 pcl::ExtractIndices<MyPoint> extraction;
//                 extraction.setInputCloud(CPC);
//                 extraction.setIndices(PPCindices[PPCnumber]);
//                 extraction.setNegative(false);
//                 extraction.filter(*PPC);
//             }
//             else{
//                 std::cout << "partial point cloud number " << PPCnumber << " cant be accesed";
//             }
//         }
// };

// void fillVectorAscending(std::vector<int>& vec, int numElements){
//     vec.reserve(numElements);
//     for(int i = 0; i < numElements; i++){
//         vec.push_back(i);
//     }
// }
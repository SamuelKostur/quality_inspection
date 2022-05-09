#include <scan_processing_node.h>

/////////////////////////////main thread 1/////////////////////////////////
void ScanProcessingNode::combCB (const sensor_msgs::PointCloud2::ConstPtr& originalPointCloud, const sensor_msgs::Image::ConstPtr& originalTexture){
    auto t1 = std::chrono::high_resolution_clock::now(); 

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
        //cv::imwrite(partialOutputDataPath + "img" + std::to_string(currIdxRobPose) + ".bmp", img);
    #endif
    addTextureToPointCloud(pointCloud, img);
    
    //ros::shutdown(); would eventually finish this callback, 
    //however i want to prevent setting promise second time
    if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        std::cout << "exiting callback " << currIdxRobPose << std::endl;
        return;
    }

    prom_vec_MT1_MT2[currIdxRobPose].set_value(pointCloud);
    currIdxRobPose++;
    currIdxRobPose = (currIdxRobPose < numRobScanPoses) ? currIdxRobPose : 0;

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Dur MT1: " << dur.count()/1000000.0 << std::endl;
}


/////////////////////////////main thread 2/////////////////////////////////
void applyBadTransf(MyPointCloud& pointCloud, std::string path){
    // // a little translation and rotation
    Eigen::Matrix4f tf = (Eigen::Matrix4f() << 0.9654, 0.251, -0.0712, -0.0234,
                                               -0.2539, 0.9667, -0.0349, 0.0963,
                                               0.060, 0.0518, 0.9968, -0.0398,
                                               0, 0, 0, 1).finished();
    // // aproximately 90 degrees
    // Eigen::Matrix4f tf = (Eigen::Matrix4f() << 0.3601, 0.9329, 0, -0.0538,
    //                                            -0.9329, 0.3601, 0, 0.5226,
    //                                            0, 0, 1, 0,
    //                                            0, 0, 0, 1).finished();
    // aproximately 135 degrees
    // Eigen::Matrix4f tf = (Eigen::Matrix4f() << -0.3992, 0.9169, 0, 0.2199,
    //                                            -0.9169, -0.3992, 0, 0.7453,
    //                                            0, 0, 1, 0,
    //                                            0, 0, 0, 1).finished();

    // aproximately 180 degrees
    // Eigen::Matrix4f tf = (Eigen::Matrix4f() << -0.9282, 0.3719, 0, 0.5711,
    //                                            -0.3719, -0.9282, 0, 0.7114,
    //                                            0, 0, 1, 0,
    //                                            0, 0, 0, 1).finished();
    //aproximately 180 degrees around Z, and flipped around (about 180 degrees as well)
    // Eigen::Matrix4f tf = (Eigen::Matrix4f() << 0.9285, -0.3693, -0.0377, 0.1417,
    //                                            -0.3706, -0.9159, -0.1546, 0.7290,
    //                                            0.0225, 0.1575, -0.9873, 0.2249,
    //                                            0, 0, 0, 1).finished();
    newPcl::transformPointCloudWithNormals(pointCloud,pointCloud,tf);
    pcl::io::savePCDFileBinary (path + "completePointCloudBadAlligned.pcd", pointCloud);
}

std::vector<int>  createPPCIndices(int PPCstartIdx,int PPCendIdx){
    std::vector<int> indices;
    indices.reserve(PPCendIdx);
    for(int i = PPCstartIdx; i < PPCendIdx; i++){
        indices.emplace_back(i);
    }
    return indices;
}

MyPointCloud ScanProcessingNode::processPointCloud(std::future<MyPointCloud> future, int currIdxRobPose){
    std::cout << "waiting for partial point cloud" << currIdxRobPose << "..." << std::endl;
    //sleep(15);
    MyPointCloud pointCloud = future.get();
    auto t1 = std::chrono::high_resolution_clock::now(); 
    std::cout << "started processing point cloud" << currIdxRobPose << "..." << std::endl;
    if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        std::cout << "exiting thread " << currIdxRobPose << std::endl;
        return pointCloud;
    }

    #if SAVE_PARTIAL_DATA
        //pcl::io::savePCDFileASCII (partialOutputDataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif
    extractUnmeasuredPoints(pointCloud);  //prepisat typ co ide do funkcie alebo spravit template
    transformPointCloudFromTCPtoRobot(robotScanPoses[currIdxRobPose], pointCloud);
    extractTable(pointCloud, tableZcoord);
    #if SAVE_PARTIAL_DATA
        //pcl::io::savePCDFileASCII (partialOutputDataPath + "pointCloud" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif
    
    std::cout << "processing point cloud" << currIdxRobPose << "done..." << std::endl;

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Dur MT2 procesPointCloud: " << dur.count()/1000000.0 << std::endl;
    return pointCloud;
}

void ScanProcessingNode::mainThread2Loop(){
    std::vector<std::future<MyPointCloud>> threadVec;
    threadVec.resize(numRobScanPoses);
    while(1){
        for (int i = 0; i < numRobScanPoses; i++){
            auto t = std::async(std::launch::async, &ScanProcessingNode::processPointCloud, this, std::move(fut_vec_MT1_MT2[i]), i);
            threadVec.at(i) = std::move(t);	
        }
        
        for (int i = 0; i < threadVec.size(); i++){
            if(threadVec[i].wait_for(std::chrono::seconds(240)) != std::future_status::ready){
                std::cout << "maximum time of scanning one part reached" << std::endl;
                raise(SIGINT);
                return;
            }
            else{                
                auto t1 = std::chrono::high_resolution_clock::now();

                //this is here, because otherwise there is a problem when later scan is processed faster then previous
                //if i want to ensure ordering of point cloud based on scan order
                completePointCloud->operator+=(threadVec[i].get());
                PPCindices[i+1] = completePointCloud->size();
                resetPromFut(prom_vec_MT1_MT2.at(i), fut_vec_MT1_MT2.at(i));

                auto t2 = std::chrono::high_resolution_clock::now();
                auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                std::cout << "Dur MT2 part 1: " << dur.count()/1000000.0 << std::endl;
            }
        }

        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting mainThread2Loop thread" << std::endl;
            return;
        }

        #if SAVE_PARTIAL_DATA
            pcl::io::savePCDFileBinary (partialOutputDataPath + "completePointCloud.pcd", *completePointCloud);
        #endif
        applyBadTransf(*completePointCloud, partialOutputDataPath);
        auto t1 = std::chrono::high_resolution_clock::now();

        sem_MT2_MT3.acquire();

        CPCindices data;
        data.CPC = *completePointCloud;
        data.PCPindices = PPCindices;
        prom_MT2_MT3.set_value(data); 


        completePointCloud->clear();

        std::cout << "all processing done..." << std::endl; 
        std_srvs::SetBool empty;        
        client_srvs_main_scanProcessing.call(empty);

        auto t2 = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        std::cout << "Dur MT2 part 2: " << dur.count()/1000000.0 << std::endl;
    }    
}

/////////////////////////////main thread 3/////////////////////////////////
void ScanProcessingNode::mainThread3Loop(){
    std::vector<std::future<void>> threadVec;
    threadVec.resize(numRobScanPoses);
    while(true){
        CPCindices data = fut_MT2_MT3.get();
        auto t1 = std::chrono::high_resolution_clock::now(); 
        MyPointCloud::Ptr CPC_MT3 = data.CPC.makeShared();
        auto tShrdPtr = std::chrono::high_resolution_clock::now(); 
        resetPromFut(prom_MT2_MT3, fut_MT2_MT3); //clear memory occupied by promise asap
        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting main thread 3" << std::endl;
            return;
        }       

        //coarse allignment CPC to CAD
        std::cout << "coarse allignment..." << std::endl;
        //without good initial guess
        //Eigen::Affine3f totalTransform =  coarseCPCtoCAD(CPC_MT3, CADcloud, partialOutputDataPath);
        //with good initial guess
        Eigen::Affine3f totalTransform =  coarseCPCtoCAD(CPC_MT3, CADcloud, initialTransform, partialOutputDataPath);
        //std::cout  << totalTransform.matrix() << std::endl;
        #if SAVE_PARTIAL_DATA
            pcl::io::savePCDFileBinary (partialOutputDataPath + "completePointCloudCoarseAlligned.pcd", *CPC_MT3);
        #endif

        auto tfine = std::chrono::high_resolution_clock::now(); 

        //fine allignment PPC to CAD
        std::cout << "fine allignment..." << std::endl;
        std::vector<MyPointCloud::Ptr> PPCvec;
        PPCvec.reserve(numRobScanPoses);
        for (int i = 0; i < numRobScanPoses; i++){
            PPCvec.emplace_back(MyPointCloud(*CPC_MT3,createPPCIndices(data.PCPindices[i] , data.PCPindices[i+1])).makeShared());
            //CAD cloud needs to be passed as new shared ptr to ensure thread safety, 
            //this way each thread has its own CAD cloud heap memory allocated
            //PPC can be pased as ptr because accesing diferent elements of vector is thread safe
            auto t = std::async(std::launch::async, fineRegPPCtoCAD<MyPoint>, PPCvec.at(i), (*CADcloud).makeShared());
            threadVec.at(i) = std::move(t);	
        }

        std::cout << "clearing previous CPC..." << std::endl;
        CPC_MT3->clear();

        auto tfine2 = std::chrono::high_resolution_clock::now(); 

        for (int i = 0; i < numRobScanPoses; i++){
            if(threadVec.at(i).wait_for(std::chrono::seconds(240)) != std::future_status::ready){
                std::cout << "maximum time of scanning one part reached" << std::endl;
                raise(SIGINT);
                return;
            }
            CPC_MT3->operator+=(*PPCvec.at(i));
        }
        
        //ICP duration
        auto t2 = std::chrono::high_resolution_clock::now();
        auto durationShPtr = std::chrono::duration_cast<std::chrono::microseconds>(tShrdPtr - t1);
        std::cout << "Duration shared ptr creation" << durationShPtr.count()/1000000.0 << std::endl;
        auto durationFineRegPrep= std::chrono::duration_cast<std::chrono::microseconds>(tfine2 - tfine);
        std::cout << "Duration fine registration prep" << durationFineRegPrep.count()/1000000.0 << std::endl;
        auto durationFineReg= std::chrono::duration_cast<std::chrono::microseconds>(t2 - tfine);
        std::cout << "Duration fine registration" << durationFineReg.count()/1000000.0 << std::endl;
        auto durationReg= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        std::cout << "Duration coarse and fine registration" << durationReg.count()/1000000.0 << std::endl;

        //create projections from register CPC
        quality_inspection::create2Dprojections CPCmsg;
        pcl::toROSMsg(*CPC_MT3, CPCmsg.request.pc2);
        client_srvs_create2Dprojections.call(CPCmsg);
        std::cout << CPCmsg.response.message << std::endl;
        #if SAVE_PARTIAL_DATA       
            pcl::io::savePCDFileBinary (partialOutputDataPath + "completePointCloudFineAlligned.pcd", *CPC_MT3);
        #endif

        sem_MT2_MT3.release();
        std::cout << "processing in MT3 done" << std::endl;
        //this has to be last to prevent creating new promise and future before exiting
        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting main thread 3" << std::endl;
            return;
        }
    }
}

/////////////////////////////////// other ///////////////////////////////////
ScanProcessingNode::ScanProcessingNode(): sem_MT2_MT3(1, 1) {
    //proper exiting
    exitFunc = boost::bind(&ScanProcessingNode::procScanExitFunc, this);
    exitFuture = exitPromise.get_future();

    //get parameters from parameter server
    if(loadRobotScanPoses() != 0){
        return;
    }
    if(loadExpInitTransform() != 0){
        return;
    }
    if(loadPartialoutputDataPath() != 0){
        return;
    }
    if(loadCADmodel() != 0){
        return;
    }
    if(loadTableZcoord() != 0){
        return;
    }

    //communication between main thread 1 and main thread 2
    numRobScanPoses = robotScanPoses.size(); //this of course has to be before all references to this variable
    fut_vec_MT1_MT2.resize(numRobScanPoses);
    prom_vec_MT1_MT2.resize(numRobScanPoses);
    for(int i  = 0; i < numRobScanPoses; i++){
        //call here to ensure that futures are ready before data callback is processed
        resetPromFut(prom_vec_MT1_MT2.at(i), fut_vec_MT1_MT2.at(i));
    }

    //communication between main thread 2 and main thread 3
    resetPromFut(prom_MT2_MT3, fut_MT2_MT3);

    //initialize Main thread 2 and main thread 3
    std::thread mainThread2 (&ScanProcessingNode::mainThread2Loop, this);
    std::thread mainThread3 (&ScanProcessingNode::mainThread3Loop, this);

    //create subscribers to scanner data topics, which are processed in main thread 1
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(n, "/phoxi_camera/pointcloud", numRobScanPoses);
    message_filters::Subscriber<sensor_msgs::Image> textureSub(n, "phoxi_camera/texture", numRobScanPoses);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pointCloudSub, textureSub, numRobScanPoses);
    sync.registerCallback(boost::bind(&ScanProcessingNode::combCB, this, _1, _2));

    //prerobit na strukturu
    PPCindices.resize(numRobScanPoses+1);
    PPCindices.emplace_back(0);

    //synchronization between Main module and Scan Processing module
    client_srvs_main_scanProcessing = n.serviceClient<std_srvs::SetBool>("comm_main_processScan");
    client_srvs_main_scanProcessing.waitForExistence();

    std_srvs::SetBool empty;
    client_srvs_main_scanProcessing.call(empty);

    //create client to service providing creation of 2D projections
    client_srvs_create2Dprojections = n.serviceClient<quality_inspection::create2Dprojections>("/quality_inspection/create2Dprojections");
    client_srvs_create2Dprojections.waitForExistence();

    ros::spin();
    mainThread2.join();
    mainThread3.join();
}

void ScanProcessingNode::procScanExitFunc(){
    std::cout << "exiting threads started" << std::endl;
    exitPromise.set_value(1);
    if(!prom_vec_MT1_MT2.empty()){
        for(int i  = 0; i < prom_vec_MT1_MT2.size(); i++){
            try{
                prom_vec_MT1_MT2[i].set_value(MyPointCloud());
            }
            //catch error which happens when sigint was evoked while promise was already set 
            catch(const std::future_error& e){}
        }
    }
   
    try{
        prom_MT2_MT3.set_value({MyPointCloud(), std::vector<int>()});
    }
    //catch error which happens when sigint was evoked while promise was already set 
    catch(const std::future_error& e){}
}

int ScanProcessingNode::loadPartialoutputDataPath(){
    if( !n.getParam("/partial_output_data_folder", partialOutputDataPath)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    boost::filesystem::create_directories(partialOutputDataPath);
    return 0;
}

int ScanProcessingNode::loadExpInitTransform(){
    //get path to file containing robot scanning positions
    std::string str1;
    if( !n.getParam("/input_data_folder", str1)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string str2;
    if( !n.getParam("/expected_initial_tranform", str2)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string path = str1 + str2;
    
    std::vector<std::vector<double>> transMatVec;
    int numRowsCols = 4; //homogenous transformation matrix is 4x4
    if(loadTxtTo2DVector(path, transMatVec, numRowsCols, numRowsCols) != 0){
        return -1;
    }
    for(int row = 0; row < numRowsCols; row++){
        for(int col = 0; col < numRowsCols; col++){
            initialTransform(row,col) = transMatVec[row][col];
        }
    }
    std::cout  << initialTransform.matrix() << std::endl;

    return 0;
}

int ScanProcessingNode::loadRobotScanPoses(){
    //get path to file containing robot scanning positions
    std::string str1;
    if( !n.getParam("/input_data_folder", str1)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string str2;
    if( !n.getParam("/robot_scanning_poses", str2)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string path = str1 + str2;
    int numCols = 6; // robot pose is defined as {x, y, z, A, B, C}
    if(loadTxtTo2DVector(path, robotScanPoses, numCols, -1) != 0){
        return -1;
    }

    return 0;
}

int ScanProcessingNode::loadCADmodel(){
    //get path to file containing robot scanning positions
    std::string str1;
    if( !n.getParam("/input_data_folder", str1)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string str2;
    if( !n.getParam("/CAD_model", str2)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }
    std::string path = str1 + str2;

    if (pcl::io::loadPCDFile (path, *CADcloud) == -1){
        PCL_ERROR (std::string("Couldn't read CAD model file").append(path).append("\n").c_str());
        return -1;
    }

    return 0;
}

int ScanProcessingNode::loadTableZcoord(){
    if( !n.getParam("/table_Z_coord",tableZcoord)){
        ROS_ERROR("Failed to get parameter from server.");
        return -1;
    }

    return 0;
}

template<typename T>
void resetPromFut(std::promise<T>& prom, std::future<T>& fut){
    std::promise<T> promise;
    fut = promise.get_future();
    prom = std::move(promise);
}

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
    ros::init(argc, argv, "scan_processing", ros::init_options::NoSigintHandler);  
    std::signal(SIGINT, sigintCB); //should be after ros::init()    
    ScanProcessingNode processScan;
    
    return 0;
}

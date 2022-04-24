#include <scanProcessingNode.h>

std::function<void()> exitFunc;

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
        cv::imwrite(dataPath + "img" + std::to_string(currIdxRobPose) + ".bmp", img);
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
    currIdxRobPose = (currIdxRobPose < numRobPoses) ? currIdxRobPose : 0;

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Dur MT1: " << dur.count()/1000000.0 << std::endl;
}


/////////////////////////////main thread 2/////////////////////////////////

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
        pcl::io::savePCDFileASCII (dataPath + "pointCloud_original" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif
    extractUnmeasuredPoints(pointCloud);  //prepisat typ co ide do funkcie alebo spravit template
    transformPointCloudFromTCPtoRobot(robotScanPoses[currIdxRobPose], pointCloud);
    extractTable(pointCloud);
    #if SAVE_PARTIAL_DATA
        pcl::io::savePCDFileASCII (dataPath + "pointCloud" + std::to_string(currIdxRobPose) + ".pcd", pointCloud);
    #endif
    
    std::cout << "processing point cloud" << currIdxRobPose << "done..." << std::endl;

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    std::cout << "Dur MT2 procesPointCloud: " << dur.count()/1000000.0 << std::endl;
    return pointCloud;
}

void ScanProcessingNode::mainThread2Loop(){
    std::vector<std::future<MyPointCloud>> threadVec;
    threadVec.resize(numRobPoses);
    while(1){
        for (int i = 0; i < numRobPoses; i++){
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

        //pcl::io::savePCDFileASCII (dataPath + "completePointCloud.pcd", *completePointCloud);
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
void fineRegPPCtoCAD(pcl::PointCloud<pcl::PointNormal> CADcloud, MyPointCloud::Ptr PPC){
    //CAD cloud passed as value, in order to ensure thread safety
    //PPC can be pased as ptr because accesing diferent elements of vector is thread safe
    //doICP(PPC, CADcloud, 100000, 300000, 4, 1e-6, 0.003);
    pcl::PointCloud<pcl::PointNormal>::Ptr CADcloudPtr = CADcloud.makeShared();
    doICP(PPC, CADcloudPtr, 100000, 100000, 4, 1e-6, 0.003);
}

void ScanProcessingNode::mainThread3Loop(){
    std::vector<std::future<void>> threadVec;
    threadVec.resize(numRobPoses);
    while(true){
        CPCindices data = fut_MT2_MT3.get();
        auto t1 = std::chrono::high_resolution_clock::now(); 
        MyPointCloud::Ptr CPC_MT3 = data.CPC.makeShared();
        resetPromFut(prom_MT2_MT3, fut_MT2_MT3); //clear memory occupied by promise asap
        if(exitFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
            std::cout << "exiting main thread 3" << std::endl;
            return;
        }
        
        //initial allignment, supposing that inspected parts are arriving with aproximately same position
        //newPcl::transformPointCloudWithNormals(*CPC_MT3, *CPC_MT3, initialAllign);

        //coarse allignment CPC to CAD
        std::cout << "coarse allignment..." << std::endl;
        //Eigen::Affine3f totalTransform =  alignPointClouds(CPC_MT3, CADcloud, 8000, 50000, 15, 1e-4, 0.05);
        Eigen::Affine3f totalTransform =  alignPointClouds(CPC_MT3, CADcloud, 8000, 100000, 6, 1e-5, 0.05);
        //std::cout  << totalTransform.matrix() << std::endl;
        //pcl::io::savePCDFileASCII (dataPath + "completePointCloudCoarseAlligned.pcd", *CPC_MT3);

        auto tfine = std::chrono::high_resolution_clock::now(); 

        //fine allignment PPC to CAD
        std::cout << "fine allignment..." << std::endl;
        std::vector<MyPointCloud::Ptr> PPCvec;
        PPCvec.reserve(numRobPoses);
        for (int i = 0; i < numRobPoses; i++){
            PPCvec.emplace_back(MyPointCloud(*CPC_MT3,createPPCIndices(data.PCPindices[i] , data.PCPindices[i+1])).makeShared());
            auto t = std::async(std::launch::async, fineRegPPCtoCAD, *CADcloud, PPCvec.at(i));
            threadVec.at(i) = std::move(t);	
        }

        std::cout << "clearing previous CPC..." << std::endl;
        CPC_MT3->clear();

        auto tfine2 = std::chrono::high_resolution_clock::now(); 

        for (int i = 0; i < numRobPoses; i++){
            if(threadVec.at(i).wait_for(std::chrono::seconds(240)) != std::future_status::ready){
                std::cout << "maximum time of scanning one part reached" << std::endl;
                raise(SIGINT);
                return;
            }
            CPC_MT3->operator+=(*PPCvec.at(i));
        }
        
        //ICP duration
        auto t2 = std::chrono::high_resolution_clock::now();
        auto durationFineRegPrep= std::chrono::duration_cast<std::chrono::microseconds>(tfine2 - tfine);
        std::cout << "Duration fine registration prep" << durationFineRegPrep.count()/1000000.0 << std::endl;
        auto durationFineReg= std::chrono::duration_cast<std::chrono::microseconds>(t2 - tfine);
        std::cout << "Duration fine registration" << durationFineReg.count()/1000000.0 << std::endl;
        auto durationReg= std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        std::cout << "Duration coarse and fine registration" << durationReg.count()/1000000.0 << std::endl;
        
        //extract distant points
        std::cout << CPC_MT3->size() << std::endl;
        auto tDistStart = std::chrono::high_resolution_clock::now();
        extractDistantPoints(CADcloud, *CPC_MT3);
        auto tDistEnd = std::chrono::high_resolution_clock::now();
        auto durationDistances= std::chrono::duration_cast<std::chrono::microseconds>(tDistEnd - tDistStart);
        std::cout << "Duration distances: " << durationDistances.count()/1000000.0 << std::endl;
        std::cout << CPC_MT3->size() << std::endl;

        //create projections from register CPC
        quality_inspection::create2Dprojections CPCmsg;
        pcl::toROSMsg(*CPC_MT3, CPCmsg.request.pc2);
        client_srvs_create2Dprojections.call(CPCmsg);
        std::cout << CPCmsg.response.message << std::endl;        
        //pcl::io::savePCDFileASCII (dataPath + "completePointCloudFineAlligned.pcd", *CPC_MT3);

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

    //communication between main thread 1 and main thread 2
    numRobPoses = robotScanPoses.size(); //this of course has to be before all references to this variable
    fut_vec_MT1_MT2.resize(numRobPoses);
    prom_vec_MT1_MT2.resize(numRobPoses);
    for(int i  = 0; i < numRobPoses; i++){
        //call here to ensure that futures are ready before data callback is processed
        resetPromFut(prom_vec_MT1_MT2.at(i), fut_vec_MT1_MT2.at(i));
    }

    //communication between main thread 2 and main thread 3
    resetPromFut(prom_MT2_MT3, fut_MT2_MT3);
    
    //create datapath and load CAD model for registration
    createDataPath();
    std::cout << dataPath << std::endl;
    loadPointCloud(CADcloud, dataPath + "pointCloudCAD407Ktransformed.pcd");

    //initialize Main thread 2 and main thread 3
    std::thread mainThread2 (&ScanProcessingNode::mainThread2Loop, this);
    std::thread mainThread3 (&ScanProcessingNode::mainThread3Loop, this);

    //create subscribers to scanner data topics, which are processed in main thread 1
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(n, "/phoxi_camera/pointcloud", numRobPoses);
    message_filters::Subscriber<sensor_msgs::Image> textureSub(n, "phoxi_camera/texture", numRobPoses);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pointCloudSub, textureSub, numRobPoses);
    sync.registerCallback(boost::bind(&ScanProcessingNode::combCB, this, _1, _2));

    //synchronization between Main module and Scan Processing module
    client_srvs_main_scanProcessing = n.serviceClient<std_srvs::SetBool>("comm_main_processScan");
    client_srvs_main_scanProcessing.waitForExistence();
    std_srvs::SetBool empty;
    client_srvs_main_scanProcessing.call(empty);

    //create client to service providing creation of 2D projections
    client_srvs_create2Dprojections = n.serviceClient<quality_inspection::create2Dprojections>("/quality_inspection/create2Dprojections");
    client_srvs_create2Dprojections.waitForExistence();

    //prerobit na strukturu
    PPCindices.resize(numRobPoses+1);
    PPCindices.emplace_back(0);

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

void ScanProcessingNode::createDataPath(){
    dataPath = ros::package::getPath("quality_inspection"); 
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
    dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
    dataPath = dataPath + "/InspectionFiles/";
    boost::filesystem::create_directories(dataPath);
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
    ros::init(argc, argv, "scanProcessingNode", ros::init_options::NoSigintHandler);  
    std::signal(SIGINT, sigintCB); //should be after ros::init()    
    ScanProcessingNode processScan;
    
    return 0;
}

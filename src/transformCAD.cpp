#include <ros/package.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <newTransform.hpp>
#include <pcl/io/ply_io.h>

class TransformCAD{
    public:
        TransformCAD(){
            createDataPath();
            // std::string cloudName = "pointCloudCAD407K";
            // pcl::io::loadPCDFile(dataPath + "1layerData/" + cloudName + ".pcd", CADcloud);
            // transformMatrix = transfToOrigin(CADcloud);            
            // pcl::io::savePCDFileBinary(dataPath + "1layerData/" + cloudName + "transformed.pcd", CADcloud);
            // std::cout  << transformMatrix.matrix() << std::endl;
            
            // loadTransSaveCloud(dataPath + "1layerData/", "pointCloudCAD101K");
            // loadTransSaveCloud(dataPath + "1layerData/", "pointCloudCAD2600K");
            genRivetPosFile();
        };
    
    private:
        std::string dataPath;
        pcl::PointCloud<pcl::PointNormal> CADcloud;
        Eigen::Affine3f transformMatrix;

        // data paths
        void createDataPath(){
            dataPath = ros::package::getPath("quality_inspection"); 
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));    
            dataPath = dataPath.substr(0, dataPath.find_last_of("/\\"));       
            dataPath = dataPath + "/cadModels/";
            boost::filesystem::create_directories(dataPath);
        }

        template<typename T>
            Eigen::Affine3f transfToOrigin(pcl::PointCloud<T>& cloud){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(cloud, centroid);
            Eigen::Affine3f translation = (Eigen::Affine3f)Eigen::Translation3f(
                                            -centroid[0], -centroid[1], -centroid[2]);

            newPcl::transformPointCloudWithNormals(cloud, cloud, translation);
            return translation;
        }        
        
        void loadTransSaveCloud(std::string path, std::string cloudName){            
            pcl::io::loadPCDFile(path + cloudName + ".pcd", CADcloud);
            newPcl::transformPointCloudWithNormals(CADcloud, CADcloud, transformMatrix);           
            pcl::io::savePCDFileBinary(path + cloudName + "transformed.pcd", CADcloud);
        }

        void genRivetPosFile(){
            pcl::PointCloud<pcl::PointXYZ> CADcloud2layer;
            pcl::io::loadPLYFile(dataPath + "2layerData/" + "dvojvrstvovy model scaledTransformed.ply", CADcloud2layer);
            // int idx = 135179;
            // std::cout << CADcloud2layer.points.at(idx).x << " " << CADcloud2layer.points.at(idx).y << " " << CADcloud2layer.points.at(idx).z << std::endl;

            std::ofstream file;
            file.open(dataPath + "2layerData/" + "rivetPositions.txt");
            if(!file){
                std::cout<<"File not created...";
                return;
            }
            std::vector<int> rivetIdxs = getRivetIdxs();
            pcl::PointXYZ point;
            for (auto& rivetIdx : rivetIdxs){
                point = CADcloud2layer.points.at(rivetIdx);                
                file << point.x << " " << point.y << " " << point.z << "\n";
            }

            file.close();
        }

        std::vector<int> getRivetIdxs(){
            std::vector<int> rivetIdxs = {
                136643, 135179, 160067, 155675, 158603, 154211, 157139, 152747, 129323, 130787,
                132251, 89338, 119075, 107363, 105899, 108827, 104435, 110291, 102971, 116147, 
                87874, 120539, 123467, 122003, 139571, 138107, 151283, 146891, 149819, 145427,
                148355, 143963, 133715, 142499, 141035, 124931, 126395, 127859, 114683, 113219,
                111755};
            return rivetIdxs;
        }
};

int main(int argc, char** argv){    
    TransformCAD transformCAD;
    return 0;
}
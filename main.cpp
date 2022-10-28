#include <iostream>
#include <vector>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


#include <pcl/filters/statistical_outlier_removal.h>

#include "iros.h"

void ccc(pcl::visualization::PCLVisualizer& viewer) {
	viewer.removeAllShapes();
	for(int i =0;i < 10;i++)
	{
		pcl::ModelCoefficients circle_coeff;
		circle_coeff.values.resize (3);    // We need 3 values
		circle_coeff.values[0] = 0;
		circle_coeff.values[1] = 0;
		circle_coeff.values[2] = i*10;
		const std::string sId("cir_"+std::to_string(i));
		
		viewer.addCircle(circle_coeff,sId,0);
		viewer.setShapeRenderingProperties(
											pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
											pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
											sId);
	}
}


std::vector<std::string> getFileNames(std::string path) {
    std::vector<std::string> filenames;
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir=opendir(path.c_str())))
        return filenames;
    while((ptr=readdir(pDir))!=0) {
        if(strcmp(ptr->d_name,".")!=0 && strcmp(ptr->d_name,"..")!=0) {
            std::string filename(ptr->d_name);
            if(filename.length()>0){
                filenames.push_back(filename);
            } else {
                std::cout << "filename length = 0. " << std::endl;
            }
        }
    }
    closedir(pDir);

    std::sort(filenames.begin(), filenames.end(), [](const std::string & a, const std::string & b) { return a<b;});

    return filenames;
}

Eigen::Matrix4f get_lidar_extern(std::string path) {
    Eigen::Matrix4f result;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "LoadExtrinsicFile Failed to open " << path << std::endl;
        exit(-1) ;
    }

    cv::Mat T44;
    fs["Rt"] >> T44;

    cv::cv2eigen(T44, result);
    Eigen::Matrix4f  result_inv = result.inverse();

    return result_inv;
}

void merge_pointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                      Eigen::Matrix4f extern_1,
                      pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                      Eigen::Matrix4f extern_2,
                      pcl::PointCloud<pcl::PointXYZ> &cloud_sum) {

    pcl::PointCloud<pcl::PointXYZ> cloud_2_in1;


    pcl::transformPointCloud(cloud_2, cloud_2_in1, extern_2);

    cloud_sum = cloud_1 + cloud_2_in1;

}

std::string find_filename_nearest(int& id, std::string name_target, std::vector<std::string>& names_source) {
    std::string result;

    std::string s = names_source[id];

    while (s < name_target && id < names_source.size()-1) {
        id++;
        s = names_source[id];
    }
    if (id > 1) {
        return names_source[id-1];
    } else {
        return names_source[0];
    }
    
}

double getTimeNow()
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch());
    std::chrono::nanoseconds nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time.time_since_epoch());

    // std::cout<<"now (s):"<<sec.count()<<std::endl;
    // std::cout<<"now (ns):"<<nsec.count()<<std::endl;
    return nsec.count()*10e-10;
}



int main(int argc, char** argv) {
    
    IROS::Params params;
    IROS::IROS_processor iros(params);

 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rear (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);

    std::string path_front = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/0/";
    std::string path_rear  = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/1/";
    std::string path_left  = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/2/";
    std::string path_right = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/3/";
    std::vector<std::string> pointcloud_filenames_front = getFileNames(path_front);
    // std::vector<std::string> pointcloud_filenames_rear  = getFileNames(path_rear);
    std::vector<std::string> pointcloud_filenames_left  = getFileNames(path_left);
    std::vector<std::string> pointcloud_filenames_right = getFileNames(path_right);


    // pcl::visualization::CloudViewer cloud_viewer("DBSCAN Cluster Result");
	
	pcl::visualization::PCLVisualizer pcl_viewer ("PCL viewer");
	// int view_point (0);
	// pcl_viewer.createViewPort (0.0, 0.0, 1.0, 1.0, view_point);
	// pcl_viewer.addCoordinateSystem(1);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud_total, 
	// 	255, 255, 255);

	std::string lidarCalib_path_front = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/calib/front.json";
    // std::string lidarCalib_path_rear  = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/
    //                                     lidar_decoded/calib/rear.json";
    std::string lidarCalib_path_left  = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/calib/left.json";
    std::string lidarCalib_path_right = "/home/yihang/DataBase/sensers_data_2022-07-27_10-36-12/lidar_decoded/calib/right.json";
    Eigen::Matrix4f extern_front = get_lidar_extern(lidarCalib_path_front);
    // Eigen::Matrix4f extern_rear = get_lidar_extern(lidarCalib_path_rear);
    Eigen::Matrix4f extern_left = get_lidar_extern(lidarCalib_path_left);
    Eigen::Matrix4f extern_right = get_lidar_extern(lidarCalib_path_right);

    
    int frame_start = 1500;
    int frame_rear = 0; 
    int frame_left = 0;
    int frame_right = 0;

	srand(time(0)); // 根据时间生成随机数

    for (int frame_id = frame_start; frame_id < pointcloud_filenames_front.size(); frame_id++) {
        
        std::string file_name_front = path_front + pointcloud_filenames_front[frame_id];
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_front, *cloud_front)) {
            std::cout<<"cannot load the '.pcd' you need"<<std::endl;
            return 0;
        }
  
        // std::string file_name_rear  = path_rear  + find_filename_nearest(frame_rear, 
        //                                                                  pointcloud_filenames_front[frame_id],
        //                                                                  pointcloud_filenames_rear);
        std::string file_name_left  = path_left  + find_filename_nearest(frame_left,
                                                                         pointcloud_filenames_front[frame_id],
                                                                         pointcloud_filenames_left);
        std::string file_name_right = path_right + find_filename_nearest(frame_right,
                                                                         pointcloud_filenames_front[frame_id],
                                                                         pointcloud_filenames_right);
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_left, *cloud_left)) {
            std::cout<<"cannot load the '.pcd' you need"<<std::endl;
            return 0;
        }
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_right, *cloud_right)) {
            std::cout<<"cannot load the '.pcd' you need"<<std::endl;
            return 0;
        } 

        Eigen::Matrix4f extern_one = Eigen::MatrixXf::Identity(4,4);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_empty(new pcl::PointCloud<pcl::PointXYZ>);
        merge_pointcloud(*cloud_empty, extern_one, *cloud_front, extern_front, *cloud_total);
        merge_pointcloud(*cloud_total, extern_one, *cloud_left , extern_left,  *cloud_total);
        merge_pointcloud(*cloud_total, extern_one, *cloud_right, extern_right, *cloud_total);

        double time_1 = getTimeNow();
        std::vector<int> label;
        iros.segment(*cloud_total, &label);
        double time_2 = getTimeNow();
        // std::cout<<time_2 - time_1<<std::endl;


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_total, *cloud_color);

        for (size_t i = 0; i < cloud_color->points.size(); ++i) {
            cloud_color->points[i].r = 255;
            cloud_color->points[i].g = 0;
            cloud_color->points[i].b = 0;
        }

        for (size_t i = 0; i < label.size(); ++i) {
            if (label[i] == 1) {
                cloud_color->points[i].r = 0;
                cloud_color->points[i].g = 255;
                cloud_color->points[i].b = 0;
            }
        }

		// cloud_viewer.showCloud(cloud_color);
		// cloud_viewer.runOnVisualizationThreadOnce(ccc);

        pcl_viewer.removeAllPointClouds();
        pcl_viewer.removeAllShapes();
        pcl_viewer.addPointCloud(cloud_color, "cloud", 0);
        pcl_viewer.spinOnce(1);

		// std::cout<<frame_id<<" "<<cloud_color->points.size()<<std::endl;
	}
	return 0;
}

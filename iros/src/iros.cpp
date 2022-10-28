#include "iros.h"

namespace IROS {

float xy_to_radius(const float &x, const float &y) {
    return sqrt(x * x + y * y);
}

float xy_to_theta(const float &x, const float &y) { // 0 ~ 2 * PI
    float angle = atan2(y, x);
    return angle > 0 ? angle : 2 * M_PI + angle;
}

IROS_processor::IROS_processor(IROS::Params params) {
    m_params = params; 

    float range = params.range_max - params.range_min;
    for (int i = 0; i < params.zones_num; i++) {
        float angle_delta = 2 * M_PI / params.each_zone_sectors_num[i];
        float range_zone = params.range_min + 
            range * (params.zones_cut_method[i+1] - params.zones_cut_method[i]);
        float range_zone_min = range * params.zones_cut_method[i];
        float range_delta = range_zone / params.each_zone_rings_num[i];

        for (int j = 0; j < params.each_zone_rings_num[i]; j++) {  
            sectors sectors_tmp;
            std::vector<float> sectors_angle;

            for (int k = 0; k < params.each_zone_sectors_num[i]; k++) {
                IROS::Patch patch_tmp;
                patch_tmp.angle_min = angle_delta * k;
                patch_tmp.angle_max = patch_tmp.angle_min + angle_delta;
                patch_tmp.range_min = range_zone_min + range_delta * j;
                patch_tmp.range_max = patch_tmp.range_min + range_delta;

                sectors_angle.push_back(patch_tmp.angle_min);
                sectors_tmp.push_back(patch_tmp);
            }
            m_rings_range.push_back(range_zone_min + range_delta * (j + 1));
            m_sectors_angle.push_back(sectors_angle);
            m_total_patches.push_back(sectors_tmp);
        }   
    }

    for (int i = 0; i < m_rings_range.size(); i++) {
        std::cout<<m_rings_range[i]<<std::endl;
    }
    std::cout<<"!!!"<<std::endl;

    // for (int i = 0; i < m_rings_range.size(); i++) {
    //     for (int j = 0; j < m_sectors_angle[i].size(); j++) {
    //         std::cout<<m_sectors_angle[i][j]<<" ";
    //     }
    //     std::cout<<std::endl;
    // }
}

IROS_processor::IROS_processor() {

}

IROS_processor::~IROS_processor() {
    
}

void IROS_processor::segment(const pcl::PointCloud<pcl::PointXYZ>& cloud, 
                             std::vector<int>* ground_points_id) {
    if (cloud.size() == 0) {
        return ;
    }
    ground_points_id->clear();
    ground_points_id->resize(cloud.size(), -1); // 将全部点的分类结果初始化为-1

    clearCache();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_input = cloud.makeShared();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> id_valid;
    preprocess(cloud_input, cloud_process, id_valid); // 对原始点云进行预处理，保留有效点
    // std::cout<<cloud_process->points.size()<<std::endl;

    separate_points(cloud_process, id_valid);
        
    // test 
    // for (int i = 0 ; i < m_total_points.size(); i++) {
    //     if (m_total_points[i].ring_id == 0 && m_total_points[i].sector_id == 15) {
    //         ground_points_id->at(m_total_points[i].order) = 1;
    //     }
    // }
    // for (int i = 0; i < m_total_patches[0][15].points.size(); i++) {
    //     ground_points_id->at(m_total_patches[0][15].points[i].order) = 1;
    // }

    update_plane();

    judge();

    for (int i = 0; i < m_total_patches.size(); i++) {
        for (int j = 0; j < m_total_patches[i].size(); j++) {
            for (int k = 0; k < m_total_patches[i][j].points.size(); k++) {
                bool is_ground = calc_correltation(i, j, k);
                if (is_ground) {
                    ground_points_id->at(m_total_patches[i][j].points[k].order) = 1;
                } else {
                    ground_points_id->at(m_total_patches[i][j].points[k].order) = 0;
                }
            }
        }
    }
    
}

void IROS_processor::preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result,
                                std::vector<int>& id_valid) {
    pcl::removeNaNFromPointCloud(*cloud, *cloud_result, id_valid);
}

void IROS_processor::separate_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                     std::vector<int>& id_valid) {
    for (int i = 0; i < cloud->points.size(); i++) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        float radis = xy_to_radius(x, y);
        float theta;
        int label;
        if (radis < m_params.range_max && radis > m_params.range_min) {
            theta = xy_to_theta(x, y);
        } else {
            m_total_points.push_back(IROS::Point(id_valid[i], x, y, z, -1));
            continue;
        }

        int ring_id;
        for (int m = 0 ; m < m_rings_range.size(); m++) {
            if (radis <= m_rings_range[m]) {
                ring_id = m;
                break;
            }
        }
        int sector_id;
        for (int n = 0 ; n < m_sectors_angle[ring_id].size(); n++) {
            if (theta < m_sectors_angle[ring_id][n]) {
                sector_id = n - 1;
                break;
            }
            sector_id = m_sectors_angle[ring_id].size()- 1;
        }
        IROS::Point p(id_valid[i], x, y, z, ring_id, sector_id, 0);
        m_total_points.push_back(p);
        m_total_patches[ring_id][sector_id].points.push_back(p);
    }
}

void IROS_processor::update_plane() {
    for (int j = 0; j < m_total_patches[0].size(); j++) {
        m_total_patches[0][j].plane_param.a = 0;
        m_total_patches[0][j].plane_param.b = 0;
        m_total_patches[0][j].plane_param.c = 0;
        m_total_patches[0][j].plane_param.d = 0;

        for (int i = 0; i < m_total_patches.size(); i++) {
            m_total_patches[i][j].plane_param.a = 0;
            m_total_patches[i][j].plane_param.b = 0;
            m_total_patches[i][j].plane_param.c = 0;
            m_total_patches[i][j].plane_param.d = 0;
        }
    }

}

void IROS_processor::judge() {
    for (int i = 0; i < m_total_patches.size(); i++) {
        for (int j = 0; j < m_total_patches[i].size(); j++) {

        }
    }
}

bool IROS_processor::calc_correltation(int ring_id, int sector_id, int point_id) {
    float result = m_total_patches[ring_id][sector_id].points[point_id].x 
                   * m_total_patches[ring_id][sector_id].plane_param.a + 
                   m_total_patches[ring_id][sector_id].points[point_id].y 
                   * m_total_patches[ring_id][sector_id].plane_param.b + 
                   m_total_patches[ring_id][sector_id].points[point_id].z 
                   * m_total_patches[ring_id][sector_id].plane_param.c + 
                   m_total_patches[ring_id][sector_id].plane_param.d;
    if (result == 0) {
        return true;
    } else {
        return false;
    }
}

void IROS_processor::clearCache() {
    m_total_points.clear();
    std::vector<IROS::Point>(m_total_points).swap(m_total_points);
    
    for (int i = 0; i < m_total_patches.size(); i++) {
        for (int j = 0; j < m_total_patches[i].size(); j++) {
            m_total_patches[i][j].points.clear();
            std::vector<IROS::Point>(m_total_patches[i][j].points).swap(m_total_patches[i][j].points);
        }
    }
    
}





} // namespace IROS

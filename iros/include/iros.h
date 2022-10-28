#ifndef __IROS_H__
#define __IROS_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define CLIMB_Z_THRESHOLD 0.1
#define CLIMB_ANGLE_THRESHOLD 15


namespace IROS {

struct Params {
    int zones_num;
    std::vector<float> zones_cut_method;
    std::vector<int> each_zone_sectors_num;
    std::vector<int> each_zone_rings_num;

    float range_max;
    float range_min;
    
    Params() {
        zones_num = 4;
        zones_cut_method = {0, 0.125, 0.25, 0.5, 1};
        each_zone_sectors_num = {16, 32, 54, 32};
        each_zone_rings_num = {2, 4, 4, 4};

        range_min = 1.0; 
        range_max = 100.0;
    }
};

struct Point {
    int order;
    float x;
    float y;
    float z;
    int ring_id;
    int sector_id;
    int label;

    Point() {}
    Point(int order_, float x_, float y_, float z_, int label_) : 
        order(order_), x(x_), y(y_), z(z_), label(label_) {}
    Point(int order_, float x_, float y_, float z_, int ring_id_, int sector_id_, int label_) 
        : order(order_), x(x_), y(y_), z(z_), ring_id(ring_id_), sector_id(sector_id_), label(label_) {}
};

struct PlaneParameter {
    float a;
    float b;
    float c;
    float d;
    PlaneParameter(float a_, float b_, float c_, float d_)
        : a(a_), b(b_), c(c_), d(d_) {};
    PlaneParameter() {}
};

struct Patch {
    float angle_min;
    float angle_max;
    float range_min;
    float range_max;

    std::vector<IROS::Point> points;

    PlaneParameter plane_param;

};



class IROS_processor {
    public:
        IROS_processor(IROS::Params params);
        IROS_processor();
        ~IROS_processor();

        void segment(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<int>* ground_points_id);
    
    public:
        IROS::Params m_params;

        typedef std::vector<IROS::Patch> sectors;
        typedef std::vector<sectors> rings;

    private:
        void preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result,
                        std::vector<int>& id_valid);
        
        void separate_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                             std::vector<int>& id_valid);

        void update_plane();
        
        void judge();

        bool calc_correltation(int ring_id, int sector_id, int point_id);

        void clearCache();

    private:
        std::vector<IROS::Point> m_total_points;
        rings m_total_patches;

        std::vector<float> m_rings_range;
        std::vector<std::vector<float>> m_sectors_angle;

        // std::vector<float> m_zones_range_min; // 4个zone各自的range最小值
        // std::vector<float> m_zones_sector_angle; // 4个zone各自的扇区角度大小
        // std::vector<float> m_zones_range_distance; // 4个zone各自的环距离大小

};



} // namespace IROS

#endif // __IROS_H__
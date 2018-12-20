#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

ros::Publisher pub;

Eigen::Vector3f yang(0.027775198, -0.085673995, 0.50992984);        // 地面法线(作为先验知识标定时计算使用)

void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *laser_ptr);
    
    std::cout << std::setprecision(15) << ros::Time::now() << std::endl;

    // 计算每个点的法线方向
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>(laser_ptr));
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    n.setInputCloud(laser_ptr);
    n.setSearchMethod(kdtree);
    n.setKSearch(30);            // 用30个点计算法线方向,可调
    n.compute(*normals);

    // 根据法线之间的夹角判断是否为地面点
    pcl::PointCloud<pcl::PointXYZ> outplane;
    for(int i = 0; i < normals->points.size(); i ++)
    {
        pcl::Normal n = normals->points[i];
        Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
        float angle = acos(yang.dot(v) / (yang.norm() * v.norm())) * 180.0 / M_PI;      // 地面点法线接近平行
        if(angle > 20 && angle < 160)                                           
        {
            outplane.push_back(laser_ptr->points[i]);
        }
    }
    
    std::cout << std::setprecision(15) << ros::Time::now() << std::endl;
    
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(outplane, out);
    out.header = msg->header;
    pub.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "out");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_200", 100, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne", 100);
    ros::spin();
    return 0;
}

#include <pcl/point_cloud.h>          //点类型的定义
#include <pcl/kdtree/kdtree_flann.h>  //kdtree类的定义

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv[])
{
    srand(time (NULL));   //使用系统时间初始化随机数种子

    //随机点云生成
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for(auto i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f); //产生数值为0到1024的浮点数
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    //创建KdTreeFLANN对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //设置搜索搜索g空间
    kdtree.setInputCloud(cloud);
    //设置查询点并随机赋值
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    
    //K近邻搜索
    //创建整数K
    int K = 10;
    //存储查询点近邻索引
    std::vector<int> pointIdxNKNSearch(K);
    //存储近邻点对应距离平方
    std::vector<float> pointNKNSquareDistance(K);
    std::cout << "K nearest neighbor search at: " << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << std::endl;

    if(kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquareDistance) > 0){
        for(auto i = 0; i < pointIdxNKNSearch.size(); ++i){
            std::cout << "  " << cloud->points[pointIdxNKNSearch[i]].x << " "
                              << cloud->points[pointIdxNKNSearch[i]].y << " "
                              << cloud->points[pointIdxNKNSearch[i]].z << " (squared distance: " 
                              << pointNKNSquareDistance[i] << ")" << std::endl;
        }
    }


    //查找给定的searchPoint的某一半径内的所有近邻
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    std::cout << "Neighbors with radius search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z 
              << ") with radius=" << radius << std::endl;

    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)){ 
        for(auto i = 0; i < pointIdxRadiusSearch.size(); ++i){
            std::cout << "  " << cloud->points[pointIdxRadiusSearch[i]].x << " "
                              << cloud->points[pointIdxRadiusSearch[i]].y << " "
                              << cloud->points[pointIdxRadiusSearch[i]].z << " (squared distance: " 
                              << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }
    }

    return 0;
}
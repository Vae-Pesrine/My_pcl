#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>  //八叉树头文件

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv)
{
    srand((unsigned int) time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    for(auto i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    /***************************************************************
     创建octree实例，设置分辨率进行初始化，该octree用它的页节点存放索引量，
     分辨率参数描述最低以及的octree的最小体素的尺寸，因此octree的深度是分辨率
     和点云空间维度的函数，如果知道点云的边界框，应该用defineBoundingbox的方法
     把它分配给octree然后通过点云指针把所有点增加到octree
     ***************************************************************/
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();  //构建octree
    
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    //存储体素近邻搜索结果对象
    std::vector<int> pointIdxVec;
    if(octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors with voxel search at:(" << searchPoint.x << " " << searchPoint.y << " " <<  searchPoint.z << ")" << std::endl;
        for(auto i = 0; i < pointIdxVec.size(); ++i)
        {
            std::cout << "  " << cloud->points[pointIdxVec[i]].x << " " << cloud->points[pointIdxVec[i]].y << " " << cloud->points[pointIdxVec[i]].z << std::endl;
        }
    }


    /**********************************************************************************
     K 被设置为10 ，K近邻搜索  方法把搜索结果写到两个分开的向量，第一个pointIdxNKNSearch包含搜索结果
   （结果点的索引的向量）  第二个向量pointNKNSquaredDistance存储搜索点与近邻之间的距离的平方。
    *************************************************************************************/

    //K 近邻搜索
    int K = 10;

    std::vector<int> pointIdxNKNSearch;         //结果点的索引的向量
    std::vector<float> pointNKNSquaredDistance; //搜索点与近邻之间的距离的平方

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径内近邻搜索

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
                      << " " << cloud->points[pointIdxRadiusSearch[i]].y
                      << " " << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    return 0;
}
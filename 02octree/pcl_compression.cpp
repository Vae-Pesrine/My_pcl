//点云压缩
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/io/openni_grabber.h>    //点云获取接口类
#include <pcl/visualization/cloud_viewer.h>  //点云可视化类
#include <pcl/compression/octree_pointcloud_compression.h> //点云压缩类

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
#define sleep(x) Sleep((x) * 1000)
#endif

class SimpleOPENNIViewer
{
public:
    SimpleOPENNIViewer(): viewer("Point Cloud Compression Example") {}

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if(!viewer.wasStopped())
        {
            //存储压缩点云的字节流对象
            std::stringstream compressedData;
            //存储输出点云
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>);
            //压缩点云
            PointCloudEncoder->encodePointCloud(cloud, compressedData);

            //解压缩点云
            PointCloudDecoder->decodePointCloud(compressedData, cloudOut);

            //可视化解压缩的点云
            viewer.showCloud(cloudOut);
        }
    }

    void run()
    {
        bool showStatistics = true;
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
    
        //创建从OPENNI获取点云的抓取对象
        pcl::Grabber *interface = new pcl::OpenNIGrabber();
        //建立回调函数
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&SimpleOPENNIViewer::cloud_cb_, this, _1);
        //建立回调函数和回调信息的绑定
        boost::signals2::connection c = interface->registerCallback(f);
        //开始接受点云的数据流
        interface->start();
        while(!viewer.wasStopped()){
            sleep(10);
        }
        interface->stop();

        //删除实例
        delete (PointCloudDecoder);
        delete (PointCloudEncoder);
    }


    pcl::visualization::CloudViewer viewer;
    
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudDecoder;
};

int main(int argc, char **argv)
{
    //创建实例
    SimpleOPENNIViewer v; 
    v.run();

    return 0;
}
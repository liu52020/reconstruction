#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// filter processing
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
/*
 * 楼梯面基本步骤
 * 1. 输入一个阶梯的点云数据   可以是ROS直接获取 也可以是深度图转换
 * 2. 按照曲率变换将其分成竖直面块和水平面块   可以有误差 但是尽量细化
 * 3. 按照每个块将其精细化  水平面固定x值   竖直面固定z值
 * */

int main() {
    // 平面类
    class Plane{
    public:
        Plane(){}

        // 参数
        int type;
        pcl::PointXYZ center; // Center of bounding rectangle  点
        int height;
    };
    //加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile ("/home/liuwei/data/tang_stairs/Positive_PCD/1_Cloud_XYZRGB.pcd", *cloud); // "/home/liuwei/data/stair_pointcloud_rs/15_pointcloud.pcd"
//    pcl::io::loadPCDFile ("/home/liuwei/data/stair_pointcloud_rs/15_pointcloud.pcd", *cloud);
    pcl::io::loadPCDFile ("/home/liuwei/data/dataset/upstair/pcd/144.pcd", *cloud);
    std::cout<<"原点云点数"<<cloud->points.size()<<std::endl;
    /// 先进行滤波
    // 对数据进行处理  直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (0.0, 3.0);        //设置在过滤字段的范围
    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud (cloud_filtered);            //设置输入点云
    pass1.setFilterFieldName ("x");         //设置过滤时所需要点云类型的y字段
    pass1.setFilterLimits (-2, 2);        //设置在过滤字段的范围
    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass1.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud (cloud_filtered);            //设置输入点云
    pass2.setFilterFieldName ("y");         //设置过滤时所需要点云类型的y字段
    pass2.setFilterLimits (-2, 0);      //设置在过滤字段的范围
    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass2.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;  //创建滤波对象
    sor.setInputCloud (cloud_filtered);            //设置需要过滤的点云给滤波对象
    sor.setLeafSize (0.02f, 0.02f, 0.02f);  //设置滤波时创建的体素体积为1cm的立方体
    sor.filter (*cloud_filtered);           //执行滤波处理，存储输出
    std::cout<<"滤波"<<cloud_filtered->points.size()<<std::endl;

    // 点云平滑 移动最小二乘
    // 对点云重采样
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling (new pcl::search::KdTree<pcl::PointXYZ>); // 创建用于最近邻搜索的KD-Tree
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);   //输出MLS
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls;  // 定义最小二乘实现的对象mls
    mls.setComputeNormals (false);  //设置在最小二乘计算中是否需要存储计算的法线
    mls.setInputCloud (cloud_filtered);        //设置待处理点云
    mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合，一般取2-5
    mls.setPolynomialFit (false);  // 设置为false可以 加速 smooth
    mls.setSearchMethod (treeSampling);    // 设置KD-Tree作为搜索方法
    mls.setSearchRadius (0.05); // 单位m.设置用于拟合的K近邻半径，越大平滑力度越大
    mls.process (*mls_points);        //输出

//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;   //创建滤波器对象
//    sor1.setInputCloud (cloud_filtered);                           //设置待滤波的点云
//    sor1.setMeanK (50);                               //设置在进行统计时考虑查询点临近点数
//    sor1.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
//    sor1.filter (*cloud_filtered);

    // 处理点云的点  以Z轴为目标将z轴的点取出来
    // 方法： 定义一个平面vecotr  存放好的平面信息
    std::vector<Plane> vPlanes; // Vector of Plane objects in the scene
    std::cout<<cloud_filtered->width<<std::endl;
    std::cout<<cloud_filtered->height<<std::endl;

    int number = 0;
//    for(size_t i=0;i<cloud_filtered->points.size();++i){
//        // 遍历所有的点
//        if(cloud_filtered->points[i].y<=-1&&cloud_filtered->points[i].y>=-1.15){
//            plane_cloud->points[number++]  = cloud_filtered->points[i];  // 赋值
//        }
//    }

    std::cout<<number<<std::endl;
    std::cout<<cloud_filtered->points[1].x<<std::endl;
    std::cout<<cloud_filtered->points[1].y<<std::endl;
    std::cout<<cloud_filtered->points[1].z<<std::endl;

    //存储
    //估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    //创建一个空的kdtree对象，并把它传递给法线估计对象
    //基于给出的输入数据集，kdtree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod (tree);
    //输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //使用半径在查询点周围3厘米范围内的所有邻元素
    ne.setRadiusSearch (0.03);
    //计算特征值
    ne.compute (*cloud_normals);
    // cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同尺寸


    // 保存
    pcl::io::savePCDFile("/home/liuwei/data/dataset/20210915/pcd/test.pcd",*cloud_filtered);
    std::cout<<"save success"<<std::endl;
    //法线可视化
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
////    viewer->initCameraParameters ();
//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer1"));
    viewer1->initCameraParameters ();
    viewer1->setCameraPosition(0, 0, -1.0, 0, -1, 0); // 设定视角

//    int v1(0); // 创建视口
//    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
//    viewer.setBackgroundColor (0.0, 0.0, 0.0,v1);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals,v1);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); //
//    int v1(0);
//    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
//    viewer->setBackgroundColor (0.0, 0.0, 0.0,v1);
//    viewer->addText("original pointcloud", 10, 10, "v1 text", v1);
//    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered,"original pointcloud",v1);
//    int v2(0);
//    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
//    viewer->setBackgroundColor (0.0, 0.0, 0.0,v2);
//    viewer->addText("filtered pointcloud", 10, 10, "v1 text", v2);
//    viewer->addPointCloud<pcl::PointXYZ>(cloud,"filtered pointcloud",v2);

//    viewer->setBackgroundColor (0.0, 0.0, 0.0);
//    viewer->addText("original pointcloud", 10, 10, "v1 text");
//    viewer->addPointCloud<pcl::PointXYZ>(cloud,"original pointcloud");
//
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }

    viewer1->setBackgroundColor (0.0, 0.0, 0.0);
    viewer1->addText("filtered pointcloud", 10, 10, "v1 text");
    viewer1->addPointCloud<pcl::PointXYZ>(mls_points,"filtered pointcloud");

    while (!viewer1->wasStopped ())
    {
        viewer1->spinOnce ();
    }




    return 0;
}

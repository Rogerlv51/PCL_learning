#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std::chrono_literals;

// 基于颜色的区域生长分割算法

int
main ()
{
  // 建立基于XYZRGB的kd树用来查找搜索
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../region_growing_rgb_tutorial.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud (*cloud, *indices);  // 去除点云中的无效点，indices保存有效点的索引

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;   // 创建区域生长对象
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);  // 用于确定邻域点的距离阈值，小于这个值的则认为是一簇
  reg.setPointColorThreshold (6);   // 颜色阈值
  reg.setRegionColorThreshold (5);   // 用在合并过程中的颜色阈值
  reg.setMinClusterSize (600);   // 确定簇的最小点数

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);   // 返回簇的索引数组

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    std::this_thread::sleep_for(100us);
  }

  return (0);
}
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>


int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: match target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }
  
  Eigen::Matrix4d tf_velo2cam;
  tf_velo2cam<<      
     4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,-1.198459927713e-02,
    -7.210626507497e-03,  8.081198471645e-03, -9.999413164504e-01,-5.403984729748e-02, 
     9.999738645903e-01,  4.859485810390e-04, -7.206933692422e-03,-2.921968648686e-01,
      0,0,0,1;
      
  // downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  *source_cloud = *downsampled;
  
  std::cout<<"target size:"<<target_cloud->size()<<" source size:"<<source_cloud->size()<<std::endl;

  ros::Time::init();
  
  /*std::cout << "--- pcl::NDT ---" << std::endl;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt->setResolution(1.0);
  ndt->setInputTarget(target_cloud);
  ndt->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr matched1(new pcl::PointCloud<pcl::PointXYZ>());
  auto t1 = ros::WallTime::now();
  ndt->align(*matched1);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;*/
  Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();
  guess<<
    0.999996, -0.000780562,  -0.00256744,       1.3343,
 0.000791729,      0.99999,   0.00435116,   0.00705603,
  0.00256401,  -0.00435318,     0.999987,    0.0161349,
           0,          0,          0,          1;
	   
  std::cout << "--- pcl::NDT_OMP ---" << std::endl;
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setInputTarget(target_cloud);
  ndt_omp->setInputSource(source_cloud);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setStepSize(0.1);
  ndt_omp->setTransformationEpsilon(0.001);
  ndt_omp->setMaximumIterations(64);
  ndt_omp->setOulierRatio(0.30);
  pcl::PointCloud<pcl::PointXYZ>::Ptr matched(new pcl::PointCloud<pcl::PointXYZ>());
  auto t1 = ros::WallTime::now();
  ndt_omp->align(*matched,guess);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl; 
  //std::cout<<"Transform: \n"<<ndt_omp->getFinalTransformation()<<std::endl;
  Eigen::Matrix4d tf_s2s=ndt_omp->getFinalTransformation().cast<double>();
  Eigen::Matrix4d tf_s2s_cam=tf_velo2cam*tf_s2s*tf_velo2cam.inverse();
  std::cout<<"tf_s2s: \n"<<tf_s2s<<std::endl;
  std::cout<<"tf_s2s_cam: \n"<<tf_s2s_cam<<std::endl;
  Eigen::Matrix4d tf_s2s_gt;
  tf_s2s_gt<<
    0.999988, 0.00489403, -0.000211991,  0.0195194,
  -0.0048943, 0.999987 ,  -0.0013073 ,  -0.0139887,
  0.000205591, 0.00130833,  0.999999 ,     1.3645,
           0,          0,          0,          1;
  tf_s2s_gt=tf_velo2cam.inverse()*tf_s2s_gt*tf_velo2cam;
  pcl::PointCloud<pcl::PointXYZ>::Ptr matched_gt(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*source_cloud, *matched_gt,tf_s2s_gt );
  pcl::io::savePCDFileASCII ("matched_gt.pcd", *matched_gt);
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("matched.pcd", *matched);
  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> matched_handler(matched, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(matched, matched_handler, "matched");
  vis.spin();

  return 0;
}
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

#include <ros/ros.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/pcd_io.h>
#include <string>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC(ros::NodeHandle& private_nh) : private_nh(private_nh) {}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  double normal_estimation_radius = private_nh.param<double>("fpfh/normal_estimation_radius", 2.0);
  double search_radius = private_nh.param<double>("fpfh/search_radius", 8.0);

  ROS_INFO_STREAM("Normal Estimation: Radius(" << normal_estimation_radius << ")");
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);

  ROS_INFO_STREAM("FPFH Extraction: Search Radius(" << search_radius << ")");
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);

  // Check if FPFH already computed and stored in a file
  ROS_INFO_STREAM("Attempting to load FPFH Features");
  if (load_features_from_file(features)){
    return features;
  }

  ROS_INFO_STREAM("Computing and saving FPFH Features");
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(cloud);
  fest.setInputNormals(normals);
  fest.compute(*features);
  
  // save computed FPFH Points to a file
  save_features_to_file(features);
  
  return features;
}

std::string GlobalLocalizationEngineFPFH_RANSAC::get_feature_filepath(){
  std::string map_filepath;
  if (!(private_nh.getParam("/globalmap_server_nodelet/globalmap_pcd", map_filepath))){
    ROS_ERROR_STREAM("Parameter /globalmap_server_nodelet/globalmap_pcd not found");
    return "";
  }
  std::string feature_extension = "_FPFH_features.pcd";
  size_t dotPosition = map_filepath.find_last_of('.');
  map_filepath = map_filepath.substr(0, dotPosition);  // get rid of the .pcd
  return map_filepath + feature_extension; // add extension to end and return it
}

void GlobalLocalizationEngineFPFH_RANSAC::save_features_to_file(pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr features){
  std::string file_name = get_feature_filepath();
  // save it
  pcl::io::savePCDFileBinary(file_name, *features);
}

bool GlobalLocalizationEngineFPFH_RANSAC::load_features_from_file(pcl::PointCloud<pcl::FPFHSignature33>::Ptr features){
  std::string file_name = get_feature_filepath();
  if (pcl::io::loadPCDFile(file_name, *features) == -1) {
    // failed to lead FPFH features from file
    return false;
  }
  return true;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  global_map = cloud;
  global_map_features = extract_fpfh(cloud);

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(private_nh));
  ransac->set_target(global_map, global_map_features);
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr cloud_features = extract_fpfh(cloud);

  ransac->set_source(cloud, cloud_features);
  auto results = ransac->estimate();

  return results.sort(max_num_candidates);
}

}  // namespace hdl_global_localization
// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/filesystem.hpp>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "cluster3d.cpp"
#include "ransac3d.cpp"

namespace lidar_obstacle_detection {

struct Color {
  float r, g, b;

  Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};

enum CameraAngle { XY, TopDown, Side, FPS };

struct Box {
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

// shorthand for point cloud pointer
template <typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

template <typename PointT>

class ProcessPointClouds {
 public:
  // constructor
  ProcessPointClouds();

  // deconstructor
  ~ProcessPointClouds();

  void numPoints(PtCdtr<PointT> cloud);

  PtCdtr<PointT> FilterCloud(PtCdtr<PointT> cloud, float filterRes,
                             Eigen::Vector4f minPoint,
                             Eigen::Vector4f maxPoint);

  std::pair<PtCdtr<PointT>, PtCdtr<PointT>> SeparateClouds(
      pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud);

  std::pair<PtCdtr<PointT>, PtCdtr<PointT>> RansacSegmentPlane(
      PtCdtr<PointT> cloud, int maxIterations, float distanceTol);

  std::pair<PtCdtr<PointT>, PtCdtr<PointT>> SegmentPlane(PtCdtr<PointT> cloud,
                                                         int maxIterations,
                                                         float distanceTol);

  std::vector<PtCdtr<PointT>> EuclideanClustering(PtCdtr<PointT> cloud,
                                                  float clusterTolerance,
                                                  int minSize, int maxSize);

  std::vector<PtCdtr<PointT>> Clustering(PtCdtr<PointT> cloud,
                                         float clusterTolerance, int minSize,
                                         int maxSize);

  Box BoundingBox(PtCdtr<PointT> cluster);

  void savePcd(PtCdtr<PointT> cloud, std::string file);

  PtCdtr<PointT> loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
}  // namespace lidar_obstacle_detection
#endif /* PROCESSPOINTCLOUDS_H_ */
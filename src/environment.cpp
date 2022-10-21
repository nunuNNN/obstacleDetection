
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


#include <pcl/visualization/pcl_visualizer.h>


using namespace lidar_obstacle_detection;

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::string name, Color color) {
  if (color.r == -1) {
    // Select color based off of cloud intensity
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
  } else {
    // Select color based off input value
    viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer->setPointCloudRenderingProperties(   // 设置点云的颜色、大小
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        name);
  }

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id,
               Color color = Color(1,0,0), float opacity=1) {
  if (opacity > 1.0) opacity = 1.0;
  if (opacity < 0.0) opacity = 0.0;

  std::string cube = "box" + std::to_string(id);

  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min,
                  box.z_max, color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

  std::string cubeFill = "boxFill" + std::to_string(id);

  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min,
                  box.z_max, color.r, color.g, color.b, cubeFill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, cubeFill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // FilterCloud
  float filterRes = 0.4;
  Eigen::Vector4f minpoint(-1, -6.5, -1.3, 1);
  Eigen::Vector4f maxpoint(30, 6.5, 2, 1);
  // SegmentPlane
  int maxIterations = 40;
  float distanceThreshold = 0.3;
  // Clustering
  float clusterTolerance = 0.5;
  int minsize = 10;
  int maxsize = 140;

  // First:Filter cloud to reduce amount of points
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
      pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint, maxpoint);

  // Second: Segment the filtered cloud into obstacles and road
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmentCloud = pointProcessorI->RansacSegmentPlane(
          filteredCloud, maxIterations, distanceThreshold);
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  // Third: Cluster different obstacle cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->EuclideanClustering(filteredCloud, clusterTolerance,
                                           minsize, maxsize);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    // Fourth: Find bounding boxes for each obstacle cluster
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;
  viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
  viewer->addCoordinateSystem(1.0);

  //  Stream cityBlock function
  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream =
      pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
    streamIterator++;
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }
    viewer->spinOnce();
  }
}

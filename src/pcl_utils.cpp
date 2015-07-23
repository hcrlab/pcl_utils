#include "picard/pcl_utils.h"
#include "picard/common.h"

// pcl stuff
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

namespace picard
{

void downsample(const PointCloudPtr &input_cloud, PointCloud output_cloud, float leaf_size /*=0.01f*/) {
  pcl::VoxelGrid<Point> vg;
  vg.setInputCloud (input_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(output_cloud);
}

bool extract(const PointCloudConstPtr &input_cloud, const pcl::PointIndices::ConstPtr &indices,
             PointCloud &output_cloud, bool negative) {
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud (input_cloud);
  extract.setIndices (indices);
  extract.setNegative (negative);
  extract.filter(output_cloud);
}

bool extract(const PointCloudConstPtr &input_cloud, const pcl::PointIndices &indices,
             PointCloud &output_cloud) {
    for (std::vector<int>::const_iterator pit = indices.indices.begin (); pit != indices.indices.end (); ++pit)
      output_cloud.points.push_back ((*input_cloud)[*pit]);
}

bool getLargestPlaneComponent(const PointCloudPtr &input_cloud, const pcl::ModelCoefficients::Ptr &coeffs,
                              const pcl::PointIndices::Ptr &planar_cluster_inliers) {

  // initialize variables
  pcl::PointIndices::Ptr planar_inliers (new pcl::PointIndices);
  PointCloudPtr cloud_plane_noisy (new PointCloud);

  /*********** RANSAC ***********/
  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (input_cloud);
  seg.segment (*planar_inliers, *coeffs);

  if (planar_inliers->indices.size () == 0)
  {
    std::cout << "getLargestPlaneComponent could not estimate a planar model for the given dataset." << std::endl;
    return false;
  }

  // get the inlying points associated with the planar surface
  extract(input_cloud, planar_inliers, *cloud_plane_noisy, false);
  std::cout << "PointCloud representing the noisy planar cloud had: " << cloud_plane_noisy->points.size () << " data points." << std::endl;

  // Euclidean Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  euclideanCluster(cloud_plane_noisy, cluster_indices);

  if (cluster_indices.size() == 0) {
    std::cout << "getLargestPlaneComponent could not get the connected components of the noisy planar cloud";
    return false;
  }

  // take only the largest euclidean cluster within the planar points
  for (std::vector<int>::const_iterator it = cluster_indices[0].indices.begin(); it != cluster_indices[0].indices.end(); ++it) {
      planar_cluster_inliers->indices.push_back(planar_inliers->indices[*it]);
  }

}

void getPointsAbove2dHull(const PointCloudConstPtr &plane_outliers, const PointCloud &cloud_hull, const PointCloudPtr &points_above_plane) {
  // Throw away anything not in the 2D XY bounding box of or not above the planar points.
  // first compute bounding box of plane points
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D (cloud_hull, min_pt, max_pt);
  std::cout << "min_pt: " << min_pt << "\n";
  std::cout << "max_pt: " << max_pt << "\n";
  // then keep only things above the plane
  max_pt[2] = FLT_MAX;
  pcl::CropBox<Point> cropbox;
  cropbox.setInputCloud(plane_outliers);
  cropbox.setMin(min_pt);
  cropbox.setMax(max_pt);
  cropbox.filter(*points_above_plane);

// // POLYGONAL PRISM METHOD
// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
// pcl::ConvexHull<pcl::PointXYZRGBA> chull;
// chull.setInputCloud (projected_cloud);
// chull.reconstruct (*cloud_hull);
// cloud_hull->push_back(cloud_hull->at(0)); // If this line is commented, the bug occurs
// // Extend the hull in both z directions on the original cloud to return the desired box.
// pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
// prism.setInputCloud(main_cloud);
// prism.setInputPlanarHull(cloud_hull);
// prism.setHeightLimits(-10, 10);
// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// prism.segment(*inliers);

}

void euclideanCluster(const PointCloudConstPtr &input_cloud, std::vector<pcl::PointIndices> &cluster_indices) {
  std::cout << "euclideanCluster() called..." << std::endl;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  tree->setInputCloud (input_cloud);
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  //ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract(cluster_indices);
  std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;
}

void interpretTableScene(const PointCloudConstPtr &input_cloud, const ModelCoefficientsPtr &coeffs, const PointCloudPtr &plane_points,
                                 const PointCloudPtr& cloud_hull, std::vector<PointCloudPtr> &object_clouds) {

  // remove NaNs
  PointCloudPtr cloud_filtered(new PointCloud);
  std::vector<int> indices;
  std::cout << "PointCloud before removing NaNs has: " << input_cloud->points.size ()  << " data points." << std::endl;
  pcl::removeNaNFromPointCloud(*input_cloud, *cloud_filtered, indices);
  std::cout << "PointCloud after removing NaNs has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  // fit the largest plane component (usually a tabletop)
  pcl::PointIndices::Ptr planar_cluster_inliers (new pcl::PointIndices);
  getLargestPlaneComponent(cloud_filtered, coeffs, planar_cluster_inliers);

  // get the inliers
  extract(cloud_filtered, planar_cluster_inliers, *plane_points, false);

  // get the outliers
  PointCloudPtr plane_outliers (new PointCloud);
  extract(cloud_filtered, planar_cluster_inliers, *plane_outliers, true);
  
  // project the inliers onto the plane
  PointCloudPtr plane_points_projected(new PointCloud);
  pcl::ProjectInliers<Point> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setIndices (planar_cluster_inliers);
  proj.setInputCloud (plane_points);
  proj.setModelCoefficients (coeffs);
  proj.filter (*plane_points_projected); 
  std::cerr << "PointCloud after projection has: "
            << plane_points_projected->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  // pcl::ConcaveHull<Point> chull;
  // chull.setInputCloud (cloud_projected);
  // chull.setAlpha (0.1);
  // chull.reconstruct (*cloud_hull);
  pcl::ConvexHull<Point> chull;
  chull.setInputCloud (plane_points_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  // get the points above the planer component
  PointCloudPtr points_above_plane(new PointCloud);
  getPointsAbove2dHull(plane_outliers, *cloud_hull, points_above_plane);

  // cluster the points above the planar component
  std::vector<pcl::PointIndices> cluster_indices;
  euclideanCluster(points_above_plane, cluster_indices);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    std::cout << "Object indices it->indices.size() = " << it->indices.size() << std::endl;
    PointCloudPtr cloud_cluster (new PointCloud);
    extract(points_above_plane, *it, *cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    object_clouds.push_back(cloud_cluster);
  }
  std::cout << "interpretTableScene() returning..." << std::endl;
}

} // namespace picard

// // Make a cloud dense
// cloud_cluster->width = cloud_cluster->points.size ();
// cloud_cluster->height = 1;
// cloud_cluster->is_dense = true;

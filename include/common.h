//
// Created by Dan Butler on 6/19/15.
//

#ifndef PICARD_COMMON_H
#define PICARD_COMMON_H

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
class ModelCoefficients;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace ros
{
class Publisher;
}

namespace picard
{

class Object;
class RealObject;
class Plane;
class World;
class PlaneRenderer;
class ObjectRenderer;
class PicardManager;
class PicardPanel;
class PoseStamped;

typedef boost::shared_ptr<World> WorldPtr;
typedef boost::shared_ptr<Object> ObjectPtr;
typedef boost::shared_ptr<PicardPanel> PicardPanelPtr;
typedef boost::shared_ptr<PicardManager> PicardManagerPtr;
typedef boost::shared_ptr<ObjectRenderer> ObjectRendererPtr;
typedef boost::shared_ptr<Plane> PlanePtr;
typedef boost::shared_ptr<PlaneRenderer> PlaneRendererPtr;
typedef boost::shared_ptr<ros::Publisher> PublisherPtr;

typedef boost::unordered_map<int, ObjectPtr> IdToObjectPtrMap;
typedef boost::unordered_map<int, ObjectRendererPtr> IdToObjectRendererPtrMap;

typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

typedef pcl::ModelCoefficients ModelCoefficients;
typedef boost::shared_ptr<ModelCoefficients> ModelCoefficientsPtr;
typedef boost::shared_ptr<const ModelCoefficients> ModelCoefficientsConstPtr;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

}

#endif //PICARD_COMMON_H

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>

#include <pcl/filters/random_sample.h>


ros::Publisher pub;
int completed = 0;
int initMax = 100;

float tableHeight = 1.3f;
int rMax = 100;
int rMin = 0;
int gMax = 255;
int gMin = 100;
int bMax = 100;
int bMin = 0;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr limited(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZRGB>());

pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond_1(new pcl::ConditionAnd<pcl::PointXYZRGB>());
pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond_2(new pcl::ConditionAnd<pcl::PointXYZRGB>());
pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr colour_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());

pcl::RandomSample<pcl::PointXYZRGB> random_sample;

// Called on each point cloud message
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg) {
	pcl::fromROSMsg(*msg, *cloud);

	random_sample.setSample(2000);
	sensor_msgs::PointCloud2 *outMsg = new sensor_msgs::PointCloud2;

	std::vector<int> idx;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

	// Build the filter
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setInputCloud(cloud);

	if (completed < initMax) {
		// Preliminary scan
		completed++;
		condrem.setCondition(range_cond_1);
	} else if (completed == initMax) {
		// Publish 1 length cloud to say we are done (client should understand)
		completed++;
		random_sample.setSample(1);
		ros::Duration(0.1).sleep();
	} else {
		// Normal operation
		condrem.setCondition(range_cond_2);
		condrem.setCondition(colour_cond);
	}

	condrem.setKeepOrganized(false); // False takes out NaN, but now 1D vector
	// Apply filter
	condrem.filter(*limited);

	// Random sampling
	random_sample.setInputCloud(limited);
	random_sample.setSeed(rand());
	random_sample.filter(*sampled);

	sampled->is_dense=false;
	pcl::toROSMsg(*sampled, *outMsg);
	outMsg->is_dense=false;

	// Publish!
	pub.publish(*outMsg);
	ros::Duration(0.01).sleep(); // No idea
}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "kinect_sampler");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("/vr/cloud", 1);

	// Set up the conditions
	range_cond_1->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, tableHeight)));
	range_cond_2->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, tableHeight)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
	colour_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
		new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));


	// Spin
	std::cout << "Running: should be /vr/cloud" << std::endl;
	ros::spin();
}


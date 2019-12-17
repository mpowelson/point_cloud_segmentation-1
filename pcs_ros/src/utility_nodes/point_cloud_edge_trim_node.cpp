#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace pcs_ros
{
/** @brief Trims the edges of a structured point cloud by some number of pixels and republishes
 *
 */
class PointCloudEdgeTrim
{
public:
  PointCloudEdgeTrim(ros::NodeHandle& nh) : nh_(nh), input_topic_("input"), output_topic_("output")
  {
    // Create publishers and subscribers
    sub_ = nh_.subscribe(input_topic_, 1, &PointCloudEdgeTrim::callback, this);
    pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(output_topic_, 1);

    // Print the topics we are using
    std::string t1 = nh_.resolveName(input_topic_);
    std::string t2 = nh_.resolveName(output_topic_);
    ROS_INFO_STREAM("Subscribing XYZRGB pointcloud on: " << t1);
    ROS_INFO_STREAM("Publishing filtered XYZRGB pointcloud on: " << t2);

    nh_.param<int>("top_border", top_border_, 10);
    nh_.param<int>("bottom_border", bottom_border_, 10);
    nh_.param<int>("left_border", left_border_, 10);
    nh_.param<int>("right_border", right_border_, 10);

    ROS_INFO_STREAM("Trimming Pointcloud to have borders of " << top_border_ << " , " << bottom_border_ << " , "
                                                              << left_border_ << " , " << right_border_ << std::endl);
  }

  void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr results(new pcl::PointCloud<pcl::PointXYZRGB>());

    results->header = input_cloud->header;
    results->width = input_cloud->width - left_border_ - right_border_;
    results->height = input_cloud->height - top_border_ - bottom_border_;
    results->is_dense = input_cloud->is_dense;
    results->points.resize(input_cloud->points.size() - (left_border_ + right_border_) * (top_border_ + bottom_border_));

    std::size_t id_new = 0;
    for(std::size_t idy = top_border_; idy < input_cloud->height - bottom_border_; idy++)
    {
      for(std::size_t idx = left_border_; idx < input_cloud->width - right_border_; idx++)
      {
        results->points[id_new] = input_cloud->points[idy * input_cloud->width + idx];
        id_new++;
      }
    }

    pub_.publish(results);
  }

private:
  ros::NodeHandle nh_;
  std::string input_topic_;
  std::string output_topic_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  int top_border_;
  int bottom_border_;
  int left_border_;
  int right_border_;
};
}  // namespace pcs_ros
int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_edge_trim_node");
  ros::NodeHandle nh("~");
  pcs_ros::PointCloudEdgeTrim filter(nh);
  ros::spin();
  return 0;
}

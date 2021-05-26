/******/
#pragma once

#include <deque>
#include <map>
#include <mutex>

// ROS includes
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace pointcloud_preprocessor
{
/** \brief @b PointCloudConcatenateFieldsSynchronizer is a special form of data
 * synchronizer: it listens for a set of input PointCloud messages on the same topic,
 * checks their timestamps, and concatenates their fields together into a single
 * PointCloud output message.
 * \author @wangkang
 */
class PointCloudConcatenateDataSynchronizerNodelet : public nodelet_topic_tools::NodeletLazy
{
public:
// 三种点云方式
  typedef sensor_msgs::PointCloud2 PointCloud2;
  typedef PointCloud2::Ptr PointCloud2Ptr;
  typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

  /** \brief Empty constructor. */
  PointCloudConcatenateDataSynchronizerNodelet() : maximum_queue_size_(3), timeout_sec_(0.1){};

  /** \brief Empty constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudConcatenateDataSynchronizerNodelet(int queue_size)
  : maximum_queue_size_(queue_size), timeout_sec_(0.1){};

  /** \brief Empty destructor. */
  virtual ~PointCloudConcatenateDataSynchronizerNodelet(){};
// 功能函数
  void onInit();
  void subscribe();
  void unsubscribe();

private:
  /** \brief The output PointCloud publisher. */
  ros::Publisher pub_output_;

  ros::Publisher pub_concat_num_;
  ros::Publisher pub_not_subscribed_topic_name_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_;

  double timeout_sec_;

  /** \brief A vector of subscriber.点云话题订阅数 */
  std::vector<boost::shared_ptr<ros::Subscriber> > filters_;
/**
 * @brief 订阅运动信息去除畸变
 * 
 */
  ros::Subscriber sub_twist_;

  ros::Timer timer_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;

  /** \brief Input point cloud topics. */
  XmlRpc::XmlRpcValue input_topics_;

  /** \brief TF listener object. */
  tf::TransformListener tf_;
// 优先级队列去畸变
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_ptr_queue_;

  std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_;
  std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_tmp_;
  std::mutex mutex_;
// tf转换
  void transformPointCloud(const PointCloud2::ConstPtr & in, PointCloud2::Ptr & out);
  void combineClouds(
    const PointCloud2::ConstPtr & in1, const PointCloud2::ConstPtr & in2, PointCloud2::Ptr & out);
  void publish();

  void convertToXYZCloud(
    const sensor_msgs::PointCloud2 & input_cloud,
    sensor_msgs::PointCloud2 & output_cloud);
  void cloud_callback(
    const sensor_msgs::PointCloud2::ConstPtr & input_ptr, const std::string & topic_name);
  void twist_callback(const geometry_msgs::TwistStamped::ConstPtr & input);
  void timer_callback(const ros::TimerEvent &);
};
}  // namespace pointcloud_preprocessor

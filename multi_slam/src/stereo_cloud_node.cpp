#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  // doTransform(PointCloud2)
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <limits>
#include <vector>

class StereoCloudNode : public rclcpp::Node
{
public:
  StereoCloudNode()
  : Node("stereo_cloud_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ------------ Params ------------
    left_topic_  = this->declare_parameter<std::string>("left_topic",  "/cam_1/image_raw");
    right_topic_ = this->declare_parameter<std::string>("right_topic", "/cam_0/image_raw");

    left_frame_  = this->declare_parameter<std::string>("left_frame",  "camera_link"); // frame_id in Image headers
    map_frame_   = this->declare_parameter<std::string>("map_frame",   "map");

    publish_in_map_ = this->declare_parameter<bool>("publish_in_map", true);

    // Use your YAML values (defaults set from your config)
    fx_ = this->declare_parameter<double>("fx", 458.654);
    fy_ = this->declare_parameter<double>("fy", 457.296);
    cx_ = this->declare_parameter<double>("cx", 367.215);
    cy_ = this->declare_parameter<double>("cy", 248.375);
    baseline_m_ = this->declare_parameter<double>("baseline_m", 0.1100741378);

    // Rectification toggle (HIGHLY recommended if images are not already rectified)
    do_rectify_ = this->declare_parameter<bool>("do_rectify", true);

    // Distortion (left)
    k1_ = this->declare_parameter<double>("k1", -0.28340811);
    k2_ = this->declare_parameter<double>("k2",  0.07395907);
    p1_ = this->declare_parameter<double>("p1",  0.00019359);
    p2_ = this->declare_parameter<double>("p2",  1.76187114e-05);
    k3_ = this->declare_parameter<double>("k3", 0.0);

    width_  = this->declare_parameter<int>("width", 752);
    height_ = this->declare_parameter<int>("height", 480);

    // Extrinsics T_c1_c2 (4x4 row-major) from your YAML
    // default = your matrix values
    Tc1c2_ = this->declare_parameter<std::vector<double>>(
      "Tc1c2",
      {
        0.999997256477797,-0.002317135723275,-0.000343393120620, 0.110074137800478,
        0.002312067192432, 0.999898048507103,-0.014090668452683,-0.000156612054392,
        0.000376008102320, 0.014089835846691, 0.999900662638081, 0.000889382785432,
        0,0,0,1
      });

    // StereoSGBM params (tuneable)
    num_disparities_ = this->declare_parameter<int>("num_disparities", 128); // multiple of 16
    block_size_      = this->declare_parameter<int>("block_size", 7);       // odd
    sgbm_p1_         = this->declare_parameter<int>("P1", 8 * 1 * block_size_ * block_size_);
    sgbm_p2_         = this->declare_parameter<int>("P2", 32 * 1 * block_size_ * block_size_);
    disp12_maxdiff_  = this->declare_parameter<int>("disp12_maxdiff", 1);
    prefilter_cap_   = this->declare_parameter<int>("prefilter_cap", 31);
    uniqueness_      = this->declare_parameter<int>("uniqueness", 10);
    speckle_window_  = this->declare_parameter<int>("speckle_window", 100);
    speckle_range_   = this->declare_parameter<int>("speckle_range", 2);

    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 60.0);
    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.2);

    // ------------ Publishers ------------
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/px4_1/stereo_cloud", rclcpp::SensorDataQoS());

    // ------------ Subscribers (Approx sync) ------------
    left_sub_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, right_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Sync>(SyncPolicy(20), *left_sub_, *right_sub_);
    sync_->registerCallback(&StereoCloudNode::stereoCb, this);

    // ------------ OpenCV Stereo matcher ------------
    sgbm_ = cv::StereoSGBM::create(
      0, num_disparities_, block_size_,
      sgbm_p1_, sgbm_p2_,
      disp12_maxdiff_,
      prefilter_cap_,
      uniqueness_,
      speckle_window_,
      speckle_range_,
      cv::StereoSGBM::MODE_SGBM_3WAY
    );

    // Precompute rectification maps if enabled
    if (do_rectify_) {
      initRectification();
    }

    RCLCPP_INFO(get_logger(), "stereo_cloud_node ready. L=%s R=%s rectify=%s",
                left_topic_.c_str(), right_topic_.c_str(), do_rectify_ ? "true" : "false");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void initRectification()
  {
    if (Tc1c2_.size() != 16) {
      RCLCPP_WARN(get_logger(), "Tc1c2 size != 16. Disabling rectification.");
      do_rectify_ = false;
      return;
    }

    cv::Mat K1 = (cv::Mat_<double>(3,3) << fx_, 0, cx_,  0, fy_, cy_,  0, 0, 1);
    cv::Mat D1 = (cv::Mat_<double>(1,5) << k1_, k2_, p1_, p2_, k3_);

    // NOTE: You provided distinct intrinsics for cam2; this node uses cam1 intrinsics for projection.
    // For rectification, we still need K2/D2. If you want, expose K2/D2 as params too.
    // Here we assume cam2 intrinsics ≈ cam1 (ok-ish for sim). You can extend later.
    cv::Mat K2 = K1.clone();
    cv::Mat D2 = D1.clone();

    cv::Mat T = (cv::Mat_<double>(3,1) << Tc1c2_[3], Tc1c2_[7], Tc1c2_[11]);

    cv::Mat R = (cv::Mat_<double>(3,3) <<
      Tc1c2_[0], Tc1c2_[1], Tc1c2_[2],
      Tc1c2_[4], Tc1c2_[5], Tc1c2_[6],
      Tc1c2_[8], Tc1c2_[9], Tc1c2_[10]);

    cv::Size img_size(width_, height_);
    cv::Mat R1, R2, P1, P2, Q;

    cv::stereoRectify(
      K1, D1, K2, D2, img_size,
      R, T, R1, R2, P1, P2, Q,
      cv::CALIB_ZERO_DISPARITY, 0, img_size
    );

    cv::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_32FC1, map1x_, map1y_);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_32FC1, map2x_, map2y_);

    // If we rectified, better to use rectified fx from P1(0,0)
    fx_rect_ = P1.at<double>(0,0);
    cx_rect_ = P1.at<double>(0,2);
    fy_rect_ = P1.at<double>(1,1);
    cy_rect_ = P1.at<double>(1,2);

    RCLCPP_INFO(get_logger(), "Rectification maps computed. fx_rect=%.3f baseline=%.6f",
                fx_rect_, baseline_m_);
  }

  void stereoCb(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
  {
    // Convert to cv::Mat
    cv_bridge::CvImageConstPtr left_cv, right_cv;
    try {
      left_cv  = cv_bridge::toCvShare(left_msg);
      right_cv = cv_bridge::toCvShare(right_msg);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat L = left_cv->image;
    cv::Mat R = right_cv->image;

    // Convert to grayscale for SGBM
    cv::Mat Lg, Rg;
    if (L.channels() == 3) cv::cvtColor(L, Lg, cv::COLOR_RGB2GRAY);
    else Lg = L;
    if (R.channels() == 3) cv::cvtColor(R, Rg, cv::COLOR_RGB2GRAY);
    else Rg = R;

    // Rectify if enabled
    if (do_rectify_) {
      cv::Mat Lr, Rr;
      cv::remap(Lg, Lr, map1x_, map1y_, cv::INTER_LINEAR);
      cv::remap(Rg, Rr, map2x_, map2y_, cv::INTER_LINEAR);
      Lg = Lr;
      Rg = Rr;
    }

    // Compute disparity (SGBM gives fixed-point disparity: disp*16)
    cv::Mat disp16;
    sgbm_->compute(Lg, Rg, disp16);

    // Convert to float disparity in pixels
    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

    // Build point cloud in LEFT optical frame, then (optionally) transform to map
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = left_msg->header;
    cloud.header.frame_id = left_msg->header.frame_id.empty() ? left_frame_ : left_msg->header.frame_id;

    // We publish unordered cloud (1 x N)
    std::vector<float> xyz;
    xyz.reserve(width_ * height_ * 3);

    const double fx_use = do_rectify_ ? fx_rect_ : fx_;
    const double fy_use = do_rectify_ ? fy_rect_ : fy_;
    const double cx_use = do_rectify_ ? cx_rect_ : cx_;
    const double cy_use = do_rectify_ ? cy_rect_ : cy_;

    for (int v = 0; v < disp.rows; ++v) {
      const float *drow = disp.ptr<float>(v);
      for (int u = 0; u < disp.cols; ++u) {
        const float d = drow[u];
        if (d <= 1.0f || !std::isfinite(d)) continue; // avoid near-zero/invalid

        const double Z = (fx_use * baseline_m_) / static_cast<double>(d);
        if (Z < min_depth_m_ || Z > max_depth_m_) continue;

        const double X = (static_cast<double>(u) - cx_use) * Z / fx_use;
        const double Y = (static_cast<double>(v) - cy_use) * Z / fy_use;

        xyz.push_back(static_cast<float>(X));
        xyz.push_back(static_cast<float>(Y));
        xyz.push_back(static_cast<float>(Z));
      }
    }

    fillPointCloud2(cloud, xyz);

    // Transform to map frame if requested
    if (publish_in_map_) {
      try {
        // Transform from cloud.frame to map at the cloud timestamp
        geometry_msgs::msg::TransformStamped tf =
          tf_buffer_.lookupTransform(map_frame_, cloud.header.frame_id, cloud.header.stamp, tf2::durationFromSec(0.05));

        sensor_msgs::msg::PointCloud2 cloud_map;
        tf2::doTransform(cloud, cloud_map, tf);
        cloud_map.header.frame_id = map_frame_;
        cloud_pub_->publish(cloud_map);
      } catch (const std::exception &e) {
        // If TF not available yet, publish in camera frame
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF to map failed (%s). Publishing in %s",
                             e.what(), cloud.header.frame_id.c_str());
        cloud_pub_->publish(cloud);
      }
    } else {
      cloud_pub_->publish(cloud);
    }
  }

  static void fillPointCloud2(sensor_msgs::msg::PointCloud2 &cloud, const std::vector<float> &xyz)
  {
    const uint32_t n_pts = static_cast<uint32_t>(xyz.size() / 3);

    cloud.height = 1;
    cloud.width  = n_pts;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    cloud.fields.resize(3);
    cloud.fields[0].name = "x"; cloud.fields[0].offset = 0; cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[0].count = 1;
    cloud.fields[1].name = "y"; cloud.fields[1].offset = 4; cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[1].count = 1;
    cloud.fields[2].name = "z"; cloud.fields[2].offset = 8; cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[2].count = 1;

    cloud.point_step = 12;
    cloud.row_step   = cloud.point_step * cloud.width;

    cloud.data.resize(static_cast<size_t>(cloud.row_step));
    std::memcpy(cloud.data.data(), xyz.data(), cloud.data.size());
  }

private:
  // Topics/frames
  std::string left_topic_, right_topic_;
  std::string left_frame_, map_frame_;
  bool publish_in_map_{true};

  // Camera params
  double fx_, fy_, cx_, cy_;
  double baseline_m_;
  int width_, height_;

  // Distortion
  double k1_, k2_, p1_, p2_, k3_;

  // Extrinsics
  std::vector<double> Tc1c2_;

  // Rectification
  bool do_rectify_{true};
  cv::Mat map1x_, map1y_, map2x_, map2y_;
  double fx_rect_{0}, fy_rect_{0}, cx_rect_{0}, cy_rect_{0};

  // Depth filtering
  double max_depth_m_{60.0}, min_depth_m_{0.2};

  // StereoSGBM params
  int num_disparities_{128}, block_size_{7};
  int sgbm_p1_{0}, sgbm_p2_{0};
  int disp12_maxdiff_{1}, prefilter_cap_{31}, uniqueness_{10}, speckle_window_{100}, speckle_range_{2};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_, right_sub_;
  std::shared_ptr<Sync> sync_;

  // OpenCV matcher
  cv::Ptr<cv::StereoSGBM> sgbm_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCloudNode>());
  rclcpp::shutdown();
  return 0;
}

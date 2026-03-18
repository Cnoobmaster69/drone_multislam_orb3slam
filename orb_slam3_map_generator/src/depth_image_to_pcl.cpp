#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <rmw/qos_profiles.h>

// For message_filters Synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std::chrono_literals;

// This code takes in a coloured image and an rgb image. Outputs the coloured pointcloud using a message filter sync.
// Uses the rgb camera intrinsics
class ColoredPointCloudNode : public rclcpp::Node
{
public:
    ColoredPointCloudNode()
        : Node("colored_pointcloud_node"), rgb_info_received_(false), depth_info_received_(false)
    {
        this->declare_parameter<std::string>("rgb_image_topic", "rgb_camera");
        this->declare_parameter<std::string>("depth_image_topic", "depth_camera");
        this->declare_parameter<std::string>("rgb_info_topic", "rgb_camera/camera_info");
        this->declare_parameter<std::string>("depth_info_topic", "camera_info");
        this->declare_parameter<std::string>("output_pointcloud_topic", "camera/colored_pointcloud");
        this->declare_parameter<std::string>("output_frame_id", "camera_link");
        this->declare_parameter<double>("sync_max_interval_sec", 1.0);
        this->declare_parameter<bool>("output_reliable_qos", true);
        
        // Get parameter values.
        std::string rgb_image_topic;
        std::string depth_image_topic;
        std::string rgb_info_topic;
        std::string depth_info_topic;
        std::string output_pointcloud_topic;
        std::string output_frame_id;
        double sync_max_interval_sec = 1.0;
        bool output_reliable_qos = true;
        this->get_parameter("rgb_image_topic", rgb_image_topic);
        this->get_parameter("depth_image_topic", depth_image_topic);
        this->get_parameter("rgb_info_topic", rgb_info_topic);
        this->get_parameter("depth_info_topic", depth_info_topic);
        this->get_parameter("output_pointcloud_topic", output_pointcloud_topic);
        this->get_parameter("output_frame_id", output_frame_id);
        this->get_parameter("sync_max_interval_sec", sync_max_interval_sec);
        this->get_parameter("output_reliable_qos", output_reliable_qos);
        output_frame_id_ = resolveFrameId(output_frame_id);

        RCLCPP_INFO_STREAM(this->get_logger(), "RGB image topic: " << rgb_image_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "Depth image topic: " << depth_image_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "RGB info topic: " << rgb_info_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "Depth info topic: " << depth_info_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "Output pointcloud topic: " << output_pointcloud_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "Output frame id: " << output_frame_id_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Output pointcloud QoS reliability: "
                                               << (output_reliable_qos ? "reliable" : "best_effort"));
        RCLCPP_INFO(this->get_logger(), "=================================");

        // Set up subscribers using message_filters with sensor-data QoS.
        // This avoids reliability mismatches with Gazebo/bridge image publishers.
        rgb_image_sub_.subscribe(this, rgb_image_topic, rmw_qos_profile_sensor_data);
        depth_image_sub_.subscribe(this, depth_image_topic, rmw_qos_profile_sensor_data);

        rgb_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            rgb_info_topic, 10,
            std::bind(&ColoredPointCloudNode::rgbInfoCallback, this, std::placeholders::_1));

        depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            depth_info_topic, 10,
            std::bind(&ColoredPointCloudNode::depthInfoCallback, this, std::placeholders::_1));

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>
            SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(50), rgb_image_sub_, depth_image_sub_);
        if (sync_max_interval_sec > 0.0)
        {
            sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_interval_sec));
        }

        // Register the synchronized callback
        sync_->registerCallback(
            std::bind(&ColoredPointCloudNode::callbackSync, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Publisher for PointCloud2
        auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5));
        if (output_reliable_qos)
        {
            cloud_qos.reliable();
        }
        else
        {
            cloud_qos.best_effort();
        }
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_pointcloud_topic, cloud_qos);
    }

private:
    void rgbInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        if (!rgb_info_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Received RGB CameraInfo once. Storing intrinsics.");
            rgb_k_ = info_msg->k; // store the 3x3 intrinsic matrix in a std::array<double, 9>
            rgb_info_received_ = true;

            // If you are sure you only need this info once, unsubscribe
            rgb_info_sub_.reset();
        }
    }

    void depthInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        if (!depth_info_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Received Depth CameraInfo once. Storing intrinsics.");
            depth_k_ = info_msg->k;
            depth_info_received_ = true;

            // If you are sure you only need this info once, unsubscribe
            depth_info_sub_.reset();
        }
    }

    void callbackSync(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
    {
        if (!rgb_info_received_ || !depth_info_received_)
            return;
        if (!received_synced_pair_) {
            received_synced_pair_ = true;
            RCLCPP_INFO(this->get_logger(), "First synchronized RGB+Depth pair received.");
        }
        // Convert ROS image messages to OpenCV images
        cv::Mat rgb_image, depth_image;
        try
        {
            rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
            // Depth image typically is TYPE_16UC1 or TYPE_32FC1
            depth_image = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // For simplicity, assume depth and RGB share same intrinsics
        double fx, fy, cx, cy;
        // fx = depth_k_[0];
        // fy = depth_k_[4];
        // cx = depth_k_[2];
        // cy = depth_k_[5];

        fx = rgb_k_[0];
        fy = rgb_k_[4];
        cx = rgb_k_[2];
        cy = rgb_k_[5];

        // Prepare PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = rgb_msg->header;
        cloud_msg.header.frame_id = output_frame_id_;
        cloud_msg.height = depth_image.rows;
        cloud_msg.width = depth_image.cols;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cloud_msg.height * cloud_msg.width);

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_rgb(cloud_msg, "rgb");

        // Example depth_scale if your depth is in mm (e.g., 16UC1)
        double depth_scale = 0.001;

        for (int v = 0; v < depth_image.rows; ++v)
        {
            for (int u = 0; u < depth_image.cols; ++u)
            {
                float Z = 0.0f;
                if (depth_image.type() == CV_16UC1)
                {
                    uint16_t d = depth_image.at<uint16_t>(v, u);
                    if (d == 0)
                    {
                        Z = std::numeric_limits<float>::quiet_NaN();
                    }
                    else
                    {
                        Z = static_cast<float>(d) * depth_scale;
                    }
                }
                else if (depth_image.type() == CV_32FC1)
                {
                    float d = depth_image.at<float>(v, u);
                    if (std::isnan(d) || d <= 0.0f)
                    {
                        Z = std::numeric_limits<float>::quiet_NaN();
                    }
                    else
                    {
                        Z = d;
                    }
                }
                else
                {
                    RCLCPP_WARN_ONCE(this->get_logger(),
                                     "Unsupported depth type. Please add handling code.");
                    Z = std::numeric_limits<float>::quiet_NaN();
                }

                float X = std::numeric_limits<float>::quiet_NaN();
                float Y = std::numeric_limits<float>::quiet_NaN();
                if (!std::isnan(Z))
                {
                    X = (static_cast<float>(u) - cx) * Z / fx;
                    Y = (static_cast<float>(v) - cy) * Z / fy;
                }

                *out_x = X;
                ++out_x;
                *out_y = Y;
                ++out_y;
                *out_z = Z;
                ++out_z;

                // Get color
                cv::Vec3b bgr_color = rgb_image.at<cv::Vec3b>(v, u);
                uint8_t b = bgr_color[0];
                uint8_t g = bgr_color[1];
                uint8_t r = bgr_color[2];

                // Pack color into a float
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                                static_cast<uint32_t>(g) << 8 |
                                static_cast<uint32_t>(b));
                memcpy(&(*out_rgb), &rgb, sizeof(uint32_t));
                ++out_rgb;
            }
        }

        pointcloud_pub_->publish(cloud_msg);
        if (published_cloud_count_ == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "First colored pointcloud published on %s",
                        pointcloud_pub_->get_topic_name());
        }
        ++published_cloud_count_;
    }

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>>>
        sync_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    // --- Subscriptions for Camera Info (only to store intrinsics once) ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

    bool rgb_info_received_;
    bool depth_info_received_;

    // Store camera intrinsics from the callback
    std::array<double, 9> rgb_k_;
    std::array<double, 9> depth_k_;
    std::string output_frame_id_;
    bool received_synced_pair_ = false;
    uint64_t published_cloud_count_ = 0;

    std::string resolveFrameId(const std::string &frame_id_in)
    {
        if (frame_id_in.empty()) return frame_id_in;
        if (frame_id_in.find('/') != std::string::npos) return frame_id_in;

        std::string ns = this->get_namespace(); // e.g. "/uav_1"
        ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
        if (ns.empty()) return frame_id_in;
        return ns + "/" + frame_id_in;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColoredPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}

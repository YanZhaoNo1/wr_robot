#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <cmath>
#include <chrono>
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace std::chrono;

typedef boost::circular_buffer<sensor_msgs::msg::Imu> ImuCircularBuffer;

class LidarMotionCalibrator : public rclcpp::Node
{
public:
    LidarMotionCalibrator() : Node("lidar_undistortion_node")
    {
        // 从参数服务器读取参数
        this->declare_parameter<std::string>("lidar_topic","/scan");
        this->declare_parameter<std::string>("imu_topic","/imu");
        this->declare_parameter<double>("imu_frequency",100.0);
        this->declare_parameter<double>("lidar_msg_delay_time",10.0);

        this->declare_parameter<std::string>("output_frame_id","laser_after");
        this->declare_parameter<bool>("pub_raw_scan_pointcloud",false);
        this->declare_parameter<bool>("pub_laserscan",false);
        this->declare_parameter<double>("laserscan_angle_increment",0.01);

        this->declare_parameter<bool>("scan_direction_clockwise",false);

        this->declare_parameter<bool>("use_range_filter",false);
        this->declare_parameter<double>("range_filter_min",0.3);
        this->declare_parameter<double>("range_filter_max",12.0);

        this->declare_parameter<bool>("use_angle_filter",false);
        this->declare_parameter<double>("angle_filter_min",-2.5);
        this->declare_parameter<double>("angle_filter_max",2.5);

        this->declare_parameter<bool>("use_radius_outlier_filter",false);
        this->declare_parameter<double>("radius_outlier_filter_search_radius",0.1);
        this->declare_parameter<int>("radius_outlier_filter_min_neighbors",1);

        this->get_parameter("lidar_topic", lidar_topic_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("imu_frequency", imu_frequency_);
        this->get_parameter("lidar_msg_delay_time", lidar_msg_delay_time_);

        this->get_parameter("output_frame_id", output_frame_id_);
        this->get_parameter("pub_raw_scan_pointcloud", pub_raw_scan_pointcloud_);
        this->get_parameter("pub_laserscan", pub_laserscan_);
        this->get_parameter("laserscan_angle_increment", laserscan_angle_increment_);

        this->get_parameter("scan_direction_clockwise", scan_direction_clockwise_);

        this->get_parameter("use_range_filter", use_range_filter_);
        this->get_parameter("range_filter_min", range_filter_min_);
        this->get_parameter("range_filter_max", range_filter_max_);

        this->get_parameter("use_angle_filter", use_angle_filter_);
        this->get_parameter("angle_filter_min", angle_filter_min_);
        this->get_parameter("angle_filter_max", angle_filter_max_);

        this->get_parameter("use_radius_outlier_filter", use_radius_outlier_filter_);
        this->get_parameter("radius_outlier_filter_search_radius", radius_outlier_filter_search_radius_);
        this->get_parameter("radius_outlier_filter_min_neighbors", radius_outlier_filter_min_neighbors_);

        lidar_sub_= this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_, 10, std::bind(&LidarMotionCalibrator::LidarCallback, this, std::placeholders::_1));
        imu_sub_= this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 100, std::bind(&LidarMotionCalibrator::ImuCallback, this, std::placeholders::_1));
        
        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_undistortion/after", 10);
        pcl_pub_origin_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_undistortion/origin", 10);
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/lidar_undistortion/scan", 10);
         
        delay_duration = rclcpp::Duration::from_seconds(lidar_msg_delay_time_);

        imuCircularBuffer_ = ImuCircularBuffer((int)(imu_frequency_ * 1.5));    
        // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_origin_;
        // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        imuCircularBuffer_.push_front(*imu_msg);
    }

    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr _lidar_msg)
    {
        // 激光点的个数
        int length = _lidar_msg->ranges.size();

        rclcpp::Duration frame_duration = rclcpp::Duration::from_seconds(_lidar_msg->scan_time);
        // frame_duration = rclcpp::Duration::from_seconds(_lidar_msg->scan_time);

        // 估计第一个激光点的时刻
        rclcpp::Time lidar_timebase = this->get_clock()->now() - frame_duration - delay_duration;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_raw_pcl;

        // 将LaserScan类型的数据转化为PCL点云格式
        LaserScanToPointCloud(_lidar_msg, pointcloud_raw_pcl);
        current_laserscan_msg_ = _lidar_msg;

        // 确保IMU缓冲区中有足够多的数据
      if (imuCircularBuffer_.size() > frame_duration.seconds() * imu_frequency_ * 1.5)
      {
        auto start = system_clock::now();

        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        sensor_msgs::msg::PointCloud2 pointcloud_raw_msg;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
        pcl::PointXYZI point_xyzi;

        current_output_timestamp = imuCircularBuffer_[0].header.stamp;
        Eigen::Quaternionf current_quat = ImuToCurrentQuaternion(imuCircularBuffer_[0].orientation);

        for (int i = 0; i < length; i++)
        {
          pcl::PointXYZI current_point = pointcloud_raw_pcl[i];

          Eigen::Vector3f point_in(current_point.x, current_point.y, 0.0);

          rclcpp::Time point_timestamp = lidar_timebase + rclcpp::Duration::from_seconds(_lidar_msg->time_increment * i);

          Eigen::Quaternionf point_quat;

          // 如果成功获取当前扫描点的姿态
          if (getLaserPose(i, point_timestamp, point_quat) == true)
          {
            Eigen::Quaternionf delta_quat = current_quat.inverse() * point_quat;

            Eigen::Vector3f point_out = delta_quat * point_in;

            point_xyzi.x = point_out(0);
            point_xyzi.y = point_out(1);
            point_xyzi.z = 0.0;
            point_xyzi.intensity = current_point.intensity;
            pointcloud_pcl.push_back(point_xyzi);
          }
          else
          {
            break;
          }
        }

        ApplyRangeFilter(pointcloud_pcl);
        ApplyAngleFilter(pointcloud_pcl);
        ApplyRadiusOutlierFilter(pointcloud_pcl);

        pcl::toROSMsg(pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = output_frame_id_;
        pcl_pub_->publish(pointcloud_msg);

        if (pub_raw_scan_pointcloud_ == true)
        {
          pcl::toROSMsg(pointcloud_raw_pcl, pointcloud_raw_msg);
          pointcloud_raw_msg.header.frame_id = output_frame_id_;
          pcl_pub_origin_->publish(pointcloud_raw_msg);
        }

        if (pub_laserscan_ == true)
        {
          PublishLaserscan(pointcloud_pcl);
        }

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        // cout <<  "Spend "  << double(duration.count()) / 1000.0 << " miliseconds" << endl;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "IMU data less than 0.2s, waiting for next packet.");
      }
    }

    bool getLaserPose(int point_index, rclcpp::Time &_timestamp, Eigen::Quaternionf &quat_out)
    {
      static int index_front = 0;
      static int index_back = 0;
      static rclcpp::Time timestamp_front;
      static rclcpp::Time timestamp_back;
      static Eigen::Quaternionf quat_front, quat_back;

      static bool index_updated_flag = false;
      static bool predict_orientation_flag = false;

      if (point_index == 0)
      {
        predict_orientation_flag = false;
        int i = 0;
        while (_timestamp < imuCircularBuffer_[i].header.stamp)
        {
          i++;
        }
        index_front = i - 1;
        index_back = i;
        index_updated_flag = true;
      }
      else
      {
        while (predict_orientation_flag == false
               && _timestamp > imuCircularBuffer_[index_front].header.stamp
               && _timestamp > imuCircularBuffer_[index_back].header.stamp)
        {
          index_front--;
          index_back--;

          if (index_front < 0)
          {
            //use prediction
            predict_orientation_flag = true;
            index_front++;
            index_back++;
          }

          index_updated_flag = true;
        }
      }

      if (index_updated_flag == true)
      {
        //cout << "index updated: " << index_front << " " << index_back << endl;
        timestamp_front = imuCircularBuffer_[index_front].header.stamp;
        timestamp_back = imuCircularBuffer_[index_back].header.stamp;
        quat_front = Eigen::Quaternionf(imuCircularBuffer_[index_front].orientation.w, imuCircularBuffer_[index_front].orientation.x, imuCircularBuffer_[index_front].orientation.y, imuCircularBuffer_[index_front].orientation.z);
        quat_back = Eigen::Quaternionf(imuCircularBuffer_[index_back].orientation.w, imuCircularBuffer_[index_back].orientation.x, imuCircularBuffer_[index_back].orientation.y, imuCircularBuffer_[index_back].orientation.z);

        index_updated_flag = false;
      }

      float alpha = (_timestamp.nanoseconds() - timestamp_back.nanoseconds()) / 
              float(timestamp_front.nanoseconds() - timestamp_back.nanoseconds());

      if (alpha < 0)
      {
        return false;
      }

      // 球面线性插值
      // Slerp.
      quat_out = quat_back.slerp(alpha, quat_front);

      return true;
    }

    void LaserScanToPointCloud(sensor_msgs::msg::LaserScan::ConstSharedPtr _laser_scan, pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
    {
      _pointcloud.clear();
      pcl::PointXYZI newPoint;
      newPoint.z = 0.0;
      double newPointAngle;

      int beamNum = _laser_scan->ranges.size();

      if (scan_direction_clockwise_ == true)
      {
        for (int i = beamNum - 1; i >= 0; i--)
        {
          newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
          newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
          newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
          newPoint.intensity = _laser_scan->intensities[i];
          _pointcloud.push_back(newPoint);
        }
      }
      else
      {
        for (int i = 0; i < beamNum; i++)
        {
          newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
          newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
          newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
          newPoint.intensity = _laser_scan->intensities[i];
          _pointcloud.push_back(newPoint);
        }
      }
    }

    Eigen::Quaternionf ImuToCurrentQuaternion(geometry_msgs::msg::Quaternion _quat)
    {
      tf2::Quaternion current_tf_quat(_quat.x, _quat.y, _quat.z, _quat.w);

      //提取航向角
      double roll, pitch, yaw;
      tf2::Matrix3x3(current_tf_quat).getRPY(roll, pitch, yaw);

      //建立当前航向的水平坐标系四元数
      geometry_msgs::msg::Quaternion horizontal_quat;
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0.0, 0.0, yaw);
      horizontal_quat = tf2::toMsg(tf2_quat);
      return Eigen::Quaternionf(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
    }

    void PublishLaserscan(pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
    {
      float angle_min, angle_max;
      if (use_angle_filter_ == true)
      {
        angle_min = angle_filter_min_;
        angle_max = angle_filter_max_;
      }
      else
      {
        angle_min = current_laserscan_msg_->angle_min;
        angle_max = current_laserscan_msg_->angle_max;
      }

      float range_min, range_max;
      if (use_range_filter_ == true)
      {
        range_min = range_filter_min_;
        range_max = range_filter_max_;
      }
      else
      {
        range_min = current_laserscan_msg_->range_min;
        range_max = current_laserscan_msg_->range_max;
      }

      unsigned int beam_size = ceil((angle_max - angle_min) / laserscan_angle_increment_);

      sensor_msgs::msg::LaserScan output;
      output.header.stamp = current_output_timestamp;
      output.header.frame_id = output_frame_id_;
      output.angle_min = angle_min;
      output.angle_max = angle_max;
      output.range_min = range_min;
      output.range_max = range_max;
      output.angle_increment = laserscan_angle_increment_;
      output.time_increment = 0.0;
      output.scan_time = 0.0;
      output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
      output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

      for (auto point : _pointcloud.points)
      {
        float range = hypot(point.x, point.y);
        float angle = atan2(point.y, point.x);
        int index = (int)((angle - output.angle_min) / output.angle_increment);
        if (index >= 0 && static_cast<unsigned int>(index) < beam_size)
        {
          if (isnan(output.ranges[index]))
          {
            output.ranges[index] = range;
          }
          else
          {
            if (range < output.ranges[index])
            {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = point.intensity;
        }
      }
      laserscan_pub_->publish(output);
    }

    void ApplyRangeFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_range_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (size_t i = 0; i < _input.size(); i++)
        {
          float range = hypot(_input[i].x, _input[i].y);
          if (range > range_filter_min_ && range < range_filter_max_)
          {
            cloud.push_back(_input[i]);
          }
        }
        _input.swap(cloud);
      }
    }

    void ApplyAngleFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_angle_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (std::size_t i = 0; i < _input.size(); i++)
        {
          float angle = atan2(_input[i].y, _input[i].x);
          if (angle > angle_filter_min_ && angle < angle_filter_max_)
          {
            cloud.push_back(_input[i]);
          }
        }
        _input.swap(cloud);
      }
    }

    void ApplyRadiusOutlierFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_radius_outlier_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(_input.makeShared());
        sor.setRadiusSearch(radius_outlier_filter_search_radius_);
        sor.setMinNeighborsInRadius(radius_outlier_filter_min_neighbors_);
        sor.setNegative(false);
        sor.filter(cloud);
        _input.swap(cloud);
      }
    }

public:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_origin_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;

    rclcpp::Duration delay_duration = rclcpp::Duration::from_seconds(lidar_msg_delay_time_);

    rclcpp::Time current_output_timestamp;

    ImuCircularBuffer imuCircularBuffer_;

    string lidar_topic_, imu_topic_, output_frame_id_;
    double imu_frequency_, lidar_msg_delay_time_;
    bool pub_raw_scan_pointcloud_;

    bool pub_laserscan_;
    double laserscan_angle_increment_;

    bool use_range_filter_;
    double range_filter_min_;
    double range_filter_max_;
    bool scan_direction_clockwise_;

    bool use_angle_filter_;
    double angle_filter_min_;
    double angle_filter_max_;

    bool use_radius_outlier_filter_;
    double radius_outlier_filter_search_radius_;
    int radius_outlier_filter_min_neighbors_;

    sensor_msgs::msg::LaserScan::ConstSharedPtr current_laserscan_msg_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LidarMotionCalibrator>());
    rclcpp::shutdown();
    return 0;
}

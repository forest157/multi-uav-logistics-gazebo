// Project-owned Gazebo Classic ray publisher. MIT; see package.xml.
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

namespace gazebo {
class LogisticsLidarPlugin : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    sensor_=std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
    if (!sensor_ || !ros::isInitialized()) {
      gzerr << "logistics lidar requires a ray sensor and gazebo_ros_api_plugin\n";
      return;
    }
    const auto ns=sdf->Get<std::string>("robotNamespace");
    frame_=sdf->Get<std::string>("frameName");
    node_.reset(new ros::NodeHandle(ns));
    publisher_=node_->advertise<sensor_msgs::PointCloud>(sdf->Get<std::string>("topicName"),1);
    // MultiRayShape's new-scan event fires BEFORE RaySensor stamps/caches its
    // data. Sensor::ConnectUpdated fires AFTER it. Do not stamp with ROS now.
    connection_=sensor_->ConnectUpdated([this]() { Publish(); });
    sensor_->SetActive(true);
  }

  ~LogisticsLidarPlugin() override { connection_.reset(); }

 private:
  void Publish() {
    if (!publisher_.getNumSubscribers()) return;
    const auto stamp=sensor_->LastMeasurementTime();
    const int width=sensor_->RangeCount(),height=sensor_->VerticalRangeCount();
    std::vector<double> ranges;
    sensor_->Ranges(ranges);
    if (width<1 || height<1 || ranges.size()!=static_cast<size_t>(width*height)) return;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp=ros::Time(stamp.sec,stamp.nsec);cloud.header.frame_id=frame_;
    cloud.points.reserve(ranges.size());
    const double yaw0=sensor_->AngleMin().Radian(),pitch0=sensor_->VerticalAngleMin().Radian();
    const double yaw_step=width>1?(sensor_->AngleMax().Radian()-yaw0)/(width-1):0.;
    const double pitch_step=height>1?(sensor_->VerticalAngleMax().Radian()-pitch0)/(height-1):0.;
    for (int j=0;j<height;++j) for (int i=0;i<width;++i) {
      const double r=ranges[j*width+i];
      if (!std::isfinite(r) || r<sensor_->RangeMin() || r>=sensor_->RangeMax()-.001) continue;
      // Exact sample angle, not the midpoint between this ray and its neighbor.
      const double yaw=yaw0+i*yaw_step,pitch=pitch0+j*pitch_step;
      geometry_msgs::Point32 point;
      point.x=r*std::cos(pitch)*std::cos(yaw);
      point.y=r*std::cos(pitch)*std::sin(yaw);
      point.z=r*std::sin(pitch);
      cloud.points.push_back(point);
    }
    publisher_.publish(cloud);
  }
  sensors::RaySensorPtr sensor_;
  event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher publisher_;
  std::string frame_;
};
GZ_REGISTER_SENSOR_PLUGIN(LogisticsLidarPlugin)
}

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#ifndef RGBD_SENSOR_DEMUX_HPP
#define RGBD_SENSOR_DEMUX_HPP

#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
#include <ros/subscriber.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <zed_interfaces/msg/rgbd_sensors.hpp>

namespace zed_nodelets
{
class RgbdSensorsDemuxNodelet : public nodelet::Nodelet
{
public:
  RgbdSensorsDemuxNodelet();
  virtual ~RgbdSensorsDemuxNodelet();

protected:
  /*! \brief Initialization function called by the Nodelet base class
   */
  virtual void onInit();

  /*! \brief Callback for full topics synchronization
   */
  void msgCallback(const zed_interfaces::msg::RGBDSensorsPtr& msg);

private:
  // Node handlers
  rclcpp::Node mNh;   // Node handler
  rclcpp::Node mNhP;  // Private Node handler

  // Publishers
  image_transport::CameraPublisher mPubRgb;
  image_transport::CameraPublisher mPubDepth;
  ros::Publisher mPubIMU;
  ros::Publisher mPubMag;

  // Subscribers
  ros::Subscriber mSub;
};

}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::RgbdSensorsDemuxNodelet, nodelet::Nodelet)

#endif  // RGBD_SENSOR_DEMUX_HPP

//
//  QRCodeReader.h
//
//
//  Created by Xun Wang on 30/07/15.
//  Copyright (c) 2015 Xun Wang. All rights reserved.
//

#ifndef QRCODE_READER_H
#define QRCODE_READER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <zbar.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pyride_common_msgs/msg/node_status.hpp>

using namespace std;

namespace qrcode_reader {

class QRCodeReader : public rclcpp::Node
{
public:
  QRCodeReader();
  virtual ~QRCodeReader();

  void init();
  void fini();

private:
  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;
  image_transport::Subscriber imgSub_;

  rclcpp::Publisher<pyride_common_msgs::msg::NodeStatus>::SharedPtr status_pub_;

  bool showResult_;

  std::mutex mutex_;

  rclcpp::TimerBase::SharedPtr qrDetectTimer_;

  sensor_msgs::msg::Image::ConstSharedPtr imgMsgPtr_;

  std::string cameraDevice_;

  zbar::ImageScanner zbarScanner_;

  void processingRawImages( const sensor_msgs::msg::Image::ConstSharedPtr& msg );

  void startDetection();
  void stopDetection();

  void doDetection();
};

} // namespace qrcode_reader

#endif /* defined(QRCODE_READER_H) */
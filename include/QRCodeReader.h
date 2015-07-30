//
//  QRCodeReader.h
//  pr2_perception
//
//  Created by Xun Wang on 30/07/15.
//  Copyright (c) 2015 Xun Wang. All rights reserved.
//

#ifndef __pr2_perception__QRCodeReader__
#define __pr2_perception__QRCodeReader__

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>

#include <zbar.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace ros;

namespace qrcode_reader {

class QRCodeReader
{
public:
  QRCodeReader();
  virtual ~QRCodeReader();
  
  void init();
  void fini();

  void continueProcessing();

private:
  NodeHandle priImgNode_;
  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;
  image_transport::Subscriber imgSub_;
  
  Publisher outputPub_;

  bool doDetection_;
  bool showResult_;

  boost::mutex mutex_;
  
  boost::thread * qr_detect_thread_;
  
  sensor_msgs::ImageConstPtr imgMsgPtr_;

  std::string cameraDevice_;

  CallbackQueue imgQueue_;
  
  AsyncSpinner * procThread_;
  zbar::ImageScanner zbarScanner_;
  
  void processingRawImages( const sensor_msgs::ImageConstPtr& msg );

  void startDetection();
  void stopDetection();

  void doDetection();
};
  
} // namespace qrcode_reader

#endif /* defined(__pr2_perception__QRCodeReader__) */

//
//  QRCodeReader.cpp
//  pr2_perception
//
//  Created by Xun Wang on 14/05/13.
//  Copyright (c) 2013 Xun Wang. All rights reserved.
//

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include "pyride_pr2/NodeStatus.h"
#include "QRCodeReader.h"

namespace qrcode_reader {

using namespace std;
using namespace cv;
  
static const int kPublishFreq = 1;
static const string kDefaultDevice = "/wide_stereo/right/image_rect_color";

QRCodeReader::QRCodeReader() :
  imgTrans_( priImgNode_ ),
  procThread_( NULL ),
  doDetection_( false ),
  showResult_( false )
{
  priImgNode_.setCallbackQueue( &imgQueue_ );
  zbarScanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRCodeReader::~QRCodeReader()
{
}

void QRCodeReader::init()
{
  NodeHandle priNh( "~" );
  
  priNh.param<std::string>( "camera", cameraDevice_, kDefaultDevice );
  priNh.param( "debug_img", showResult_, false );
  
  procThread_ = new AsyncSpinner( 1, &imgQueue_ );
  procThread_->start();

  if (showResult_) {
    imgPub_ = imgTrans_.advertise( "/qrcode_reader/debug_view", 1 );
  }

  outputPub_ = priImgNode_.advertise<pyride_pr2::NodeStatus>( "/pyride_pr2/node_status", 1 );

  this->startDetection();
}

void QRCodeReader::fini()
{
  this->stopDetection();
  imgSub_.shutdown();

  if (procThread_) {
    delete procThread_;
    procThread_ = NULL;
  }
}

void QRCodeReader::continueProcessing()
{
  ros::spin();
}
  
void QRCodeReader::doDetection()
{
  ros::Rate publish_rate( kPublishFreq );
  ros::Time ts;
  cv_bridge::CvImagePtr cv_ptr;

  while (doDetection_) {
    {
      boost::mutex::scoped_lock lock( mutex_ );
      if (imgMsgPtr_.get()) {
        try {
          cv_ptr = cv_bridge::toCvCopy( imgMsgPtr_, sensor_msgs::image_encodings::MONO8 );
          ts = imgMsgPtr_->header.stamp;
        }
        catch (cv_bridge::Exception & e) {
          ROS_ERROR( "Unable to convert image message to mat." );
          imgMsgPtr_.reset();
          continue;
        }
        imgMsgPtr_.reset();
      }
    }
    zbar::Image zbar_image( cv_ptr->image.cols, cv_ptr->image.rows, "Y800",
        cv_ptr->image.data, cv_ptr->image.cols * cv_ptr->image.rows );
    zbarScanner_.scan( zbar_image );

    // iterate over all barcode readings from image
    zbar::SymbolSet symbols = zbarScanner_.get_results();
    if (symbols.get_size() > 0) {
      std::stringstream ss;

      for (zbar::SymbolIterator symbol = symbols.symbol_begin();
           symbol != symbols.symbol_end(); ++symbol)
      {
        std::string barcode = symbol->get_data();
        // verify if repeated barcode throttling is enabled
        ROS_INFO( "got bar code %s.", barcode.c_str() );
        ss << barcode << ";";
      }
      pyride_pr2::NodeStatus msg;
      msg.header.stamp = ros::Time::now();
      msg.priority = 2;
      msg.for_console = false;
      msg.node_id = "qrcode_reader";
      msg.status_text = ss.str();
      outputPub_.publish( msg );
    }
    publish_rate.sleep();
  }
}
  
void QRCodeReader::processingRawImages( const sensor_msgs::ImageConstPtr& msg )
{
  // assume we cannot control the framerate (i.e. default 30FPS)
  boost::mutex::scoped_lock lock( mutex_ );

  imgMsgPtr_ = msg;
}

/*
bool QRCodeReader::enableHDTService( pr2ht::DetectTrackControl::Request &req,
                      pr2ht::DetectTrackControl::Response &res )
{
  if (req.tostart) {
    this->startDetection();
  }
  else {
    this->stopDetection();
  }
  res.ret = true;
  return true;
}
*/

void QRCodeReader::startDetection()
{
  if (doDetection_) {
    ROS_INFO( "QR code reader service has already started!" );
    return;
  }

  doDetection_ = true;
  
  imgSub_ = imgTrans_.subscribe( cameraDevice_, 1,
                                  &QRCodeReader::processingRawImages, this );

  qr_detect_thread_ = new boost::thread( &QRCodeReader::doDetection, this );

  ROS_INFO( "Starting QR code reader service." );
}

void QRCodeReader::stopDetection()
{
  if (!doDetection_)
    return;

  doDetection_ = false;
 
  if (qr_detect_thread_) {
    qr_detect_thread_->join();
    delete qr_detect_thread_;
    qr_detect_thread_ = NULL;
  }

  imgSub_.shutdown();

  ROS_INFO( "Stopping QR code reader service." );
}

} // namespace qrcode_reader
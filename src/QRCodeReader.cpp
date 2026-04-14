//
//  QRCodeReader.cpp
//  pr2_perception
//
//  Created by Xun Wang on 14/05/13.
//  Copyright (c) 2013 Xun Wang. All rights reserved.
//

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include <pyride_common_msgs/msg/node_status.hpp>
#include "QRCodeReader.h"

namespace qrcode_reader {

using namespace std;
using namespace cv;

static const string kDefaultDevice = "/wide_stereo/right/image_rect_color";
static const int kDetectionRateMs = 100;

QRCodeReader::QRCodeReader()
  : Node( "qrcode_reader" ),
    imgTrans_( shared_from_this() ),
    showResult_( false ),
    debugSubscriberCount_( 0 )
{
  zbarScanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  rclcpp::on_shutdown( [this]() { this->fini(); } );
}

QRCodeReader::~QRCodeReader()
{
}

void QRCodeReader::init()
{
  this->declare_parameter<std::string>( "camera", kDefaultDevice );
  this->declare_parameter<bool>( "debug_img", false );

  this->get_parameter( "camera", cameraDevice_ );
  this->get_parameter( "debug_img", showResult_ );

  if (showResult_) {
    imgPub_ = imgTrans_.advertise( "/qrcode_reader/debug_view", 1 );
  }

  rclcpp::PublisherOptions statusPub_options;
  statusPub_options.event_callbacks.matched_callback =
    [this]( const rmw_matched_status_t & status ) {
      statusSubscriberCount_.store( status.current_count );
      debugSubscriberCount_.store( imgPub_.getNumSubscribers() );
      RCLCPP_INFO( this->get_logger(), "Subscriber event! Status: %zu, Debug: %zu",
                   status.current_count, imgPub_.getNumSubscribers() );
      this->updateDetectionState();
    };

  status_pub_ = this->create_publisher<pyride_common_msgs::msg::NodeStatus>( "/pyride/node_status", 1, statusPub_options );
}

void QRCodeReader::fini()
{
  this->stopDetection();
  imgSub_.shutdown();
}

void QRCodeReader::doDetection()
{
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::msg::Image::ConstSharedPtr localImgMsg;

  {
    std::unique_lock<std::mutex> lock( mutex_ );
    if (!imgMsgPtr_) {
      return;
    }
    localImgMsg = imgMsgPtr_;
    try {
      cv_ptr = cv_bridge::toCvCopy( imgMsgPtr_, "mono8" );
    }
    catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR( this->get_logger(), "Unable to convert image message to mat." );
      imgMsgPtr_.reset();
      return;
    }
    imgMsgPtr_.reset();
  }
  zbar::Image zbar_image( cv_ptr->image.cols, cv_ptr->image.rows, "Y800",
      cv_ptr->image.data, cv_ptr->image.cols * cv_ptr->image.rows );
  zbarScanner_.scan( zbar_image );

  zbar::SymbolSet symbols = zbarScanner_.get_results();
  if (symbols.get_size() > 0) {
    std::stringstream ss;

    for (zbar::SymbolIterator symbol = symbols.symbol_begin();
         symbol != symbols.symbol_end(); ++symbol)
    {
      std::string barcode = symbol->get_data();
      RCLCPP_INFO( this->get_logger(), "got bar code %s.", barcode.c_str() );
      ss << barcode << ";";
    }
    pyride_common_msgs::msg::NodeStatus msg;
    msg.header.stamp = this->now();
    msg.priority = 2;
    msg.for_console = false;
    msg.node_id = "qrcode_reader";
    msg.status_text = ss.str();
    status_pub_->publish( msg );

    if (imgPub_.getNumSubscribers() > 0) {
      cv_bridge::CvImagePtr annotated_ptr;
      try {
        annotated_ptr = cv_bridge::toCvCopy( localImgMsg, "bgr8" );
      }
      catch (cv_bridge::Exception & e) {
        return;
      }
      std::string text = ss.str();
      int baseline = 0;
      cv::Size textSize = cv::getTextSize( text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline );
      cv::Point textOrg( (annotated_ptr->image.cols - textSize.width) / 2, textSize.height + 10 );
      cv::putText( annotated_ptr->image, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                   cv::Scalar( 0, 255, 0 ), 2 );
      imgPub_.publish( annotated_ptr->toImageMsg() );
    }
  }
}

void QRCodeReader::processingRawImages( const sensor_msgs::msg::Image::ConstSharedPtr& msg )
{
  std::unique_lock<std::mutex> lock( mutex_ );

  imgMsgPtr_ = msg;
}

void QRCodeReader::startDetection()
{
  std::unique_lock<std::mutex> lock( mutex_ );
  if (qrDetectTimer_) {
    return;
  }

  imgSub_ = imgTrans_.subscribe( cameraDevice_, 1,
                                  &QRCodeReader::processingRawImages, this );

  qrDetectTimer_ = this->create_wall_timer(
      std::chrono::milliseconds( kDetectionRateMs ),
      [this]() { this->doDetection(); } );

  RCLCPP_INFO( this->get_logger(), "Starting QR code detection." );
}

void QRCodeReader::stopDetection()
{
  std::unique_lock<std::mutex> lock( mutex_ );
  if (!qrDetectTimer_) {
    return;
  }

  qrDetectTimer_->cancel();
  qrDetectTimer_ = nullptr;

  imgSub_.shutdown();

  RCLCPP_INFO( this->get_logger(), "Stopping QR code detection." );
}

void QRCodeReader::updateDetectionState()
{
  size_t totalSubscribers = statusSubscriberCount_.load() + debugSubscriberCount_.load();
  if (totalSubscribers > 0) {
    this->startDetection();
  }
  else {
    this->stopDetection();
  }
}

} // namespace qrcode_reader
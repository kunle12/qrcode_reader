//
//  main.cpp
//  pr2_perception
//
//  Created by Xun Wang on 30/07/15
//  Copyright (c) 2015 Xun Wang. All rights reserved.
//

#include <rclcpp/rclcpp.hpp>
#include "QRCodeReader.h"

using namespace qrcode_reader;

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  
  auto qrReader = std::make_shared<QRCodeReader>();
  
  qrReader->init();
  
  rclcpp::spin( qrReader );
  
  qrReader->fini();
  
  rclcpp::shutdown();
  
  return 0;
}
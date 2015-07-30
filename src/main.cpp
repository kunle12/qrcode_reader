//
//  main.cpp
//  pr2_perception
//
//  Created by Xun Wang on 30/07/15
//  Copyright (c) 2015 Xun Wang. All rights reserved.
//

#include <ros/ros.h>
#include "QRCodeReader.h"

using namespace qrcode_reader;

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "qrcode_reader" );
  
  QRCodeReader qrReader;
  
  qrReader.init();
  
  qrReader.continueProcessing();
  
  qrReader.fini();

  return 0;
}

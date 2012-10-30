/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
 \file camera.cpp
 \brief 
 */


#include "viz.h"
#include "names.h"
#include <sstream>
#include "conversions/image.h"
#include "conversions/camera.h"
#include "conversions/3dpose.h"

#include <visp/vpImageIo.h>
#include <visp/vpVideoReader.h>
#include <visp/vpVideoWriter.h>
#include <visp/vpV4l2Grabber.h>
#include <iostream>
#include <vector>
#include <string>


#include <visp/vpDisplayX.h>
#include <boost/thread/thread.hpp>
#include "threading.h"
#include "events.h"
#include "logfilewriter.hpp"

namespace ar_visp
{
void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}
Viz::Viz() :
            n_("~"),
            spinner_(0),
            queue_size_(100),
            I(480,640),
            raw_image_subscriber_(n_, ar_visp::raw_image_topic, queue_size_),
            camera_info_subscriber_(n_, ar_visp::camera_info_topic, queue_size_),
            ar_marker_subscriber_(n_, ar_visp::ar_marker_topic, queue_size_),
            image_info_marker_sync_(raw_image_subscriber_, camera_info_subscriber_,ar_marker_subscriber_, queue_size_*10),
            iter_(0)
{

  spinner_.start();
  std::string config;
  std::string log;
  n_.param(ar_visp::tracker_config_param.c_str(), config, std::string("config.cfg"));
  n_.param(ar_visp::display_ar_tracker_param.c_str(), display_ar_tracker_, true);
  n_.param(ar_visp::display_mb_tracker_param.c_str(), display_mb_tracker_, true);
  n_.param(ar_visp::tracker_log_param.c_str(), log, std::string("/log/%08d.jpg"));


  cmd_line_ = new CmdLine(config);
  writer_.setFileName((cmd_line_->get_data_dir() + log).c_str());
  if(cmd_line_->using_var_file()){
    varfile_.open((cmd_line_->get_var_file()+std::string(".ar")).c_str(),std::ios::out);
    //ROS_INFO(cmd_line_->get_var_file().c_str());
  }
  writer_.open(logI);
  d = new vpDisplayX();
  d->init(I);
  image_info_marker_sync_.registerCallback(boost::bind(&Viz::frameCallback,this, _1, _2, _3));

  detectors::DetectorBase* detector = NULL;
  if (cmd_line_->get_detector_type() == CmdLine::ZBAR)
    detector = new detectors::qrcode::Detector;
  else if(cmd_line_->get_detector_type() == CmdLine::DTMX)
    detector = new detectors::datamatrix::Detector;
  t_ = new tracking::Tracker(*cmd_line_,detector,false);
  //t_->start();

  TrackerThread* tt = new TrackerThread(*t_);
  boost::thread* bt = new boost::thread(*tt);
  t_->process_event(tracking::select_input(I));

}

void Viz::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const ar_pose::ARMarkerConstPtr& marker){
  tracking::LogFileWriter writer(varfile_);
  cam_ = visp_bridge::toVispCameraParameters(*cam_info);

  this->I = visp_bridge::toVispImageRGBa(*image);
  vpDisplay::display(this->I);

  vpHomogeneousMatrix cMo;
  if(display_ar_tracker_){
    cMo = visp_bridge::toVispHomogeneousMatrix(marker->pose.pose);

    if(cmd_line_->using_var_file()){
      writer.write(iter_);
      for(int i=0;i<6*6;i++)
        writer.write(marker->pose.covariance[i]);

    }

  }

  if(display_mb_tracker_)
    t_->process_event(tracking::input_ready(this->I,cam_,iter_));

  if(display_ar_tracker_){
    vpHomogeneousMatrix cMo_err;//static error matrix
    cMo_err[0][0] = -0.44389436959795;
    cMo_err[0][1] = 0.84731901068995;
    cMo_err[0][2] = 0.29156179941363;
    cMo_err[0][3] = 0;//-0.13808424723959;

    cMo_err[1][0] = -0.018874688966252;
    cMo_err[1][1] = -0.33414412377255;
    cMo_err[1][2] = 0.94233298293487;
    cMo_err[1][3] = 0;//0.065815147353372;

    cMo_err[2][0] = 0.89588031279147;
    cMo_err[2][1] = 0.41279316708782;
    cMo_err[2][2] = 0.16431757791075;
    cMo_err[2][3] = 0;//0.27235909228323;

    cMo_err[3][0] = 0;
    cMo_err[3][1] = 0;
    cMo_err[3][2] = 0;
    cMo_err[3][3] = 1;

    if(display_mb_tracker_)
      vpDisplay::displayFrame(this->I,cMo/**cMo_err*/,cam_,.1,vpColor::gray,4);
    else
      vpDisplay::displayFrame(this->I,cMo,cam_,.1,vpColor::none, 2);
  }

  vpDisplay::flush(this->I);
  d->getImage(logI);
  writer_.saveFrame(logI);
  iter_++;
}

Viz::~Viz()
{
  delete cmd_line_;
  delete d;
  writer_.close();
  varfile_.close();
}
}

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

namespace ar_visp
{
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


  std::string config;
  std::string log;
  n_.param(ar_visp::tracker_config_param.c_str(), config, std::string("config.cfg"));
  n_.param(ar_visp::pose_file_param.c_str(), pose_file_name_, std::string("var.txt"));
  n_.param(ar_visp::tracker_log_param.c_str(), log, std::string("/log/%08d.jpg"));

  writer_.setFileName(log.c_str());
  ROS_INFO("%s",pose_file_name_.c_str());

  varfile_.open(pose_file_name_.c_str(),std::ios::out);

  writer_.open(logI);
  d = new vpDisplayX();
  d->init(I);
  image_info_marker_sync_.registerCallback(boost::bind(&Viz::frameCallback,this, _1, _2, _3));
  raw_image_subscriber_.registerCallback(boost::bind(&Viz::frameCallbackWithoutMarker,this, _1));

  ros::spin();
}
void Viz::frameCallbackWithoutMarker(const sensor_msgs::ImageConstPtr& image){
  art_potentially_untracked_.push_back(*image);
  iter_++;
}

void Viz::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const ar_pose::ARMarkerConstPtr& marker){

  for(std::list<sensor_msgs::Image>::iterator i=art_potentially_untracked_.begin();i!=art_potentially_untracked_.end();){
    this->I = visp_bridge::toVispImageRGBa(*i);
    vpDisplay::display(this->I);
    if(i->header.stamp.toNSec() < image->header.stamp.toNSec()){
      writer_.saveFrame(this->I);
      vpDisplay::flush(this->I);
      i=art_potentially_untracked_.erase(i);
      continue;
    }else if(i->header.stamp.toNSec() == image->header.stamp.toNSec()){
      vpHomogeneousMatrix cMo = visp_bridge::toVispHomogeneousMatrix(marker->pose.pose);
      cam_ = visp_bridge::toVispCameraParameters(*cam_info);
      vpDisplay::displayFrame(this->I,cMo,cam_,.1,vpColor::none, 2);
      vpDisplay::flush(this->I);
      d->getImage(logI);
      writer_.saveFrame(logI);
      vpPoseVector p(cMo);
      varfile_ << iter_ << "\t";
      for(unsigned int j=0;j<p.getRows();j++)
        varfile_ << p[j] << "\t";
      i=art_potentially_untracked_.erase(i);
      varfile_ << std::endl;
      continue;
    }else i++;
  }

}

Viz::~Viz()
{
  for(std::list<sensor_msgs::Image>::iterator i=art_potentially_untracked_.begin();i!=art_potentially_untracked_.end();i++){
    this->I = visp_bridge::toVispImageRGBa(*i);
    writer_.saveFrame(this->I);
  }
  delete d;
  writer_.close();
  varfile_.close();
}
}

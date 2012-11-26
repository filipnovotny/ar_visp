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
 \file viz.h
 \brief 
 */
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "ar_pose/ARMarker.h"
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpVideoWriter.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <fstream>
#include <list>

#ifndef __AR_VISP_VIZ_H__
#define __AR_VISP_VIZ_H__
namespace ar_visp
{
class Viz
{
private:
  ros::NodeHandle n_;
  ros::AsyncSpinner spinner_;
  unsigned int queue_size_;
  vpImage<vpRGBa> I,logI;
  std::list<sensor_msgs::Image > art_potentially_untracked_;
  vpVideoWriter writer_;
  unsigned int iter_;

  std::string pose_file_name_;

  message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber_;
  message_filters::Subscriber<ar_pose::ARMarker> ar_marker_subscriber_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, ar_pose::ARMarker> image_info_marker_sync_;
  vpDisplayX* d;

  vpCameraParameters cam_;
  std::ofstream varfile_;
  /*!
    \brief subscriber callback. Synchronizes ar marker and camera.

    Goes through list and separates untracked images from tracked ones.
   */
  void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const ar_pose::ARMarkerConstPtr& marker);

  /*!
      \brief subscriber callback. adds image to queue .

     */

  void frameCallbackWithoutMarker(const sensor_msgs::ImageConstPtr& image);

public:
  Viz();
  virtual ~Viz();
};
}
#endif /* CAMERA_H_ */

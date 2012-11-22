/****************************************************************************
 *
 * $Id: file.h 3496 2011-11-22 15:14:32Z fnovotny $
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
 * File containing names of topics or services used all accross the package
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file names.cpp
  \brief File containing names of topics or services used all accross the package
*/

#include "names.h"
#include "ros/ros.h"

namespace ar_visp
{
  std::string camera_prefix("");
  std::string pose_file_param("pose_file");
  std::string tracker_config_param("tracker_config");
  std::string tracker_log_param("tracker_log");
  std::string display_ar_tracker_param("display_ar_tracker");
  std::string display_mb_tracker_param("display_mb_tracker");
  std::string raw_image_topic(camera_prefix + "/image_raw");
  std::string camera_info_topic(camera_prefix + "/camera_info");
  std::string ar_marker_topic("/ar_pose_marker");

  void remap(){
    if (ros::names::remap("camera_prefix") != "camera_prefix") {
      camera_prefix = ros::names::remap("camera_prefix");
      //tracker_config = ros::names::remap("tracker_config");
      raw_image_topic = camera_prefix + "/image_raw";
      camera_info_topic = camera_prefix + "/camera_info";
    }
  }
}



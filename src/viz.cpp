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


namespace ar_visp
{
void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}
Viz::Viz() :
            spinner_(0),
            queue_size_(1),
            I(480,640),
            raw_image_subscriber_(n_, ar_visp::raw_image_topic, queue_size_),
            camera_info_subscriber_(n_, ar_visp::camera_info_topic, queue_size_),
            ar_marker_subscriber_(n_, ar_visp::ar_marker_topic, queue_size_),
            image_info_marker_sync_(raw_image_subscriber_, camera_info_subscriber_,ar_marker_subscriber_, queue_size_*10)


{

  d = new vpDisplayX();
  d->init(I);
  image_info_marker_sync_.registerCallback(boost::bind(&Viz::frameCallback,this, _1, _2, _3));
  spinner_.start();


}
void Viz::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const ar_pose::ARMarkerConstPtr& marker){
  cam_ = visp_bridge::toVispCameraParameters(*cam_info);
  this->I = visp_bridge::toVispImageRGBa(*image);
  vpHomogeneousMatrix cMo = visp_bridge::toVispHomogeneousMatrix(marker->pose.pose);

  vpDisplay::display(this->I);
  vpDisplay::displayFrame(this->I,cMo,cam_,.1,vpColor::none,2);
  vpDisplay::flush(this->I);
}

Viz::~Viz()
{
  // TODO Auto-generated destructor stub
}
}

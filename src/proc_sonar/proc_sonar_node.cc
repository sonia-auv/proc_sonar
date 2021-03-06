/**
 * \file	proc_sonar_node.cc
 * \author	Marc-Antoine Couture <coumarc9@outlook.com>
 * \date	2019-07-14
 *
 * \copyright Copyright (c) 2019 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_sonar_node.h"


namespace proc_sonar {

    ProcSonarNode::ProcSonarNode(ros::NodeHandlePtr &nh) :
        imgConverter(1500, 1000){

        sonarPointCloudSubscriber = nh->subscribe("/provider_sonar/point_cloud2", 200, &ProcSonarNode::ProviderSonarPointCloudCallback, this);

        imgPublisher = nh->advertise<sensor_msgs::Image>("/proc_sonar/img", 100);

    }


    void ProcSonarNode::Spin() {
        ros::Rate r(15); // 15 Hz

        while (ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProcSonarNode::ProviderSonarPointCloudCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud) {

        cv::Mat image = imgConverter.Convert(point_cloud);

        cv_bridge::CvImage ros_image;

        ros_image.image = image;
        ros_image.encoding = sensor_msgs::image_encodings::MONO8;

        imgPublisher.publish(ros_image.toImageMsg());


    }

}
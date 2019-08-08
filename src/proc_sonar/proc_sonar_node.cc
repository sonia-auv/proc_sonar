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

    ProcSonarNode::ProcSonarNode(ros::NodeHandlePtr &nh) {

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

        ROS_INFO("Callback");

        int mat_width = 1500;
        int mat_height = 1000;

        int mat_half_width = mat_width / 2;
        int mat_half_height = mat_height / 2;

        cv::Mat image(mat_height, mat_width, CV_8UC1);
        image.setTo(0);


        float min_x = 0, max_x = 0, min_y = 0, max_y = 0, max_intensity = 0;

        for (pcl::PointXYZI point : point_cloud->points) {
            if (min_x > point.x){
                min_x = point.x;
            }

            if (min_y > point.y){
                min_y = point.y;
            }

            if (max_x < point.x){
                max_x = point.x;
            }

            if (max_y < point.y){
                max_y = point.y;
            }

            if (max_intensity < point.intensity){
                max_intensity = point.intensity;
            }
        }

ROS_INFO_STREAM(point_cloud->points.size() << "");

        for (pcl::PointXYZI point : point_cloud->points) {
            int x = std::round(point.x / max_x * mat_width/2 + mat_half_width), y = std::round(mat_height - point.y / max_y * mat_height);
            float intensity = point.intensity;


            if(0 <= x && x < mat_width &&
               0 <= y && y < mat_height) {

                image.at<uchar>(y, x) = uchar(intensity/max_intensity * 255);
            } else {
                ROS_INFO("OUT");
            }

        }

        cv_bridge::CvImage ros_image;

        ros_image.image = image;
        ros_image.encoding = sensor_msgs::image_encodings::MONO8;

        imgPublisher.publish(ros_image.toImageMsg());


    }

}
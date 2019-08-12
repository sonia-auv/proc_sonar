/**
 * \file	PointCloudToImgConverter.cc
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

#include "PointCloudToImgConverter.h"

namespace proc_sonar {
    PointCloudToImgConverter::PointCloudToImgConverter(int mat_width, int mat_height)
      : mat_width(mat_width), mat_height(mat_height)
    {

    }

    cv::Mat PointCloudToImgConverter::Convert(const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
    {
        int mat_half_width = mat_width / 2;

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

        for (pcl::PointXYZI point : point_cloud->points) {
            int x = std::round(point.x / max_x * mat_width/2 + mat_half_width);
            int y = std::round(mat_height - point.y / max_y * mat_height);

            float intensity = point.intensity;


            if(0 <= x && x < mat_width &&
               0 <= y && y < mat_height) {

                image.at<uchar>(y, x) = uchar(intensity/max_intensity * 255);
            }

        }

        return image;
    }
}
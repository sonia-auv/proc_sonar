/**
 * \file	PointCloudToImgConverter.h
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

#ifndef PROC_SONAR_POINTCLOUDTOIMGCONVERTER_H
#define PROC_SONAR_POINTCLOUDTOIMGCONVERTER_H

#include <opencv/cv.h>
#include <pcl_ros/io/pcd_io.h>

namespace proc_sonar {
    class PointCloudToImgConverter {
    public:
        PointCloudToImgConverter(int mat_width, int mat_height);

        cv::Mat Convert(const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud);

    private:

        int mat_width;
        int mat_height;

    };
}


#endif //PROC_SONAR_POINTCLOUDTOIMGCONVERTER_H

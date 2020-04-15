/**
 * @file
 * @brief
 *
 * Publishes ~/
 */

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>

#include <pcl/conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>

#include <rosbag/bag.h>

#include <ouster/os1.h>
#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/os1_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <unique_id/unique_id.h>
#include <boost/uuid/random_generator.hpp>

#include <opencv2/opencv.hpp>

#include <cartographer_ros_msgs/LandmarkEntry.h>
#include <cartographer_ros_msgs/LandmarkList.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include "ouster_ros/point_os1.h"

#define foreach BOOST_FOREACH

namespace OS1 = ouster::OS1;
using namespace Eigen;

void parseToXYZ (pcl::PointCloud<ouster_ros::OS1::PointOS1> os1cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &xyzCloud)
{
    foreach (ouster_ros::OS1::PointOS1 pointOs1, os1cloud.points ) {
        pcl::PointXYZ pointXyz;
        pointXyz.x = pointOs1.x;
        pointXyz.y = pointOs1.y;
        pointXyz.z = pointOs1.z;
        xyzCloud->push_back(pointXyz);
    }
}

double areaSumOfAllChildrenOf(const std::vector<std::vector<cv::Point>> contours,const std::vector<cv::Vec4i> hierarchy, int i, cv::RotatedRect *rotRect) {
    double sumOfHoles = 0;
    int idx = i;

    cv::Vec4i currentChild = hierarchy[idx];

    if (currentChild[2] != -1) {
        idx = currentChild[2];
        currentChild = hierarchy[idx];

        sumOfHoles = sumOfHoles + contourArea(contours[idx]);
        if (contours[idx].size() > 5) {
            *rotRect = fitEllipse(contours[idx]);
        }
    }

    return sumOfHoles;
}

pcl::PointXYZ pointAtIndex(const cv::Point2f coords,const cv::Mat indexImage,const ouster_ros::OS1::CloudOS1 ptCloud) {
    size_t index = indexImage.at<int>((int)round(coords.y),(int)round(coords.x)); //due to rotation in prev func
    pcl::PointXYZ point(ptCloud.points[index].x,ptCloud.points[index].y,ptCloud.points[index].z);
    return point;
}

std::vector<cv::Point2f> ptCloudIndicesOfLandMarks (const cv::Mat image, const float binaryThreshhold, const cv::Mat indexImage, const ouster_ros::OS1::CloudOS1 ptCloud) {
    std::vector<cv::Point2f> indices;

    double min_im, max_im;

    cv::minMaxLoc(image, &min_im, &max_im);

    cv::Mat im_norm(image);
    if (min_im == max_im) {
        PCL_INFO("Cannot normalize intensity image...\n");
    } else {
        im_norm = (im_norm - min_im) / (max_im - min_im);


        cv::Mat threshIm;
        threshIm = image > .5171f;
        cv::Mat drawing(threshIm);

        cv::Mat dst = cv::Mat::zeros(64, 2048, CV_8UC3);
        rotate(im_norm, im_norm, cv::ROTATE_90_CLOCKWISE);
        //TODO is everything kosher
        cvtColor(drawing, drawing, CV_GRAY2RGB);

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        std::vector<std::vector<cv::Point> > contours_poly;
        std::vector<cv::Rect> boundRect;
        std::vector<cv::RotatedRect> outerEllipseRects;
        std::vector<cv::RotatedRect> innerEllipseRects;
        std::vector<int> areaIdx;

        findContours(threshIm, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr contourPtCloud (new pcl::PointCloud<pcl::PointXYZ> ());
        int idx = 0;
        for (size_t i = 0; i < contours.size(); i++) {

            double area = contourArea(contours[i]);
            if (area > 45 && area < 600  && contours[i].size() > 4) {     //to fit ellipse you need 6 points
                double subArea;

                cv::RotatedRect innerRotRect;
                subArea = areaSumOfAllChildrenOf(contours, hierarchy, i, &innerRotRect);

                for (size_t x = 0; x < contours[i].size(); x++) {
                    pcl::PointXYZ contourPoint;
                    contourPoint = pointAtIndex(contours[i][x],indexImage,ptCloud);
                    contourPtCloud->push_back(contourPoint);

                }
                pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr
                        modelCircle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ> (contourPtCloud));

                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (modelCircle3D);
                ransac.setDistanceThreshold (.01);
                ransac.computeModel();
                float circle_radius = ransac.model_coefficients_[3];
                contourPtCloud->clear();


                if (subArea > 5 && circle_radius >0.07 && circle_radius <0.08) {
                    innerEllipseRects.push_back(innerRotRect);
                    std::vector<cv::Point> polygon;
                    cv::Rect rect;

                    approxPolyDP(cv::Mat(contours[i]), polygon, 3, true);
                    contours_poly.push_back(polygon);

                    rect = boundingRect(cv::Mat(contours_poly[idx]));
                    boundRect.push_back(rect);

                    areaIdx.push_back(i);
                    cv::RotatedRect rotRect = fitEllipse(contours[i]);
                    outerEllipseRects.push_back(rotRect);

                    indices.push_back(rotRect.center);

                    idx = idx + 1;
                }
            }
        }
        for (int i = 0; i < areaIdx.size(); i++) {
            cv::Scalar color = cv::Scalar(0, 0, 255);
            rectangle(drawing, outerEllipseRects[i].boundingRect(), color, 2);
           //cv::Scalar color2 = cv::Scalar(0, 255, 0);
            //rectangle(drawing,innerEllipseRects[i].boundingRect(),color2,2);
        }

            if (areaIdx.size() >0) {
                namedWindow("Targets", cv::WINDOW_AUTOSIZE);
                imshow("Targets", drawing);
                cv::waitKey(0);
            }
    }

    return indices;
}


bool isDefinedPoint(pcl::PointXYZ point) {
    if (point.x == 0 && point.y == 0 && point.z == 0) {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    rosbag::Bag bag;
    std::cout << "Loading bag ... " << std::endl;
    bag.open("/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/debug/map_gen_0.bag");
    std::cout << "Finished loading bag." << std::endl;

    ouster_ros::OS1::CloudOS1 cloud{};
    auto H = OS1::pixels_per_column;
    auto W = OS1::n_cols_of_lidar_mode(OS1::lidar_mode::MODE_1024x10);
    const auto px_offset = ouster::OS1::get_px_offset(W);
    int ptCloudCounter = 0;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        sensor_msgs::PointCloud2::ConstPtr ptCloud = m.instantiate<sensor_msgs::PointCloud2>();
               if (ptCloud != nullptr) {
                   pcl::fromROSMsg(*ptCloud, cloud);
                   ptCloudCounter = ptCloudCounter + 1;

                   cv::Mat intensity_image = cv::Mat(H, W, CV_32F);
                   cv::Mat index_image = cv::Mat(H, W, CV_32SC1);

                   for (int u = 0; u < H; u++) {
                       for (int v = 0; v < W; v++) {
                           const size_t vv = (v + px_offset[u]) % W;
                           const size_t index = vv * H + u;
                           const auto& pt = cloud[index];
                           index_image.at<int>(u, v) = (int)  index;
                           intensity_image.at<float>(u, v) = pt.intensity;

                       }
                   }
                   if (ptCloudCounter> 1750) {
                       std::vector<cv::Point2f> intensityImageIndices;
                       intensityImageIndices = ptCloudIndicesOfLandMarks(intensity_image, 0.5271f,index_image,cloud);
                       std::cout << ptCloudCounter << "th point cloud contains {" << intensityImageIndices.size()
                                 << "} targets" << std::endl;
                   }
               }
    }

    bag.close();

    return EXIT_SUCCESS;
}
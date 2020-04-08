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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ouster/os1.h>
#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/os1_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <unique_id/unique_id.h>
#include <boost/uuid/random_generator.hpp>

#include <opencv2/opencv.hpp>

#include <cartographer_ros_msgs/LandmarkEntry.h>
#include <cartographer_ros_msgs/LandmarkList.h>
#include <signal.h>

namespace OS1 = ouster::OS1;
using namespace pcl;
using namespace Eigen;

double areaSumOfAllChildrenOf(const std::vector<std::vector<cv::Point>> contours,const std::vector<cv::Vec4i> hierarchy, int i, cv::RotatedRect *rotRect) {
    double sumOfHoles = 0;
    int idx = i;

    cv::Vec4i currentChild = hierarchy[idx];

    while (currentChild[2] != -1) {
        idx = currentChild[2];
        currentChild = hierarchy[idx];

        sumOfHoles = sumOfHoles + contourArea(contours[idx]);
        if (contours[idx].size() > 5 && rotRect->size.width==0) {
            *rotRect = fitEllipse(contours[idx]);
        }
    }

    return sumOfHoles;
}
std::vector<cv::Point2f> ptCloudIndicesOfLandMarks (const cv::Mat image, const float binaryThreshhold) {
    std::vector<cv::Point2f> indices;

    double min_im, max_im;

    cv::minMaxLoc(image, &min_im, &max_im);

    cv::Mat im_norm(image);
    if (min_im == max_im) {
        PCL_INFO("Cannot normalize intensity image...\n");
    } else {
        im_norm = (im_norm - min_im) / (max_im - min_im);

        cv::Mat threshIm;

        threshIm = image > binaryThreshhold;

        cv::Mat dst = cv::Mat::zeros(64, 2048, CV_8UC3);
        rotate(im_norm, im_norm, cv::ROTATE_90_CLOCKWISE);
        cv::Mat drawing(threshIm);
        cvtColor(drawing, drawing, CV_GRAY2RGB);

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        std::vector<std::vector<cv::Point> > contours_poly;
        std::vector<cv::Rect> boundRect;
        std::vector<cv::RotatedRect> outerEllipseRects;
        std::vector<cv::RotatedRect> innerEllipseRects;
        std::vector<int> areaIdx;

        findContours(threshIm, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        int idx = 0;
        for (size_t i = 0; i < contours.size(); i++) {

            double area = contourArea(contours[i]);
            if (area > 85 && area < 600) {
                double subArea;

                cv::RotatedRect innerRotRect;
                subArea = areaSumOfAllChildrenOf(contours, hierarchy, i, &innerRotRect);

                if (subArea > 0) {
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
    }

    return indices;
}

pcl::PointXYZ pointAtIndex(const cv::Point2f coords,const cv::Mat indexImage,const ouster_ros::OS1::CloudOS1 ptCloud) {
    size_t index = indexImage.at<int>((int)round(coords.y),(int)round(coords.x));
    pcl::PointXYZ point(ptCloud.points[index].x,ptCloud.points[index].y,ptCloud.points[index].z);
    return point;
}

bool isDefinedPoint(pcl::PointXYZ point) {
    if (point.x == 0 && point.y == 0 && point.z == 0) {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "landmark_node");
    ros::NodeHandle nh("~");

    boost::uuids::random_generator generator;

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    auto H = OS1::pixels_per_column;
    auto W = OS1::n_cols_of_lidar_mode(
            OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    const auto px_offset = ouster::OS1::get_px_offset(W);
    ros::Publisher landmarkListPublisher =
            nh.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 100);

    ouster_ros::OS1::CloudOS1 cloud{};

    auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
        pcl::fromROSMsg(*m, cloud);

        cartographer_ros_msgs::LandmarkList landmarks;
        landmarks.header = m->header;
        landmarks.header.frame_id = "os1_lidar";
        std::vector<cartographer_ros_msgs::LandmarkEntry>  landmarkEntries;


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

        std::vector<cv::Point2f> intensityImageIndices;
        intensityImageIndices = ptCloudIndicesOfLandMarks(intensity_image,0.5271f);

        for (size_t j = 0; j < intensityImageIndices.size(); ++j) {
            pcl::PointXYZ landmarkPoint;
            landmarkPoint = pointAtIndex(intensityImageIndices[j],index_image,cloud);
            if (isDefinedPoint(landmarkPoint)) {
                cartographer_ros_msgs::LandmarkEntry landmarkEntry;
                geometry_msgs::Pose pose;
                pose.position.x = -landmarkPoint.x;
                pose.position.y = -landmarkPoint.y;
                pose.position.z = -landmarkPoint.z;
                pose.orientation = geometry_msgs::Quaternion();
                landmarkEntry.tracking_from_landmark_transform = pose;


                boost::uuids::uuid uuid = generator();
                landmarkEntry.id = boost::uuids::to_string(uuid);
                landmarkEntry.rotation_weight = 0;
                landmarkEntry.translation_weight = 0;

                landmarkEntries.push_back(landmarkEntry);
            }
        }


        landmarks.landmark = landmarkEntries;
        landmarkListPublisher.publish(landmarks);
    };

    auto pc_sub =
            nh.subscribe<sensor_msgs::PointCloud2>("points", 100, cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}
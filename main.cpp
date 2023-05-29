//
// Created by guli on 2/14/23.
//
//
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <iomanip>
#include "open3d/Open3D.h"
#include "src/PointCloudProcessor.h"


int main() {
    // read original point cloud
    std::shared_ptr<open3d::geometry::PointCloud> pOrigPointcloud = open3d::io::CreatePointCloudFromFile("/home/guli/CLionProjects/EPFL/data/DemoICPPointClouds/cloud_bin_0.pcd");
    // read target point cloud
    std::shared_ptr<open3d::geometry::PointCloud> pTargetPointcloud = open3d::io::CreatePointCloudFromFile("/home/guli/CLionProjects/EPFL/data/DemoICPPointClouds/cloud_bin_1.pcd");

    PointCloudProcessor Obj(pOrigPointcloud, pTargetPointcloud);

    //register 2 point clouds
    Eigen::Matrix4d transMatrix;
    transMatrix << 0.862, 0.011, -0.507, 0.5,
            -0.139, 0.967, -0.215, 0.7,
            0.487, 0.255, 0.835, -1.4,
            0.0, 0.0, 0.0, 1.0;
    Obj.ICPRegistration(pOrigPointcloud, pTargetPointcloud, transMatrix);

    open3d::geometry::PointCloud combinedCloud = Obj.CombineRegistration(*pOrigPointcloud, *pTargetPointcloud, transMatrix);

    // segment the ground
    std::shared_ptr<open3d::geometry::PointCloud> pCombinedCloud =
            std::make_shared<open3d::geometry::PointCloud>(combinedCloud);

    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::PointCloud>> resultSegment =
                                                                                Obj.Segmentation(pCombinedCloud);

    std::shared_ptr<open3d::geometry::PointCloud> inlier = std::get<0>(resultSegment);;
    std::shared_ptr<open3d::geometry::PointCloud> outlier = std::get<1>(resultSegment);

    open3d::visualization::DrawGeometries({inlier, outlier});

    return 0;
}


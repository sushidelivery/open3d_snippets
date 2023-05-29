//
// Created by guli on 5/29/23.
//

#ifndef EPFL_POINTCLOUDPROCESSOR_H
#define EPFL_POINTCLOUDPROCESSOR_H
#include "open3d/Open3D.h"

class PointCloudProcessor {
    public:
        PointCloudProcessor(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr_one,
                            const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr_two);
        ~PointCloudProcessor();
        PointCloudProcessor() = delete;
        PointCloudProcessor(PointCloudProcessor&) = delete;
        PointCloudProcessor& operator = (PointCloudProcessor&) = delete;

        std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::PointCloud>>
                Segmentation(std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr);

        void ICPRegistration(const std::shared_ptr<open3d::geometry::PointCloud> &pOrigPointcloud,
                             const std::shared_ptr<open3d::geometry::PointCloud> &pTargetPointcloud,
                             Eigen::Matrix4d &transInit,
                             const double &voxel_size = 0.05);
        open3d::geometry::PointCloud CombineRegistration(const open3d::geometry::PointCloud &source,
                                                                              const open3d::geometry::PointCloud &target,
                                                                              const Eigen::Matrix4d &Transformation);

private:
        std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr_one_;
        std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr_two_;

};

#endif //EPFL_POINTCLOUDPROCESSOR_H

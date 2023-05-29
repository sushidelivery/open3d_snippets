//
// Created by guli on 5/29/23.
//

#include "PointCloudProcessor.h"
#include "iostream"
#include <iomanip>


std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::PointCloud>>
PointCloudProcessor::Segmentation(std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr) {
    if(pointcloud_ptr == nullptr || !pointcloud_ptr)
    {
        std::cerr << "PointCloud Error" << std::endl;
        std::make_tuple(nullptr, nullptr);
    }
    std::tuple<Eigen::Vector4d , std::vector<size_t>> segmentResult = pointcloud_ptr->SegmentPlane(0.01,3,1000);
    Eigen::Vector4d planeModel = std::get<0>(segmentResult);
    std::vector<size_t> inlier = std::get<1>(segmentResult);

    double a = planeModel[0];
    double b = planeModel[1];
    double c = planeModel[2];
    double d = planeModel[3];
    std::cout << "Segmentation Result\nPlane equation: " << std::fixed << std::setprecision(3) << a << "x + " << b << "y + " << c << "z + " << d << std::endl;

    std::shared_ptr<open3d::geometry::PointCloud> inlierCloud = pointcloud_ptr->SelectByIndex(inlier);
    inlierCloud->PaintUniformColor({1.0,0,0});
    std::shared_ptr<open3d::geometry::PointCloud> outlierCloud = pointcloud_ptr->SelectByIndex(inlier, true);
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::PointCloud>> resultSegment = std::make_tuple(inlierCloud, outlierCloud);
    return resultSegment;

}

PointCloudProcessor::PointCloudProcessor(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr_one,
                                         const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud_ptr_two)
                    :pointcloud_ptr_one_(pointcloud_ptr_one),pointcloud_ptr_two_(pointcloud_ptr_two){
    std::cout << "Object Constructed" << std::endl;
}

PointCloudProcessor::~PointCloudProcessor(){
    std::cout << "Object Destructed" << std::endl;
}

open3d::geometry::PointCloud PointCloudProcessor::CombineRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(
            new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    source_transformed_ptr->PaintUniformColor({1, 0.706, 0});
    target_ptr->PaintUniformColor({0, 0.651, 0.929});
    open3d::geometry::PointCloud combinedCloud = *source_transformed_ptr + *target_ptr;
    return combinedCloud;

}

void PointCloudProcessor::ICPRegistration(const std::shared_ptr<open3d::geometry::PointCloud> &pOrigPointcloud,
                                          const std::shared_ptr<open3d::geometry::PointCloud> &pTargetPointcloud,
                                          Eigen::Matrix4d &transInit,
                                          const double &voxel_size){

    auto source_down = pOrigPointcloud->VoxelDownSample(voxel_size);
    source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
            voxel_size * 2.0, 30));

    auto target_down = pTargetPointcloud->VoxelDownSample(voxel_size);
    target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
            voxel_size * 2.0, 30));

    auto result = open3d::pipelines::registration::RegistrationICP(
            *source_down, *target_down, 0.07, transInit,
            open3d::pipelines::registration::
            TransformationEstimationPointToPoint(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,
                                                                    1000));
    transInit = result.transformation_;
    std::cout << "Registration Result\n" << transInit << std::endl;
};

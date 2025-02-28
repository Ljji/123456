#ifndef POINT_CLOUD_UTILS_H
#define POINT_CLOUD_UTILS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

namespace tomato_slam {

template<typename PointT>
Eigen::Vector3f computeCentroid(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    return centroid.head<3>();
}

template<typename PointT>
void computePrincipalDirections(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                               Eigen::Matrix3f& eigenvectors,
                               Eigen::Vector3f& eigenvalues) {
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::computeCovarianceMatrix(*cloud, centroid, covariance_matrix);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
    eigenvalues = eigen_solver.eigenvalues();
    eigenvectors = eigen_solver.eigenvectors();
}

template<typename PointT>
void transformPointCloud(const typename pcl::PointCloud<PointT>::Ptr& input_cloud,
                        typename pcl::PointCloud<PointT>::Ptr& output_cloud,
                        const Eigen::Matrix4f& transform) {
    pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}

} // namespace tomato_slam

#endif // POINT_CLOUD_UTILS_H

#ifndef NOISE_VOXEL_GRID_H
#define NOISE_VOXEL_GRID_H

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class noise_voxel_grid : public pcl::VoxelGrid<pcl::PointXYZ> {
public:
    // add stuff from super class to be able to use its members unmodified
    typedef pcl::VoxelGrid<pcl::PointXYZ> super;
    typedef super::PointCloud PointCloud;
    typedef pcl::PointXYZ PointT;
protected:
    using super::filter_name_;
    using super::getClassName;
    using super::input_;
    using super::indices_;
    int min_points; // minimum points in a voxel required to add it as a point
    int skip_points; // just keep every skip_points points
public:
    noise_voxel_grid(int min_points, int skip_points = 1);
    void applyFilter(PointCloud &output);
};

#endif // NOISE_VOXEL_GRID_H

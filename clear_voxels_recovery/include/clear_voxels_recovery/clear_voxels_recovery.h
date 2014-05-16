#ifndef RUN_BACKWARDS_RECOVERY_H
#define RUN_BACKWARDS_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/voxel_layer.h>

namespace clear_voxels_recovery {

    class ClearVoxelsRecovery : public nav_core::RecoveryBehavior {
    public:
        /**
        * @brief Constructor, make sure to call initialize in addition to actually initialize the object
        * @param
        * @return
        */
        ClearVoxelsRecovery();

        /**
        * @brief Initialization function for the ClearCostmapRecovery recovery behavior
        * @param tf A pointer to a transform listener
        * @param global_costmap A pointer to the global_costmap used by the navigation stack
        * @param local_costmap A pointer to the local_costmap used by the navigation stack
        */
        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        /**
        * @brief Run the ClearCostmapRecovery recovery behavior. Reverts the
        * costmap to the static map outside of a user-specified window and
        * clears unknown space around the robot.
        */
        void runBehavior();

    private:
        void clear(costmap_2d::Costmap2DROS* costmap);
        void clearMap(boost::shared_ptr<costmap_2d::VoxelLayer> costmap, 
                      double pose_x, double pose_y);
        std::string name_;
        bool initialized_;
        tf::TransformListener* tf_;
        costmap_2d::Costmap2DROS* global_costmap_;
        costmap_2d::Costmap2DROS* local_costmap_;
    };
    
}; // run_backwards_recovery
#endif // RUN_BACKWARDS_RECOVERY_H

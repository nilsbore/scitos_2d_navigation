#include <clear_voxels_recovery/clear_voxels_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(clear_voxels_recovery, ClearVoxelsRecovery, clear_voxels_recovery::ClearVoxelsRecovery, nav_core::RecoveryBehavior)

namespace clear_voxels_recovery {

    ClearVoxelsRecovery::ClearVoxelsRecovery() : initialized_(false)
    {
    
    }
    
    void ClearVoxelsRecovery::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
    {
        if(!initialized_){
            name_ = name;
            tf_ = tf;
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            //get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name_);

            //private_nh.param("reset_distance", reset_distance_, 3.0);
            //private_nh.param("layer_search_string", layer_search_string_, std::string("obstacle"));

            initialized_ = true;
        }
        else {
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }
    
    void ClearVoxelsRecovery::runBehavior()
    {
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        if(global_costmap_ == NULL || local_costmap_ == NULL){
            ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
            return;
        }
        ROS_WARN("Clearing chest camera costmap in front to unstuck robot.");
        clear(global_costmap_);
        clear(local_costmap_);
    }
    
    
    void ClearVoxelsRecovery::clear(costmap_2d::Costmap2DROS* costmap){
        std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

        tf::Stamped<tf::Pose> pose;

        if(!costmap->getRobotPose(pose)){
            ROS_ERROR("Cannot clear map because pose cannot be retrieved");
            return;
        }

        double x = pose.getOrigin().x();
        double y = pose.getOrigin().y();
        
        std::string layer_search_string("chest"); // or voxel, depending on what this is

        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
            if(plugin->getName().find(layer_search_string)!=std::string::npos){
                ROS_INFO("Clearing voxel layer: %s", plugin->getName().c_str());
                boost::shared_ptr<costmap_2d::VoxelLayer> costmap;
                costmap = boost::static_pointer_cast<costmap_2d::VoxelLayer>(plugin);
                clearMap(costmap, x, y);
            }
        }
    }

    void ClearVoxelsRecovery::clearMap(boost::shared_ptr<costmap_2d::VoxelLayer> costmap, 
                                        double pose_x, double pose_y){
        boost::unique_lock< boost::shared_mutex > lock(*(costmap->getLock()));

        double reset_distance = 1.2;
        double start_point_x = pose_x - reset_distance / 2.0;
        double start_point_y = pose_y - reset_distance / 2.0;
        double end_point_x = start_point_x + reset_distance;
        double end_point_y = start_point_y + reset_distance;

        int start_x, start_y, end_x, end_y;
        costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
        costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

        unsigned char* grid = costmap->getCharMap();
        for (int x=0; x<(int)costmap->getSizeInCellsX(); x++){
            bool xrange = x < start_x && x > end_x;
                           
            for(int y=0; y < (int)costmap->getSizeInCellsY(); y++){
                    if (xrange && y < start_y && y > end_y)
                        continue;
                    int index = costmap->getIndex(x,y);
                    if (grid[index] != costmap_2d::NO_INFORMATION) {
                        grid[index] = costmap_2d::NO_INFORMATION;
                    }
            }
        }

        double ox = costmap->getOriginX(), oy = costmap->getOriginY();
        double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
        costmap->setResetBounds(ox, ox + width, oy, oy + height);
        return;
    }
        
}; // run_backwards_recovery

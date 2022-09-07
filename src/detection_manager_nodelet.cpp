#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_object_detection/point_cloud_processor.hpp>
#include <pcl_object_detection/ObjectDetectionParameterConfig.h>
#include <pcl_object_detection/DetectionModeCtr.h>

#include <sobit_common_msg/RunCtrl.h>
#include <sobit_common_msg/ObjectPose.h>
#include <sobit_common_msg/ObjectPoseArray.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

namespace pcl_object_detection {
    enum Mode {
        OFF = 0,
        TABLE_MODE,
        FLOOR_MODE,
        SHELF_MODE,
        PLACEABLE_POSITION
    };

    class DetectionManager : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::ServiceServer srv_subscriber_switch_;
            ros::ServiceServer srv_detection_mode_switch_;
            ros::ServiceClient client_table_;
            ros::ServiceClient client_floor_;
            ros::ServiceClient client_shelf_;
            ros::ServiceClient client_placeable_;
            int detection_mode_;

            dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>* server_;
            dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(pcl_object_detection::ObjectDetectionParameterConfig& config, uint32_t level);
            bool callbackSubscriberSwitch( sobit_common_msg::RunCtrl::Request &req, sobit_common_msg::RunCtrl::Response &res  );
            bool detctionModeSwitchCb ( pcl_object_detection::DetectionModeCtr::Request &req, pcl_object_detection::DetectionModeCtr::Response &res );

        public:
            virtual void onInit();

    };
}

void pcl_object_detection::DetectionManager::callbackDynamicReconfigure(pcl_object_detection::ObjectDetectionParameterConfig& config, uint32_t level) {
    sobit_common_msg::RunCtrl table, floor, shelf, placeable;
    table.request.request = false;
    floor.request.request = false;
    shelf.request.request = false;
    placeable.request.request = false;
        //オン（再定義）
    if ( config.detection_mode == Mode::OFF ) {
        NODELET_INFO ("Setting a mode = OFF" );
    } else if( config.detection_mode == Mode::TABLE_MODE ) {
        NODELET_INFO ("Setting a mode = TABLE_MODE" );
        table.request.request = true;
    } else if( config.detection_mode == Mode::FLOOR_MODE ) {
        NODELET_INFO ("Setting a mode = FLOOR_MODE" );
        floor.request.request = true;
    } else if( config.detection_mode == Mode::SHELF_MODE ) {
        NODELET_INFO ("Setting a mode = SHELF_MODE" );
        shelf.request.request = true;
    } else if ( config.detection_mode == Mode::PLACEABLE_POSITION ) {
        NODELET_INFO ("Setting a mode = PLACEABLE_POSITION" );
        placeable.request.request = true;
    }
    if ( !client_table_.call(table) ) ROS_ERROR("Failed to call service object_detection_table_run_ctr");
    if ( !client_floor_.call(floor) ) ROS_ERROR("Failed to call service object_detection_floor_run_ctr");
    if ( !client_shelf_.call(shelf) ) ROS_ERROR("Failed to call service object_detection_shelf_run_ctr");
    if ( !client_placeable_.call(placeable) ) ROS_ERROR("Failed to call service placeable_detection_run_ctr");
    std::cout << "\n" << std::endl;
    return;
}

bool pcl_object_detection::DetectionManager::detctionModeSwitchCb ( pcl_object_detection::DetectionModeCtr::Request &req, pcl_object_detection::DetectionModeCtr::Response &res ) {
    sobit_common_msg::RunCtrl table, floor, shelf, placeable;
    table.request.request = false;
    floor.request.request = false;
    shelf.request.request = false;
    placeable.request.request = false;
    if ( req.detection_mode == Mode::OFF ) {
        NODELET_INFO ("Setting a mode = OFF" );
    } else if( req.detection_mode == Mode::TABLE_MODE ) {
        NODELET_INFO ("Setting a mode = TABLE_MODE" );
        table.request.request = true;
    } else if( req.detection_mode == Mode::FLOOR_MODE ) {
        NODELET_INFO ("Setting a mode = FLOOR_MODE" );
        floor.request.request = true;
    } else if( req.detection_mode == Mode::SHELF_MODE ) {
        NODELET_INFO ("Setting a mode = SHELF_MODE" );
        shelf.request.request = true;
    } else if ( req.detection_mode == Mode::PLACEABLE_POSITION ) {
        NODELET_INFO ("Setting a mode = PLACEABLE_POSITION" );
        placeable.request.request = true;
    } else {
        ROS_INFO ("Setting a mode that does not exist. mode = %d", req.detection_mode );
        return false;
    }
    if ( !client_table_.call(table) ) ROS_ERROR("Failed to call service object_detection_table_run_ctr");
    if ( !client_floor_.call(floor) ) ROS_ERROR("Failed to call service object_detection_floor_run_ctr");
    if ( !client_shelf_.call(shelf) ) ROS_ERROR("Failed to call service object_detection_shelf_run_ctr");
    if ( !client_placeable_.call(placeable) ) ROS_ERROR("Failed to call service placeable_detection_run_ctr");
    std::cout << "\n" << std::endl;
    detection_mode_ = req.detection_mode;
    res.response = true;
    return true;
}


bool pcl_object_detection::DetectionManager::callbackSubscriberSwitch( sobit_common_msg::RunCtrl::Request &req, sobit_common_msg::RunCtrl::Response &res ) {
    sobit_common_msg::RunCtrl table, floor, shelf, placeable;
    table.request.request = false;
    floor.request.request = false;
    shelf.request.request = false;
    placeable.request.request = false;
    if ( req.request ) {
         //オン（再定義）
        if ( detection_mode_ == Mode::OFF ) {
            NODELET_INFO ("Setting a mode = OFF" );
        } else if( detection_mode_ == Mode::TABLE_MODE ) {
            NODELET_INFO ("Setting a mode = TABLE_MODE" );
            table.request.request = true;
        } else if( detection_mode_ == Mode::FLOOR_MODE ) {
            NODELET_INFO ("Setting a mode = FLOOR_MODE" );
            floor.request.request = true;
        } else if( detection_mode_ == Mode::SHELF_MODE ) {
            NODELET_INFO ("Setting a mode = SHELF_MODE" );
            shelf.request.request = true;
        } else if ( detection_mode_ == Mode::PLACEABLE_POSITION ) {
            NODELET_INFO ("Setting a mode = PLACEABLE_POSITION" );
            placeable.request.request = true;
        }
    } else {
        ROS_INFO ("Turn off the PCL Object Detection" );
    }
    if ( !client_table_.call(table) ) ROS_ERROR("Failed to call service object_detection_table_run_ctr");
    if ( !client_floor_.call(floor) ) ROS_ERROR("Failed to call service object_detection_floor_run_ctr");
    if ( !client_shelf_.call(shelf) ) ROS_ERROR("Failed to call service object_detection_shelf_run_ctr");
    if ( !client_placeable_.call(placeable) ) ROS_ERROR("Failed to call service placeable_detection_run_ctr");
    std::cout << "\n" << std::endl;
    res.response = true;
    return true;
}

void pcl_object_detection::DetectionManager::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    srv_subscriber_switch_ = nh_.advertiseService( "run_ctr", &DetectionManager::callbackSubscriberSwitch, this);
    srv_detection_mode_switch_ = nh_.advertiseService( "detection_mode_ctr", &DetectionManager::detctionModeSwitchCb, this);
    client_table_ = nh_.serviceClient<sobit_common_msg::RunCtrl>("object_detection_table_run_ctr");
    client_floor_ = nh_.serviceClient<sobit_common_msg::RunCtrl>("object_detection_floor_run_ctr");
    client_shelf_ = nh_.serviceClient<sobit_common_msg::RunCtrl>("object_detection_shelf_run_ctr");
    client_placeable_ = nh_.serviceClient<sobit_common_msg::RunCtrl>("placeable_detection_run_ctr");

    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>(pnh_);
    f_ = boost::bind(&DetectionManager::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    // bool subscriber_on = pnh_.param<bool>( "subscriber_on", true );
}

PLUGINLIB_EXPORT_CLASS(pcl_object_detection::DetectionManager, nodelet::Nodelet)

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPMoveBase class.
 * RPMoveBase is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by id from the parameter server.
 */

namespace KCL_rosplan {

    class RPMoveBase: public RPActionInterface
    {

      public:

        /**
         * @brief constructor
         */
        RPMoveBase(std::string &actionserver);

        /**
         * @brief query wp coordinates from parameter server, where the waypoint values (x, y, theta) are assumed to be stored
         * @param wpID an integer with the waypoint id from which to fetch the pose values (x, y, theta)
         * @param result return value gets written here by reference
         * @return true if waypoint values were found in parameter server, false otherwise
        */
        bool wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped &result);

        /**
         * @brief listen to and process action_dispatch topic
         * @param msg this parameter comes from the topic subscription, it contains the request (id, name, params, etc)
         * @return true if execution was successful
         */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

      private:

        // action lib client that will make the communication with move_base action server
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;

        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;

        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle nh_;

        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;
    };
}
#endif

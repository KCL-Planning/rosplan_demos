/**
 * 
 * Copyright [2019] <KCL Kings College London>  
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * 
 * RPRoadmapFilter class is used to filter an existing waypoint set to N waypoints
 * The filtering is performed by sampling base on an existing costmap.
 */

#include <sstream>
#include <string>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <std_srvs/Trigger.h>

#ifndef KCL_rp_roadmap_filter
#define KCL_rp_roadmap_filter

namespace KCL_rosplan {

    class RPRoadmapFilter
    {

      public:

        RPRoadmapFilter();

        /**
         * @brief check if wps are available in param server, if so, load them for filtering
         * @return True if waypoints were found in param server and are in list format, False otherwise
         */
        bool loadParams();

        /** 
         * @brief callback that gets executed upon receiving a costmap msg
         * @param msg the costmap information is encoded in this variable and its value comes from the ROS network
         */
        void costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

        /**
         * @brief given a waypoint unique id (string) and a pose this function stores the waypoint in the ROS parameter server
         * @param wp_id string that identifies uniquely the waypoint
         * @param pose x, y, theta coordinates and reference frame of the waypoint encoded as pose stamped msg
         */
        void uploadWPToParamServer(std::string wp_id, geometry_msgs::PoseStamped pose);

        /**
         * @brief Callback function provided by this server node, when user makes a request to generate a roadmap
         * @param req the request msg from the user, contains the parameters specified by the user from which the node
         * will generate the waypoints
         * @param res the response that we need to fill and provide to the user as feedback
         */
        bool filterRoadmap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

      private:

        /// visualisation functions
        void pubWPGraph();
        void clearWPGraph();

        /// Update KB function
        void updateKB();

        /// manage communication of this node and the ROS network
        ros::NodeHandle nh_;
        /// topic that this node needs
        ros::Subscriber map_sub_;
        /// topics that this node offers
        ros::Publisher waypoints_pub_; // for visualisation purposes
        /// services that this node will query
        ros::ServiceClient update_kb_client_, update_kb_client_array_;
        /// services that are offered by this node
        ros::ServiceServer filter_waypoint_service_server_;

        /// topic and service names
        std::string costmap_topic_;
        bool costmap_received_;
        nav_msgs::OccupancyGrid cost_map_;
        std::string wp_reference_frame_;

        /// how much to wait in seconds for the costmap and KB update services
        int srv_timeout_;

        /// Roadmap
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        std::vector<geometry_msgs::PoseStamped> filtered_waypoints_;
        int waypoint_count_;

        /// to store the namespace in which the waypoints are stored in the parameter server
        std::string wp_namespace_input_;
        std::string wp_namespace_output_;
    };
}
#endif

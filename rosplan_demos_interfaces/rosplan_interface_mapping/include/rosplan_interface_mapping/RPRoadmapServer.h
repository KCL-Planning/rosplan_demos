/**
 * 
 * Copyright [2019] <KCL Kings College London>  
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Oscar Lima (oscar.lima@dfki.de)
 * 
 * RPRoadmapServer class is used to generate N free collision waypoints from a costmap subscription
 * 
 * - Currently this works throught the nav_msgs/GetMap service.
 * - Waypoints are stored symbolically in the Knowledge Base.
 * - Waypoint coordinates are stored in the parameter server.
 * - Connectivity between waypoints is also computed.
 * - Loading existing waypoints from a file is supported.
 */

#include <sstream>
#include <string>

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

#include "rosplan_interface_mapping/CreatePRM.h"
#include "rosplan_interface_mapping/AddWaypoint.h"
#include "rosplan_interface_mapping/RemoveWaypoint.h"

#ifndef KCL_rp_roadmap_server
#define KCL_rp_roadmap_server

namespace KCL_rosplan {

    struct Waypoint
    {
        /**
         * @brief empty struct constructor, struct Waypoint used to store waypoints
         */
        Waypoint();

        /**
         * @brief constructor, struct Waypoint used to store waypoints
         * @param id string waypoint unique identifier
         * @param xCoord waypoint x coordinates in cell coordinates
         * @param yCoord waypoint y coordinates in cell coordinates
         * @param map_meta_data map information: resolution, width, height and origin
         */
        Waypoint(const std::string &id, unsigned int xCoord, unsigned int yCoord, const nav_msgs::MapMetaData& map_meta_data);

        /**
        * @brief Get the distance between this waypoint and @ref{other}.
        * @param other The other waypoint to get the distance from.
        * @return The distance between the waypoints.
        */
        float getDistance(const Waypoint& other);

        /**
        * @brief Update the location of this waypoint such that it's no furter than @ref{max_casting_range} away from @ref{other}.
        * @param other The Waypoint this waypoint was casted from.
        * @param max_casting_range The maximum distance this waypoint can be from the given waypoint.
        * @param map_meta_data The meta data of the occupancy grid.
        */
        void update(const Waypoint& other, float max_casting_range, const nav_msgs::MapMetaData& map_meta_data);

        int grid_x;
        int grid_y;
        double real_x;
        double real_y;
        std::string wpID;
        std::vector<std::string> neighbours;
    }; // end of struct Waypoint

    struct Edge
    {
        Edge(const std::string &s, const std::string &e) : start(s), end(e) {}

        std::string start;
        std::string end;
    }; // end of struct Edge

    class RPRoadmapServer
    {

      public:

        RPRoadmapServer();

        /**
         * @brief check if wps are available in param server, if so, load them in symbolic KB and visualise them
         * @return True if waypoints were found in param server and are in list format, False otherwise
         */
        bool loadParams();

        /** 
         * @brief callback that gets executed upon receiving a odom msg, this is used to get the robot pose
         * @param msg the odom information is encoded in this variable and its value comes from the ROS network
         */
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);

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
        bool generateRoadmap(rosplan_interface_mapping::CreatePRM::Request &req, rosplan_interface_mapping::CreatePRM::Response &res);

        /**
         * @brief Callback function provided by this server node, when user makes a request to add a waypoint
         * this function gets executed, then connects a new waypoint and stores it in the knowledge base and parameter server
         * @param req the request msg from the user
         * @param res the response that we need to fill and provide to the user as feedback
         */
        bool addWaypointSrv(rosplan_interface_mapping::AddWaypoint::Request &req, rosplan_interface_mapping::AddWaypoint::Response &res);

        /**
         * @brief Create new waypoint and stores it in the knowledge base
         * @param store_in_param_server if true, the parameter will be installed on param server
         * @return true for success, false otherwise
         */
        bool addWaypoint(std::string id, geometry_msgs::PoseStamped waypoint, float connecting_distance, bool store_in_param_server);

        /**
         * @brief Callback function provided by this server node, when user makes a request to remove a waypoint
         * this function gets executed
         * @param req the request msg from the user
         * @param res the response that we need to fill and provide to the user as feedback
         */
        bool removeWaypoint(rosplan_interface_mapping::RemoveWaypoint::Request &req, rosplan_interface_mapping::RemoveWaypoint::Response &res);

        /**
         * @brief Service provided by this node to let know the RoadmapServer iterface that waypoints have been loaded in param server
         * and trigger loading them into symbolic KB + visualisation via rviz
         * @param req empty request
         * @param res bool success - indicate successful run of triggered service, string message - information for error msgs
         * @return true if success, false otherwise
         */
        bool loadWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        /**
         * @brief Waypoint generation function, deletes all previous data and generates a new waypoint set
         * @param map the costmap expressing the occupied and free cells of the map such that we do not
         * generate waypoints on top of occupied cells
         * @param nr_waypoints the desired number of waypoint to generate
         * @param min_distance the minimum distance allowed between any pair of waypoints
         * @param casting_distance the maximum distance a waypoint can be cast
         * @param connecting_distance the maximum distance that can exists between waypoints for them to be considered connected
         * @param total_attempts maximum amount of attempts to generate random valid waypoints
         */
        void createPRM(nav_msgs::OccupancyGrid &map, unsigned int nr_waypoints, double min_distance, double casting_distance, double connecting_distance, double occupancy_threshold, int total_attempts);

      private:

        /**
         * @brief remove waypoint from parameter server and from ROSPlan Knowledge base
         * @param id the waypoint id (string) of the waypoint to delete
         * @return true if removed succesfully, false otherwise
         */
        bool clearWaypoint(const std::string &id);

        /**
        * @brief Check if two waypoints can be connected without colliding with any known scenery. The line should not
        * come closer than @ref{min_width} than any known obstacle.
        * @param w1 The first waypoint.
        * @param w2 The second waypoint.
        * @return True if the waypoints can be connected, false otherwise.
        */
        bool canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, nav_msgs::OccupancyGrid &map, double occupancy_threshold);

        /// visualisation functions
        void pubWPGraph();
        void clearWPGraph();

        /// manage communication of this node and the ROS network
        ros::NodeHandle nh_;
        /// topic that this node needs
        ros::Subscriber odom_sub_, map_sub_;
        /// topics that this node offers
        ros::Publisher waypoints_pub_, edges_pub_; // for visualisation purposes
        /// services that this node will query
        ros::ServiceClient update_kb_client_, update_kb_client_array_, get_map_client_;
        /// services that are offered by this node
        ros::ServiceServer remove_waypoint_service_server_, waypoint_service_server_, prm_service_server_, load_wp_service_server_;

        /// topic and service names
        std::string costmap_topic_;

        bool use_static_map_, costmap_received_, update_connectivity_, update_waypoints_;
        std::string wp_reference_frame_;
        nav_msgs::OccupancyGrid cost_map_;
        geometry_msgs::PoseStamped base_odom_;
        tf::TransformListener tf_;

        /// how much to wait in seconds for the costmap and KB update services
        int srv_timeout_;
        /// threshold for a costmap cell to be considered occupied
        int occupancy_threshold_;

        /// Roadmap
        std::map<std::string, Waypoint*> waypoints_;
        std::vector<Edge> edges_;

        /// to store the namespace in which the waypoints are stored in the parameter server
        std::string wp_namespace_;

    };
}
#endif

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include "rosplan_interface_mapping/CreatePRM.h"
#include "rosplan_interface_mapping/AddWaypoint.h"
#include "rosplan_interface_mapping/RemoveWaypoint.h"

#ifndef KCL_roadmap
#define KCL_roadmap

/**
 * RPRoadmapServer is used to generate waypoints from a subscribed costmap.
 * Currently this works throught the nav_msgs/GetMap service.
 * Waypoints are stored symbolically in the Knowledge Base.
 * Waypoint coordinates are stored in the parameter server.
 */
namespace KCL_rosplan {

    struct Waypoint
    {
        Waypoint(const std::string &id, unsigned int xCoord, unsigned int yCoord, const nav_msgs::MapMetaData& map_meta_data)
            : wpID(id), grid_x(xCoord), grid_y(yCoord) {
            occupancy_grid_utils::Cell cell;
            cell.x = grid_x;
            cell.y = grid_y;

            geometry_msgs::Point real_point = occupancy_grid_utils::cellCenter(map_meta_data, cell);
            real_x = real_point.x;
            real_y = real_point.y;
        }

        Waypoint()
            : wpID("wp_err"), grid_x(0), grid_y(0) {}

        /**
         * Get the distance between this waypoint and @ref{other}.
         * @param other The other waypoint to get the distance from.
         * @return The distance between the waypoints.
         */
        float getDistance(const Waypoint& other) {
            return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
        }

        /**
         * Update the location of this waypoint such that it's no furter than @ref{max_casting_range} away from @ref{other}.
         * @param other Waypoint this waypoint was casted from.
         * @param max_casting_range The maximum distance this waypoint can be from the given waypoint.
         * @param resolution The resolution of the occupancy grid.
         * @param origin The origin of the occupancy grid.
         */
        void update(const Waypoint& other, float max_casting_range, const nav_msgs::MapMetaData& map_meta_data) {
            float distance = getDistance(other);
            if (distance > max_casting_range) {
                float scale = max_casting_range / distance;

                real_x = other.real_x + (real_x - other.real_x) * scale;
                real_y = other.real_y + (real_y - other.real_y) * scale;
                geometry_msgs::Point point;
                point.x = real_x;
                point.y = real_y;

                occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(map_meta_data, point);
                grid_x = cell.x;
                grid_y = cell.y;
            }
        }

        std::string wpID;
        int grid_x;
        int grid_y;
        double real_x;
        double real_y;
        std::vector<std::string> neighbours;
    }; // end of struct Waypoint

    struct Edge
    {
        Edge(const std::string &s, const std::string &e)
            : start(s), end(e) {}

        std::string start;
        std::string end;
    };

    class RPRoadmapServer
    {

    public:

        RPRoadmapServer();

        /* callbacks to maps and odom */
        void odomCallback( const nav_msgs::OdometryConstPtr& msg );
        void costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg );

        /* service to (re)generate waypoints */
        void uploadWPToParamServer(std::string wp_id, geometry_msgs::PoseStamped pose);
        bool generateRoadmap(rosplan_interface_mapping::CreatePRM::Request &req, rosplan_interface_mapping::CreatePRM::Response &res);
        bool addWaypoint(rosplan_interface_mapping::AddWaypoint::Request &req, rosplan_interface_mapping::AddWaypoint::Response &res);
        bool removeWaypoint(rosplan_interface_mapping::RemoveWaypoint::Request &req, rosplan_interface_mapping::RemoveWaypoint::Response &res);

        void createPRM(nav_msgs::OccupancyGrid map, unsigned int nr_waypoints, double min_distance,
                double casting_distance, double connecting_distance, int occupancy_threshold, int total_attempts);

    private:

        void publishWaypointMarkerArray();
        void publishEdgeMarkerArray();
        void clearMarkerArrays();
        bool clearWaypoint(const std::string &id);
        /**
         * Check if two waypoints can be connected without colliding with any known scenery. The line should not
         * come closer than @ref{min_width} than any known obstacle.
         * @param w1 The first waypoint.
         * @param w2 The second waypoint.
         * @param threshold A value between -1 and 255 above which a cell is considered to be occupied.
         * @return True if the waypoints can be connected, false otherwise.
         */
        bool canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int threshold) const;

        ros::NodeHandle nh_;
        std::string static_map_service_;
        bool use_static_map_;
        std::string fixed_frame_;

        // odometry
        nav_msgs::OccupancyGrid cost_map_;
        geometry_msgs::PoseStamped base_odom_;
        ros::ServiceClient map_client_;
        tf::TransformListener tf_;

        // Knowledge base
        ros::ServiceClient update_knowledge_client_;

        // Roadmap
        std::map<std::string, Waypoint*> waypoints_;
        std::map<std::string, std::string> db_name_map_;
        std::vector<Edge> edges_;

        // visualisation
        ros::Publisher waypoints_pub_;
        ros::Publisher edges_pub_;

    };
}
#endif

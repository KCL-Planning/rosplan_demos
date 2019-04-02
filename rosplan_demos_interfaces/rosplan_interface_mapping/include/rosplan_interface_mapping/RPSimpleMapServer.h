#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>

#ifndef KCL_simple_roadmap
#define KCL_simple_roadmap

namespace KCL_rosplan {

    struct Waypoint
    {
        Waypoint(const std::string &id, double xCoord, double yCoord)
            : wpID(id), real_x(xCoord), real_y(yCoord) {}

        Waypoint()
            : wpID("wp_err"), real_x(0), real_y(0) {}

        float getDistance(const Waypoint& other) {
            return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
        }

        std::string wpID;
        double real_x;
        double real_y;
        std::vector<std::string> neighbours;
    };

    class RPSimpleMapServer
    {

    public:

        RPSimpleMapServer();

        // service to (re)generate waypoints
        bool setupRoadmap();

    private:

        // Roadmap
        std::map<std::string, Waypoint*> waypoints_;
        std::map<std::string, std::string> db_name_map;
        void parsePose(geometry_msgs::PoseStamped &pose, std::string line);
        bool getWPCoordinates();

        // visualisation
        void publishWaypointMarkerArray();
        void clearMarkerArrays();

        ros::NodeHandle nh_;
        ros::ServiceClient update_knowledge_client_; // to update KB
        ros::Publisher waypoints_pub_;
        std::string fixed_frame_;
    };
}
#endif

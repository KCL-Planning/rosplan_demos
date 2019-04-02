#include "rosplan_interface_mapping/RPSimpleMapServer.h"
#include <tf/transform_listener.h>

namespace KCL_rosplan {

    RPSimpleMapServer::RPSimpleMapServer() : nh_("~") {

        // knowledge interface
        std::string kb;
        nh_.param<std::string>("knowledge_base", kb, "knowledge_base");
        std::stringstream ss;
        ss << "/" << kb << "/update";
        update_knowledge_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ros::service::waitForService(ss.str(), ros::Duration(20));

        // query params from param server
        nh_.param<std::string>("fixed_frame", fixed_frame_, "map");

        // visualisation
        ss.str("");
        ss << "/" << kb << "/viz/waypoints";
        waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 10, true);
    }

    /*-----------*/
    /* build PRM */
    /*-----------*/

    /**
    * parses a pose with yaw from strings: "[f, f, f]"
    */
    void RPSimpleMapServer::parsePose(geometry_msgs::PoseStamped &pose, std::string line) {

        int curr,next;
        curr=line.find("[")+1;
        next=line.find(",",curr);

        pose.pose.position.x = (double)atof(line.substr(curr,next-curr).c_str());
        curr=next+1; next=line.find(",",curr);

        pose.pose.position.y = (double)atof(line.substr(curr,next-curr).c_str());
        curr=next+1; next=line.find("]",curr);

        float theta = atof(line.substr(curr,next-curr).c_str());
        tf::Quaternion q;
        q.setEuler(theta, 0 ,0);


        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.z();
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.z = q.y();
    }

    bool RPSimpleMapServer::getWPCoordinates() {

        // function that gets an unknown amount of waypoints from param server under a specific namespace

        XmlRpc::XmlRpcValue waypoint_list;

        if (!nh_.getParam("/waypoints", waypoint_list)) {
            ROS_ERROR("Could not found waypoint list in parameter server");
            return false;
        }

        if(!(waypoint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
            ROS_ERROR("Failed to assert that waypoints are a list");
            return false;
        }

        // make sure that waypoint list is greater than 0
        if(!(waypoint_list.size() > 0)) {
            ROS_ERROR("waypoint list cannot be empty");
            return false;
        }

        // iterate over list
        for(auto wit=waypoint_list.begin(); wit!=waypoint_list.end(); wit++) {
            if(!wit->second.valid()) {
                ROS_ERROR("waypoint value not set");
                return false;
            }

            ROS_INFO_STREAM(wit->first << " : " << wit->second);
            // TODO: store waypoints in struct?
        }

        return true;
    }

    void RPSimpleMapServer::publishWaypointMarkerArray()
    {
        visualization_msgs::MarkerArray marker_array;
        size_t counter = 0;
        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = fixed_frame_;
            marker.header.stamp = ros::Time();
            marker.ns = "mission_waypoint";
            marker.id = counter; counter++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.pose.position.x = wit->second->real_x;
            marker.pose.position.y = wit->second->real_y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.3;
            marker.color.g = 1.0;
            marker.color.b = 0.3;
            marker.text = wit->first;
            marker_array.markers.push_back(marker);
        }
        waypoints_pub_.publish( marker_array );
    }

    /* clears all waypoints and edges */
    void RPSimpleMapServer::clearMarkerArrays()
    {
        visualization_msgs::MarkerArray marker_array;
        size_t counter = 0;
        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = fixed_frame_;
            marker.header.stamp = ros::Time();
            marker.ns = "mission_waypoint";
            marker.id = counter; counter++;
            marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(marker);
        }
        waypoints_pub_.publish( marker_array );
    }

    bool RPSimpleMapServer::setupRoadmap() {

        ROS_INFO("KCL: (RPSimpleMapServer) Loading roadmap from parameter server");

        // TODO: replace function below with getWPCoordinates()

        // load configuration file
//         std::ifstream infile(filename.c_str());
//         std::string line;
//         int curr,next;
//         while(std::getline(infile, line)) {
//             // read waypoint
//             curr=line.find("[");
//             std::string name = line.substr(0,curr);
//
//             // instance
//             rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
//             updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//             updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
//             updateSrv.request.knowledge.instance_type = "waypoint";
//             updateSrv.request.knowledge.instance_name = name;
//             update_knowledge_client_.call(updateSrv);
//
//             // data
//             geometry_msgs::PoseStamped pose;
//             pose.header.frame_id = fixed_frame_;
//             parsePose(pose, line);
//
//             // save here for viz
//             Waypoint* wp = new Waypoint(name, pose.pose.position.x, pose.pose.position.y);
//             waypoints_[wp->wpID] = wp;
//         }
//         infile.close();
//
//         // publish visualization
//         publishWaypointMarkerArray();
        return true;
    }

} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_simple_map_server");

    // init
    KCL_rosplan::RPSimpleMapServer sms;
    sms.setupRoadmap();

    ROS_INFO("KCL: (RPSimpleMapServer) Ready to receive.");
    ros::spin();
    return 0;
}

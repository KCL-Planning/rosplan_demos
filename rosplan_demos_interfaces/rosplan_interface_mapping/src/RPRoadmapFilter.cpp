/**
 * 
 * Copyright [2019] <KCL King's College London>
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 * Maintainer: Michael Cashmore (michael.cashmore@kcl.ac.uk)
 */

#include "rosplan_interface_mapping/RPRoadmapFilter.h"

namespace KCL_rosplan {

    /*-----------------*/
    /* RPRoadmapFilter */
    /*-----------------*/

    /**
     * Constructor
     */
    RPRoadmapFilter::RPRoadmapFilter() : nh_("~"), costmap_received_(false), srv_timeout_(3.0) {

        std::string rosplan_kb_name;

        // get required parameters from param server
        nh_.param<int>("srv_timeout", srv_timeout_, 3.0);
        nh_.param<int>("waypoint_count", waypoint_count_, 10);
        nh_.param<std::string>("wp_namespace_input", wp_namespace_input_, "/rosplan_demo_waypoints");
        nh_.param<std::string>("wp_namespace_output", wp_namespace_output_, "/filtered_waypoints");
        nh_.param<std::string>("wp_reference_frame", wp_reference_frame_, "map");
        nh_.param<std::string>("rosplan_kb_name", rosplan_kb_name, "rosplan_knowledge_base");
        nh_.param<std::string>("costmap_topic", costmap_topic_, "costmap_topic");

        // subscriptions of this node, robot odometry and costmap
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(costmap_topic_, 1, &RPRoadmapFilter::costMapCallback, this);

        // publications of this node (for visualisation purposes), waypoints and connectivity information (edges)
        waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz/waypoints", 10, true);

        // services offered by this node, create random wp (create prm), add single wp, remove a waypoint
        filter_waypoint_service_server_ = nh_.advertiseService("filter_waypoints",&RPRoadmapFilter::filterRoadmap, this);

        // services required by this node, to update rosplan KB and to query map from server
        std::stringstream ss;
        ss << "/" << rosplan_kb_name << "/update";
        update_kb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << "/" << rosplan_kb_name << "/update_array";
        update_kb_client_array_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

        ROS_INFO("KCL: (%s) Ready to receive.", ros::this_node::getName().c_str());
    }

    // update the costmap with received information coming from callback (topic subscription)
    void RPRoadmapFilter::costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        cost_map_ = *msg;
        costmap_received_ = true;
    }

    /*------------------------------*/
    /* Loading WPs from param server */
    /*------------------------------*/

    bool RPRoadmapFilter::loadParams() {

        if(!nh_.hasParam(wp_namespace_input_)) {
            ROS_INFO("KCL: (%s) No waypoints found in param server under namespace: %s", ros::this_node::getName().c_str(), wp_namespace_input_.c_str());
            return false;
        }

        // ensure there is costmap data before proceeding (every 2 seconds)
        ros::Rate loop_rate(0.5);
        while(!costmap_received_ && ros::ok()) {
            ROS_WARN("KCL: (%s) Costmap not received, ensure that costmap topic has data (%s)", ros::this_node::getName().c_str(), costmap_topic_.c_str());
            ROS_INFO("KCL: (%s) Checking for costmap data at 0.5 hz...", ros::this_node::getName().c_str());
            loop_rate.sleep();
            // listen to callbacks
            ros::spinOnce();
        }

        // get all waypoints under a namespace
        XmlRpc::XmlRpcValue waypoints;
        std::string wp_reference_frame;
        if(nh_.getParam(wp_namespace_input_, waypoints)){
            if(waypoints.getType() == XmlRpc::XmlRpcValue::TypeStruct){
                for(auto wit=waypoints.begin(); wit!=waypoints.end(); wit++) {
                    if(wit->second.getType() == XmlRpc::XmlRpcValue::TypeString) {
                        ROS_INFO("KCL: (%s) parsing string parameter (wp reference frame)", ros::this_node::getName().c_str());
                        if(wit->first.c_str() == std::string("waypoint_frameid")) {
                            std::string wp_reference_frame = static_cast<std::string>(wit->second);
                            ROS_INFO("KCL: (%s) Setting new waypoint reference frame : %s", ros::this_node::getName().c_str(), wp_reference_frame.c_str());
                        }
                        else {
                            ROS_ERROR("KCL: (%s) Error: Expected waypoint_frameid, received instead: %s", ros::this_node::getName().c_str(), wit->first.c_str());
                            return false;
                        }
                    }
                    else if(wit->second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                        // iterate over waypoint coordinates (x, y , theta)
                        std::vector<double> waypoint;
                        std::string wp_id = "";
                        for (size_t i = 0; i < wit->second.size(); i++) {
                            double coordinate = -1.0;
                            if(wit->second[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                                ROS_DEBUG("Warning: coordinate stored in param server as integer, consider adding .0 to your values");
                                coordinate = static_cast<double>(static_cast<int>(wit->second[i]));
                            }
                            else if(wit->second[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                                ROS_DEBUG("coordinates properly stored as double, all ok");
                                coordinate = static_cast<double>(wit->second[i]);
                            }
                            if(i == 0) {
                                ROS_DEBUG("%s x coordinate : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                                wp_id = wit->first.c_str();
                            }
                            else if(i == 1) {
                                ROS_DEBUG("%s y coordinate : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                            }
                            else if(i == 2) {
                                ROS_DEBUG("%s theta angle : %lf", wit->first.c_str(), coordinate);
                                waypoint.push_back(coordinate);
                            }
                            else {
                                ROS_ERROR("KCL: (%s) Error: waypoint must only consist of 3 coordinates: x, y and theta", ros::this_node::getName().c_str());
                                return false;
                            }
                        }
                        // add symbolic wp to KB and store in memoryf
                        // convert wp to PoseStamped
                        ROS_ASSERT(waypoint.size() == 3);
                        geometry_msgs::PoseStamped wp_as_pose;
                        wp_as_pose.header.frame_id = wp_reference_frame;
                        wp_as_pose.pose.position.x = waypoint.at(0);
                        wp_as_pose.pose.position.y = waypoint.at(1);
                        // convert yaw to quaternion
                        tf2::Quaternion q;
                        q.setRPY(0.0, 0.0, waypoint.at(2));
                        wp_as_pose.pose.orientation.x = q[0];
                        wp_as_pose.pose.orientation.y = q[1];
                        wp_as_pose.pose.orientation.z = q[2];
                        wp_as_pose.pose.orientation.w = q[3];

                        waypoints_.push_back(wp_as_pose);
                    }
                    else {
                        ROS_WARN("KCL (%s) Unrecognized parameter format, allowed formats are: string, list", ros::this_node::getName().c_str());
                    }
                }
            }
            else {
                ROS_ERROR("KCL: (%s) Waypoints are not in list format, failed to load", ros::this_node::getName().c_str());
                return false;
            }
        }
        else{
            ROS_ERROR("KCL: (%s) Parameters exist but still failed to load them", ros::this_node::getName().c_str());
            return false;
        }

        return true;
    }

    /*---------------*/
    /* Filtering PRM */
    /*---------------*/

    bool RPRoadmapFilter::filterRoadmap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

        // check if wps are available in param server, if so, load them in symbolic KB and visualise them
        loadParams();
        ROS_INFO("KCL: (%s) Waypoints loaded", ros::this_node::getName().c_str());

        // calculate total weight
        int w_sum = 0;
        nav_msgs::OccupancyGrid *map = &cost_map_;
        std::vector<int> wp_weight;
        for(int i=0;i<waypoints_.size();i++) {
            // get cell
            int cell_x = (int) (waypoints_[i].pose.position.x/map->info.resolution); 
            int cell_y = (int) (waypoints_[i].pose.position.y/map->info.resolution);
            int index =  cell_x + cell_y*map->info.width;
            w_sum += map->data[index];
            wp_weight.push_back(map->data[index]);
        }

        // sample waypoints
        int count = 0;
        filtered_waypoints_.clear();
        for(int i=0;i<waypoint_count_ && i<waypoints_.size();i++) {
            int sample = rand() % w_sum;
            int counter = 0;
            while(sample > 0 && counter < waypoints_.size()) {
                sample = sample - wp_weight[counter];
                if(sample > 0) counter++;
            }
            filtered_waypoints_.push_back(waypoints_[counter]);
            std::stringstream ss;
            ss << "wp" << count;
            uploadWPToParamServer(ss.str(), waypoints_[counter]);
            count++;
        }

        // visualise
        clearWPGraph();
        pubWPGraph();

        // upload to KB
        updateKB();

        return true;
    }

    void RPRoadmapFilter::uploadWPToParamServer(std::string wp_id, geometry_msgs::PoseStamped waypoint) {

        // get theta from quaternion
        double roll, pitch, yaw;
        tf::Quaternion q(waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::vector<double> pose_as_array;
        pose_as_array.push_back(waypoint.pose.position.x);
        pose_as_array.push_back(waypoint.pose.position.y);
        pose_as_array.push_back(yaw);
        nh_.setParam(wp_namespace_output_ + "/" + wp_id, pose_as_array);
    }

    void RPRoadmapFilter::pubWPGraph() {

        visualization_msgs::MarkerArray marker_array;
        
        // publish nodes as marker array
        int count = 0;
        for (std::vector<geometry_msgs::PoseStamped>::iterator wit=filtered_waypoints_.begin(); wit!=filtered_waypoints_.end(); ++wit) {
            visualization_msgs::Marker node_marker;
            node_marker.header.stamp = ros::Time();
            node_marker.header.frame_id = wp_reference_frame_;
            node_marker.ns = wp_namespace_output_;
            node_marker.type = visualization_msgs::Marker::CUBE;
            node_marker.action = visualization_msgs::Marker::MODIFY;
            node_marker.pose = wit->pose;
            node_marker.scale.x = 0.35f;
            node_marker.scale.y = 0.35f;
            node_marker.scale.z = 0.35f;
            node_marker.color.a = 1.0f;
            node_marker.color.r = 1.0f;
            node_marker.color.g = 0.3f;
            node_marker.color.b = 0.3f;
            node_marker.id = count;
            
            marker_array.markers.push_back(node_marker);
            count++;
        }
        waypoints_pub_.publish(marker_array);
    }

    // clears all waypoints and edges
    void RPRoadmapFilter::clearWPGraph() {

        visualization_msgs::MarkerArray marker_array;
        int count = 0;
        for (std::vector<geometry_msgs::PoseStamped>::iterator wit=filtered_waypoints_.begin(); wit!=filtered_waypoints_.end(); ++wit) {
            visualization_msgs::Marker node_marker;
            node_marker.header.stamp = ros::Time();
            node_marker.header.frame_id = wp_reference_frame_;
            node_marker.ns = wp_namespace_output_;
            node_marker.action = visualization_msgs::Marker::DELETE;
            node_marker.id = count;

            marker_array.markers.push_back(node_marker);
            count++;
        }

        waypoints_pub_.publish( marker_array );
    }

    void RPRoadmapFilter::updateKB() {

        // generate waypoints
        ROS_INFO("KCL: (%s) Updating KB", ros::this_node::getName().c_str());

        // add roadmap to knowledge base
        ROS_INFO("KCL: (%s) Adding knowledge", ros::this_node::getName().c_str());
        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateInstSrv;
        int count = 0;
        std::stringstream ss;
        for (std::vector<geometry_msgs::PoseStamped>::iterator wit=filtered_waypoints_.begin(); wit!=filtered_waypoints_.end(); ++wit) {

            ss.str("");
            ss << "wp" << count;
            count ++;

            // instance
            rosplan_knowledge_msgs::KnowledgeItem item;
            updateInstSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
            item.instance_type = "waypoint";
            item.instance_name = ss.str();
            updateInstSrv.request.knowledge.push_back(item);
        }

        // wait for service existence
        if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
            ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
            return;
        }

        if(!update_kb_client_array_.call(updateInstSrv)) {
            ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
        }
    }

} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_filter");
    KCL_rosplan::RPRoadmapFilter rms;
    ros::spin();
    return 0;
}

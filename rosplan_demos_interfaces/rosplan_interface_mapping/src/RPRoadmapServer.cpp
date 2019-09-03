/**
 * 
 * Copyright [2019] <KCL King's College London>
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
 * 
 */

#include "rosplan_interface_mapping/RPRoadmapServer.h"

namespace KCL_rosplan {

    /*-----------------*/
    /* Struct Waypoint */
    /*-----------------*/

    Waypoint::Waypoint() : wpID("wp_err"), grid_x(0), grid_y(0) {}

    Waypoint::Waypoint(const std::string &id, unsigned int xCoord, unsigned int yCoord, const nav_msgs::MapMetaData& map_meta_data)
            : wpID(id), grid_x(xCoord), grid_y(yCoord)
    {
        occupancy_grid_utils::Cell cell;
        cell.x = grid_x;
        cell.y = grid_y;

        geometry_msgs::Point real_point = occupancy_grid_utils::cellCenter(map_meta_data, cell);
        real_x = real_point.x;
        real_y = real_point.y;
    }

    float Waypoint::getDistance(const Waypoint& other) {
        return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
    }

    void Waypoint::update(const Waypoint& other, float max_casting_range, const nav_msgs::MapMetaData& map_meta_data) {
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

    /*-----------------*/
    /* RPRoadmapServer */
    /*-----------------*/

    /**
     * Constructor
     */
    RPRoadmapServer::RPRoadmapServer() : nh_("~"), costmap_received_(false), srv_timeout_(3.0), occupancy_threshold_(10) {

        std::string rosplan_kb_name;
        std::string odom_topic;
        std::string get_map_srv_name;

        // get required parameters from param server
        nh_.param("use_static_map_", use_static_map_, false);
        nh_.param("update_connectivity", update_connectivity_, true);
        nh_.param("update_waypoints", update_waypoints_, true);
        nh_.param<std::string>("wp_reference_frame", wp_reference_frame_, "map");
        nh_.param<int>("srv_timeout", srv_timeout_, 3.0);
        nh_.param<int>("occupancy_threshold", occupancy_threshold_, 10);
        nh_.param<std::string>("wp_namespace", wp_namespace_, "/rosplan_demo_waypoints");
        nh_.param<std::string>("rosplan_kb_name", rosplan_kb_name, "rosplan_knowledge_base");
        nh_.param<std::string>("costmap_topic", costmap_topic_, "costmap_topic");
        nh_.param<std::string>("odom_topic", odom_topic, "odom_topic");
        nh_.param<std::string>("get_map_srv_name", get_map_srv_name, "get_map_srv_name");

        // subscriptions of this node, robot odometry and costmap
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, &RPRoadmapServer::odomCallback, this);
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(costmap_topic_, 1, &RPRoadmapServer::costMapCallback, this);

        // publications of this node (for visualisation purposes), waypoints and connectivity information (edges)
        waypoints_pub_ = nh_.advertise<visualization_msgs::Marker>("viz/waypoints", 10, true);
        edges_pub_ = nh_.advertise<visualization_msgs::Marker>("viz/edges", 10, true);

        // services offered by this node, create random wp (create prm), add single wp, remove a waypoint
        prm_service_server_ = nh_.advertiseService("create_prm",&RPRoadmapServer::generateRoadmap, this);
        waypoint_service_server_ = nh_.advertiseService("add_waypoint",&RPRoadmapServer::addWaypointSrv, this);
        remove_waypoint_service_server_ = nh_.advertiseService("remove_waypoint",&RPRoadmapServer::removeWaypoint, this);

        load_wp_service_server_ = nh_.advertiseService("load_waypoints",&RPRoadmapServer::loadWaypoints, this);

        // services required by this node, to update rosplan KB and to query map from server
        std::stringstream ss;
        ss << "/" << rosplan_kb_name << "/update";
        update_kb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << "/" << rosplan_kb_name << "/update_array";
        update_kb_client_array_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());
        get_map_client_ = nh_.serviceClient<nav_msgs::GetMap>(get_map_srv_name);

        // check if wps are available in param server, if so, load them in symbolic KB and visualise them
        loadParams();

        ROS_INFO("KCL: (%s) Ready to receive.", ros::this_node::getName().c_str());
    }

    // update the costmap with received information coming from callback (topic subscription)
    void RPRoadmapServer::costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        cost_map_ = *msg;
        costmap_received_ = true;
    }

    // update position of the robot
    void RPRoadmapServer::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        // we assume that the odometry is published in the frame of the base
        base_odom_.header = msg->header;
        base_odom_.pose.position = msg->pose.pose.position;
        base_odom_.pose.orientation = msg->pose.pose.orientation;
    }

    /*------------------------------*/
    /* Loading WP from param server */
    /*------------------------------*/

    bool RPRoadmapServer::loadParams() {

        if(!nh_.hasParam(wp_namespace_)) {
            ROS_INFO("KCL: (%s) No waypoints found in param server under namespace: %s", ros::this_node::getName().c_str(), wp_namespace_.c_str());
            return false;
        }

        ROS_INFO("KCL: (%s) Found waypoints in param server, will load them now into KB", ros::this_node::getName().c_str());

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

        if(nh_.getParam(wp_namespace_, waypoints)){
            if(waypoints.getType() == XmlRpc::XmlRpcValue::TypeStruct){
                for(auto wit=waypoints.begin(); wit!=waypoints.end(); wit++) {
                    if(wit->second.getType() == XmlRpc::XmlRpcValue::TypeString) {
                        ROS_INFO("KCL: (%s) parsing string parameter (wp reference frame)", ros::this_node::getName().c_str());
                        if(wit->first.c_str() == std::string("waypoint_frameid")) {
                            wp_reference_frame_ = static_cast<std::string>(wit->second);
                            ROS_INFO("KCL: (%s) Setting new waypoint reference frame : %s", ros::this_node::getName().c_str(), wp_reference_frame_.c_str());
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
                        wp_as_pose.header.frame_id = wp_reference_frame_;
                        wp_as_pose.pose.position.x = waypoint.at(0);
                        wp_as_pose.pose.position.y = waypoint.at(1);
                        // convert yaw to quaternion
                        tf2::Quaternion q;
                        q.setRPY(0.0, 0.0, waypoint.at(2));
                        wp_as_pose.pose.orientation.x = q[0];
                        wp_as_pose.pose.orientation.y = q[1];
                        wp_as_pose.pose.orientation.z = q[2];
                        wp_as_pose.pose.orientation.w = q[3];
                        double connecting_distance = 8.0;
                        ROS_ASSERT(wp_id.c_str() != std::string(""));
                        addWaypoint(wp_id, wp_as_pose, connecting_distance, false);
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

    /*----------------*/
    /* Generating PRM */
    /*----------------*/

    bool RPRoadmapServer::generateRoadmap(rosplan_interface_mapping::CreatePRM::Request &req, rosplan_interface_mapping::CreatePRM::Response &res) {

        if(update_waypoints_) {
            // clear previous roadmap from knowledge base
            ROS_INFO("KCL: (%s) Cleaning old roadmap", ros::this_node::getName().c_str());

            // clear waypoint instances from KB
            rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
            updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
            updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
            updateSrv.request.knowledge.instance_type = "waypoint";

            // wait for service existence
            if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
                ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
                return false;
            }

            if(!update_kb_client_.call(updateSrv)) {
                ROS_ERROR("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
                return false;
            }
        }

        // read map
        nav_msgs::OccupancyGrid map;
        if(use_static_map_) {
            ROS_INFO("KCL: (%s) Reading in map", ros::this_node::getName().c_str());
            nav_msgs::GetMap mapSrv;

            // check for service existence
            if(!get_map_client_.waitForExistence(ros::Duration(srv_timeout_))) {
                ROS_ERROR("KCL: (%s) Map service not found (%s)", ros::this_node::getName().c_str(), get_map_client_.getService().c_str());
                return false;
            }

            get_map_client_.call(mapSrv);
            map = mapSrv.response.map;
        } else {
            if(!costmap_received_) {
                ROS_ERROR("KCL: (%s) Costmap not received, ensure that costmap topic has data (%s)", ros::this_node::getName().c_str(), costmap_topic_.c_str());
                return false;
            }
            map = cost_map_;
        }

        // generate waypoints
        ROS_INFO("KCL: (%s) Generating roadmap", ros::this_node::getName().c_str());
        createPRM(map, req.nr_waypoints, req.min_distance, req.casting_distance, req.connecting_distance, req.occupancy_threshold, req.total_attempts);

        // add roadmap to knowledge base and scene database
        if(update_waypoints_) {
            ROS_INFO("KCL: (%s) Adding knowledge", ros::this_node::getName().c_str());
            rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateInstSrv;
            for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {

                res.waypoints.push_back(wit->first);

                // instance
                rosplan_knowledge_msgs::KnowledgeItem item;
                updateInstSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
                item.instance_type = "waypoint";
                item.instance_name = wit->first;
                updateInstSrv.request.knowledge.push_back(item);
            }

            // wait for service existence
            if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
                ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
                return false;
            }

            if(!update_kb_client_array_.call(updateInstSrv))
                ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
        }

        if(update_waypoints_ && update_connectivity_) {

            rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
            rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateFuncSrv;
            for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {

                // predicates
                for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {

                    rosplan_knowledge_msgs::KnowledgeItem item;

                    item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                    item.attribute_name = "connected";
                    diagnostic_msgs::KeyValue pairFrom;
                    pairFrom.key = "from";
                    pairFrom.value = wit->first;
                    item.values.push_back(pairFrom);
                    diagnostic_msgs::KeyValue pairTo;
                    pairTo.key = "to";
                    pairTo.value = *nit;
                    item.values.push_back(pairTo);

                    updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                    updatePredSrv.request.knowledge.push_back(item);
                }

                // functions
                for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {

                    rosplan_knowledge_msgs::KnowledgeItem item;

                    item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
                    item.attribute_name = "distance";
                    diagnostic_msgs::KeyValue pairFrom;
                    pairFrom.key = "wp1";
                    pairFrom.value = wit->first;
                    item.values.push_back(pairFrom);
                    diagnostic_msgs::KeyValue pairTo;
                    pairTo.key = "wp2";
                    pairTo.value = *nit;
                    item.values.push_back(pairTo);
                    double dist = sqrt(
                            (wit->second->real_x - waypoints_[*nit]->real_x)*(wit->second->real_x - waypoints_[*nit]->real_x)
                            + (wit->second->real_y - waypoints_[*nit]->real_y)*(wit->second->real_y - waypoints_[*nit]->real_y));
                    item.function_value = dist;

                    updateFuncSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                    updateFuncSrv.request.knowledge.push_back(item);

                }
            }

            // service calls
            if(!update_kb_client_array_.call(updatePredSrv))
                ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
            if(!update_kb_client_array_.call(updateFuncSrv))
                ROS_INFO("KCL: (%s) Failed to call update service.", ros::this_node::getName().c_str());
        }

        // add waypoints to param server
        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {
            //data
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = map.header.frame_id;
            pose.pose.position.x = wit->second->real_x;
            pose.pose.position.y = wit->second->real_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            uploadWPToParamServer(wit->first, pose); // waypoint id, pose
        }

        // publish waypoint graph as markers for visualisation purposes
        pubWPGraph();

        ROS_INFO("KCL: (%s) Done", ros::this_node::getName().c_str());
        return true;
    }

    void RPRoadmapServer::createPRM(nav_msgs::OccupancyGrid &map,
                unsigned int nr_waypoints,
                double min_distance,
                double casting_distance,
                double connecting_distance,
                double occupancy_threshold,
                int total_attempts) {

        // map info
        int width = map.info.width;
        int height = map.info.height;
        double resolution = map.info.resolution; // m per cell

        if(width==0 || height==0) {
            ROS_INFO("KCL: (%s) Empty map", ros::this_node::getName().c_str());
            return;
        }

        // V <-- empty set; E <-- empty set.
        for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
            delete (*ci).second;
        }
        waypoints_.clear();
        edges_.clear();

        // create robot start point
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped start_pose_transformed;
        start_pose.header = base_odom_.header;
        start_pose.pose.position = base_odom_.pose.position;
        start_pose.pose.orientation = base_odom_.pose.orientation;
        try {
            tf_.waitForTransform( base_odom_.header.frame_id, "map", ros::Time::now(), ros::Duration(500) );
            tf_.transformPose("map", start_pose, start_pose_transformed);
        } catch(tf::LookupException& ex) {
            ROS_ERROR("KCL: (%s) Lookup Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        } catch(tf::ConnectivityException& ex) {
            ROS_ERROR("KCL: (%s) Connectivity Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        } catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("KCL: (%s) Extrapolation Error: %s", ros::this_node::getName().c_str(), ex.what());
            return;
        }

        occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, start_pose_transformed.pose.position);
        Waypoint* start_wp = new Waypoint("wp0", start_cell.x, start_cell.y, map.info);
        waypoints_[start_wp->wpID] = start_wp;

        int loop_counter = 0;
        while(waypoints_.size() < nr_waypoints && ++loop_counter < total_attempts) {

            // Sample a random waypoint.
            std::map<std::string, Waypoint*>::iterator item = waypoints_.begin();
            std::advance(item, rand() % waypoints_.size());
            Waypoint* casting_wp = (*item).second;

            // sample collision-free configuration at random
            int x = rand() % width;
            int y = rand() % height;

            std::stringstream ss;
            ss << "wp" << waypoints_.size();
            Waypoint* wp = new Waypoint(ss.str(), x, y, map.info);

            // Move the waypoint closer so it's no further than @ref{casting_distance} away from the casting_wp.
            wp->update(*casting_wp, casting_distance, map.info);

            if(map.data[y*width+x] > occupancy_threshold) {
                delete wp;
                continue;
            }

            // Check whether this waypoint is connected to any of the existing waypoints.
            geometry_msgs::Point p1, p2;
            p1.x = wp->real_x;
            p1.y = wp->real_y;

            // Ignore waypoint that are too close to existing waypoints.
            float min_distance_to_other_wp = std::numeric_limits<float>::max();
            for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                float distance = wp->getDistance(*(*ci).second);
                if (distance < min_distance_to_other_wp)
                    min_distance_to_other_wp = distance;
            }

            if (min_distance_to_other_wp < min_distance) {
                delete wp;
                continue;
            }

            for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
                Waypoint* other_wp = (*ci).second;
                p2.x = other_wp->real_x;
                p2.y = other_wp->real_y;

                if (wp->getDistance(*other_wp) < connecting_distance && canConnect(p1, p2, map, occupancy_threshold)) {
                    wp->neighbours.push_back(other_wp->wpID);
                    other_wp->neighbours.push_back(wp->wpID);
                    Edge e(wp->wpID, other_wp->wpID);
                    edges_.push_back(e);
                }
            }

            if (wp->neighbours.size() > 0) {
                waypoints_[wp->wpID] = wp;
            }
        }
    }

    bool RPRoadmapServer::addWaypointSrv(rosplan_interface_mapping::AddWaypoint::Request &req, rosplan_interface_mapping::AddWaypoint::Response &res) {
        // add wp and store in param server
        return addWaypoint(req.id, req.waypoint, req.connecting_distance, true);
    }

    bool RPRoadmapServer::addWaypoint(std::string id, geometry_msgs::PoseStamped waypoint, float connecting_distance, bool store_in_param_server) {

        ROS_INFO("KCL: (%s) Adding new waypoint", ros::this_node::getName().c_str());

        if(id == "") {
            ROS_ERROR("KCL: (%s) Waypoint id cannot be empty, please name your waypoint", ros::this_node::getName().c_str());
            return false;
        }

        nav_msgs::OccupancyGrid map;
        
        // option 1: get map via service call from map server, srv name is "/static_map"
        if(use_static_map_) {
            ROS_INFO("KCL: (%s) Reading in map", ros::this_node::getName().c_str());
            nav_msgs::GetMap mapSrv;

            // check for service existence
            if(!get_map_client_.waitForExistence(ros::Duration(srv_timeout_))) {
                ROS_ERROR("KCL: (%s) Costmap service not found (%s)", ros::this_node::getName().c_str(), get_map_client_.getService().c_str());
                return false;
            }

            get_map_client_.call(mapSrv);
            map = mapSrv.response.map;
        } else {

            // option 2: get map from topic subcription
            if(!costmap_received_) {
                ROS_ERROR("KCL: (%s) Costmap not received, ensure that costmap topic has data (%s)", ros::this_node::getName().c_str(), costmap_topic_.c_str());
                return false;
            }
            map = cost_map_;
        }

        // at this point map is available, either via service call to map server of via subscription to move base global costmap

        // get properties from received map object
        int width = map.info.width;
        int height = map.info.height;
        double resolution = map.info.resolution; // m per cell

        if(width==0 || height==0) {
            ROS_INFO("KCL: (%s) Empty map", ros::this_node::getName().c_str());
            return false;
        }

        // clear old waypoint
        clearWaypoint(id);

        // add new waypoint
        occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, waypoint.pose.position);
        Waypoint* new_wp = new Waypoint(id, start_cell.x, start_cell.y, map.info);
        waypoints_[new_wp->wpID] = new_wp;

        // connect to neighbours
        geometry_msgs::Point p1, p2;
        p1.x = new_wp->real_x;
        p1.y = new_wp->real_y;

        for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
            Waypoint* other_wp = (*ci).second;
            p2.x = other_wp->real_x;
            p2.y = other_wp->real_y;

            try {
                if (new_wp->getDistance(*other_wp) <= connecting_distance && canConnect(p1, p2, map, occupancy_threshold_)) {
                    new_wp->neighbours.push_back(other_wp->wpID);
                    other_wp->neighbours.push_back(new_wp->wpID);
                    Edge e(new_wp->wpID, other_wp->wpID);
                    edges_.push_back(e);
                }
            }
            catch (const std::exception& e) {
                ROS_WARN("KCL (%s) Failed to connect waypoint, skipping %s", ros::this_node::getName().c_str(), id.c_str());
                ROS_ERROR("KCL: (%s) Error: %s", ros::this_node::getName().c_str(), e.what());
                return false;
            }
        }

        // instance
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";
        updateSrv.request.knowledge.instance_name = new_wp->wpID;

        // wait for service existence
        if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
            ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
            return false;
        }

        if (!update_kb_client_.call(updateSrv)) {
            ROS_ERROR("KCL: (%s) Failed to add a new waypoint instance.", ros::this_node::getName().c_str());
            return false;
        }

        ROS_INFO("KCL: (%s) Process the %lu neighbours of this new waypoint.", ros::this_node::getName().c_str(), new_wp->neighbours.size());

        // predicates
        for (std::vector<std::string>::iterator nit=new_wp->neighbours.begin(); nit!=new_wp->neighbours.end(); ++nit) {
            // connected new->old
            rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
            updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            updatePredSrv.request.knowledge.attribute_name = "connected";
            diagnostic_msgs::KeyValue pairFrom;
            pairFrom.key = "from";
            pairFrom.value = new_wp->wpID;
            updatePredSrv.request.knowledge.values.push_back(pairFrom);
            diagnostic_msgs::KeyValue pairTo;
            pairTo.key = "to";
            pairTo.value = *nit;
            updatePredSrv.request.knowledge.values.push_back(pairTo);
            update_kb_client_.call(updatePredSrv);

            // connected old->new
            updatePredSrv.request.knowledge.values.clear();
            pairFrom.value = *nit;
            updatePredSrv.request.knowledge.values.push_back(pairFrom);
            pairTo.value = new_wp->wpID;
            updatePredSrv.request.knowledge.values.push_back(pairTo);
            update_kb_client_.call(updatePredSrv);
        }

        // functions
        for (std::vector<std::string>::iterator nit=new_wp->neighbours.begin(); nit!=new_wp->neighbours.end(); ++nit) {

            // distance new->old
            rosplan_knowledge_msgs::KnowledgeUpdateService updateFuncSrv;
            updateFuncSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            updateFuncSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
            updateFuncSrv.request.knowledge.attribute_name = "distance";
            diagnostic_msgs::KeyValue pairFrom;
            pairFrom.key = "wp1";
            pairFrom.value = new_wp->wpID;
            updateFuncSrv.request.knowledge.values.push_back(pairFrom);
            diagnostic_msgs::KeyValue pairTo;
            pairTo.key = "wp2";
            pairTo.value = *nit;
            updateFuncSrv.request.knowledge.values.push_back(pairTo);
            double dist = sqrt(
                    (new_wp->real_x - waypoints_[*nit]->real_x)*(new_wp->real_x - waypoints_[*nit]->real_x)
                    + (new_wp->real_y - waypoints_[*nit]->real_y)*(new_wp->real_y - waypoints_[*nit]->real_y));
            updateFuncSrv.request.knowledge.function_value = dist;
            update_kb_client_.call(updateFuncSrv);

            // distance old->new
            updateFuncSrv.request.knowledge.values.clear();
            pairFrom.value = *nit;
            updateFuncSrv.request.knowledge.values.push_back(pairFrom);
            pairTo.value = new_wp->wpID;
            updateFuncSrv.request.knowledge.values.push_back(pairTo);
            updateFuncSrv.request.knowledge.function_value = dist;
            update_kb_client_.call(updateFuncSrv);
        }

        // upload waypoint to param server
        uploadWPToParamServer(id, waypoint);

        // publish wp for visualisation purposes
        pubWPGraph();

        ROS_INFO("KCL: (%s) Finished adding new waypoint", ros::this_node::getName().c_str());

        return true;
    }


    bool RPRoadmapServer::canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, nav_msgs::OccupancyGrid &map, double occupancy_threshold) {

        if(!costmap_received_ && !use_static_map_) {
            ROS_ERROR("KCL: (%s) Costmap not received, ensure that costmap topic has data (%s)", ros::this_node::getName().c_str(), costmap_topic_.c_str());
            return false;
        }

        occupancy_grid_utils::Cell sta = occupancy_grid_utils::pointCell(map.info, w1);
        occupancy_grid_utils::Cell end = occupancy_grid_utils::pointCell(map.info, w2);

        // Check if the turtlebot is going to collide with any known obstacle.
        occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(map.info, w1, w2);

        for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
        {
            const occupancy_grid_utils::Cell& cell = *i;
            // Check if this cell is occupied.
            if (map.data[cell.x + cell.y * map.info.width] > occupancy_threshold) return false;
        }
        return true;
    }

    void RPRoadmapServer::uploadWPToParamServer(std::string wp_id, geometry_msgs::PoseStamped waypoint) {

        // get theta from quaternion
        double roll, pitch, yaw;
        tf::Quaternion q(waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::vector<double> pose_as_array;
        pose_as_array.push_back(waypoint.pose.position.x);
        pose_as_array.push_back(waypoint.pose.position.y);
        pose_as_array.push_back(yaw);
        nh_.setParam(wp_namespace_ + "/" + wp_id, pose_as_array);
    }


    bool RPRoadmapServer::removeWaypoint(rosplan_interface_mapping::RemoveWaypoint::Request &req,
                                        rosplan_interface_mapping::RemoveWaypoint::Response &res) {

        // this function will get executed upon receiving a srv call request to this server node

        // delete waypoint from param server and KB
        clearWaypoint(req.id);

        ROS_INFO("KCL: (%s) Successfully removed waypoint from param server and Knowledge Base.", ros::this_node::getName().c_str());

        // srv call executed succesfully
        return true;
    }

    bool RPRoadmapServer::loadWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        res.success = loadParams();
        return true;
    }

    bool RPRoadmapServer::clearWaypoint(const std::string &name) {

        // erase waypoint from param server
        nh_.deleteParam(wp_namespace_ + "/" + name);

        // remove waypoint instance from ROSPlan KB
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";
        updateSrv.request.knowledge.instance_name = name;

        // wait for service existence
        if(!update_kb_client_.waitForExistence(ros::Duration(srv_timeout_))) {
            ROS_ERROR("KCL: (%s) Update KB service not found (%s)", ros::this_node::getName().c_str(), update_kb_client_.getService().c_str());
            return false;
        }

        if (!update_kb_client_.call(updateSrv)) {
            ROS_INFO("KCL: (%s) Failed to remove old waypoint instance from KB.", ros::this_node::getName().c_str());
        }

        return true;
    }

    void RPRoadmapServer::pubWPGraph() {

        // publish nodes as marker array
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = wp_reference_frame_;
        node_marker.header.stamp = ros::Time();
        node_marker.ns = wp_namespace_;
        node_marker.type = visualization_msgs::Marker::CUBE_LIST;
        node_marker.action = visualization_msgs::Marker::MODIFY;
        node_marker.pose.position.x = 0.0;
        node_marker.pose.position.y = 0.0;
        node_marker.pose.position.z = 0.0;
        node_marker.pose.orientation.x = 0.0;
        node_marker.pose.orientation.y = 0.0;
        node_marker.pose.orientation.z = 0.0;
        node_marker.pose.orientation.w = 1.0;
        node_marker.scale.x = 0.25f;
        node_marker.scale.y = 0.25f;
        node_marker.scale.z = 0.25f;
        node_marker.color.a = 1.0f;
        node_marker.color.r = 1.0f;
        node_marker.color.g = 1.0f;
        node_marker.color.b = 1.0f;
        node_marker.id = 0;

        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {
            geometry_msgs::Point point;
            point.x = wit->second->real_x;
            point.y = wit->second->real_y;
            point.z = 0;
            node_marker.points.push_back(point);
        }
        waypoints_pub_.publish(node_marker);

        // publish edges as marker array
        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = wp_reference_frame_;
        edge_marker.header.stamp = ros::Time();
        edge_marker.ns = wp_namespace_;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::Marker::MODIFY;
        edge_marker.scale.x = 0.02;
        edge_marker.color.a = 1.0;
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 0.3;
        edge_marker.color.b = 1.0;
        node_marker.id = 1;

        for (std::vector<Edge>::iterator eit=edges_.begin(); eit!=edges_.end(); ++eit) {
            geometry_msgs::Point start;
            start.x = waypoints_[eit->start]->real_x;
            start.y = waypoints_[eit->start]->real_y;
            start.z = 0;
            edge_marker.points.push_back(start);

            geometry_msgs::Point end;
            end.x = waypoints_[eit->end]->real_x;
            end.y = waypoints_[eit->end]->real_y;
            end.z = 0;
            edge_marker.points.push_back(end);
        }
        edges_pub_.publish(edge_marker);
    }

    // clears all waypoints and edges
    void RPRoadmapServer::clearWPGraph() {

        visualization_msgs::Marker marker;
        marker.header.frame_id = wp_reference_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = wp_namespace_;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;
        waypoints_pub_.publish( marker );

        //visualization_msgs::Marker marker;
        marker.header.frame_id = wp_reference_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = wp_namespace_;
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::DELETE;
        edges_pub_.publish( marker );
    }
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_server");
    KCL_rosplan::RPRoadmapServer rms;
    ros::spin();
    return 0;
}

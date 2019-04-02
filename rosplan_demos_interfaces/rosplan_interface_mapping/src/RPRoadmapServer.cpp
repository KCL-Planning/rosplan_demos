#include "rosplan_interface_mapping/RPRoadmapServer.h"
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

/* implementation of rosplan_interface_mapping::RPRoadmapServer */
namespace KCL_rosplan {

    /* constructor */
    RPRoadmapServer::RPRoadmapServer() : nh_("~") {

        // config
        std::string dataPath("common/");
        std::string staticMapService("/static_map");
        nh_.param("static_map_service_", static_map_service_, staticMapService);
        nh_.param("use_static_map_", use_static_map_, false);
        nh_.param<std::string>("fixed_frame", fixed_frame_, "map");

        // subscriptions and publications
        ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &RPRoadmapServer::odomCallback, this);
        ros::Subscriber map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, &RPRoadmapServer::costMapCallback, this);
        ros::ServiceServer createPRMService = nh_.advertiseService("/kcl_rosplan/roadmap_server/create_prm", &RPRoadmapServer::generateRoadmap, this);
        ros::ServiceServer addWaypointService = nh_.advertiseService("/kcl_rosplan/roadmap_server/add_waypoint", &RPRoadmapServer::addWaypoint, this);
        ros::ServiceServer removeWaypointService = nh_.advertiseService("/kcl_rosplan/roadmap_server/remove_waypoint", &RPRoadmapServer::removeWaypoint, this);

        // knowledge interface
        std::string kb = "knowledge_base";
        nh_.getParam("knowledge_base", kb);
        std::stringstream ss;
        ss << "/" << kb << "/update";
        update_knowledge_client_ = nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());

        // visualisation
        ss.str("");
        ss << "/" << kb << "/viz/waypoints";
        waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 10, true);
        ss.str("");
        ss << "/" << kb << "/viz/edges";
        edges_pub_ = nh_.advertise<visualization_msgs::Marker>(ss.str(), 10, true);

        // map interface
        map_client_ = nh_.serviceClient<nav_msgs::GetMap>(static_map_service_);

        ROS_INFO("KCL: (RPRoadmapServer) Ready to receive.");
    }

    /*------------------*/
    /* callback methods */
    /*------------------*/

    /* update the costmap */
    void RPRoadmapServer::costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg ) {
        cost_map_ = *msg;
    }

    /* update position of the robot */
    void RPRoadmapServer::odomCallback( const nav_msgs::OdometryConstPtr& msg ) {
        //we assume that the odometry is published in the frame of the base
        base_odom_.header = msg->header;
        base_odom_.pose.position = msg->pose.pose.position;
        base_odom_.pose.orientation = msg->pose.pose.orientation;
    }

    /*-----------*/
    /* build PRM */
    /*-----------*/

    /**
     * Check if two waypoints can be connected without colliding with any known scenery. The line should not
     * come closer than @ref{min_width} than any known obstacle.
     * @param w1 The first waypoint.
     * @param w2 The second waypoint.
     * @param threshold A value between -1 and 255 above which a cell is considered to be occupied.
     * @return True if the waypoints can be connected, false otherwise.
     */
    bool RPRoadmapServer::canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int threshold) const {

        // Check if the turtlebot is going to collide with any known obstacle.
        occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(cost_map_.info, w1, w2);

        for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
        {
            const occupancy_grid_utils::Cell& cell = *i;

            // Check if this cell is occupied.
            if (cost_map_.data[cell.x + cell.y * cost_map_.info.width] > threshold)
            {
                return false;
            }
        }
        return true;
    }

    void RPRoadmapServer::publishWaypointMarkerArray() // TODO: remove copy pasted function
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

    void RPRoadmapServer::publishEdgeMarkerArray()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "mission_edges";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.3;
        marker.color.b = 1.0;
        for (std::vector<Edge>::iterator eit=edges_.begin(); eit!=edges_.end(); ++eit) {
            geometry_msgs::Point start;
            start.x = waypoints_[eit->start]->real_x;
            start.y = waypoints_[eit->start]->real_y;
            start.z = 0;
            marker.points.push_back(start);

            geometry_msgs::Point end;
            end.x = waypoints_[eit->end]->real_x;
            end.y = waypoints_[eit->end]->real_y;
            end.z = 0;
            marker.points.push_back(end);

        }
        edges_pub_.publish( marker );
    }

    /* clears all waypoints and edges */
    void RPRoadmapServer::clearMarkerArrays() {

        visualization_msgs::MarkerArray marker_array;
        size_t counter = 0;
        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "mission_waypoint";
            marker.id = counter; counter++;
            marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(marker);
        }
        waypoints_pub_.publish( marker_array );

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "mission_edges";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::DELETE;
        edges_pub_.publish( marker );
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
        nh_.setParam("/rosplan_demo_waypoints/" + wp_id, pose_as_array);
    }

    /**
     * Generates waypoints and stores them in the parameter server
     */
    bool RPRoadmapServer::generateRoadmap(rosplan_interface_mapping::CreatePRM::Request &req, rosplan_interface_mapping::CreatePRM::Response &res) {

        // clear previous roadmap from knowledge base
        ROS_INFO("KCL: (RPRoadmapServer) Cleaning old roadmap");
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";
        update_knowledge_client_.call(updateSrv);

        // clear from visualization
        clearMarkerArrays();

        // read map
        nav_msgs::OccupancyGrid map;
        if(use_static_map_) {
            ROS_INFO("KCL: (RPRoadmapServer) Reading in map");
            nav_msgs::GetMap mapSrv;
            map_client_.call(mapSrv);
            map = mapSrv.response.map;
        } else {
            map = cost_map_;
        }

        // generate waypoints
        ROS_INFO("KCL: (RPRoadmapServer) Generating roadmap");
        /*
         * nr_waypoints, number of waypoints to generate before function terminates.
         * min_distance, the minimum distance allowed between any pair of waypoints.
         * casting_distance, the maximum distance a waypoint can be cast.
         * connecting_distance, the maximum distance that can exists between waypoints for them to be connected.
         * occupancy_threshold, a number between 0 and 255; determines above which value a cell is considered occupied.
         */
        createPRM(map, req.nr_waypoints, req.min_distance, req.casting_distance, req.connecting_distance, req.occupancy_threshold, req.total_attempts);

        // publish visualization
        publishWaypointMarkerArray();
        publishEdgeMarkerArray();

        // add roadmap to knowledge base and scene database
        ROS_INFO("KCL: (RPRoadmapServer) Adding knowledge");
        for (std::map<std::string, Waypoint*>::iterator wit=waypoints_.begin(); wit!=waypoints_.end(); ++wit) {

            // instance
            rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
            updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
            updateSrv.request.knowledge.instance_type = "waypoint";
            updateSrv.request.knowledge.instance_name = wit->first;
            if(!update_knowledge_client_.call(updateSrv))
                ROS_INFO("KCL: (RPRoadmapServer) Failed to call update service.");

            res.waypoints.push_back(wit->first);

            // predicates
            for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {
                rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
                updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                updatePredSrv.request.knowledge.attribute_name = "connected";
                diagnostic_msgs::KeyValue pairFrom;
                pairFrom.key = "from";
                pairFrom.value = wit->first;
                updatePredSrv.request.knowledge.values.push_back(pairFrom);
                diagnostic_msgs::KeyValue pairTo;
                pairTo.key = "to";
                pairTo.value = *nit;
                updatePredSrv.request.knowledge.values.push_back(pairTo);
                if(!update_knowledge_client_.call(updatePredSrv))
                    ROS_INFO("KCL: (RPRoadmapServer) Failed to call update service.");
            }

            // functions
            for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {
                rosplan_knowledge_msgs::KnowledgeUpdateService updateFuncSrv;
                updateFuncSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                updateFuncSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
                updateFuncSrv.request.knowledge.attribute_name = "distance";
                diagnostic_msgs::KeyValue pairFrom;
                pairFrom.key = "wp1";
                pairFrom.value = wit->first;
                updateFuncSrv.request.knowledge.values.push_back(pairFrom);
                diagnostic_msgs::KeyValue pairTo;
                pairTo.key = "wp2";
                pairTo.value = *nit;
                updateFuncSrv.request.knowledge.values.push_back(pairTo);
                double dist = sqrt(
                        (wit->second->real_x - waypoints_[*nit]->real_x)*(wit->second->real_x - waypoints_[*nit]->real_x)
                        + (wit->second->real_y - waypoints_[*nit]->real_y)*(wit->second->real_y - waypoints_[*nit]->real_y));
                updateFuncSrv.request.knowledge.function_value = dist;
                if(!update_knowledge_client_.call(updateFuncSrv))
                    ROS_INFO("KCL: (RPRoadmapServer) Failed to call update service.");
            }

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

            // add waypoint to param server
            uploadWPToParamServer(wit->first, pose); // waypoint id, pose
        }

        // robot start position
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        updateSrv.request.knowledge.attribute_name = "robot_at";
        diagnostic_msgs::KeyValue pair1, pair2;
        pair1.key = "v";
        pair1.value = "kenny";
        updateSrv.request.knowledge.values.push_back(pair1);
        pair2.key = "wp";
        pair2.value = "wp0";
        updateSrv.request.knowledge.values.push_back(pair2);
        update_knowledge_client_.call(updateSrv);

        ROS_INFO("KCL: (RPRoadmapServer) Done");
        return true;
    }

    /*
     * Input:
     *     nr_waypoints, number of waypoints to generate before function terminates.
     *     min_distance, the minimum distance allowed between any pair of waypoints.
     *     casting_distance, the maximum distance a waypoint can be cast.
     *     connecting_distance, the maximum distance that can exists between waypoints for them to be connected.
     *     occupancy_threshold, a number between 0 and 255; determines above which value a cell is considered occupied.
     * Output: A roadmap G = (V, E)
     */
    void RPRoadmapServer::createPRM(nav_msgs::OccupancyGrid map, unsigned int nr_waypoints, double min_distance, double casting_distance, double connecting_distance, int occupancy_threshold, int total_attempts) {

        // map info
        int width = map.info.width;
        int height = map.info.height;
        double resolution = map.info.resolution; // m per cell

        if(width==0 || height==0) {
            ROS_INFO("KCL: (RPRoadmapServer) Empty map");
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
            tf_.waitForTransform( base_odom_.header.frame_id, "map", ros::Time::now(), ros::Duration( 500 ) );
            tf_.transformPose( "map", start_pose,  start_pose_transformed);
        } catch(tf::LookupException& ex) {
            ROS_ERROR("Lookup Error: %s\n", ex.what());
            return;
        } catch(tf::ConnectivityException& ex) {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return;
        } catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
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

                if (wp->getDistance(*other_wp) < connecting_distance && canConnect(p1, p2, occupancy_threshold)) {
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

    /**
        * Connects a new waypoint and stores it in the knowledge base and scene database
        */
    bool RPRoadmapServer::addWaypoint(rosplan_interface_mapping::AddWaypoint::Request &req, rosplan_interface_mapping::AddWaypoint::Response &res) {

        ROS_INFO("KCL: (RPRoadmapServer) Adding new waypoint");

        // read map
        nav_msgs::OccupancyGrid map;
        if(use_static_map_) {
            ROS_INFO("KCL: (RPRoadmapServer) Reading in map");
            nav_msgs::GetMap mapSrv;
            map_client_.call(mapSrv);
            map = mapSrv.response.map;
        } else {
            map = cost_map_;
        }

        // map info
        int width = map.info.width;
        int height = map.info.height;
        double resolution = map.info.resolution; // m per cell

        if(width==0 || height==0) {
            ROS_INFO("KCL: (RPRoadmapServer) Empty map");
            return false;
        }

        // clear old waypoint
        clearWaypoint(req.id);

        // add new waypoint
        occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, req.waypoint.pose.position);
        Waypoint* new_wp = new Waypoint(req.id, start_cell.x, start_cell.y, map.info);
        waypoints_[new_wp->wpID] = new_wp;

        // connect to neighbours
        geometry_msgs::Point p1, p2;
        p1.x = new_wp->real_x;
        p1.y = new_wp->real_y;

        for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci) {
            Waypoint* other_wp = (*ci).second;
            p2.x = other_wp->real_x;
            p2.y = other_wp->real_y;

            std::cout << "Try to connect " << new_wp->wpID << " to " << other_wp->wpID << "." << std::endl;

            if (new_wp->getDistance(*other_wp) < req.connecting_distance && canConnect(p1, p2, req.occupancy_threshold)) {
                new_wp->neighbours.push_back(other_wp->wpID);
                other_wp->neighbours.push_back(new_wp->wpID);
                Edge e(new_wp->wpID, other_wp->wpID);
                edges_.push_back(e);
            } else {
                std::cout << "Do not connect these waypoints because: ";
                if (new_wp->getDistance(*other_wp) < req.connecting_distance) {
                    std::cout << "collision detected." << std::endl;
                } else {
                    std::cout << "the distance between them is too large. " << new_wp->getDistance(*other_wp) << ">= " <<  req.connecting_distance << "." << std::endl;
                }
            }
        }


        // instance
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";
        updateSrv.request.knowledge.instance_name = new_wp->wpID;
        if (!update_knowledge_client_.call(updateSrv)) {
            ROS_ERROR("Failed to add a new waypoint instance.");
            return false;
        }

        // publish visualization
        publishWaypointMarkerArray();
        publishEdgeMarkerArray();

        ROS_INFO("Process the %lu neighbours of this new waypoint.", new_wp->neighbours.size());

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
            update_knowledge_client_.call(updatePredSrv);

            // connected old->new
            updatePredSrv.request.knowledge.values.clear();
            pairFrom.value = *nit;
            updatePredSrv.request.knowledge.values.push_back(pairFrom);
            pairTo.value = new_wp->wpID;
            updatePredSrv.request.knowledge.values.push_back(pairTo);
            update_knowledge_client_.call(updatePredSrv);
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
            update_knowledge_client_.call(updateFuncSrv);

            // distance old->new
            updateFuncSrv.request.knowledge.values.clear();
            pairFrom.value = *nit;
            updateFuncSrv.request.knowledge.values.push_back(pairFrom);
            pairTo.value = new_wp->wpID;
            updateFuncSrv.request.knowledge.values.push_back(pairTo);
            updateFuncSrv.request.knowledge.function_value = dist;
            update_knowledge_client_.call(updateFuncSrv);
        }

        // upload wp to param server
        uploadWPToParamServer(req.id, req.waypoint);

        ROS_INFO("KCL: (RPRoadmapServer) Finished adding new waypoint");

        return true;
    }

    bool RPRoadmapServer::removeWaypoint(rosplan_interface_mapping::RemoveWaypoint::Request &req,
                                        rosplan_interface_mapping::RemoveWaypoint::Response &res) {

        if ( clearWaypoint(req.id) ) {
            // publish visualization
            publishWaypointMarkerArray();
            publishEdgeMarkerArray();
        }
        return true;
    }

    bool RPRoadmapServer::clearWaypoint(const std::string &name) {

        if ( db_name_map_.count( name ) == 0 ) {
            return false;
        }

        // erase waypoint from param server
        nh_.deleteParam("/rosplan_demo_waypoints/" + name);

        std::vector<Edge>::iterator eit = edges_.begin();
        while( eit != edges_.end() ) {
            if ( name == eit->start || name == eit->end ) {
                eit = edges_.erase( eit );
            } else {
                eit++;
            }
        }
        // remove instance
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        updateSrv.request.knowledge.instance_type = "waypoint";
        updateSrv.request.knowledge.instance_name = name;
        if (!update_knowledge_client_.call(updateSrv)) {
            ROS_INFO("Failed to remove old waypoint instance.");
        }

        return true;

    }

} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_roadmap_server");
    KCL_rosplan::RPRoadmapServer rms;
    ros::spin();
    return 0;
}

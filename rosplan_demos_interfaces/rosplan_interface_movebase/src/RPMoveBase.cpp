#include "rosplan_interface_movebase/RPMoveBase.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

    /* constructor */
    RPMoveBase::RPMoveBase(ros::NodeHandle &nh, std::string &actionserver)
     : action_client(actionserver, true) {

        // costmap client
        clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

        // get waypoints reference frame from param server
        nh.param<std::string>("waypoint_frameid", waypoint_frameid_, "map");

        // remove
        geometry_msgs::PoseStamped pose1;
        std::string wpid = "wp1";
        wpIDtoPoseStamped(wpid, pose1);
    }

    bool RPMoveBase::wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped &result)
    {
        // input: waypoint ID, output: by reference it will return the wp coordinates, return: true if
        // waypoint ID was found on parameter server, false if not found

        // TODO: query pose from param server
        // also make sure we upload those waypoints on the server side

        ros::NodeHandle nh;
        std::vector<double> wp;
        if(nh.hasParam("/rosplan_demo_waypoints/" + wpID)) {
            if(nh.getParam("/rosplan_demo_waypoints/" + wpID, wp)) {
                if(wp.size() == 3) {
                    result.header.frame_id = waypoint_frameid_;
                    result.pose.position.x = wp[0];
                    result.pose.position.y = wp[1];

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, wp[2]);
                    result.pose.orientation.x = q[0];
                    result.pose.orientation.x = q[1];
                    result.pose.orientation.x = q[2];
                    result.pose.orientation.x = q[3];

                    return true;
                }
                else {
                    ROS_ERROR("wp size must be equal to 3 : (x, y, and theta)");
                    return false;
                }
            }
        }
        else
            return false;
    }

    /* action dispatch callback */
    bool RPMoveBase::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // get waypoint ID from action dispatch
        std::string wpID;
        bool found = false;
        for(size_t i=0; i<msg->parameters.size(); i++) {
            if(0==msg->parameters[i].key.compare("to") or 0==msg->parameters[i].key.compare("w1")) {
                wpID = msg->parameters[i].value;
                found = true;
            }
        }
        if(!found) {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
            return false;
        }

        // get waypoint coordinates from its ID via query to parameter server
        geometry_msgs::PoseStamped pose;
        if(!wpIDtoPoseStamped(wpID, pose)) {
            ROS_ERROR("Waypoint not found in parameter server");
            return false;
        }

        ROS_INFO("KCL: (%s) waiting for move_base action server to start", params.name.c_str());
        action_client.waitForServer();

        // dispatch MoveBase action
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = pose;
        action_client.sendGoal(goal);

        bool finished_before_timeout = action_client.waitForResult();
        if (finished_before_timeout) {

            actionlib::SimpleClientGoalState state = action_client.getState();
            ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

                // publish feedback (achieved)
                return true;

            } else {

                // clear costmaps
                std_srvs::Empty emptySrv;
                clear_costmaps_client.call(emptySrv);

                // publish feedback (failed)
                return false;
            }
        } else {
            // timed out (failed)
            action_client.cancelAllGoals();
            ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
            return false;
        }
    }
} // close namespace

    /*-------------*/
    /* Main method */
    /*-------------*/

    int main(int argc, char **argv) {

        ros::init(argc, argv, "rosplan_interface_movebase");
        ros::NodeHandle nh("~");

        std::string actionserver;
        nh.param("action_server", actionserver, std::string("/move_base"));

        // create PDDL action subscriber
        KCL_rosplan::RPMoveBase rpmb(nh, actionserver);

        rpmb.runActionInterface();

        return 0;
    }

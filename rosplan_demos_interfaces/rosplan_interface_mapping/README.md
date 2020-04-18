# rosplan mapping interface

The mapping interface offers 3 services to interact with it:

    /kcl_rosplan/rosplan_roadmap_server/add_waypoint
    /kcl_rosplan/rosplan_roadmap_server/create_prm
    /kcl_rosplan/rosplan_roadmap_server/remove_waypoint

### add single waypoint srv:

Add a custom waypoint, you need to provide with: wp id, coordinates and reference frame.

Example of how to call this service from terminal:

    rosservice call /kcl_rosplan/rosplan_roadmap_server/add_waypoint "id: 'wp0'
    waypoint:
    header:
        seq: 0
        stamp:
        secs: 0
        nsecs: 0
        frame_id: 'map'
    pose:
        position:
        x: -0.5
        y: -0.5
        z: 0.0
        orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    connecting_distance: 1.0"

The waypoint gets added both to parameter server under /rosplan_demo_waypoints/foo_wp_id_provided_name.

You can confirm it was indeed uploaded to param server by doing:

    rosparam get /rosplan_demo_waypoints/wp0

You should see something like:

    [-0.5, -0.5, 0.0]

Which are the wp coordinates: x, y and theta. The reference frame is used to match it
against the one provided as a node parameter in /rosplan_roadmap_server/wp_reference_frame
when it does not match the system will issue an error and the waypoint will not be added.
This is done to prevent using multiple reference frames. A cleaner solution would be to
transform the waypoint using tf from a user specified

Note that the waypoint includes orientation. While the rviz visualisation shows only nodes and edges (wp connectivity graph)
keep in mind that each point has an orientation associated with it. By visualising the wp as a point.
    
### remove single waypoint

Remove an existing waypoint by id. It will remove from rosplan KB and parameter server.
More in detail from rosplan KB it removes wp instance and related facts (e.g. connected wp0 wp0, etc).

Example of how to call this service from terminal:

    rosservice call /kcl_rosplan/rosplan_roadmap_server/remove_waypoint "id: 'wp0'"

You can confirm that it was indeed deleted from parameter server by doing:

    rosparam get /rosplan_demo_waypoints/wp0

The system should then complain with an error like:

    ERROR: Parameter [/rosplan_demo_waypoints/wp0] is not set

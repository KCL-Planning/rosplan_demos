(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    wp0 - waypoint
)
(:init
    (robot_at kenny wp0)
)
(:goal (and
    (robot_at kenny wp0)
))
)

(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 wp11 wp12 wp13 wp14 wp15 wp16 wp17 wp18 wp19 - waypoint
    d00 d01 d02 d03 d04 d05 d06 d07 d08 - pedestal
    b00 b01 - painting
)
(:init
)
(:goal (and
    (inspected d00)
    (inspected d01)
    (inspected d02)
    (inspected d03)
    (inspected d04)
    (inspected d05)
    (inspected d06)
    (inspected d07)
    (inspected d08)
))
)

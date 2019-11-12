(define (domain turtlebot_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	robot
    exhibit
    painting doughnut - exhibit
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
	(inspected ?e - exhibit)
    (doughnut_visible_from ?from - waypoint ?e - doughnut)
)

(:functions
    (distance ?a ?b - waypoint)
)

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 10) ;; TODO (distance ?from ?to))
	:condition (and
		(at start (robot_at ?v ?from)))
	:effect (and
		(at end (visited ?to))
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to)))
)

;; inspect a doughnut
(:durative-action inspect_doughnut
	:parameters (?v - robot ?from - waypoint ?p - doughnut)
	:duration ( = ?duration 5)
	:condition (and
		(at start (doughnut_visible_from ?from ?p))
		(over all (robot_at ?v ?from))
        )
	:effect (and
		(at end (inspected ?p))
        )
)


)

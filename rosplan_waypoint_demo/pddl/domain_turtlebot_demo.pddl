(define (domain turtlebot_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	robot
    doughnut
    order
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
	(inspected ?e - doughnut)
	(not_inspected ?e - doughnut)
    (doughnut_visible_from ?from - waypoint ?e - doughnut)
    (part_of  ?e - doughnut ?o - order)
)

(:functions
    (distance ?a ?b - waypoint)
    (order_complete ?o - order)
)

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration (distance ?from ?to))
	:condition (and
		(at start (robot_at ?v ?from)))
	:effect (and
		(at end (visited ?to))
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to)))
)

;; inspect a doughnut
(:durative-action inspect_doughnut
	:parameters (?v - robot ?from - waypoint ?p - doughnut ?o - order)
	:duration ( = ?duration 5)
	:condition (and
		(at start (doughnut_visible_from ?from ?p))
    	(at start (not_inspected ?p))
        (at start (part_of ?p ?o))
		(over all (robot_at ?v ?from))
        )
	:effect (and
    	(at start (not (not_inspected ?p)))
		(at end (inspected ?p))
		(at end (increase (order_complete ?o) 1))
        )
)
)

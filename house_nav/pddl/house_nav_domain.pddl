(define (domain house_nav)
(:requirements :strips :universal-preconditions :equality :typing :durative-actions :negative-preconditions :adl)

(:types
  robot
  room corridor zone - location
  object
)


(:predicates
  (robot_at ?r - robot ?z - location)
  (connected ?from ?to - location)
  (zone_in_room ?z - zone ?room - room)
 ; (object_at_room ?o - object ?room - room)
  (object_at ?o - object ?z - location)
  (object_in_robot ?o - object ?r - robot)
)


(:durative-action move_between_rooms
  :parameters (?r - robot ?from ?to - location)
  :duration ( = ?duration 5)
  :condition (and
    (at start(and
      (robot_at ?r ?from)
      ; (robot_available ?r)
    ))
    (over all(and
      (not (= ?from ?to))
      (or
        (connected ?from ?to)
        (connected ?to ?from)
      )
      (forall (?zone - zone)
        (not(robot_at ?r ?zone))         
      )
    ))
  )
  :effect (and
      (at start(and
        (not(robot_at ?r ?from))
        ; (not(robot_available ?r))
      ))
      (at end(and
        (robot_at ?r ?to)
        ; (robot_available ?r)
      ))
  )
  )



;; Function to move between zones that are in the same room
(:durative-action move_between_zones
    :parameters (?r - robot ?from ?to - zone ?room - room)
    :duration (= ?duration 2)
    :condition (and 
        (at start (and 
          (robot_at ?r ?from)
          (not (= ?from ?to))
         ; (robot_available ?r) 
        ))
        (over all (and 
          (zone_in_room ?from ?room)
          (zone_in_room ?to ?room)
          (robot_at ?r ?room)
        ))
    )
    :effect (and 
        (at start (and 
          (not(robot_at ?r ?from))
          ;(not(robot_available ?r))
        ))
        (at end (and 
          (robot_at ?r ?to)
         ; (robot_available ?r)
        ))
    )
)


;; Function to leave a zone and stay only in the room
(:durative-action leave_zone
    :parameters (?r - robot ?from - zone ?to - room)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
          ;(robot_available ?r)
          (robot_at ?r ?from)
        ))
        (over all (and 
          (zone_in_room ?from ?to)
          (robot_at ?r ?to)
        ))
    )
    :effect (and 
        (at start (and 
         ; (not(robot_available ?r))
          (not(robot_at ?r ?from))
        ))
        ; (at end (robot_available ?r)
        ; )
    )
)


;; Function to cross enter to a zone and be in room and zone
(:durative-action enter_zone
    :parameters (?r - robot ?from - room ?to - zone)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
          ;(robot_available ?r)
          (forall (?zone - zone)
            (forall (?zone - zone)
            (not(robot_at ?r ?zone))
            )
          )
        ))
        (over all (and 
          (zone_in_room ?to ?from)
          (robot_at ?r ?from)
        ))
    )
    :effect (and 
        (at start (and 
         ; (not(robot_available ?r))
        ))
        (at end (and 
        ;  (robot_available ?r)
          (robot_at ?r ?to)
        ))
    )
)


;; Function that let the robot take an object if is in the room
(:durative-action take_object
    :parameters (?r - robot ?o - object ?room - location)
    :duration (= ?duration 2)
    :condition (and 
        (at start (and 
          (object_at ?o ?room)
          ;(robot_available ?r)
        ))
        (over all (robot_at ?r ?room)
        )
    )
    :effect (and 
        (at start (and 
          (not(object_at ?o ?room))
          ;(not (robot_available ?r))
        ))
        (at end (and 
          (object_in_robot ?o ?r)
         ; (robot_available ?r)
        ))
    )
)



;; Function that let the robot leave an object if is in the room
(:durative-action leave_object
    :parameters (?r - robot ?o - object ?room - location)
    :duration (= ?duration 2)
    :condition (and 
        (at start (and 
          (object_in_robot ?o ?r)
         ; (robot_available ?r)
        ))
        (over all (robot_at ?r ?room)
        )
    )
    :effect (and 
        (at start (and 
         ; (not(robot_available ?r))
          (not(object_in_robot ?o ?r))
        ))
        (at end (and
          (object_at ?o ?room) 
         ; (robot_available ?r)
        ))
    )
)

)

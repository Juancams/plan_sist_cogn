(define (domain cognitive_arch)
(:requirements :strips :universal-preconditions :typing :durative-actions)

(:types
  location
  robot
)


(:predicates
  (robot_at ?r - robot ?l - location)
  (explored ?l - location)
  (connected ?from ?to - location)
)

;; Function to move
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration ( = ?duration 5)
  :condition (and
    (at start(and
      (robot_at ?r ?from)
      ; (robot_available ?r)
    ))
    (over all(and
      (connected ?from ?to)
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


(:durative-action explore
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
      (at start(and(
        robot_at ?r ?l))
      )
    )
    :effect (and
      (at end(and(
        explored ?l))
      )
    )
)

)

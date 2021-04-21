(define (domain cognitive_arch)
(:requirements :strips :universal-preconditions :typing :durative-actions)

(:types
  room corridor zone - location
  robot
)


(:predicates
  (robot_at ?r - robot ?l - location)
  (explored ?l - location)
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

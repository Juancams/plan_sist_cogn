(define (domain treasures)
(:requirements :strips :typing :universal-preconditions :equality :negative-preconditions :disjunctive-preconditions)
(:types
  robot island treasure
)

(:predicates 
  (treasureInRobot ?t - treasure ?r - robot)
  (robotAt ?r - robot ?i - island)
  (treasureAt ?t - treasure ?i - island)
  (is_route ?from - island ?to - island)
  (robotFinished ?r - robot)
)

(:action sail
  :parameters (?r - robot ?from ?to - island)
  :precondition 
    (and 
      (or
        (is_route ?from ?to)
        (is_route ?to ?from)
      )
      (robotAt ?r ?from)
      (not (= ?from ?to))
    )
  :effect 
    (and 
      (not (robotAt ?r ?from))
      (robotAt ?r ?to)
    )
)

(:action take
    :parameters (?r - robot ?t - treasure ?i - island)
    :precondition (and 
      (not (treasureInRobot ?t ?r))
      (robotAt ?r ?i)
      (treasureAt ?t ?i)
    )
    :effect (and 
      (treasureInRobot ?t ?r)
      (not (treasureAt ?t ?i))
    )
)

(:action all_treasures
    :parameters (?r - robot)
    :precondition (and 
      (forall (?t - treasure) (treasureInRobot ?t ?r))
    )
    :effect (and 
      (robotFinished ?r)
    )
)
)


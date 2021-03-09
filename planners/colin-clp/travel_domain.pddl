(define (domain travel)

(:requirements :strips :fluents :durative-actions :typing :equality)

(:types
  vehicle
  city
)

(:predicates
  (car_at ?v - vehicle ?c - city)
  (is_road ?from ?to - city)
)

(:functions
  (distance ?from ?to - city)
  (speed ?v - vehicle)
  (distance_driven)
)

(:durative-action drive
    :parameters (?v - vehicle ?from ?to - city)
    :duration (= ?duration (/ (distance ?from ?to) (speed ?v)))
    :condition (and 
        (at start (and
          (car_at ?v ?from)
        ))
        (over all (and
          (is_road ?from ?to)
        ))
    )
    :effect (and 
        (at start (and 
          (not (car_at ?v ?from))
        ))
        (at end (and 
          (car_at ?v ?to)
          (increase (distance_driven) (distance ?from ?to))
        ))
    )
)
)
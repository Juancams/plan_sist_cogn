(define (problem travel_problem1) (:domain travel)
(:objects
    minicooper - vehicle
    fuenlabrada leganes mostoles alcorcon madrid - city
)

(:init
    (car_at minicooper fuenlabrada)
    (= (speed minicooper) 100)

    (is_road fuenlabrada mostoles)  (is_road mostoles fuenlabrada)
    (is_road fuenlabrada leganes)   (is_road leganes fuenlabrada)
    (is_road alcorcon mostoles)     (is_road mostoles alcorcon)
    (is_road alcorcon madrid)       (is_road madrid alcorcon)
    (is_road leganes madrid)        (is_road madrid leganes)
    (is_road alcorcon leganes)      (is_road leganes alcorcon)

    (= (distance fuenlabrada mostoles) 10)
    (= (distance mostoles fuenlabrada) 10)

    (= (distance fuenlabrada leganes) 10)
    (= (distance leganes fuenlabrada) 10)

    (= (distance alcorcon mostoles) 5)
    (= (distance mostoles alcorcon) 5)

    (= (distance alcorcon leganes) 10)
    (= (distance leganes alcorcon) 10)

    (= (distance alcorcon madrid) 15)
    (= (distance madrid alcorcon) 15)

    (= (distance leganes madrid) 15)
    (= (distance madrid leganes) 15)

    (= (distance_driven) 0)
)

(:goal (and
    (car_at minicooper madrid)
))

(:metric minimize (distance_driven))
)

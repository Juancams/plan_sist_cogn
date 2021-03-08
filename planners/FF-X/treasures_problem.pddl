(define (problem treasures1)
(:domain treasures)
(:objects 
  robot - robot 
  Formentera Ibiza Mallorca Menorca Cabrera - island
  Money Jewelry - treasure
)
(:init 
  (treasureAt Money Ibiza)
  (treasureAt Jewelry Cabrera)
  (robotAt robot Formentera)

  (is_route Formentera Ibiza)
  (is_route Ibiza Mallorca)
  (is_route Mallorca Menorca)
  (is_route Menorca Cabrera)
  (is_route Cabrera Formentera)
)

(:goal (and
  (robotFinished robot)
  )
)
)

;; Copyright 2021 The Rebooters
;;
;; Licensed under the Apache License, Version 2.0 (the "License");
;; you may not use this file except in compliance with the License.
;; You may obtain a copy of the License at
;;
;;     http://www.apache.org/licenses/LICENSE-2.0
;;
;; Unless required by applicable law or agreed to in writing, software
;; distributed under the License is distributed on an "AS IS" BASIS,
;; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
;; See the License for the specific language governing permissions and
;; limitations under the License.

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
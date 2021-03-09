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

(define (domain treasures)
(:requirements :strips :typing :universal-preconditions :equality :negative-preconditions :disjunctive-preconditions)
(:types
  robot island treasure
)

;; Example for FF-X planner with universal preconditions,
;; negative preconditions and disjunctive preconditions

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


# MIPS-XXL
## Description

MIPS-XXL is a planning software which supports PDDL Level 3. The ‘XXL’ in the name is due to the fact that it supports exploration of state spaces that are much larger than the available RAM. It implements External Memory Planning Algorithms that utilize hard-disk in an efficient manner. 
This planner uses hill-climbing search to solve problems.

## Installation

You can download mips-xxl executable [here](https://github.com/tvaquero/itsimple/blob/master/myPlanners/mips-xxl)

## How to use?

```console
./mips-xxl -o domain.pddl -f problem.pddl
```

You can include more options:

```
OPTIONS     DESCRIPTIONS

-p <str>    path for operator and fact file
-o <str>    operator file name
-f <str>    fact file name

-E          don't do enforced hill-climbing try before bestfirst

 - External - 
-X          do external enforced hill-climbing
-B          do external breadth-first search
-k <num>    beam size for pruning; valid for both Ex-EHC and Ex-BFS
-M          do external merge
---------------
-g <num>    set weight w_g in w_g*g(s) + w_h*h(s) [preset: 0]
-h <num>    set weight w_h in w_g*g(s) + w_h*h(s) [preset: 0]

-O          switch on optimization expression (default is plan length)
-G          output normal task
-A          output ADL task instead of STRIPS
```

## Support

- [x] :strips
- [x] :typing
- [x] :disjunctive-preconditions
- [x] :equality 
- [x] :existential-preconditions 
- [x] :universal-preconditions 
- [x] :quantified-preconditions 
- [x] :conditional-effects 
- [ ] :action-expansions 
- [ ] :foreach-expansions 
- [ ] :dag-expansions 
- [ ] :domain-axioms 
- [ ] :subgoal-through-axioms 
- [ ] :safety-constraints 
- [ ] :expression-evaluation 
- [x] :fluents 
- [ ] :open-world 
- [ ] :true-negation 
- [x] :adl 
- [ ] :ucpop 
- [ ] :numeric-fluents 
- [x] :durative-actions 
- [ ] :continuous-effects 
- [x] :negative-preconditions
- [ ] :derived-predicates
- [x] :timed-initial-literals
- [ ] :constraints
- [x] :preferences
- [ ] :action-costs
- [ ] :goal-utilities

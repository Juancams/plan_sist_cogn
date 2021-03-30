# FF-X
## Description

Fast forward is a forward chaining heuristic state space planner. It ignores the delete list of all operators, extracting an explicit solution by using a GRAPHPLAN-style algorithm. 
The number of actions in the relaxed solutions us used as a goal distance estimate, with a hill-climbing local search, which uses breadth first search to find a successor.

## Installation

You can download and install FF-X visiting its [homepage](http://fai.cs.uni-saarland.de/hoffmann/ff.html)

## How to use?

```console
./ff -o domain.pddl -f problem.pddl
```

You can include more options:

```
OPTIONS   DESCRIPTIONS

-p <str>    path for operator and fact file
-o <str>    operator file name
-f <str>    fact file name

-i <num>    run-time information level( preset: 1 )
      0     only times
      1     problem name, planning process infos
    101     parsed problem data
    102     cleaned up ADL problem
    103     collected string tables
    104     encoded domain
    105     predicates inertia info
    106     splitted initial state
    107     domain with Wff s normalized
    108     domain with NOT conds translated
    109     splitted domain
    110     cleaned up easy domain
    111     unaries encoded easy domain
    112     effects multiplied easy domain
    113     inertia removed easy domain
    114     easy action templates
    115     cleaned up hard domain representation
    116     mixed hard domain representation
    117     final hard domain representation
    118     reachability analysis results
    119     facts selected as relevant
    120     final domain and problem representations
    121     connectivity graph
    122     fixpoint result on each evaluated state
    123     1P extracted on each evaluated state
    124     H set collected for each evaluated state
    125     False sets of goals <GAM>
    126     detected ordering constraints leq_h <GAM>
    127     the Goal Agenda <GAM>

-d <num>    switch on debugging
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
- [ ] :fluents 
- [ ] :open-world 
- [ ] :true-negation 
- [x] :adl 
- [ ] :ucpop 
- [ ] :numeric-fluents 
- [ ] :durative-actions 
- [ ] :continuous-effects 
- [x] :negative-preconditions
- [x] :derived-predicates
- [ ] :timed-initial-literals
- [ ] :constraints
- [ ] :preferences
- [ ] :action-costs
- [ ] :goal-utilities

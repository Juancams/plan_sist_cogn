# METRIC-FF
## Description
```
Version of FF that can plan with numerical state variables and effects.
MetricFF considers the numeric conditions of the actions in
the heuristic estimation, but ignores the decreasing effects. 
Regarding to, Metric-FF expands the same planning
graph (the traditional one, in which a level of propositions
is followed by a level with all applicable actions given these
propositions) whether when a metric expression is defined or
not.
```
## How to use?

```console
./metric_ff -o domain.pddl -f problem.pddl
```
##Usage of metric-ff:
```
OPTIONS   DESCRIPTIONS

-p <str>    path for operator and fact file
-o <str>    operator file name
-f <str>    fact file name

-E          don't do enforced hill-climbing try before bestfirst

-g <num>    set weight w_g in w_g*g(s) + w_h*h(s) [preset: 0]
-h <num>    set weight w_h in w_g*g(s) + w_h*h(s) [preset: 0]

-O          switch on optimization expression (default is plan length)
```
## Support

- [x] :strips
- [x] :typing
- [ ] :disjunctive-preconditions
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
- [ ] :adl 
- [ ] :ucpop 
- [x] :numeric-fluents 
- [ ] :durative-actions 
- [ ] :continuous-effects 
- [ ] :negative-preconditions
- [ ] :derived-predicates
- [ ] :timed-initial-literals
- [ ] :constraints
- [ ] :preferences
- [ ] :action-costs
- [ ] :goal-utilities

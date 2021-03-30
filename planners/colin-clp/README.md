# COLIN-CLP
## Description

Colin is a forward-chaining heuristic search planner. In addition to the full temporal semantics of PDDL, this planner is able to reasoning with COntinuous LINear numeric change.  Combines FF-style forward chaining search with the use of a linear program (which lets reduce the range of actions)

## Installation

You can download colin-clp executable [here](https://github.com/tvaquero/itsimple/blob/master/myPlanners/colin-clp)

## How to use?

```console
./colin domain.pddl problem.pddl
```

You can include more options:

```
Options are: 

	-citation	Display citation to relevant conference paper (ICAPS 2010);
	-b		Disable best-first search - if EHC fails, abort;
	-E		Skip EHC: go straight to best-first search;
	-e		Use standard EHC instead of steepest descent;
	-h		Disable helpful-action pruning;
	-k		Disable compression-safe action detection;
	-M		Disable the tie-breaking in search that favours plans with shorter makespans;
	-F		Full FF helpful actions (rather than just those in the RP applicable in the current state);
	-r		Read in a plan instead of planning;
	-T		Rather than building a partial order, build a total-order
	-v<n>		Verbose to degree n (n defaults to 1 if not specified).
	-L<n>		LP verbose to degree n (n defaults to 1 if not specified).
```
## Support

- [x] :strips
- [x] :typing
- [ ] :disjunctive-preconditions
- [x] :equality 
- [ ] :existential-preconditions 
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
- [x] :numeric-fluents 
- [x] :durative-actions 
- [x] :continuous-effects 
- [x] :negative-preconditions
- [ ] :derived-predicates
- [ ] :timed-initial-literals
- [ ] :constraints
- [ ] :preferences
- [ ] :action-costs
- [ ] :goal-utilities

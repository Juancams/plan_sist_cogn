# LPG-TD-1.0
## Description

LPG is a fully automated system for solving planning problems specified
using version 2.2 (2004) of the standard planning language PDDL. LPG-td 
is a new version of LPG including several extensions and improvements. 
LPG-td supports Timed Initial Literals and Derived Predicates, the two 
main new features of PDDL2.2.

## How to use?

```console
./lpg_td_1.0 -o domain.pddl -f problem.pddl -speed -noout
```

LPG-td-1.0 settings:

NECESSARY SETTINGS
```
-o <string>              specifies the file of the operators 

-f <string>              specifies the file of (init/goal) facts

-n <number>              specifies the desired number of solutions;
                         alternative options are -speed and -quality

```
OPTIONAL SETTINGS
```
-p <string>              specifies the path for the operator/fact files

-out <string>            specifies the file name for computed plans

-noout                   does not save computed plans

-v off                   switches off verbose mode

-search_steps <number>   specifies the number of steps of the first
                         restart of the local search [default 500]

-restarts <number>       specifies the max number of the restarts [default 9]

-repeats <number>        specifies the maximum number of the repeats [default 5]

-noise <0..1>            specifies the initial noise value of Walkplan [default 0.10]

-static_noise            set the noise value to a fixed static value

-seed <number>           sets the seed of the random number generator

-lowmemory               computes mutex relations between actions at runtime

-cputime <number>        specifies the maximum CPU-time (in seconds) [default 1800]

-cputime_localsearch <number>        specifies the maximum CPU-time for 
                         the local search procedure (in seconds) [default 1200] 

-nobestfirst             switches off best-first search

-onlybestfirst           immediately runs best-first search

-timesteps               sets the plan quality metric as #time-steps
```

## Support

- [x] :strips
- [x] :typing
- [x] :disjunctive-preconditions
- [x] :equality 
- [x] :existential-preconditions 
- [x] :universal-preconditions 
- [x] :quantified-preconditions 
- [ ] :conditional-effects 
- [ ] :action-expansions 
- [ ] :foreach-expansions 
- [ ] :dag-expansions 
- [x] :domain-axioms 
- [ ] :subgoal-through-axioms 
- [ ] :safety-constraints 
- [ ] :expression-evaluation 
- [x] :fluents 
- [ ] :open-world 
- [ ] :true-negation 
- [ ] :adl 
- [ ] :ucpop 
- [ ] :numeric-fluents 
- [x] :durative-actions 
- [ ] :continuous-effects 
- [ ] :negative-preconditions
- [x] :derived-predicates
- [ ] :timed-initial-literals
- [ ] :constraints
- [ ] :preferences
- [ ] :action-costs
- [ ] :goal-utilities

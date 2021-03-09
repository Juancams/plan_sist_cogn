# PLAN-A
## Description
Converts a planning instance into an E-MAJSAT instance, and then draws on techniques from Boolean satisfiability and dynamic programming to solve the E-MAJSAT instance. 
Techniques as backward level reduction, accumulative learning of clauses, and search-space pruning based on multi-valued domain for-mulation are included.
## How to use?

```console
./plan-a -o domain.pddl -f problem.pddl
```

You can include more options:

```
OPTIONS   DESCRIPTIONS

-p -path <str>    path for operator and fact file
-o -domain <str>    operator strips file name
-f -problem <str>    fact strips file name

-cgdomain <str>    operator ADL/strips file name for producing Causal Graph
-cgproblem <str>    fact ADL/strips file name for producing Causal Grapth

-l  <num>   goal layer for CNF

-G <0 or 1> (0) create CNF output or (1) build final solution
-b <str>    CNF output file name
-t <0 or 1> (1) CNF output includes only unary/binary clauses - others ignored
-S <str>    Input Solution File Name (only when -G 1 is used)
-F -out <str>    Final Output Solution File Name (only when -G 1 is used)
-V <str>    Variables File Name - list all variables (only when -G 1 is used)
-londexm <0 or 1>  (0) just use mutex, (1) use londex
-C          CNF formula output (preset: -1); at layer <-l>
      0     none
      1     action-based
      2     gp-style action-based
      3     gp-based
      4     thin gp-based
```
## Support

- [x] :strips
- [x] :typing
- [ ] :disjunctive-preconditions
- [ ] :equality 
- [ ] :existential-preconditions 
- [ ] :universal-preconditions 
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
- [ ] :negative-preconditions
- [ ] :derived-predicates
- [ ] :timed-initial-literals
- [ ] :constraints
- [ ] :preferences
- [x] :action-costs
- [ ] :goal-utilities

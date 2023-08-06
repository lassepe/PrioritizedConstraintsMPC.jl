# PrioritizedConstraintsMPC

This repository serves as an entry-point to the development of an MPC framework with prioritized constraints.

As an initial assessment task, complete the following steps

## Setup and Preliminaries

1. Clone this repository.
2. Install Julia 1.9.2. I recommend to use [juliaup](https://github.com/JuliaLang/juliaup) for the installation.
3. Set up an IDE. You probably want to use VS code; it has a nice [julia extension](https://www.julia-vscode.org/).
4. Familiarize yourself with a basic development workflow in Julia. Specifically, in Julia, we usually don't call a script from the terminal (i.e., we do *not* run `julia my_script.jl`). Instead, we start a REPL, with `julia` and issue commands from there. Here are some tips to this end. Use [Revise.jl](https://timholy.github.io/Revise.jl/stable/). You will need this tool to make sure that changes in your code are reflected in the REPL without having to restart Julia.
5. Familiarize yourself with the basics of the Julia package manager, [pkg](https://docs.julialang.org/en/v1/stdlib/Pkg/). You will need it to pull in new dependencies and install or remove packages. Most importantly:
    - Learn about how to `activate` a project environment.
    - Learn about how to install the dependencies of a project via `instantiate`.
6. If necessary, read up on the language basics/syntax in the [manual](https://docs.julialang.org/en/v1/manual/getting-started/). If you already know Python/C++, you will be able to pick up most of this on the fly.
7. Set up the license for that PATH solver. Consult the [installation instructions of `ParametricMCPs.jl`](https://github.com/lassepe/parametricmcps.jl#installation) for more details.

## The Assessment Task

This assessment task serves two purposes: (i) it allows you familiarize with the new tools that are required for this project, and (ii) it is meant to check your level of understanding of the basics in constrained optimization.
For these reasons, we will keep the task simple but we will use rather "bare-bone" tools to implement the solution instead of spoiling all the fun with an off-the-shelf library.

You will implement a simple MPC controller for a point mass robot with input and state constraints. There will be no noise, no model mismatch, and initial states will be chosen so that the problem is feasible. Hence, this task does *not* yet involve implementing any prioritization of constraints; just a vanilla MPC.

After activating and instantiating this project, the entry-point to this project will be as follow:

```julia
using PrioritizedConstraintsMPC
PrioritizedConstraintsMPC.run_demo()
```

If you set up Julia correctly, this should show a simple plot with a Jackal robot spawned at a
random position and moving towards the goal position (star) using a simple P-controller. Of course,
this naive controller does not consider constraints. Thus, the simulation will likely terminate
with warning like below:

```Julia
┌ Warning: State at time 60 is out of bounds.
└ @ PrioritizedConstraintsMPC ~/worktree/PrioritizedConstraintsMPC.jl/src/validation.jl:46
┌ Warning: Step was not feasible; terminating
└ @ PrioritizedConstraintsMPC ~/worktree/PrioritizedConstraintsMPC.jl/src/visualization.jl:42
┌ Warning: Goal not reached!
└ @ PrioritizedConstraintsMPC ~/worktree/PrioritizedConstraintsMPC.jl/src/visualization.jl:53
```

Once you have successfully completed the task by completing the steps below, the robot should move
to the goal position without violating any constraints.

Complete the following sub-tasks. The corresponding places to implement these are highlighted in the code with `<TODO IMPLEMENT STEP>` in the `src/` directory:

- TASK 1: Implement a quadratic cost function that penalizes the distance to the goal position.
- TASK 2: Implement the solver constructor. More details on sub-steps in `src/solver.jl`
- TASK 3: Invoke your solver in a receding-horizon fashion.

## Other tips:

- In case you are wondering why Julia is initially rather slow: when ever you call a function for
the first time, Julia first has to compile that function. The second time you call a function, it
can directly use the compiled code. When you close the session, the compiled code is deleted. That
is a key reason why we work in a long-running interactive session rather than calling `julia
my_script.jl`.
- If you have programming-language related questions, the [Julia
manual](https://docs.julialang.org/en/v1/) should be the first thing to check.
- For debugging, I like to use [Infiltrator.jl](https://github.com/JuliaDebug/Infiltrator.jl). You
can use it to (conditionally) stop the REPL at a specific point in your code.
- The task assumes that you know the basics of constrained optimization. If you need a refresher, on
  this or related topics, consult the resources listed on my website: https://lasse-peters.net/primer.

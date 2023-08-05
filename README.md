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

## The Assessment Task

This assessment task serves two purposes: (i) it allows you familiarize with the new tools that are required for this project, and (ii) it is meant to check your level of understanding of the basics in constrained optimization.
For these reasons, we will keep the task simple but we will use rather "bare-bone" tools to implement the solution instead of spoiling all the fun with an off-the-shelf library.

You will implement a simple MPC controller for a point mass robot with input and state constraints. There will be no noise, no model mismatch, and initial states will be chosen so that the problem is feasible. Hence, this task does *not* yet involve implementing any prioritization of constraints; just a vanilla MPC.

After activating and instantiating this project, the entry-point to this project will be as follow:

```julia
using PrioritizedConstraintsMPC
PrioritizedConstraintsMPC.run_demo()
```

If you set up Julia correctly, this should show a simple plot with a robot standing at the origin of a squared environment.
Once you have successfully completed the task by completing the steps below, the robot should move to the goal position (marked with the red cross) and stop there.

Complete the following sub-tasks. The corresponding places to implement these are highlighted in the code with `<TODO IMPLEMENT STEP>` in the `src/` directory:

1. 

## Other tips:

- If you have programming-language related questions, the [Julia manual](https://docs.julialang.org/en/v1/) should be the first thing to check.
- For debugging, I like to use [Infiltrator.jl](https://github.com/JuliaDebug/Infiltrator.jl). You can use it to (conditionally) stop the REPL at a specific point in your code.

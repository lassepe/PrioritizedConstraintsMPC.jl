function simulate!(
    strategy,
    dynamics,
    environment,
    initial_state,
    goal_position;
    canvas = Makie.Figure(),
    robot_marker = load(joinpath(@__DIR__, "..", "assets", "jackal.png")),
    max_simulation_time = 200,
)
    validator = Validator(; environment, dynamics, initial_state)

    axis = TGE.create_environment_axis(canvas[1, 1], environment)
    robot_state = Makie.Observable(initial_state)

    Makie.scatter!(
        axis,
        goal_position[1],
        goal_position[2];
        marker = :star4,
        markerspace = :data,
        markersize = 0.3,
        color = :darkred,
    )
    Makie.scatter!(
        axis,
        Makie.@lift($robot_state[1]),
        Makie.@lift($robot_state[2]);
        marker = robot_marker,
        markerspace = :data,
        markersize = 0.3,
        rotations = Makie.@lift(atan($robot_state[4], $robot_state[3])),
    )

    display(canvas)

    Makie.record(canvas, "simulation.gif") do io
        for time in 1:max_simulation_time
            Makie.recordframe!(io)
            input = strategy(robot_state[], time)
            is_feasible = validate_step!(validator, robot_state[], input, time)

            if !is_feasible
                @warn "Step was not feasible; terminating"
                break
            end
            sleep(0.1)
            # update the robot state
            robot_state[] = dynamics(robot_state[], input, time)
        end
    end

    if norm(robot_state[][1:2] - goal_position) < 0.1
        @info "Goal reached!"
    else
        @warn "Goal not reached!"
    end

    canvas
end

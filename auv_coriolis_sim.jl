using Oceananigans
using Oceananigans.Units
using LinearAlgebra
using Random
using CairoMakie

# ==========================================
# 1. SETUP THE INDIAN OCEAN ENVIRONMENT
# ==========================================

τ₀ = 0.1 # Wind stress magnitude 
wind_stress(x, y, t) = -τ₀ * cos(π * y / 5000kilometers)

# Boundary conditions
u_top_bc = FluxBoundaryCondition(wind_stress)
u_bcs = FieldBoundaryConditions(top = u_top_bc)

# Mumbai to Madagascar is roughly 5000 km. 
grid = RectilinearGrid(size=(128, 128, 1), 
                       extent=(5000kilometers, 5000kilometers, 100meters), 
                       topology=(Bounded, Bounded, Bounded))

# The Core Physics: Beta-Plane Approximation
coriolis_model = BetaPlane(latitude=10)

# Initialize the Oceananigans model
model = NonhydrostaticModel(grid; 
                            coriolis=coriolis_model, 
                            boundary_conditions=(u=u_bcs,), 
                            closure=ScalarDiffusivity(ν=1e4, κ=1e4))

set!(model, u=0.1, v=0.05) 

# ==========================================
#  AUV KINEMATICS & HELPER FUNCTIONS
# ==========================================

mutable struct AUV
    name::String
    position::Vector{Float64}
    target::Vector{Float64}
    velocity_mag::Float64
end

function get_ocean_current(model, x, y)
    # Placeholder for actual field queries
    return [0.1, 0.05] 
end

function calculate_coriolis_acceleration(velocity, y_pos, coriolis_model)
    f0 = coriolis_model.f₀
    beta = coriolis_model.β
    f_local = f0 + (beta * y_pos)
    return [-f_local * velocity[2], f_local * velocity[1]]
end

function non_linear_physics_prediction(x_current, thrust_accel, coriolis_model, dt)
    p_x, p_y, v_x, v_y = x_current
    u_x, u_y = thrust_accel

    # Earth rotation at current estimated Y-position
    f_local = coriolis_model.f₀ + (coriolis_model.β * p_y)

    # Predict next state using Euler integration
    p_x_next = p_x + (v_x * dt)
    p_y_next = p_y + (v_y * dt)
    
    # Note: We align with our `calculate_coriolis_acceleration` sign convention
    v_x_next = v_x + (u_x - f_local * v_y) * dt
    v_y_next = v_y + (u_y + f_local * v_x) * dt

    return [p_x_next, p_y_next, v_x_next, v_y_next]
end

function calculate_jacobian(x_current, coriolis_model, dt)
    p_y = x_current[2]
    v_x = x_current[3]
    v_y = x_current[4]

    f0 = coriolis_model.f₀
    beta = coriolis_model.β
    f_local = f0 + (beta * p_y)

    # The mathematically derived 4x4 Jacobian Matrix
    F = [
        1.0  0.0                 dt              0.0;
        0.0  1.0                 0.0             dt;
        0.0  (-beta * v_y * dt)  1.0             (-f_local * dt);
        0.0  (beta * v_x * dt)   (f_local * dt)  1.0
    ]
    return F
end

# ==========================================
# INITIALIZATION
# ==========================================

mumbai_pos = [4000kilometers, 4000kilometers]
madagascar_pos = [1000kilometers, 1000kilometers]
auv_speed = 2.0 # 2 m/s cruising speed

naive_auv = AUV("Dead Reckoning AUV", copy(mumbai_pos), madagascar_pos, auv_speed)
smart_auv = AUV("Coriolis Compensated AUV", copy(mumbai_pos), madagascar_pos, auv_speed)

dt = 10minutes 
simulation_days = 30
total_steps = Int((simulation_days * 24 * 60 * 60) / dt)

# ==========================================
# MAIN UNIFIED SIMULATION LOOP
# ==========================================
function animate_transit(model, naive_auv, smart_auv, coriolis_model, total_steps, dt)
    println("Setting up Makie visualization...")

    # -SETUP HISTORY ARRAYS (Converted to km for plotting) ---
    hist_naive_x = [naive_auv.position[1] / 1000]
    hist_naive_y = [naive_auv.position[2] / 1000]
    
    hist_true_x = [smart_auv.position[1] / 1000]
    hist_true_y = [smart_auv.position[2] / 1000]
    
    hist_ekf_x = [smart_auv.position[1] / 1000]
    hist_ekf_y = [smart_auv.position[2] / 1000]
    
    hist_sensor_x = Float64[]
    hist_sensor_y = Float64[]

    # --- EKF INITIALIZATION ---
    P = Matrix{Float64}(I, 4, 4) * 100.0 
    Q = Matrix{Float64}(I, 4, 4) * 0.1   
    R = Matrix{Float64}(I, 2, 2) * 2000.0 # High sensor noise for visual effect
    H = [1.0 0.0 0.0 0.0;                
         0.0 1.0 0.0 0.0]
    x_current = [smart_auv.position[1], smart_auv.position[2], 0.0, 0.0]

    # --- GENERATING THE FIGURE ---
    fig = Figure(size = (900, 900))
    ax = Axis(fig[1, 1],
              title = "Trans-Oceanic Navigation: Coriolis Drift & EKF Tracking",
              xlabel = "East (km)",
              ylabel = "North (km)",
              limits = (0, 5000, 0, 5000))

    # Plotting static locations
    scatter!(ax, [mumbai_pos[1]/1000], [mumbai_pos[2]/1000], color=:blue, markersize=15, label="Mumbai")
    scatter!(ax, [madagascar_pos[1]/1000], [madagascar_pos[2]/1000], color=:green, markersize=15, label="Madagascar")

    # Creating Observables
    obs_naive_x = Observable(hist_naive_x)
    obs_naive_y = Observable(hist_naive_y)
    obs_true_x = Observable(hist_true_x)
    obs_true_y = Observable(hist_true_y)
    obs_ekf_x = Observable(hist_ekf_x)
    obs_ekf_y = Observable(hist_ekf_y)
    obs_sensor_x = Observable(hist_sensor_x)
    obs_sensor_y = Observable(hist_sensor_y)

    # Drawing dynamic lines/scatters tied to the observables
    lines!(ax, obs_naive_x, obs_naive_y, color=:red, linewidth=3, label="Naive (Dead Reckoning)")
    scatter!(ax, obs_sensor_x, obs_sensor_y, color=(:gray, 0.3), markersize=5, label="Noisy ADCP/IMU")
    lines!(ax, obs_ekf_x, obs_ekf_y, color=:lightgreen, linewidth=4, label="EKF Estimate")
    lines!(ax, obs_true_x, obs_true_y, color=:blue, linewidth=2, linestyle=:dash, label="True Smart Path")
    
    axislegend(ax, position=:rt)

    # --- RECORDING THE ANIMATION ---
    println("Simulating and recording mp4... (This may take a minute or two)")
    
    # We are stepping the simulation multiple times per frame to keep the video duration reasonable
    steps_per_frame = 24 
    frames = 1:Int(floor(total_steps / steps_per_frame))

    record(fig, "indian_ocean_transit.mp4", frames; framerate = 30) do frame
        
        for _ in 1:steps_per_frame
            time_step!(model, dt) 
            
            # -- NAIVE UPDATE --
            dir_naive = normalize(naive_auv.target - naive_auv.position)
            thrust_naive = dir_naive * naive_auv.velocity_mag
            naive_auv.position += (thrust_naive + get_ocean_current(model, 0, 0)) * dt
            
            # -- SENSOR NOISE SIMULATION --
            # Injecting +/- 2km of noise into the position ping
            noise_x = randn() * 2000 
            noise_y = randn() * 2000
            noisy_measurement = [smart_auv.position[1] + noise_x, smart_auv.position[2] + noise_y]

            # -- EKF PREDICT --
            actual_thrust_accel = [0.0, 0.0] # Simplified for tracking: assume steady state thrust mapping
            x_predict = non_linear_physics_prediction(x_current, actual_thrust_accel, coriolis_model, dt)
            F = calculate_jacobian(x_current, coriolis_model, dt)
            P_predict = F * P * transpose(F) + Q

            # -- EKF UPDATE --
            y_residual = noisy_measurement - (H * x_predict)
            S = H * P_predict * transpose(H) + R
            K = P_predict * transpose(H) * inv(S) 
            
            x_current = x_predict + (K * y_residual)
            P = (I - K * H) * P_predict
            
            # -- SMART AUV KINEMATICS (Using EKF Estimate to navigate) --
            dir_to_target = normalize(smart_auv.target - x_current[1:2])
            currents_at_smart = get_ocean_current(model, 0, 0)
            
            intended_velocity = dir_to_target * smart_auv.velocity_mag
            coriolis_drift = calculate_coriolis_acceleration(intended_velocity, x_current[2], coriolis_model) * dt
            
            compensation_vector = -currents_at_smart - (coriolis_drift / dt)
            adjusted_thrust_vector = (dir_to_target * smart_auv.velocity_mag) + compensation_vector
            actual_thrust = normalize(adjusted_thrust_vector) * smart_auv.velocity_mag
            
            # Move the actual robot through the water
            smart_auv.position += (actual_thrust + currents_at_smart) * dt

            # Updating History Arrays
            push!(hist_naive_x, naive_auv.position[1] / 1000)
            push!(hist_naive_y, naive_auv.position[2] / 1000)
            push!(hist_true_x, smart_auv.position[1] / 1000)
            push!(hist_true_y, smart_auv.position[2] / 1000)
            push!(hist_ekf_x, x_current[1] / 1000)
            push!(hist_ekf_y, x_current[2] / 1000)
            push!(hist_sensor_x, noisy_measurement[1] / 1000)
            push!(hist_sensor_y, noisy_measurement[2] / 1000)
        end
        
        # Triggering Makie to redraw the frame with the new data
        obs_naive_x[] = hist_naive_x
        obs_naive_y[] = hist_naive_y
        obs_true_x[] = hist_true_x
        obs_true_y[] = hist_true_y
        obs_ekf_x[] = hist_ekf_x
        obs_ekf_y[] = hist_ekf_y
        obs_sensor_x[] = hist_sensor_x
        obs_sensor_y[] = hist_sensor_y
    end

    println("Success! Check your directory for 'indian_ocean_transit.mp4'")
end

# Execute the visualization
animate_transit(model, naive_auv, smart_auv, coriolis_model, total_steps, dt)
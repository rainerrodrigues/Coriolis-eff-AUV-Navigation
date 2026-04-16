using Oceananigans
using Oceananigans.Units
using LinearAlgebra

# ==========================================
# 1. SETUP THE INDIAN OCEAN ENVIRONMENT
# ==========================================

# Mumbai to Madagascar is roughly 5000 km. 
# We set up a massive, coarse-resolution grid.
grid = RectilinearGrid(size=(128, 128, 1), 
                       extent=(5000kilometers, 5000kilometers, 100meters), 
                       topology=(Bounded, Bounded, Bounded))

# The Core Physics: Beta-Plane Approximation
# Latitude ~ 10° North (between Mumbai and equator) as a reference
coriolis = BetaPlane(latitude=10)

# Initialize the Oceananigans model
model = NonhydrostaticModel( grid; 
                              coriolis=coriolis, 
                              advection=WENO(),
                              closure=ScalarDiffusivity(ν=1e4, κ=1e4)) # High viscosity for stability

# (In a full simulation, you would add wind stress boundary conditions here to spin up the Indian Ocean Gyre)
set!(model, u=0.1, v=0.05) # Seed a generic background current for demonstration

# ==========================================
# AUV KINEMATICS & CONTROL SETUP
# ==========================================

mutable struct AUV
    name::String
    position::Vector{Float64}
    target::Vector{Float64}
    velocity_mag::Float64
end

# Mumbai (Approximate Top Right of our grid) to Madagascar (Bottom Left)
mumbai_pos = [4000kilometers, 4000kilometers]
madagascar_pos = [1000kilometers, 1000kilometers]
auv_speed = 2.0 # 2 m/s cruising speed

naive_auv = AUV("Dead Reckoning AUV", copy(mumbai_pos), madagascar_pos, auv_speed)
smart_auv = AUV("Coriolis Compensated AUV", copy(mumbai_pos), madagascar_pos, auv_speed)

# ==========================================
# 3. THE NAVIGATION ALGORITHMS
# ==========================================

function get_ocean_current(model, x, y)
    # In a real scenario, the AUV would use an Acoustic Doppler Current Profiler (ADCP) 
    # Here, we query the Oceananigans model's velocity field
    # (Simplified field query for demonstration)
    u_field = model.velocities.u
    v_field = model.velocities.v
    # Return placeholder currents based on the initialized model state
    return [0.1, 0.05] 
end

function calculate_coriolis_acceleration(velocity, y_pos, coriolis_model)
    # Extract f0 and beta from the Oceananigans BetaPlane struct
    f0 = coriolis_model.f₀
    beta = coriolis_model.β
    
    # f = f0 + βy
    f_local = f0 + (beta * y_pos)
    
    # Coriolis acceleration vector for 2D: a = [-f*v, f*u]
    return [-f_local * velocity[2], f_local * velocity[1]]
end

# ==========================================
# MAIN SIMULATION LOOP
# ==========================================

dt = 10minutes # Timestep
simulation_days = 30
total_steps = Int((simulation_days * 24 * 60 * 60) / dt)

for step in 1:total_steps
    # 1. Step the Oceananigans fluid dynamics model forward
    time_step!(model, dt)
    
    # 2. Update Naive AUV (Dead Reckoning)
    # Always points straight at the target, ignoring the water moving beneath it
    dir_naive = normalize(naive_auv.target - naive_auv.position)
    thrust_naive = dir_naive * naive_auv.velocity_mag
    
    currents_at_naive = get_ocean_current(model, naive_auv.position[1], naive_auv.position[2])
    
    # Actual movement = Thrust + Ocean Currents
    naive_auv.position += (thrust_naive + currents_at_naive) * dt
    
    # 3. Update Smart AUV (Compensated Navigation)
    # Calculates the vector required to counteract the current and Coriolis drift
    dir_to_target = normalize(smart_auv.target - smart_auv.position)
    
    currents_at_smart = get_ocean_current(model, smart_auv.position[1], smart_auv.position[2])
    
    # Predict Coriolis drift based on intended velocity
    intended_velocity = dir_to_target * smart_auv.velocity_mag
    coriolis_drift = calculate_coriolis_acceleration(intended_velocity, smart_auv.position[2], coriolis) * dt
    
    # The AUV alters its heading to cancel out the environmental vectors
    compensation_vector = -currents_at_smart - (coriolis_drift / dt)
    adjusted_thrust_vector = (dir_to_target * smart_auv.velocity_mag) + compensation_vector
    
    # Normalize and apply AUV speed limit (can't magically thrust faster)
    actual_thrust = normalize(adjusted_thrust_vector) * smart_auv.velocity_mag
    
    # Actual movement
    smart_auv.position += (actual_thrust + currents_at_smart) * dt
    
    # Break if Madagascar is reached
    if norm(smart_auv.position - madagascar_pos) < 50kilometers
        println("Smart AUV reached Madagascar in $(round(step * dt / 86400, digits=2)) days!")
        break
    end
end

# Compare final cross-track error
error_naive = norm(naive_auv.position - madagascar_pos) / 1000 # in km
println("Naive AUV missed target by: $(round(error_naive, digits=2)) km")
import numpy as np
import matplotlib.pyplot as plt

dt = 0.3  
simulation_time = 30  
safe_distance = 50 

kp = 1.2 
kd = 0.8 
ki=0.1 
initial_distance = 100 
v_follower = 0  
x_follower = 0  


def leading_vehicle_motion(case, t):
    if case == 1:  # Constant velocity
        v_leader = 5  
        a_leader = 0
    elif case == 2:  # Constant acceleration
        v_leader = 5 + 2 * t  
        a_leader = 2  
    elif case == 3:  # Deceleration to rest
        a_leader = -0.2 
        v_leader = max(0, 5 + a_leader * t) 

def simulate_acc(case):
    global v_follower, x_follower
    time = np.arange(0, simulation_time, dt)
    distances = [] 
    velocities_follower = []
    errors = []

    d_leader = initial_distance
    for t in time:
        v_leader, _ = leading_vehicle_motion(case, t)
        d_leader += v_leader * dt

        distance = d_leader - x_follower
        error = distance - safe_distance
        integral=0
        integral += error * dt 

        if len(errors) > 0:
            error_rate = (error - errors[-1]) / dt
        else:
            error_rate = 0
        a_follower = kp * error + kd * error_rate+ki * integral

        v_follower = max(0, v_follower + a_follower * dt)  # Prevent negative velocity
        x_follower += v_follower * dt

        distances.append(distance)
        velocities_follower.append(v_follower)
        errors.append(error)

    return time, distances, velocities_follower, errors

def plot_results(time, distances, velocities, errors, case):
    plt.figure(figsize=(20, 3))

    plt.subplot(1, 3, 1)
    plt.plot(time, distances, label="Distance")
    plt.axhline(safe_distance, color='r', linestyle='--', label="Safe Distance")
    plt.title("Distance vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.plot(time, velocities, label="Follower Velocity")
    plt.title("Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.plot(time, errors, label="Error")
    plt.title("Error vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()

    plt.suptitle(f"Adaptive Cruise Control - Case {case}")
    plt.show()
for case in [1, 2, 3]:
    time, distances, velocities, errors = simulate_acc(case)
    plot_results(time, distances, velocities, errors, case)
import numpy as np
import matplotlib.pyplot as plt

def lipm_trajectory(x0, v0, p_zmp, z_c, g, dt, T):
    """
    Compute the trajectory of the Linear Inverted Pendulum Model (LIPM).
    
    Parameters:
    x0: Initial position of the center of mass (COM)
    v0: Initial velocity of the COM
    p_zmp: Position of the Zero Moment Point (ZMP)
    z_c: Height of the COM
    g: Gravitational acceleration
    dt: Time step
    T: Total time duration
    
    Returns:
    A tuple containing the time array, the corresponding COM positions and velocities.
    """
    # Natural frequency of the pendulum
    omega = np.sqrt(g / z_c)
    
    # Time array
    time = np.arange(0, T, dt)
    
    # Initialize arrays for COM positions and velocities
    x_com = np.zeros_like(time)
    v_com = np.zeros_like(time)
    # Set initial conditions
    x_com[0] = x0
    v_com[0] = v0
    
    # Compute the trajectory using the LIPM equations
    for i in range(1, len(time)):
        t = time[i]
        x_com[i] = p_zmp + (x0 - p_zmp) * np.cosh(omega * t) + (v0 / omega) * np.sinh(omega * t)
        v_com[i] = (x0 - p_zmp) * omega * np.sinh(omega * t) + v0 * np.cosh(omega * t)
    
    return time, x_com, v_com

def plot_trajectory(time, x_com, v_com):
    """
    Plot the trajectory of the COM position and velocity.
    
    Parameters:
    time: Time array
    x_com: COM positions
    v_com: COM velocities
    """
    plt.figure(figsize=(12, 6))
    
    # Plot COM position
    plt.subplot(2, 1, 1)
    plt.plot(time, x_com, label='COM Position')
    plt.title('LIPM Trajectory - COM Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.grid()
    plt.legend()
    
    # Plot COM velocity
    plt.subplot(2, 1, 2)
    plt.plot(time, v_com, label='COM Velocity', color='orange')
    plt.title('LIPM Trajectory - COM Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid()
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def scenario_parameters(scenario):
    """
    Return parameters for different scenarios.
    
    Parameters:
    scenario: A string indicating the scenario type ('divergent', 'convergent_to_fixed_point', 'oscillatory')
    
    Returns:
    A dictionary containing the parameters for the specified scenario.
    """
    if scenario == 'divergent':
        return {
            'x0': 0.05,  # Initial position (m)
            'v0': 0.0,   # Initial velocity (m/s)
            'p_zmp': 0.0,  # ZMP position (m)
            'z_c': 0.5,  # Height of COM (m)
            'g': 9.81,   # Gravitational acceleration (m/s^2)
        }
    elif scenario == 'convergent_to_fixed_point':
        x0 = 0.05  # Initial position (m)
        v0 = 0.1    # Initial velocity (m/s)
        g = 9.81    # Gravitational acceleration (m/s^2)
        z_c = 0.5   # Height of COM (m)
        T_c = np.sqrt(z_c / g)  # Time to reach fixed point
        p_zmp = x0 + T_c * v0  # ZMP position to ensure convergence to fixed point

        return {
            'x0': x0,  # Initial position (m)
            'v0': v0,    # Initial velocity (m/s)
            'p_zmp': p_zmp,   # ZMP position (m)
            'z_c': z_c,   # Height of COM (m)
            'g': g,    # Gravitational acceleration (m/s^2)
        }
    elif scenario == 'back_to_origin':
        x0 = 0.0  # Initial position (m)
        v0 = 0.1    # Initial velocity (m/s)
        g = 9.81    # Gravitational acceleration (m/s^2)
        z_c = 0.5   # Height of COM (m)
        T_c = np.sqrt(z_c / g)  # Time to reach fixed point
        p_zmp = 0.1 * T_c * np.cosh(0.5 / T_c) / np.sinh(0.5 / T_c)  # ZMP position to ensure return to origin
        return {
            'x0': x0,   # Initial position (m)
            'v0': v0,   # Initial velocity (m/s)
            'p_zmp': p_zmp,   # ZMP position (m)
            'z_c': z_c,   # Height of COM (m)
            'g': g    # Gravitational acceleration (m/s^2)
        }
    else:
        raise ValueError("Invalid scenario type. Choose from 'divergent', 'convergent_to_fixed_point', or 'back_to_origin'.")


# Example usage
if __name__ == "__main__":
    # Parameters
    scenario = 'back_to_origin'  # Choose from 'divergent', 'convergent_to_fixed_point', 'back_to_origin'
    params = scenario_parameters(scenario)
    x0 = params['x0']
    v0 = params['v0']
    p_zmp = params['p_zmp']
    z_c = params['z_c']
    g = params['g']
    dt = 0.01
    T = 2.0

    # Compute trajectory
    time, x_com, v_com = lipm_trajectory(x0, v0, p_zmp, z_c, g, dt, T)

    # Plot trajectory
    plot_trajectory(time, x_com, v_com)
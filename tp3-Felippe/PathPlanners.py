import numpy as np


def att_force(q, goal, katt=5):
    """
    Calculates the attractive force between a robot and a goal position.

    Parameters:
    - q: The current position of the robot.
    - goal: The goal position.
    - katt: The scaling factor for the attractive force (default: 5).

    Returns:
    - Fatt: The attractive force vector.

    """
    Fatt = katt * (goal - q)
    return Fatt

def rep_force(q, obs, R=5, krep=0.005):
    """
    Calculates the repulsive force exerted by obstacles on a given point.

    Parameters:
    - q (numpy.ndarray): The coordinates of the point.
    - obs (list): A list of obstacle coordinates.
    - R (float): The radius of influence for the obstacles. Default is 5.
    - krep (float): The scaling factor for the repulsive force. Default is 0.005.

    Returns:
    - numpy.ndarray: The repulsive force vector.

    """
    Frep = np.zeros(2)
    for obstacle in obs:
        v = q[0:2] - obstacle
        d = np.linalg.norm(v) 

        # Se a distância for menor que o raio de influência calcular a força repulsiva
        if (d < R):  
            rep = (1/d**2)*((1/d)-(1/R))*(v/d) 
            Frep += rep

    return krep*Frep

def tt_force(q, goal, laser_data, obs, obs_pts, HWL, max_sensor_range=5):
    """
    Calculates the total force acting on the robot given its current configuration, goal position,
    laser data, obstacle positions, and a transformation matrix.

    Parameters:
        q (list): The current configuration of the robot.
        goal (list): The goal position.
        laser_data (list): The laser data containing angle and distance values.
        obs (list): The list to store obstacle positions.
        obs_pts (list): The list to store obstacle points.
        HWL (numpy.ndarray): The transformation matrix.
        max_sensor_range (float, optional): The maximum sensor range. Defaults to 5.

    Returns:
        list: A list containing the total force, attractive force, and repulsive force.

    """
    Frep = np.zeros(2)
    for i in range(len(laser_data)):
        ang, dist = laser_data[i]  # get angle and distance values

        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)  # x coordinate
            y = dist * np.sin(ang)  # y coordinate
            point = np.array([x, y, 0, 1])

            if len(HWL) != 0:
                point = HWL @ point
                obs.append(point[0:2])
                obs_pts.append(point)
    Frep = rep_force(q, obs)
    Fatt = att_force(q, goal)
    Ft = Fatt + Frep

    return [Ft, Fatt, Frep]
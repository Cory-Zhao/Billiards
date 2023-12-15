import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# function that checks if a ball hits the wall
# input: out- position vector
# output: T/F if the out- position is already past a wall
def hitsWall(x_out):
    if x_out[0] >= right_wall - radius or x_out[0] <= left_wall + radius or x_out[1] >= top_wall - radius or x_out[1] <= bottom_wall + radius:
        return True
    return False

# function that calculates the time until it hits a wall
# input: in- and out- position vectors and in- velocity vector
# ouput: time until it hits a wall and the specific wall
def timeToWall(x_in, x_out, v_in):
    dt_hit_wall = float('inf')
    time_to_hit = []
    if x_out[0] >= right_wall - radius:
        time_to_hit.append((abs((right_wall - radius - x_in[0]) / v_in[0]), "RIGHT"))
    if x_out[0] <= left_wall + radius:
        time_to_hit.append((abs((left_wall + radius - x_in[0]) / v_in[0]), "LEFT"))
    if x_out[1] >= top_wall - radius:
        time_to_hit.append((abs((top_wall - radius - x_in[1]) / v_in[1]), "TOP"))
    if x_out[1] <= bottom_wall + radius:
        time_to_hit.append((abs((bottom_wall + radius - x_in[1]) / v_in[1]), "BOTTOM"))

    for time, wall in time_to_hit:
        if time < dt_hit_wall:
            dt_hit_wall = time
            collided_wall = wall

    return dt_hit_wall, collided_wall

# function that updates velocity when the ball hits the wall
# input: in- velocity vector and a specific wall
# output: out- velocity vector calculated by which wall it hits
def updateWallV(v_in, collided_wall):
    if collided_wall == "LEFT" or collided_wall == "RIGHT":
        v_out = np.array([v_in[0] * -damping, v_in[1] * friction])
    if collided_wall == "TOP" or collided_wall == "BOTTOM":
        v_out = np.array([v_in[0] * friction, v_in[1] * -damping])
    return v_out

# function that handles when the balls collide
# input: in- position and velocity vectors of both balls
# output: out- position and velocity vectors of both balls and the time it takes for the balls to collide
def collision(xR_in, xB_in, vR_in, vB_in):
    dt_shrunk = abs((np.linalg.norm(xR_in - xB_in) - 2 * radius) / (np.linalg.norm(vR_in - vB_in)))

    xR_out = np.array(xR_in + dt_shrunk * vR_in)
    xB_out = np.array(xB_in + dt_shrunk * vB_in)

    n = np.array((xB_out - xR_out) / np.linalg.norm(xB_out - xR_out))
    z = np.array([n[1], -n[0]])

    vR_out = np.dot(vB_in,n) * n + np.dot(vR_in,z) * z
    vB_out = np.dot(vR_in,n) * n + np.dot(vB_in,z) * z
    return xB_out, xR_out, vB_out, vR_out, dt_shrunk

# function that draws each frame of the animation
def animate(i):
    t = 0
    t_final = 50
    dt = 0.02

    # initial red ball position (x,y)
    xR_in = np.array([0.75,0.5],dtype=np.float64)
    # initial red ball velocity (u,v)
    vR_in = np.array([0,0],dtype=np.float64)

    # initial blue ball position (x,y)
    xB_in = np.array([0.25,0.5],dtype=np.float64)
    # initial red ball velocity (u,v)
    vB_in = np.array([1,0],dtype=np.float64)

    while t < t_final:     
        # update out vectors   
        xR_out = xR_in + dt * vR_in
        xB_out = xB_in + dt * vB_in
        vR_out = vR_in
        vB_out = vB_in

        dt_new = dt

        # collision between balls
        if (np.linalg.norm(xR_out - xB_out) <= 2 * radius):
            xB_out, xR_out, vB_out, vR_out, dt_new = collision(xR_in, xB_in, vR_in, vB_in)

        # red or blue ball will hit a wall
        if hitsWall(xR_out) or hitsWall(xB_out):
            # both balls will hit a wall 
            if hitsWall(xR_out) and hitsWall(xB_out):
                time_R, wall_R = timeToWall(xR_in, xR_out, vR_in)
                time_B, wall_B = timeToWall(xB_in, xB_out, vB_in)
                if abs(time_R - time_B) < 0.0000005:
                    dt_new = time_R
                    vR_out = updateWallV(vR_in, wall_R)
                    vB_out = updateWallV(vB_in, wall_B)
                elif time_R < time_B:
                    dt_new = time_R
                    vR_out = updateWallV(vR_in, wall_R)
                else:
                    dt_new = time_B
                    vB_out = updateWallV(vB_in, wall_B)
            # only the red ball will hit a wall
            elif hitsWall(xR_out):
                dt_new, wall_R = timeToWall(xR_in, xR_out, vR_in)
                vR_out = updateWallV(vR_in, wall_R)
            # only the blue ball will hit a wall
            elif hitsWall(xB_out):
                dt_new, wall_B = timeToWall(xB_in, xB_out, vB_in)
                vB_out = updateWallV(vB_in, wall_B)

            xB_out = xB_in + dt_new * vB_in
            xR_out = xR_in + dt_new * vR_in
        
        # update in-vectors
        xR_in = xR_out
        xB_in = xB_out
        vR_in = vR_out
        vB_in = vB_out

        xR_animation.append(xR_in[0])
        yR_animation.append(xR_in[1])
        xB_animation.append(xB_in[0])
        yB_animation.append(xB_in[1])
        t += dt_new

    ax.clear()
    ax.set_aspect(1)
    circleR = plt.Circle((xR_animation[i], yR_animation[i]), radius, color="red")
    circleB = plt.Circle((xB_animation[i], yB_animation[i]), radius, color="blue")
    ax.add_artist(circleR)
    ax.add_artist(circleB)
    ax.set_facecolor("forestgreen")
    ax.set_xlim([left_wall, right_wall])
    ax.set_ylim([bottom_wall, top_wall])

# radius of the balls
radius = 0.05

# damping and friction coefficients
damping = 0.8
friction = 0.98

# wall dimensions
left_wall = 0
right_wall = 1
bottom_wall = 0
top_wall = 1

# create empty lists for the x and y coordinates of the red and blue balls
xR_animation = []
yR_animation = []
xB_animation = []
yB_animation = []

# create the figure and axes objects
fig, ax = plt.subplots()

# run the animation
ani = FuncAnimation(fig, animate, frames=1000, interval=100, repeat=False)

plt.show()
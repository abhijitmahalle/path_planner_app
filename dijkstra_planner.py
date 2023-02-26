# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Hrushikesh Budhale
# Created Date: Saturday 26 February 2022
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from matplotlib.widgets import Slider
import matplotlib.pyplot as plt
import heapq as hq  # heapq shows ~20% better performance over PriorityQueue
import numpy as np
import time

# =============================================================================
# GUI functions
# =============================================================================

def on_click(event):
    global click_pos
    try: click_pos = (int(event.ydata), int(event.xdata))
    except Exception: pass

def get_locations():
    inputs, i = [0,0], 0
    ax.set_title("Select Start location")
    while inputs[1] == 0:
        if plt.waitforbuttonpress() == False:    # returns false on mouse click
            try:
                if click_pos[0] > 0 and click_pos[1] > 0 and (obs[click_pos[0],click_pos[1]] == False):
                    inputs[i] = click_pos
                    ax.scatter(click_pos[1], click_pos[0], s=10, c='g')
                    ax.set_title("Select Goal location")
                    i += 1
                else: raise Exception
            except Exception as e:
                ax.set_title("Location not in available space, Please select another location")
    return inputs

# =============================================================================
# Planning functions
# =============================================================================

def check_in_poly(pts, poly):
    count = np.zeros(pts.shape[0])
    for i, _ in enumerate(poly[:-1]):
        # pts.y should be within y limits of line and pts.x should be less than intersection.x
        intersection_x = (poly[i+1,0] - poly[i,0]) * (pts[:,1]-poly[i,1]) / (poly[i+1,1] - poly[i,1]) + poly[i,0]
        count += (((pts[:,1] > poly[i,1]) != (pts[:,1] > poly[i+1,1])) & (pts[:,0] < intersection_x))*1
    return count % 2 # point is outside if even number of intersections

def back_track(came_from, start, goal):
    path = [goal]
    while path[-1] != start: path.append(came_from[path[-1]])
    path.reverse()
    return np.array(path).reshape(-1,2)

def get_obstacle_map(pts, tol=0):
    # add Concave polygon
    polygon = np.array([(36-tol, 185), (115+(tol*1.5), 210+(tol*0.86)), (80+tol, 180), (105+(tol), 100-(tol*1.5)), 
                        (36-tol, 185)])
    obs = check_in_poly(pts, polygon.reshape(-1,2))
    
    # add Hexagon
    hexagon = np.array([(165-(tol*0.86), 079.8-(tol*0.5)), (165-(tol*0.86), 120.2+(tol*0.5)), (200, 140.5+tol), 
                        (235+(tol*0.86), 120.2+(tol*0.5)), (235+(tol*0.86), 079.8-(tol*0.5)), (200, 059.6-tol), 
                        (165-(tol*0.86), 079.8-(tol*0.5))])
    obs2 = check_in_poly(pts, hexagon.reshape(-1,2))
    obs = np.logical_or(obs, obs2)

    # add circle
    center = np.array([300,185])
    obs2 = np.linalg.norm(pts-center, axis=1) < 40+tol
    obs = np.logical_or(obs, obs2)

    # add border
    obs2 = np.zeros((height,width), dtype=bool)
    obs2[:,0:1+tol] = obs2[0:1+tol,:] = obs2[:,-(1+tol):] = obs2[-(1+tol):,:] = True
    obs2 = obs2.flatten()
    obs = np.logical_or(obs, obs2)

    return obs.reshape(height,width)

def get_neighbors(current):
    neighbors = current + actions
    valid = ~obs[neighbors[:,0],neighbors[:,1]]
    return neighbors[valid], costs[valid]

def find_path(start, goal, animate=False):
    frontier, tmp = [], []
    came_from, cost_so_far = dict(), dict() 
    came_from[start] = cost_so_far[start] = 0
    hq.heappush(frontier, (0, start))

    while len(frontier) > 0:
        cost, current = hq.heappop(frontier)
        
        if animate: # visualization of map exploration
            tmp.append(current)
            if len(tmp) > (slider.val*10) or current == goal:
                show_pt = np.array(tmp)
                ax.scatter(show_pt[:,1], show_pt[:,0], s=1, c='cyan')
                plt.pause(0.001)
                tmp = []

        if current == goal: return back_track(came_from, start, goal)

        for nxt, cost in zip(*get_neighbors(current)):
            new_cost = cost_so_far[current] + cost
            if (cost_so_far.get(tuple(nxt), False) == False) or new_cost < cost_so_far[tuple(nxt)]:
                cost_so_far[tuple(nxt)] = new_cost
                hq.heappush(frontier, (new_cost, tuple(nxt)))
                came_from[tuple(nxt)] = current

def run_application():        
    # Receive start and goal locations
    start, goal = get_locations()
    
    # find path
    ax.set_title("Searching Path")
    plt.pause(0.001)    # Necessary for updating title
    start_time = time.perf_counter()
    path = find_path(start, goal)
    message = f"Path Found! in {round(time.perf_counter() - start_time,2)} sec."
    ax.set_title(message)
    print(message)
    ax.plot(path[:,1],path[:,0], c='r')
    
    # show animation
    find_path(start, goal, animate=True)
    plt.show()

# =============================================================================    
# Main logic
# =============================================================================    

def main():
    global slider, actions, costs, width, height, obs, ax, click_pos
    
    # setup
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.15)
    slider_ax = plt.axes([0.15, 0.05, 0.7, 0.03])
    slider = Slider(slider_ax, 'Speed', 1, 100, 20)
    fig.canvas.mpl_connect('button_press_event', on_click)

    actions = np.array([[1,0], [1,-1], [0,-1], [-1,-1], [-1,0], [-1,1], [0,1], [1,1]])
    costs = np.array([1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414])
    width, height = 400, 250
    
    # create exploration space
    Y, X = np.mgrid[0:height, 0:width]
    pts = np.array([X.ravel(), Y.ravel()]).T

    # create obstacle map
    obs = get_obstacle_map(pts) # for visual
    ax.scatter(pts[obs.flatten(),0], pts[obs.flatten(),1], s=1, c='k')
    ax.axis('equal')
    obs = get_obstacle_map(pts, tol=5) # (tolerance = 5) for planning 

    run_application()

if __name__ == "__main__":
    main()
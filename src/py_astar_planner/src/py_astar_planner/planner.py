import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from py_astar_planner import astar

def costmap_plot(costmap, x_size, y_size, x_start, y_start, x_goal, y_goal):
    """Plot costmap """
    # plot    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    plt.imshow(costmap.reshape((x_size, y_size)), interpolation=None)
    circ_start = Circle((x_start, y_start), 5, color='r')
    ax.add_patch(circ_start)
    
    circ_goal = Circle((x_goal, y_goal), 5, color='b')
    ax.add_patch(circ_goal)

    length = 3
    x_list = np.linspace(x_start, x_goal, length).astype(int)
    y_list = np.linspace(y_start, y_goal, length).astype(int)
    path = np.column_stack((x_list, y_list)).tolist()
    ## for i in range(len(path)):
    ##     plt.plot(path[i][0], path[i][1], 'ro-' )
    plt.plot(path[0][0], path[0][1], 'ro' )
    
    plt.tight_layout()
    plt.show()
    #from IPython import embed; embed(); sys.exit()
        
def make_plan(costmap, x_size, y_size, x_start, y_start, x_goal, y_goal):
    """ 
    A function to generate obstacles

    Parameters
    ----------
    costmap : list
        a list of cost values at obstacles
    x_size: int
        the row length
    y_size: int
        the column length
    x_start: int
        x coordinate of the start
    y_start: int
        y coordinate of the start
    x_goal: int
        x coordinate of the goal
    y_goal: int
        y coordinate of the goal

    Returns
    -------
    path : list
        a list of waypoints
    """
    print("Make a plan via python script!")
    # initialize variables
    start       = [x_start, y_start]
    goal        = [x_goal, y_goal]
    robot_size  = 12
    threshold   = 252
    grid_limits = [[0, 0], [x_size, y_size]] # square
    
    # action
    #actions = [[-1,0], [0,-1], [1,0], [0,1]]
    actions = [[-1,0], [0,-1], [1,0], [0,1],
               [-1,-1],[-1,1],[1,-1],[1,1],]

    # run your algorithm
    path = astar.astar_planning(start, goal, actions,
                                grid_limits,
                                costmap, robot_size,
                                threshold)
    return path



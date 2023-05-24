import math
import numpy as np
import os
import matplotlib.pyplot as plt

CLOSED_SET = None
OPEND_SET = None

class Node:
    """
    Node class for dijkstra search
    """
    def __init__(self, pos, idx, cost, h, prev_idx):
        self.pos  = np.array(pos)
        self.idx  = idx # current node's index
        self.cost = cost
        self.h    = h
        self.prev_idx = prev_idx # previous node's index

    def __str__(self):
        return str(self.idx) + "," + str(self.cost) + "," + str(self.prev_idx)


def get_grid_index(state, grid_limits):
    """ 
    A function to convert a state to the index of grid.

    Parameters
    ----------
    state : list
        a list of coordinate values
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[0,0,0],[max_x,max_y,max_z]])

    Returns
    -------
     : Integer
        The index of the input state in grid.
    """
    #if type(state) in [list, tuple]: state = np.array(state)
        
    if len(state)==2:
        idx = state[0]*grid_limits[1,1]+state[1]
    elif len(state)==3:
        return NotImplemented        
    else:
        return NotImplemented
    
    if idx<0 or idx>=grid_limits[1][0]*grid_limits[1][1]:
        print("out of grid: {}".format(idx) )
    return idx


def is_valid(state, grid_limits, costmap, robot_size, threshold):
    """ 
    A function to check the validity of the input state.

    Parameters
    ----------
    state : array
        a list of coordinate values
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.
        (e.g., [[0,0,0],[max_x,max_y,max_z]])
    costmap : 2D list
        a cost map of obstacles.
    robot_size : float
        a radius of a robot. This value is used for collision checking.       

    Returns
    -------
     : boolean
        True if the input state is valid
    """

    # check collision
    if type(state) is list: state = np.array(state)
    if len(np.shape(state))>=2:
        #state=state[np.newaxis,:]
        print("Invalid shape of state")
        return NotImplemented                

    # check workspace limits
    if any(state[i] < grid_limits[0][i] for i in range(len(grid_limits[0])) ):
        return False
    if any(state[i] >= grid_limits[1][i] for i in range(len(grid_limits[1])) ):
        return False
    
    area = get_robot_indices(state, robot_size, grid_limits)
    #from IPython import embed; embed(); sys.exit()
        
    if any(costmap[area]>threshold): return False
    return True


def get_robot_indices(state, robot_size, grid_limits):
    
    if type(state) is list: state = np.array(state)
    #if len(np.shape(state))==1: state=state[np.newaxis,:]
    if len(np.shape(state))>=2:
        print("Invalid shape of state")
        return NotImplemented                

    area = []
    for i in range(robot_size):
        x = state[0]-int(robot_size/2)+i
        #limit check
        if x < grid_limits[0][0] or x >= grid_limits[1][0]:
            continue
        for j in range(robot_size):
            y = state[1]-int(robot_size/2)+j
            #limit check
            if y < grid_limits[0][1] or y >= grid_limits[1][1]:
                continue
            area.append(x*grid_limits[1][1]+y)
    return area

def astar_planning(start, goal, actions, grid_limits,
                   costmap, robot_size, threshold, **kwargs):
    """ 
    A function to generate a path from a start to a goal 
    using A* search-based planning algorithm.  

    Parameters
    ----------
    start : list
        a list of start coordinate values (e.g., [x,y,z]).  
    goal : list
        a list of goal coordinate values (e.g., [x,y,z]).  
    actions : list
        a list of available actions, where each action is a list
        of float values (e.g., [[vx,vy,vz],[vx,vy,vz], ..., ]).  
    grid_limits : list
        a list of minimum and maximum configuration limits, where
        each limit is a list of float values.   
        (e.g., [[min_x,_min_y,min_z],[max_x,max_y,max_z]])
    costmap : n-dimensional array 
        a costmap.
    robot_size : float
        a radius of a robot. This value is used for collision checking.       

    Returns
    -------
    path : list
        a list of coordinate values that is the shortest path from 
        the start to the goal.
    """
    if type(grid_limits) is not np.ndarray: grid_limits = np.array(grid_limits)
    if type(costmap) is list: costmap = np.array(costmap)
    
    openset, closedset = dict(), dict()

    # Set a start node
    start_node = Node(start, get_grid_index(start, grid_limits),
                      0, np.linalg.norm(np.array(start)-goal), -1)
    openset[start_node.idx] = start_node

    if is_valid(start, grid_limits, costmap, robot_size, threshold) is False:
        print("Start is not valid, so made it zero cost")
        costmap[get_robot_indices(start, robot_size, grid_limits)] = 0
    
    # Set a goal node
    goal_node = Node(goal,
                     get_grid_index(goal, grid_limits),
                     0, 0, -1)
    if start_node.idx==goal_node.idx: return []
    print("Start and goal indices: {} and {}".format(start_node.idx,
                                                         goal_node.idx))

    if is_valid(goal, grid_limits, costmap, robot_size, threshold) is False:
        print("Goal is not valid, so made it zero cost")
        costmap[get_robot_indices(goal, robot_size, grid_limits)] = 0
    

    while True:
        # Empty openset
        if len(openset) == 0: return None

        #------------------------------------------------------------
        # PLACE YOUR CODE HERE
        # You need to implement according to the pseudo-code provided
        #------------------------------------------------------------
        #
        # HINT
        # ----
        # cur_idx  = GET_A_INDEX_OF_NODE_WITH_MINIMUM_HEURISTICS_FROM_OPENSET
        # cur_node = GET_A_NODE_WITH_THE_INDEX_FROM_OPENSET
        #
        # if cur_node is the goal node then break
        # Find the node with the minimum heuristic value from the open set
        cur_idx = min(openset, key=lambda x: openset[x].h + openset[x].cost)
        cur_node = openset[cur_idx]

        if cur_node.idx == goal_node.idx:
            goal_node.prev_idx = cur_node.prev_idx
            break
        #------------------------------------------------------------

        # Remove the item from the open set
        del openset[cur_idx]

        # Add it to the closed set
        closedset[cur_idx] = cur_node

        # expand nodes based on available actions
        for i, action in enumerate(actions): 
            
            #------------------------------------------------------------
            # PLACE YOUR CODE HERE
            # You need to implement according to the pseudo-code provided
            #------------------------------------------------------------
            #
            # HINT
            # ----
            # next_pos = NEXT_POSITION_BY_THE_ACTION
            # if IS_NEXT_POS_VALID:
            #   ...
            #
            # node = DEFINE_NODE_WITH_F_VALUE
            # ...
            # ADD_THE_NODE_TO_OPENSET
            # Otherwise if it is already in the open set

            next_pos = [cur_node.pos[j] + action[j] for j in range(len(cur_node.pos))]

            if is_valid(next_pos, grid_limits, costmap, robot_size, threshold):
                next_idx = get_grid_index(next_pos, grid_limits)
                if next_idx not in closedset:
                    cost = cur_node.cost + 1  #we suppose that the travel cost to go to an adjacent node is the same for every cell one.
                    heuristic = np.linalg.norm(np.array(next_pos) - np.array(goal_node.pos))
                    if next_idx not in openset:
                        next_node = Node(next_pos, next_idx, cost, heuristic, cur_node.idx)
                        openset[next_idx] = next_node
                    else:
                        openset[next_idx].cost = cost
                        openset[next_idx].heuristic = heuristic
                        openset[next_idx].parent = cur_node.idx


            #------------------------------------------------------------
        #print(cur_node.__str__())

    global CLOSED_SET, OPEN_SET
    CLOSED_SET = closedset.copy()
    OPEN_SET = openset.copy()
    # Track the path from goal to start
    path = [goal_node.pos.tolist()]
    
    #------------------------------------------------------------
    # PLACE YOUR CODE HERE
    # You need to track back the best path from the goal node
    #------------------------------------------------------------
    #
    # HINT
    # ----
    # ...
    # while ....
    #     path.append(...)
    current_node = goal_node 
    while current_node.prev_idx != start_node.idx:
        current_node = closedset[current_node.prev_idx]
        path.append(current_node.pos.tolist())
    
    path.append(start_node.pos.tolist())
    print("Node expand : ", len(CLOSED_SET.keys()))
    #------------------------------------------------------------
    return path[::-1]


def read_cost_map(cost_map_path):
    """ 
    A function that reads a text file describing the cost map.

    Parameters
    ----------
    cost_map_path : str or path-like object
        a path to the text file.

    Returns
    -------
    cost_map_array : list
        the flattened array of the cost map read.
    (H, W) : Tuple(Int, Int)
        tuple describing the size of the cost map read.
    """
    with open(cost_map_path, 'r') as f:
        cost_map = f.readlines()

    def mapper(entity):
        assert entity in ['O', 'X']

        if entity =='O':
            return 0
        if entity == 'X':
            return 1
    H = len(cost_map)
    cost_map_array = np.array(list(map(
            lambda x: list(map(
                lambda y: mapper(y),
                x.split()
            )),
            cost_map
        ))).flatten()
    W = len(cost_map_array) // H
    return list(cost_map_array), (H, W)

def plot_trajs(map_size, trajectories, costmap, visited_nodes):
    """Plot the input trajectories"""
    from matplotlib.colors import LinearSegmentedColormap
    n,m = map_size
    V = np.zeros_like( costmap )
    for idx in visited_nodes:
        V[idx] = 1
    visited = np.array(V).reshape([n,m])
    obstacles = np.array(costmap).reshape([n,m])
    dark_low = ((0., 1., 1.),
            (.3, 1., 0.),
            (1., 0., 0.))
    cdict3 = {'red':  dark_low,
        'green': dark_low,
        'blue': dark_low,
        'alpha': ((0.0, 0.0, 0.0),
                  (0.3, 0.0, 1.0),
                  (1.0, 1.0, 1.0))
        }

    # Plot the explored nodes.
    symbol =  [' ', '.']
    for sx in range(n):
        for sy in range(m):
            plt.plot([sy], [sx], marker=symbol[int(visited[sx, sy])], linestyle='none', color='k')

    # Plot the obstacles (costs).
    dropout_high = LinearSegmentedColormap('Dropout', cdict3)
    plt.imshow(obstacles, cmap = dropout_high)

    # Plot the trajectory generated.
    for trajectory in trajectories:
        traj_2d = np.array([ s for s in trajectory ])
        y = traj_2d[:, 0]
        x = traj_2d[:, 1]
        plt.plot(x, y, alpha=1, color='r')
    plt.show()

if __name__ == '__main__':
    cost_map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'map.txt')   
    costmap , (H, W) = read_cost_map(cost_map_path)
    start = [2, 18]
    goal  = [18, 7]
    robot_size = 1
    threshold  = 0.0
    grid_limits = [[0, 0], [H, W]] 
    actions = [[-1,0], [0,-1], [1,0], [0,1]]

    path = astar_planning(start, goal, actions,
                              grid_limits,
                              costmap, robot_size,
                              threshold)
    plot_trajs((H,W), [path], costmap, list(CLOSED_SET.keys())) # + list(OPEN_SET.keys()))

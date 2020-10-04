# search.py
# ---------
ACTIONS = (
    (-1,  0), # go up
    ( 0, -1), # go left
    ( 1,  0), # go down
    ( 0,  1)  # go right
)

# Define action in matrix form
import numpy as np
action = np.array([[-1, 0], [0, -1], [1, 0], [0, 1]])

from utils.search_app import OrderedSet, Stack, Queue, PriorityQueue

def depth_first_search(grid_size, start, goal, obstacles, costFn, logger):
   
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = Stack() # Use Stack for openset
    closed_set = Queue()

    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set

    parents = [ # the immediate predecessor cell from which to reach a cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [ # the action that the parent took to reach the cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
   
    movement = []
     
    # Define DFS
    open_set.add(start)
    while (open_set != []):
        parent = open_set.pop()
        closed_set.add(parent)
        
        # Search each direction of the parent
        for i in range(4):
            
            # Moving direction
            action_rows = action[i][0]
            action_cols = action[i][1]
                       
            # Calculate child position
            parent_rows, parent_cols = parent
            child_rows, child_cols = parent_rows + action_rows, parent_cols + action_cols
            child = child_rows, child_cols
            
            # Restrict the node in the map
            if (child not in obstacles) and (child_rows < n_rows) and (child_cols < n_cols) and (child_rows >= 0) and (child_cols >= 0):
                if (child not in open_set) and (child not in closed_set):
                    
                    # Record parent and action for each step
                    parents[child_rows][child_cols] = parent
                    actions[child_rows][child_cols] = tuple(action[i])
                    
                    if child == goal:
                        
                        # Find Path
                        for j in range(n_cols * n_rows):
                            
                            # find action for each step
                            act = actions[child_rows][child_cols]
                            
                            # Save action in movement[]
                            movement.insert(0, act)
                            
                            # find parent for each step
                            parent = parents[child_rows][child_cols]
                            child_rows,child_cols = parent
                            
                            # Stop and return results
                            if parent == start:
                                return movement, closed_set                            
                    else:
                        open_set.add(child)

#############################################################################

def uniform_cost_search(grid_size, start, goal, obstacles, costFn, logger):

    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = PriorityQueue(order="min", f=lambda v: v)
    closed_set = Queue()

    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set

    parents = [ # the immediate predecessor cell from which to reach a cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [ # the action that the parent took to reach the cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]

    movement = []
    
    # Variables initialization
    open_set.put(start,0)
    child_rows = 0
    child_cols = 0
    g = 0 # Path cost
    
    # Define UCS
    while (open_set != []):
        
        # Take out (x,y)(parent) and path v(cost) from openset
        temp_parent = open_set.pop()
        parent,v = temp_parent
        
        # Determine whether the agent reach to goal
        if parent == goal:
            
            # Search path
            for j in range(n_cols * n_rows):                
                parent_rows, parent_cols = parent            
                
                # Stop and return
                if parent == start:
                    print(movement)
                    return movement, closed_set
                
                # record action and put it in to movement()
                act = actions[parent_rows][parent_cols]
                movement.insert(0, act)
                
                # Update parent
                parent = parents[parent_rows][parent_cols]
        
        # Add parent to closedset
        closed_set.add(parent)

        # Search each direction of the parent
        for i in range(4):
            
            # Moving direction
            action_rows = action[i][0]
            action_cols = action[i][1]
            
            # Calculate child position
            parent_rows, parent_cols = parent
            child_rows, child_cols = parent_rows + action_rows, parent_cols + action_cols
            child = child_rows, child_cols
            
            # Update g(path cost)
            g = v + costFn((parent_rows,parent_cols))
            
            # Restrict the node in the map
            if (child not in obstacles) and (child_rows < n_rows) and (child_cols < n_cols) and (child_rows >= 0) and (child_cols >= 0):
                if (child not in open_set) and (child not in closed_set):
                    
                    # Add into openset 
                    open_set.put(child,g)

                    # Record parent and action for each step
                    parents[child_rows][child_cols] = parent
                    actions[child_rows][child_cols] = tuple(action[i])
                
                # Update path cost if agent already in open list
                elif(child in open_set)and(g < open_set.get(child)):
                    
                    # Add into openset 
                    open_set.put(child,g)
                    
                    # Record parent and action for each step
                    parents[child_rows][child_cols] = parent
                    actions[child_rows][child_cols] = tuple(action[i])

def astar_search(grid_size, start, goal, obstacles, costFn, logger):

    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = PriorityQueue(order="min", f=lambda v: v)
    closed_set = OrderedSet()

    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    
    parents = [ # the immediate predecessor cell from which to reach a cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [ # the action that the parent took to reach the cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]

    movement = []

    def heuristic(row, col):
        h = abs(goal_row - row) + abs(goal_col - col)
        #print(h)
        return h
        pass    
    # Variable Initialization
    g = 0                                    # g(n)
    h = heuristic(start_row, start_col)      # h(n)
    f = g + h                                # f(n) Path cost
    
    # Add start point to openset
    open_set.put(start,f)
    
    # Define A*
    while(open_set != []): 
        
        # Take out (x,y)(parent) and path v(cost) from openset
        temp_parent = open_set.pop()
        parent,v = temp_parent
        parent_rows, parent_cols = parent
        
        # Update h
        h = heuristic(parent_rows, parent_cols)
        
        # Determine whether the agent reach to goal
        if parent == goal:
            
            # Search path
            for j in range(n_cols * n_rows):
                parent_rows, parent_cols = parent            
                
                # Stop and return
                if parent == start:
                    return movement, closed_set
                
                # record action and put it in to movement()
                act = actions[parent_rows][parent_cols]
                movement.insert(0, act)
                
                # Update parent
                parent = parents[parent_rows][parent_cols]
        
        # Add parent to closedset
        closed_set.add(parent)
        
        # Search each direction of the parent
        for i in range(4):
            
            # Moving direction
            action_rows = action[i][0]
            action_cols = action[i][1]

            # # Calculate child position
            child_rows, child_cols = parent_rows + action_rows, parent_cols + action_cols
            child = child_rows, child_cols
            
            # Update g, h and f value
            g = (v - h) + costFn((parent_rows,parent_cols))
            h_child = heuristic(child_rows, child_cols)
            f = g + h_child
            
            # Restrict the node in the map
            if (child not in obstacles) and (child_rows < n_rows) and (child_cols < n_cols) and (child_rows >= 0) and (child_cols >= 0):
                if (child not in open_set) and (child not in closed_set):
                    
                    # Add into openset 
                    open_set.put(child,f)
                    
                    # Record parent and action for each step
                    parents[child_rows][child_cols] = parent
                    actions[child_rows][child_cols] = tuple(action[i])
                
                elif(child in open_set)and(f < open_set.get(child)):
                    
                    # Add into openset 
                    open_set.put(child,f)
                    
                    # Record parent and action for each step
                    parents[child_rows][child_cols] = parent
                    actions[child_rows][child_cols] = tuple(action[i])

if __name__ == "__main__":
    # make sure actions and cost are defined correctly
    from utils.search_app import App
    assert(ACTIONS == App.ACTIONS)

    import tkinter as tk

    algs = {
        "Depth-First": depth_first_search,
        "Uniform Cost Search": uniform_cost_search,
        "A*": astar_search
    }

    root = tk.Tk()
    App(algs, root)
    root.mainloop()


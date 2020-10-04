# prmplanner.py
# ---------
from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import math
import numpy as np

disk_robot = True #(change this to False for the advanced extension) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.
    

    # the roadmap 
    graph = Roadmap()

    # Generate Vertice    
    n = 375 # Sampling numbers
    while graph.getNrVertices() < n:
        for a in range(n):
            # Genernate a vertice
            q_current = [np.random.randint(x_limit[0],x_limit[1]), np.random.randint(y_limit[0],y_limit[1])]
            append = True
            # Determine whether current vertice is valuable 
            for vertice in graph.getVertices():
                if distance(vertice.getConfiguration(),q_current) < 3:
                    append = False      
            if not collision(q_current) and append:
                graph.addVertex(q_current)
    
    # Find all valuable path for the robot off line
    path = []
    Vertices = graph.getVertices() # Get vertices from " build_roadmap"
    
    # Find nearest neighbors for current vertice
    for v in Vertices:
        q_neighbors, q_distance = k_nearest_neighbors(graph,v.getConfiguration())
        for i in range(len(q_neighbors)):
            graph.addEdge(v,q_neighbors[i],q_distance[i],path)

    #uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph
    
def find_path(q_start, q_goal, graph):
    path  = [] 
    
     # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # define start point
    x1 = q_start[0]
    y1 = q_start[1]
    
    # define the goal point
    x2 = q_goal[0]
    y2 = q_goal[1]
    
    # Define as inf
    distances = np.inf
    distanceg = np.inf
    
    start_list = graph.addVertex([x1,y1])
    end_list = graph.addVertex([x2,y2])
    
    V = graph.getVertices() 
    parent = ["" for i in range(len(V))] 
       
    for ver in V:
        distancec_s = distance(ver.q,start_list.q)
        if ver.id != start_list.id and distancec_s < distances:
            min_s = distancec_s
            distances = distancec_s
            near_s = ver
    
    for ver in V:
        distancec_g = distance(ver.q,end_list.q)
        if ver.id != end_list.id and distancec_g < distanceg:
            min_g = distancec_g
            distanceg = distancec_g
            near_g = ver
    
    graph.addEdge(start_list,near_s,min_s)
    graph.addEdge(end_list,near_g,min_g)
    
    
    # Apply A*
    ver_list = graph.getVertices()
    n = graph.getNrVertices()
    
    # define the heuristic
    heuristic = []
    for v in ver_list:
        d = distance(v.q, [q_goal[0],q_goal[1]])
        heuristic.append(d)
        
    # parameters    
    start_pos = ver_list[n-2]
    h = heuristic[start_pos.id]
    g = 0
    f = g+h
    open_set.put(start_pos, Value(f=f,g=g))

    while len(open_set) > 0:
        q, v = open_set.pop()
        g = v.g
        closed_set.add(q)
        if q.getConfiguration() == [q_goal[0],q_goal[1]]:
            print("Reach Goal Successfully")
            break
        for k in q.getEdges():
            ind = k.getDestination()
            ver_can = ver_list[ind]
            if ver_can not in closed_set:
                g_new = g + k.getDist()
                if ver_can not in open_set or open_set.get(ver_can).g > g_new:
                    f_new = g_new + heuristic[ind]
                    parent[ind] = k.getSource()
                    # Update f and g
                    open_set.put(ver_can, Value(f=f_new, g=g_new))         
    
    if(q.getConfiguration() != [q_goal[0],q_goal[1]]):
        print('Fail to get the path')
    else:
        while(q.getConfiguration() != [q_start[0],q_start[1]]):
            q = ver_list[parent[q.getId()]]  
            path.insert(0,q.getConfiguration())
    return path   

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """
    vertices = graph.getVertices()
    q_neighbor = []
    q_distance = []
    
    for v in vertices:
        if distance(q,v.getConfiguration()) < max_dist:
            q_neighbor.append(v)
            q_distance.append(distance(q,v.getConfiguration()))

    return q_neighbor, q_distance


def k_nearest_neighbors(graph, q, K=5):
    """distance()
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances
    """
    s = PriorityQueue(order=min)
    vertices = graph.getVertices()
    near_neighbor = []
    near_distance = []

    for vertice in vertices:
        if(vertice.getConfiguration()!=q and interpolate(q,vertice.getConfiguration(),1/30)==False):
            s.put(vertice,distance(q,vertice.getConfiguration()))
    for i in range(min(K,len(s))):
        vertice = s.pop()
        near_neighbor.append(vertice[0])
        near_distance.append(vertice[1])
    return near_neighbor,near_distance

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """
    distance = math.sqrt((q1[0] - q2[0])*(q1[0] - q2[0])+(q1[1] - q2[1])*(q1[1] - q2[1]))
    return distance

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """
    # Collision detection
    for obstacle in obstacles:
        # Work space to Configuration space
        if (q[0] > obstacle.x_min - robot_radius) and (q[0] < obstacle.x_max + robot_radius):  
            if (q[1] > obstacle.y_min - robot_radius) and (q[1] < obstacle.y_max + robot_radius):
                return True
    return False 
     
def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """   
    N = int(1/stepsize)
    dx = (q2[0] - q1[0])/N
    dy = (q2[1] - q1[1])/N
    for i in range(N):
        q_current = [q1[0] + (i+1) * dx, q1[1] + (i+1) * dy]
        inter = collision(q_current)
        if inter:
            return True
    return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()

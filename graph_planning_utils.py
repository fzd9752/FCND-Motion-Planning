from enum import Enum
from queue import PriorityQueue

import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString

# import sys
# !{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)



def extract_polygons(data):

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        
        # TODO: Compute the height of the polygon
        height = alt + d_alt

        p = Poly(corners, height)
        polygons.append(p)

    return polygons

class Sampler:

    def __init__(self, data, start=(317, 445, 0), goal=(150, 50, 5)):
        self._polygons = extract_polygons(data)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        # limit z-axis
        self._zmax = 20
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

        self.samples = []
        self._dist = LA.norm(np.array(goal[:2])-np.array(start[:2]))
        self._center = (np.array(goal[:2]) + np.array(start[:2])) / 2
        RAD_DEVIATION = 5
        self._radius = (self._dist / 2) + RAD_DEVIATION

        self.start = start
        self.goal = goal


    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts

    ## circular_random
    def circular_random(self, num_samples):
        '''
        Generate uniform random 2D samples in a circular area
        num_samples: number of samples (int)
        The idea and code modified based on https://github.com/ywiyogo/FCND2-3D-MotionPlanning/blob/master/prob_roadmap.py
        '''
        i = 0
        xvals = []
        yvals = []
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)

        while i < num_samples:
            r_sample, theta = np.sqrt(np.random.uniform(0, self._radius)) * np.sqrt(self._radius), 2 * np.pi * np.random.uniform(0, 1)
            x = int(self._center[0] + r_sample * np.cos(theta))
            y = int(self._center[1] + r_sample * np.sin(theta))
            if x > self._xmin or x < self._xmax:
                if y > self._ymin or y < self._ymax:
                    xvals.append(x)
                    yvals.append(y)
                    i = i + 1

        self.samples = list(zip(np.array(xvals), np.array(yvals), zvals))
        self.samples.append(self.start)
        self.samples.append(self.goal)
                      
        return self.check_collision()

    def add_nodes(self, node):
        self.samples.append(node)

    def check_collision(self):
        pts = []
        for s in self.samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)

        return pts

        # if False:  #for debugging the function
        #     plt.figure()
        #     plt.title("Uniform Random Sampling in Circle")
        #     plt.scatter(x_samples, y_samples)
        #     plt.show()

    @property
    def polygons(self):
        return self._polygons


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(nodes, k, polygons):
    g = nx.Graph()
    tree = KDTree(nodes)
    n = 0
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
        n += 1

        if n%10 == 0:
            print("Connecting %i node" %(n+1))
        
    return g

def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    # TODO: complete

    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    n = 0
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)

        n += 1
        if n%10 == 0:
            print('Hi, I\'m working on the %i loop.' %n)
             
    path = []
    path_cost = 0
    if found:      
        # retrace steps
        path.append(goal)
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost


def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))

## Add function to load first row of csv
def load_csv(filename):
    with open(filename) as f:
        lines=f.readlines()
        for line in lines:
            #print(line.strip().replace(' ', ',').split(','))
            data = line.strip().replace(' ', ',').split(',')
            lat0 = float(data[1])
            lon0 = float(data[4])
            break
    return lat0, lon0

## Add util function for collinearity check
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-5):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

## Using collinerity to prune the path
def path_prune(path, epsilon=1e-5):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        collinear = collinearity_check(p1, p2, p3)
        if collinear:
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation
import math
from shapely import geometry

from utils import *
from grid import *

def reset_grid():
    for x in range(50):
        for y in range(50):
            Point(x, y).reset_state

def reset_node(source, dest):
    source.reset_state()
    dest.reset_state()

def best_cost(cost, frontier):
    for node in frontier:
        if (cost <= node.cost):
            return True
    return False

def gen_polygons(worldfilepath):
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
            polygons.append(polygon)
    return polygons

method_counters = {"BFS" : 0, "DFS" : 0, "GBFS" : 0, "A*" : 0}
def print_to_summary(key, total_cost, nodes_expanded):
    with open("summary.txt", "a") as f:
        f.write(f"Name: {key}{method_counters[key]}\n")
        f.write(f"Path cost: {str(total_cost)}\n" )
        f.write(f"Nodes expanded: {nodes_expanded}\n\n")

def distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

# starts at the end, and keeps grabbing the parent to trace to the big parent
# solution path should never go inside an enclosed polygon so we can simply count the child.inside without worry
# set tf = True if you are running searches that require path cost calculation
def reconstruct_solution_path(node, tf):
    solution_path = []
    path_cost = 0
    while node is not None:
        solution_path.append(node)
        if node.inside and tf:
            path_cost += 1.5
        else:
            path_cost += 1
        node = node.parent
    solution_path.reverse()
    return solution_path, path_cost-1

# checks for enclosure for both enclosed polygons and turfs
def is_enclosed(point, polygon_list):
    #print(f"({point.x}, {point.y})")
    point = geometry.Point(point.x, point.y)
    for polygon in polygon_list:
        enclosure = geometry.Polygon(polygon)
        buffered_enclosure = enclosure.buffer(0.2)
        if buffered_enclosure.contains(point) or point.x < 0 or point.x >= 50 or point.y < 0 or point.y >= 50: # the point can also be outside the canvas
            return True
    return False

# actions for each searching algorithm
# set a_star=True when using A* algorithm
def search_actions(point, dest, enc_vertices, explored, a_star, turf_vertices):
    children = []
    
    # need new location by adding displacement for each direction
    for dx, dy in point.directions:
        child_x = dx + point.x
        child_y = dy + point.y
        child_node = Point(child_x, child_y)

        if is_enclosed(child_node, enc_vertices) or child_node in explored:  #skip the actions already done
            continue
        
        # this line requires all algorithms to enter turf_vertices whether or not they need path cost
        # the solution was to add a seperate check on the path cost calculation so that if we need path cost with turfs, it will always exist
        if is_enclosed(child_node, turf_vertices):
            child_node.inside = True

        if a_star:
            # h(n) is straight line distance
            hn = distance(child_node, dest)
            # g(n) is path cost 
            gn = distance(point, child_node)

            # if inside turf, add the path cost
            if child_node.inside:
                gn += 0.5

            child_node.parent = point
            child_node.heuristic = hn + gn
        else:
            child_node.parent = point
            child_node.heuristic = distance(child_node, dest)

        children.append(child_node)


    return children

# breadth first search implementation taking a source point, destination point, and the enclosed polygons
def breadth_first_search(source, dest, enc_vertices, turf_vertices):
    key = "BFS"
    method_counters[key] += 1
    nodes_expanded = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(key, 0, nodes_expanded)
        return reconstruct_solution_path(node)

    frontier = Queue()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node) 

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        actions = search_actions(node, dest, enc_vertices, explored, False, turf_vertices)
        node.set_children(actions)
        explored.append(node)

        for child in node.children:
            if child not in explored or child not in frontier:
                if child.__eq__(dest):
                    SOLUTION, path_cost = reconstruct_solution_path(child, False)
                    print_to_summary(key, path_cost, nodes_expanded)
                    return SOLUTION
                frontier.push(child)
            # each child should also be marked as visited
            explored.append(child)

# depth first search implementation taking a source point, destination point, and the enclosed polygons
def depth_first_search(source, dest, enc_vertices, turf_vertices):
    key = "DFS"
    method_counters[key] += 1
    nodes_expanded = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(key, 0, nodes_expanded)
        return reconstruct_solution_path(node)

    frontier = Stack()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node) 

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        actions = search_actions(node, dest, enc_vertices, explored, False, turf_vertices)
        node.set_children(actions)
        explored.append(node)

        if node.__eq__(dest):
            SOLUTION, path_cost = reconstruct_solution_path(node, False)
            print_to_summary(key, path_cost, nodes_expanded)
            return SOLUTION
        
        for child in node.children:
            if child not in explored or child not in frontier:
                frontier.push(child)
                explored.append(child)
                
# Greedy Best-First Search implementation taking a source point, destination point, and the enclosed polygons
def greedy_bfs_search(source, dest, enc_vertices, turf_vertices):
    key = "GBFS"
    method_counters[key] += 1
    nodes_expanded = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(key, 0, nodes_expanded)
        return reconstruct_solution_path(node)

    frontier = PriorityQueue()
    reached = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node, node.heuristic)

    while not frontier.isEmpty():        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        if node.__eq__(dest):
            SOLUTION, path_cost = reconstruct_solution_path(node, True)
            print_to_summary(key, path_cost, nodes_expanded)
            return SOLUTION
        
        actions = search_actions(node, dest, enc_vertices, reached, False, turf_vertices)
        node.set_children(actions)
        #explored.append(node)

        for child in node.children:
            if child not in reached or best_cost(child.heuristic, reached):
                reached.append(child)
                frontier.push(child, child.heuristic)


# A* Search implementation taking a source point, destination point, and the enclosed polygons
def a_star_search(source, dest, enc_vertices, turf_vertices):
    key = "A*"
    method_counters[key] += 1
    nodes_expanded = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(key, 0, nodes_expanded)
        return reconstruct_solution_path(node)

    frontier = PriorityQueue()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node, node.heuristic)

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        nodes_expanded += 1

        actions = search_actions(node, dest, enc_vertices, explored, True, turf_vertices)
        node.set_children(actions)
        explored.append(node)

        for child in node.children:
            if child not in explored or child not in frontier:
                if child.__eq__(dest):
                    SOLUTION, path_cost = reconstruct_solution_path(child, True)
                    print_to_summary(key, path_cost, nodes_expanded)
                    return SOLUTION
                frontier.push(child, child.heuristic)

        if not node.__eq__(dest):
            explored.append(node)

if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    #source = Point(24,17)
    #dest = Point(28,20)

    source = Point(8,10)
    dest = Point(43,45)

    source_copy = source
    dest_copy = dest
    #dest = Point(8,10)

    enc_polygons = []
    turf_polygons = []

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)  # source point
    draw_dest(ax, dest.x, dest.y)  # destination point
    
    # Draw enclosure polygons
    for polygon in epolygons:
        #cleared to make every individual shape
        shape = []
        for p in polygon:
            draw_point(ax, p.x, p.y)
            shape.append((p.x, p.y))
        enc_polygons.append(shape)

    for polygon in epolygons:
        for i in range(0, len(polygon)):
            draw_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])
    
    # Draw turf polygons
    for polygon in tpolygons:
        #cleared to make every individual shape
        shape = []
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
            shape.append((p.x, p.y))
        turf_polygons.append(shape)

    for polygon in tpolygons:
        for i in range(0, len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    #### Here call your search to compute and collect res_path
    
    def show_plot(res_path):
        for i in range(len(res_path)-1):
            draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
            plt.pause(0.01)
    
    def clear_plot(res_path):
        for i in range(len(res_path)-1):
            draw_result_white(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
            clear_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
             

    #add more menu here!
    menu = ["1. Breadth First Search", 
            "2. Depth First Search",
            "3. Greedy Best-First Search",
            "4. A* Search",
            "5. Run All",
            "0. Quit"
            ]

    exit_flag = False
    while not exit_flag:
        try:
            print("Please enter an option: (Please keep resulting window open)")
            print(*menu, sep="\n")
            user_input = int(input())
            res_path = None
            match user_input:
                case 1:
                    res_path = breadth_first_search(source, dest, enc_polygons, turf_polygons)
                    show_plot(res_path)
                case 2:
                    res_path = depth_first_search(source, dest, enc_polygons, turf_polygons)
                    show_plot(res_path)
                case 3:
                    res_path = greedy_bfs_search(source, dest, enc_polygons, turf_polygons)
                    show_plot(res_path)
                case 4:
                    res_path = a_star_search(source, dest, enc_polygons, turf_polygons)
                    show_plot(res_path)
                case 5:
                    print("will crash the plot lol, prints everything into summary")
                    # jank fix to not crash
                    res_path = [Point(0, 0)]
                    breadth_first_search(source, dest, enc_polygons, turf_polygons)
                    reset_node(source, dest)
                    reset_grid()
                    depth_first_search(source, dest, enc_polygons, turf_polygons)
                    reset_node(source, dest)
                    reset_grid()
                    greedy_bfs_search(source, dest, enc_polygons, turf_polygons)
                    reset_node(source, dest)
                    reset_grid()
                case 0:
                    exit_flag = True
                    res_path = [Point(0, 0)]
                case _:
                    print("Invalid option. Please try again.\n")

            reset_node(source, dest)
            reset_grid()
            clear_plot(res_path)
        except ValueError:
            print("Please enter a valid integer.\n")        
    

    # res_path = [Point(24,17), Point(25,17), Point(26,17), Point(27,17),  
    #             Point(28,17), Point(28,18), Point(28,19), Point(28,20)]
    # if not exit_flag:
    #     for i in range(len(res_path)-1):
    #         draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
    #         plt.pause(0.00001)
    plt.show()
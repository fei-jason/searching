import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation
import math
from shapely import geometry

from utils import *
from grid import *
 
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

def print_to_summary(name, total_cost, nodes_expanded):
    with open("summary.txt", "w") as f:
        f.write(f"{name}\n")
        f.write(f"Path cost: {str(total_cost)}\n" )
        f.write(f"Nodes expanded: {nodes_expanded}\n")

def distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

# starts at the end, and keeps grabbing the parent to trace to the big parent
def reconstruct_solution_path(node):
    solution_path = []
    while node is not None:
        solution_path.append(node)
        node = node.parent
    solution_path.reverse()
    return solution_path
    
def is_enclosed(point, polygon_list):
    #print(f"({point.x}, {point.y})")
    point = geometry.Point(point.x, point.y)
    for polygon in polygon_list:
        enclosure = geometry.Polygon(polygon)
        buffered_enclosure = enclosure.buffer(0.2)
        if buffered_enclosure.contains(point) or point.x < 0 or point.x >= 50 or point.y < 0 or point.y >= 50: # the point can also be outside the canvas
            return True
    return False

def is_turf(point, polygon_list):
    #print(f"({point.x}, {point.y})")
    point = geometry.Point(point.x, point.y)
    for polygon in polygon_list:
        enclosure = geometry.Polygon(polygon)
        buffered_enclosure = enclosure.buffer(0.2)
        if buffered_enclosure.contains(point) or point.x < 0 or point.x >= 50 or point.y < 0 or point.y >= 50: # the point can also be outside the canvas
            return True
    return False

def actions_bfs_dfs(point, dest, enc_vertices, explored):
    children = []
    
    # need new location by adding displacement for each direction
    for dx, dy in point.directions:
        child_x = dx + point.x
        child_y = dy + point.y
        child_node = Point(child_x, child_y)

        if is_enclosed(child_node, enc_vertices) or child_node in explored:  #skip the actions already done
            continue
        else:
            child_node.parent = point
            child_node.heuristic = distance(child_node, dest)
            children.append(child_node)

    return children

# breadth first search implementation taking a source point, destination point, and the enclosed polygons
def breadth_first_search(source, dest, enc_vertices):
    name = "Breadth First Search"
    nodes_expanded = 0
    path_cost = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(name, path_cost, nodes_expanded)
        return reconstruct_solution_path(child)

    frontier = Queue()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node) 
    explored.append(node)

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        actions = actions_bfs_dfs(node, dest, enc_vertices, explored)
        node.set_children(actions)
        explored.append(node)

        for child in node.children:
            if child.__eq__(dest):
                SOLUTION = reconstruct_solution_path(child)
                print_to_summary(name, len(SOLUTION)-1, nodes_expanded)
                return SOLUTION
            elif child not in explored:
                frontier.push(child)
            explored.append(child)

# depth first search implementation taking a source point, destination point, and the enclosed polygons
def depth_first_search(source, dest, enc_vertices):
    name = "Depth First Search"
    nodes_expanded = 0
    path_cost = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(name, path_cost, nodes_expanded)
        return reconstruct_solution_path(child)

    frontier = Stack()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node) 
    explored.append(node)

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        actions = actions_bfs_dfs(node, dest, enc_vertices, explored)
        node.set_children(actions)
        explored.append(node)

        for child in node.children:
            if child.__eq__(dest):
                SOLUTION = reconstruct_solution_path(child)
                print_to_summary(name, len(SOLUTION)-1, nodes_expanded)
                return SOLUTION
            elif child not in explored:
                frontier.push(child)
            explored.append(child)

# Greedy Best-First Search implementation taking a source point, destination point, and the enclosed polygons
def greedy_bfs(source, dest, enc_vertices):
    name = "Breadth First Search"
    nodes_expanded = 0
    path_cost = 0
    node = source
    node.heuristic = distance(source, dest)

    # return here
    if source.__eq__(dest):
        print_to_summary(name, path_cost, nodes_expanded)
        return reconstruct_solution_path(child)

    frontier = PriorityQueue()
    explored = [] # i couldn't use a set bc Point isn't iterable
    frontier.push(node, node.heuristic)
    explored.append(node)

    while True:
        if frontier.isEmpty():
            #failure
            return False
        
        node = frontier.pop()
        #print(f"{nodes_expanded}")
        nodes_expanded += 1

        actions = actions_bfs_dfs(node, dest, enc_vertices, explored)
        node.set_children(actions)
        explored.append(node)

        for child in node.children:
            if child.__eq__(dest):
                SOLUTION = reconstruct_solution_path(child)
                print_to_summary(name, len(SOLUTION)-1, nodes_expanded)
                return SOLUTION
            elif child not in explored:
                frontier.push(child, child.heuristic)
            explored.append(child)

if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    #source = Point(24,17)
    #dest = Point(28,20)
    source = Point(8,10)
    dest = Point(43,45)

    enc_polygons = []
    tur_polygons = []

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
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
    for polygon in tpolygons:
        for i in range(0, len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    #### Here call your search to compute and collect res_path
    #res_path = breadth_first_search(source, dest, enc_polygons)
    #res_path = depth_first_search(source, dest, enc_polygons)
    res_path = greedy_bfs(source, dest, enc_polygons)

    # res_path = [Point(24,17), Point(25,17), Point(26,17), Point(27,17),  
    #             Point(28,17), Point(28,18), Point(28,19), Point(28,20)]

    for i in range(len(res_path)-1):
        draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
        plt.pause(0.00001)

    
    plt.show()

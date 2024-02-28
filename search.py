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

def reconstruct_solution_path(node):
    solution_path = []
    while node is not None:
        solution_path.append(node)
        node = node.parent
    solution_path.reverse()
    return solution_path

def is_enclosed(point, polygon_list):
    for polygon in polygon_list:
        enclosure = geometry.Polygon(polygon)
        if enclosure.contains(geometry.Point(point.x, point.y)):
            return True
    return False

def actions_bfs_dfs(point, dest, enc_vertices):
    x = point.x
    y = point.y

    children = []

    # need new location by adding dispalcement
    for dx, dy in point.directions:
        child_x = x + dx
        child_y = y + dy
        child_node = Point(child_x, child_y)

        if is_enclosed(child_node, enc_vertices):
            continue
        else:
            child_node.parent = point
            child_node.heuristic = distance(child_node, dest)
            children.append(child_node)

    return children

def breadth_first_search(source, dest, enc_vertices):
    total_cost = 0
    nodes_expanded = 0

    node = source
    node.heuristic = distance(node, dest)

    # return here
    

    frontier = Queue()
    explored = []
    frontier.push(node)
    explored.append(node)

    while not frontier.isEmpty():
        if frontier.isEmpty():
            print("failure")
            return None
        
        node = frontier.pop()
        nodes_expanded += 1
        total_cost += node.heuristic
        explored.append(node)

        actions = actions_bfs_dfs(node, dest, enc_vertices)
        node.set_children(actions)

        for child in node.children:
            if child not in explored:
                explored.append(child)
                frontier.push(child)
            elif child.__eq__(dest):
                explored.append(child)
                print_to_summary("bfs1", total_cost, nodes_expanded)
                return reconstruct_solution_path(child)
    return []

if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    #source = Point(24,17)
    #dest = Point(28,20)
    source = Point(8,10)
    dest = Point(43,45)

    enc_polgons = []
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
        enc_polgons.append(shape)

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
    res_path = breadth_first_search(source, dest, enc_polgons)

    # res_path = [Point(24,17), Point(25,17), Point(26,17), Point(27,17),  
    #             Point(28,17), Point(28,18), Point(28,19), Point(28,20)]

    for i in range(len(res_path)-1):
        draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
        plt.pause(0.1)

    
    plt.show()

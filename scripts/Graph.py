#!/usr/bin/env python

import rospy
import networkx as nx
import math as m
from geometry_msgs.msg import Point
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
from matplotlib import collections as mc

xlim = [0, 2000]
ylim = [0, 2000]
node_radius = 50


class Graph:
    def __init__(self):
        pass

    # Determines if a node should be deleted
    def obstacle_in_node(self, node, x, y, r):
        # Obstacle projection to a square
        figure1 = Polygon([(x-r, y-r), (x+r, y-r), (x+r, y+r), (x-r, y+r)])
        # Node projection to a square
        figure2 = Polygon([(node[1][0], node[2][0]), (node[1][1], node[2][0]), (node[1][1], node[2][1]), (node[1][0], node[2][1])])
        # True if both elements intersect
        # False otherwise
        return figure1.intersects(figure2)

    # Principal method
    def graph_building(self, obstacle_list):
        # Node extension/resolution
        # Limits to determine graph edges
        distance_low_trigger = node_radius
        distance_sup_trigger = m.sqrt(2*node_radius**2)
        # Total nodes to be created in each direction
        x_total = int((xlim[1]-xlim[0])/node_radius)
        y_total = int((ylim[1]-ylim[0])/node_radius)
        # Node creation
        # add_node[0] Node identifier
        # add_node[1] Node x interval
        # add_node[2] Node y interval
        # add_node[3] Node center x coordinate
        # add_node[4] Node center y coordinate
        # add_node[5] Node center
        nodes = []
        for i in range(0, x_total):
            for j in range(0, y_total):
                add_node = [0, 0, 0, 0, 0, 0]
                add_node[0] = y_total*i + j
                add_node[1] = [xlim[0] + i * node_radius, (xlim[0] + i * node_radius) + node_radius]
                add_node[2] = [ylim[0] + j * node_radius, (ylim[0] + j * node_radius) + node_radius]
                add_node[3] = (add_node[1][0] + add_node[1][1]) / 2
                add_node[4] = (add_node[2][0] + add_node[2][1]) / 2
                add_node[5] = (add_node[3], add_node[4])
                nodes.append(add_node)
        # List containing nodes with obstacle
        nodes_to_remove = []
        nodes_to_remove_id = []
        for j in obstacle_list:
            for i in range(0, len(nodes)):
                if self.obstacle_in_node(nodes[i], j[0], j[1], j[2]+250):
                    nodes_to_remove.append(nodes[i])
                    nodes_to_remove_id.append(nodes[i][0])

        # Removing nodes
        for i in nodes_to_remove:
            if i in nodes:
                nodes.remove(i)
        # Graph object creation
        G = nx.Graph()
        # Adding nodes
        for i in nodes:
            G.add_node(i[5])
        edges = []
        lines = []
        # Existing edges according to limits
        for i in nodes:
            center_i = i[3] + i[4] * 1j
            for j in nodes:
                center_j = j[3] + j[4] * 1j
                distance = abs(center_i-center_j)
                if (distance >= distance_low_trigger) and (distance <= distance_sup_trigger):
                    edges.append((i[5], j[5], {'weight': distance}))
                    # Creating lines list
                    lines.append((i[5], j[5]))
                    # edges.append((i[0], j[0]))
        # Adding edges
        G.add_edges_from(edges)
        # Plotting graph
        aux_x = [x[3] for x in nodes]
        aux_y = [x[4] for x in nodes]
        #lc = mc.LineCollection(lines)
        #fig, ax = plt.subplots()
        #ax.add_collection(lc)
        #plt.scatter(aux_x, aux_y)
        #plt.show()
        return G
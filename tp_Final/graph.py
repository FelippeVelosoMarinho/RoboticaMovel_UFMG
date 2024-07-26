#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  graph.py
#
#  Copyright 2012 Kevin R <KRPent@gmail.com>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#

import os
import json
import random
from graphviz import Graphviz

## @brief Represents a directed graph of nodes and edges.
# 
# The directed graph is identified with a supplied name and stored in the 
# "data/json/" folder if saved. Initially the graph contains zero nodes, but
# can be populated automatically by loading the data of the supplied name. The
# graph can also be randomly generated, or manually configured.
class DiGraph:
    ## The location that the graph data is stored as json objects.
    _directory_data = "data/json/"
    
    ## The Graphviz object that will be used to display the graph.
    _painter = None
    
    ## An edge with this cost signifies that it has been removed from the graph.
    # This value implies that any edge in the graph must be very small in 
    # comparison.
    INFINITY = 10000
    
    ## Represents a NULL predecessor.
    UNDEFINED = None
    
    ## Specifies the identifier for the graph.
    _name = "graph"
    
    ## The dictionary of the graph. Each key represents a node and the value of
    # the associated node is a dictionary of all the edges. The key of the edges
    # dictionary is the node the edge terminates at and the value is the cost of
    # the edge.
    _data = {}
    _hdata = {}
    
    ## Initializes the graph with an identifier and Graphviz object.
    #    
    # @post The graph will contain the data specified by the identifier, if that
    # data exist. If not, then the graph will be empty.
    #
    # @param self The object pointer.
    # @param name The identifier for the graph by which the data is stored as or
    # will be stored as.
    # 
    def __init__(self, name=None):
        if name:
            self._name = name
        
        self.load()
        
        self._painter = Graphviz()
        return

    ## Gets the edges of a specified node.
    #
    # @param self The object pointer.
    # @param node The node whose edges are being queried.
    # @retval {} A dictionary of the edges and their cost if the node exists 
    # within the graph or None if the node is not in the graph.
    #
    def __getitem__(self, node):
        if node in self._data:
            return self._data[node]
        else:
            return None

    def getSignature(self, node1, node2):
        if node1 in self._hdata:
            return self._hdata[node1][node2]
        else:
            return None

    ## Iterator for the digraph object.
    #
    # @param self The object pointer.
    # @retval iter An iterator that can be used to process each node of the 
    # graph.
    #
    def __iter__(self):
        return iter(self._data)

    ## Adds a node to the graph.
    #
    # @param self The object pointer.
    # @param node The name of the node that is to be added to the graph.
    # @retval bool True if the node was added or False if the node already 
    # existed in the graph.
    #
    def add_node(self, node):
        if node in self._data:
            return False

        self._data[node] = {}
        self._hdata[node] = {}

        return True

    ## Adds an edge to the graph.
    #
    # @post The two nodes specified exist within the graph and there exists an
    # edge between them of the specified value.
    #
    # @param self The object pointer.
    # @param node_from The node that the edge starts at.
    # @param node_to The node that the edge terminates at.
    # @param cost The cost of the edge, if the cost is not specified a random
    # cost is generated from 1 to 10.
    #
    def add_edge(self, node_from, node_to, cost=None, hSignature=[]):
        if not cost:
            cost = random.randrange(1, 11)
        
        self.add_node(node_from)
        self.add_node(node_to)
        
        self._data[node_from][node_to] = cost
        self._hdata[node_from][node_to] = hSignature
    
        return

    ## Removes an edge from the graph.
    #
    # @param self The object pointer.
    # @param node_from The node that the edge starts at.
    # @param node_to The node that the edge terminates at.
    # @param cost The cost of the edge, if the cost is not specified all edges
    # between the nodes are removed.
    # @retval int The cost of the edge that was removed. If the nodes of the 
    # edge does not exist, or the cost of the edge was found to be infinity, or 
    # if the specified edge does not exist, then -1 is returned.
    #
    def remove_edge(self, node_from, node_to, cost=None):
        if node_from not in self._data:
            return -1
        
        if node_to in self._data[node_from]:
            if not cost:
                cost = self._data[node_from][node_to]
                
                if cost == self.INFINITY:
                    return -1
                else:
                    self._data[node_from][node_to] = self.INFINITY
                    return cost
            elif self._data[node_from][node_to] == cost:
                self._data[node_from][node_to] = self.INFINITY
                
                return cost
            else:
                return -1
        else:
            return -1
    
    ## Populates the graph with the data of the graph identifier.
    #
    # @pre The _name variable has been set and there exists a ".json" file at
    # the directory specified by _directory_data with the graph data.
    # @post The _data dictionary will contain all the nodes and edges of the 
    # graph.
    #
    # @param self The object pointer.
    # @retval bool True if the graph was populated from the specified file, 
    # False otherwise.
    # 
    def load(self):
        path_json = f"{self._directory_data}{self._name}.json"
        if not os.path.exists(path_json):
            return False
        
        with open(path_json, 'r') as fhandle:
            self._data = json.load(fhandle)
        
        return True
    
    ## Stores the nodes and edges of the graph.
    #
    # @pre The _name variable has been set and the _data dictionary contains all
    # the nodes and edges of the graph.
    # @post There exists a ".json" file at the directory specified by 
    # _directory_data with the graph data.
    # 
    # @param self The object pointer.
    # 
    def save(self):
        if not os.path.exists(self._directory_data):
            os.makedirs(self._directory_data)
        
        with open(f"{self._directory_data}{self._name}.json", 'w') as fhandle:
            json.dump(self._data, fhandle)
        
        return
    
    ## Generates an image of the graph.
    #
    #

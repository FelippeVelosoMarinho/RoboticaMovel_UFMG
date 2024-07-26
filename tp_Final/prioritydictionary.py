#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  prioritydictionary.py
#  
#  Copyright 2002 David Eppstein, UC Irvine
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
# http://code.activestate.com/recipes/117228/
#

import heapq

class priorityDictionary(dict):
    def __init__(self):
        '''Initialize PriorityDictionary by creating a binary heap of pairs 
        (value, key). Note that changing or removing a dict entry will not 
        remove the old pair from the heap until it is found by smallest() or
        until the heap is rebuilt.'''
        self.__heap = []
        self.__entry_finder = {}  # mapping of tasks to entries
        self.__REMOVED = '<removed-task>'  # placeholder for a removed task
        super().__init__()

    def smallest(self):
        '''Find the smallest item after removing deleted items from the heap.'''
        while self.__heap:
            value, key = self.__heap[0]
            if key is not self.__entry_finder or self.__entry_finder[key] != value:
                heapq.heappop(self.__heap)
            else:
                return key
        raise IndexError("smallest of empty PriorityDictionary")

    def __iter__(self):
        '''Create a destructive sorted iterator of PriorityDictionary.'''
        while self:
            x = self.smallest()
            yield x
            del self[x]

    def __setitem__(self, key, val):
        '''Change value stored in dictionary and add corresponding pair to heap.  
        Rebuilds the heap if the number of deleted items grows too large, to 
        avoid memory leakage.'''
        if key in self.__entry_finder:
            self.remove_task(key)
        self.__entry_finder[key] = val
        heapq.heappush(self.__heap, (val, key))
        super().__setitem__(key, val)

    def remove_task(self, key):
        '''Mark an existing task as REMOVED.'''
        self.__entry_finder[key] = self.__REMOVED

    def setdefault(self, key, val):
        '''Reimplement setdefault to call our customized __setitem__.'''
        if key not in self:
            self[key] = val
        return self[key]

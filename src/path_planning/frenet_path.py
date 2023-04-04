#!/usr/bin/env python
#-*-coding:utf-8-*-
"""
Created on Mon Aug 24 15:12:36 2020

@author IHS
"""

class Frenet_path:
    
    def __init__(self):
        self.s = []
        self.q = []

        self.x = []
        self.y = []
        self.yaw = []
        self.k = []
        
        self.offset_cost = 0
        self.obs_distance = 0
        self.consistency_cost = 0
        self.total_cost = 0
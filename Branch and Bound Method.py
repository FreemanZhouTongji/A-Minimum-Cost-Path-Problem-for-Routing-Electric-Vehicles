# -*- coding: utf-8 -*-
"""
Created on Sat Jan  4 21:46:48 2020

@author: 吴冕
"""

from gurobipy import *
import pandas as pd

#%%
# Network Config
f = open(r'edge.csv')
df_edge = pd.read_csv(f)
f = open(r'node.csv')
df_node = pd.read_csv(f)

#%%
edge = {}
for row in range(len(df_edge)):
    i = int(df_edge.loc[row,'point1'])
    j = int(df_edge.loc[row,'point2'])
    dis = df_edge.loc[row,'dis']
    edge[i, j] = dis
    edge[j, i] = dis

arcs, dist = multidict(edge)
arcs = tuplelist(arcs)

#%%
# Parameters Setting
M = 1e4
VOT = 0.5   # ￥/h
speed = 10   # km/h

node_num = len(df_node)
charging_node = list(df_node['charger'])     # whether charging node or not
charging_price = list(df_node['price'])     # charging price

battery_capacity = 100      # battery capacity
battery_init = 75      # battery initial state
elec_consuming_rate = 0.2   # power consuming rate

#%% 
#Modelling
m = Model('VehicleCharging')
x = {}
e = {}
a = {}
b = {}

for arc in arcs:
    x[arc[0], arc[1]] = m.addVar(0, 1, 0, GRB.BINARY, 'x')       

for i in range(node_num):
    e[i] = m.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, 'e%s'%i)
    a[i] = m.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, 'a%s'%i)
    b[i] = m.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, 'b%s'%i)
    
m.update()

m.addConstr(a[0] == battery_init, 'a_r')
m.addConstr(b[0] == battery_init, 'b_r')
m.addConstr(a[node_num-1] >= 0, 'a_s')

# flow conservation constraints for r
outflow = quicksum(x[0, j] for i, j in arcs.select(0, '*'))
m.addConstr(outflow == 1, 'flow_conservation%s'%i)

# flow conservation constraints for s
inflow = quicksum(x[i, node_num-1] for i, j in arcs.select('*', node_num-1))
m.addConstr(inflow == 1, 'flow_conservation%s'%i)

for i in range(1, node_num-1):
    # flow conservation constraints
    inflow = quicksum(-x[i, j] for i, j in arcs.select('*', i))
    outflow = quicksum(x[i, j] for i, j in arcs.select(i, '*'))
    m.addConstr(inflow + outflow == 0, 'flow_conservation%s'%i)  

    # electricity constraints
    m.addConstr(a[i] >= 0, 'e_alpha%s'%i)
    
    # charging constraints
    m.addConstr(a[i] + e[i] == b[i], 'charging%s'%i)
    m.addConstr(e[i] >= 0, 'e_min%s'%i)   
    m.addConstr(e[i] <= charging_node[i] * M, 'e_max%s'%i)
    m.addConstr(b[i] <= battery_capacity, 'e_beta%s'%i)

for i,j in arcs:
    # electricity consumption
    elec = a[j] - b[i] + elec_consuming_rate * dist[i, j]
    m.addConstr(elec >= M * (x[i, j] - 1), 'power_consuming_lb_%s_%s'%(i,j)) 
    m.addConstr(elec <= M * (1 - x[i, j]), 'power_consuming_lb_%s_%s'%(i,j))

obj1 = quicksum(e[i]*charging_price[i] for i in range(0, node_num))
obj2 = quicksum(x[i, j] * (VOT * dist[i, j] / speed) for i, j in arcs)
m.setObjective(obj1 + obj2, GRB.MINIMIZE)
m.optimize()

path = []
charging = {}
if m.status == GRB.Status.OPTIMAL:
    print('Successfully Solved!')
    
    for arc in arcs:
        if x[arc[0], arc[1]].x > 0:
            path.append(arc)
            
    for i in range(node_num):
        if e[i].x > 0:
            charging[i] = e[i].x
        
    print('Shortest path:\n', path)
    print('Charging strategy:\n', charging)
 
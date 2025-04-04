#!/usr/bin/env python3
# encoding: utf-8
"""
tests.test_path
~~~~~~~~~~~~~~~

:author: Wannes Meert
:copyright: Copyright 2017-2018 DTAI, KU Leuven and Sirris.
:license: Apache License, Version 2.0, see LICENSE for details.
"""
# import sys
import os
# import logging
from pathlib import Path
import numpy as np

# import leuvenmapmatching as mm
from leuvenmapmatching.map.inmem import InMemMap
from leuvenmapmatching.matcher.simple import SimpleMatcher
from leuvenmapmatching import visualization as mmviz
from leuvenmapmatching.matcher.distance import DistanceMatcher

# logger = mm.logger
# directory = None


# def test_path1():
#     path = [(0.8, 0.7), (0.9, 0.7), (1.1, 1.0), (1.2, 1.5), (1.2, 1.6), (1.1, 2.0),
#             (1.1, 2.3), (1.3, 2.9), (1.2, 3.1), (1.5, 3.2), (1.8, 3.5), (2.0, 3.7),
#             (2.1, 3.3), (2.4, 3.2), (2.6, 3.1), (2.9, 3.1), (3.0, 3.2), (3.1, 3.8),
#             (3.0, 4.0), (3.1, 4.3), (3.1, 4.6), (3.0, 4.9)]
#     # path_sol = ['A', ('A', 'B'), 'B', ('B', 'D'), 'D', ('D', 'E'), 'E', ('E', 'F')]
#     path_sol_nodes = ['A', 'B', 'D', 'E', 'F']
#     mapdb = InMemMap("map", graph={
#         "A": ((1, 1), ["B", "C"]),
#         "B": ((1, 3), ["A", "C", "D"]),
#         "C": ((2, 2), ["A", "B", "D", "E"]),
#         "D": ((2, 4), ["B", "C", "D", "E"]),
#         "E": ((3, 3), ["C", "D", "F"]),
#         "F": ((3, 5), ["D", "E"])
#     }, use_latlon=False)

#     matcher = SimpleMatcher(mapdb, max_dist=None, min_prob_norm=None,
#                             non_emitting_states=False, only_edges=False)
#     path_pred, _ = matcher.match(path, unique=True)
    
#     matcher.print_lattice_stats()
#     matcher.print_lattice()
    
#     mmviz.plot_map(mapdb, matcher=matcher, show_labels=True, show_matching=True,
#                     show_graph=True, show_lattice=True,
#                     filename=str(directory / "test_path1.png"))
#     # assert path_pred == path_sol, f"Paths not equal:\n{path_pred}\n{path_sol}"
#     nodes_pred = matcher.path_pred_onlynodes
#     assert nodes_pred == path_sol_nodes, f"Nodes not equal:\n{nodes_pred}\n{path_sol_nodes}"

# def test_path1_dist():
#     path = [(0.8, 0.7), (0.9, 0.7), (1.1, 1.0), (1.2, 1.5), (1.2, 1.6), (1.1, 2.0),
#             (1.1, 2.3), (1.3, 2.9), (1.2, 3.1), (1.5, 3.2), (1.8, 3.5), (2.0, 3.7),
#             (2.1, 3.3), (2.4, 3.2), (2.6, 3.1), (2.9, 3.1), (3.0, 3.2), (3.1, 3.8),
#             (3.0, 4.0), (3.1, 4.3), (3.1, 4.6), (3.0, 4.9)]
#     # path_sol = ['A', ('A', 'B'), 'B', ('B', 'D'), 'D', ('D', 'E'), 'E', ('E', 'F')]
#     path_sol_nodes = ['A', 'B', 'D', 'E', 'F']
#     mapdb = InMemMap("map", graph={
#         "A": ((1, 1), ["B", "C"]),
#         "B": ((1, 3), ["A", "C", "D"]),
#         "C": ((2, 2), ["A", "B", "D", "E"]),
#         "D": ((2, 4), ["B", "C", "D", "E"]),
#         "E": ((3, 3), ["C", "D", "F"]),
#         "F": ((3, 5), ["D", "E"])
#     }, use_latlon=False)

#     matcher = DistanceMatcher(mapdb, max_dist=None, min_prob_norm=None,
#                               obs_noise=0.5,
#                               non_emitting_states=False)
#     matcher.match(path)
#     mmviz.plot_map(mapdb, matcher=matcher, show_labels=True, show_matching=True, show_graph=True,
#                     filename=str(directory / "test_path1_dist.png"))
#     nodes_pred = matcher.path_pred_onlynodes
#     assert nodes_pred == path_sol_nodes, f"Nodes not equal:\n{nodes_pred}\n{path_sol_nodes}"

def my_test():
    path = [(0.8, 0.7), (0.9, 0.7), (1.1, 1.0), (1.2, 1.5), (1.2, 1.6), (1.1, 2.0),
            (1.1, 2.3), (1.3, 2.9), (1.2, 3.1), (1.5, 3.2), (1.8, 3.5), (2.0, 3.7),
            (2.1, 3.3), (2.4, 3.2), (2.6, 3.1), (2.9, 3.1), (3.0, 3.2), (3.1, 3.8),
            (3.0, 4.0), (3.1, 4.3), (3.1, 4.6), (3.0, 4.9)]
    # path_sol = ['A', ('A', 'B'), 'B', ('B', 'D'), 'D', ('D', 'E'), 'E', ('E', 'F')]
    path_sol_nodes = ['A', 'B', 'D', 'E', 'F']
    mapdb = InMemMap("map", graph={
        "A": ((1, 1), ["B", "C"]),
        "B": ((1, 3), ["A", "C", "D"]),
        "C": ((2, 2), ["A", "B", "D", "E"]),
        "D": ((2, 4), ["B", "C", "D", "E"]),
        "E": ((3, 3), ["C", "D", "F"]),
        "F": ((3, 5), ["D", "E"]),
        "G": ((2,1),["A","ad"])
    }, use_latlon=False)

    matcher = SimpleMatcher(mapdb, max_dist=None, min_prob_norm=None,
                            non_emitting_states=False, only_edges=False)
    path_pred, _ = matcher.match(path, unique=True)
    
    # matcher.print_lattice_stats()
    # matcher.print_lattice()
    nodes_pred = matcher.path_pred_onlynodes
    mmviz.plot_map(mapdb, path=path,nodes=nodes_pred, show_labels=True, show_matching=True,
                    show_graph=True, show_lattice=False,
                    filename=str(directory / "my_test.png"))
    # assert path_pred == path_sol, f"Paths not equal:\n{path_pred}\n{path_sol}"
    # nodes_pred = matcher.path_pred_onlynodes
    # assert nodes_pred == path_sol_nodes, f"Nodes not equal:\n{nodes_pred}\n{path_sol_nodes}"

def read_mmpath(path,index):
    file=open(path, "r")
    mmpath=[]
    while(True):
        line=file.readline()
        # print(line)
        if len(line)>0:
            mmpath.append(int(line))
        else:
            break

    file.close()
    return mmpath
def read_trace(path,index):
    file=open(path, "r")
    # num=file.readline()
    # print(num)
    trace=[]
    start_record=False
    tmp=None
    while(True):
        a=file.readline().replace('\n', '').split(sep=" ")
        if len(a)==3:
            trace.append((float(a[1]),float(a[2])))
        else:
            break
    file.close()
    return trace
def read_road(path):
    file=open(path, "r")
    # num=int(file.readline())
    # print(num)
    roads=[]
    # for i in range(num):
    while True:
        road=file.readline().split(sep=" ")[0:-1]
        if road:
            # print(a)
            roads.append(road)
        else:
            break
    file.close()
    return roads
def build_neighbor_map(roads,mmpath):
    graph={}
    for road in roads:

        graph=road2graph(road,graph)
        # nodes.append(road[0]+'_0')
        # graph[road[0]]= ((float(road[7]), float(road[6])), [road[1], road[2]])
        # graph[predecessor[0]]= ((float(predecessor[7]), float(predecessor[6])), [predecessor[1], predecessor[2]])
        # graph[successor[0]]= ((float(successor[7]), float(successor[6])), [successor[1], successor[2]])
    mapdb = InMemMap("map", graph=graph, use_latlon=True)
    return mapdb,graph
def build_map(roads,mmpath):
    graph={}
    roads_id=[int(i[0]) for i in roads]
    for index in mmpath:
        
        # print(roads_id)
        i=roads_id.index(index)
        road=roads[i]
        # predecessor=roads[int(roads[index][1])]
        # successor=roads[int(roads[index][1])]
        graph=road2graph(road,graph)
        # nodes.append(road[0]+'_0')
        # graph[road[0]]= ((float(road[7]), float(road[6])), [road[1], road[2]])
        # graph[predecessor[0]]= ((float(predecessor[7]), float(predecessor[6])), [predecessor[1], predecessor[2]])
        # graph[successor[0]]= ((float(successor[7]), float(successor[6])), [successor[1], successor[2]])
    mapdb = InMemMap("map", graph=graph, use_latlon=True)
    return mapdb,graph
# def build_nodes(graph,trace):
#     nodes=[]
#     for pos in trace:
#         dis=10000
#         node=None
#         for point in graph:
#             x_diff=graph[point][0][0]-pos[0]
#             y_diff=graph[point][0][1]-pos[1]
#             d_diff=np.sqrt(x_diff**2+y_diff**2)
#             # print(x_diff,y_diff)
#             if d_diff<dis:
#                 dis=d_diff
#                 node=point
#         nodes.append(node)
#     nodes_unique=[]
#     [nodes_unique.append(i) for i in nodes if not i in nodes_unique]
#     return nodes_unique
def build_nodes(roads,mm_id,trace):
    nodes=[]
    roads_id=[int(i[0]) for i in roads]
    for i in range(len(trace)):
        dis=10000
        node=None
        # print(roads_id)
        road_index=roads_id.index(mm_id[i])
        road=roads[road_index]
        num=int(road[5])
        for j in range(num):
            lon=float(road[7+2*j])
            lat=float(road[6+2*j])
            x_diff=lon-trace[i][1]
            y_diff=lat-trace[i][0]
            d_diff=np.sqrt(x_diff**2+y_diff**2)
            if d_diff<dis:
                dis=d_diff
                node=road[0]+'_'+str(j)
        nodes.append(node)
    # print("len nodes:",len(nodes))
    # print(nodes)
    nodes_unique=[]
    [nodes_unique.append(i) for i in nodes if not i in nodes_unique]
    # print("len nodes:",len(nodes_unique))
    return nodes_unique            
def road2graph(road,graph):
    num=int(road[5])
    for i in range(num):
        point_name=road[0]+'_'+str(i)
        lon=float(road[7+2*i])
        lat=float(road[6+2*i])
        link1=road[0]+'_'+str(i-1)
        link2=road[0]+'_'+str(i+1)
        if i==0:
            link1=road[1]
        if i==num-1:
            link2=road[2]
        graph[point_name]= ((lat, lon), [link1, link2])
    
    return graph
#TODO
#dynamic visualization
if __name__ == "__main__":
    directory = Path(os.environ.get('TESTDIR', Path(__file__).parent))
    print(f"Saving files to {directory}")
    # my_test()
    roads=read_road("./madmap_candidate_road.txt")
    index=[0,1000000]
    mm_id=read_mmpath("./mm_out_standard.txt",index)
    # print(mmpath)
    print("mm_id",len(mm_id))
    trace=read_trace("./trace.txt",index)
    #trace_corrected=read_trace("./trace_corrected.txt",index)
    # trace=read_trace("./hp_out.txt",index)
    print("trace",len(trace))


    mapdb,graph=build_map(roads,mm_id)

    nodes=build_nodes(roads,mm_id,trace)
    mapdb_full,graph_full=build_neighbor_map(roads,mm_id)
    mmviz.plot_map(mapdb_full,path=trace, nodes=nodes,
                    show_labels=False, show_matching=True,
                    show_graph=True, show_lattice=False,filename=str(directory / "full.png"),
                    dpi=300)
    mmviz.plot_map(mapdb,path=trace, nodes=nodes,
                    show_labels=False, show_matching=True,
                    show_graph=True, show_lattice=False,filename=str(directory / "mmroads.png"),
                    dpi=300)
    '''mmviz.plot_map(mapdb_full,path=trace_corrected, nodes=nodes,
                    show_labels=False, show_matching=True,
                    show_graph=True, show_lattice=False,filename=str(directory / "full_corrected.png"),
                    dpi=300)
    mmviz.plot_map(mapdb,path=trace_corrected, nodes=nodes,
                    show_labels=False, show_matching=True,

                    show_graph=True, show_lattice=False,filename=str(directory / "mmroads_corrected.png"),
                    dpi=300)'''

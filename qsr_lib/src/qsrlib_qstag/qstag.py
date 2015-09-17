#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Qualitative Spatio-Temporal Activity Graph module

"""
from __future__ import print_function
import sys, copy
import argparse
from datetime import datetime
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import World_Trace, Object_State
from qsrlib_io.world_qsr_trace import World_QSR_Trace

from igraph import Graph
import itertools

class Activity_Graph():
    '''
    Activity Graph class:
    Lower level is a set of only nodes of type 'object'. Middle level nodes are only of
    type 'spatial_relation'. Top level nodes are only of type 'temporal_relation'.
    Accepts input file which is a plain text file with each line an interaction of
    a pair of objects.
    Header: object1, object1_type, object2, object2_type, spatial relation, start time, end time

    Example content:
    ----------------
    o1,mug,o2,hand,sur,3,7
    o1,mug,o3,head,con,4,9
    o2,hand,o3,head,dis,1,9

    OR

    Accepts a list of episodes where each episode is a tuple with above structure.
    '''

    def __init__(self, episodes, params):
        self.spatial_obj_edges  = []
        self.temp_spatial_edges = []
        self.episodes           = episodes
        self.graph              = self.get_activity_graph(episodes)


    @property
    def abstract_graph(self):
        #abstract_graph = Graph.copy(self.graph)
        abstract_graph = self

        for o_node in self.object_nodes:
            #Activity Graph code:

            #Set the name of the object node equal to the type.
            o_node['name']=o_node['obj_type']

            #to remove object node type and name:
            """o_node['obj_type'] = 'Unknown'
            o_node['name']='Unknown_object'
            """
        return abstract_graph

    @property
    def object_nodes(self):
        # Get object nodes from graph
        object_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'object':
                object_nodes.append(node)
        return object_nodes

    @property
    def abstract_object_nodes(self):
        # Get object nodes from abstract graph
        object_nodes = []
        for node in self.abstract_graph.vs():
            if node['node_type'] == 'object':
                object_nodes.append(node)
        return object_nodes

    @property
    def spatial_nodes(self):
        # Get spatial relation nodes from graph
        spatial_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'spatial_relation':
                spatial_nodes.append(node)
        return spatial_nodes

    @property
    def temporal_nodes(self):
        # Get temporal relation nodes from graph
        temporal_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'temporal_relation':
                temporal_nodes.append(node)
        return temporal_nodes



    def get_E_set(self, objects, episodes_selection):
        """returns the Starting episode set (E_s) and the Endding episode set (E_s)
        See Sridar_aaai_2010 for more details
        """
        objects = objects.values()
        start, end = {}, {}
        E_s = [], E_f

        for obj1, obj2 in combinations(objects, 2):
            added=0
            for epi in episodes_selection:
                if (epi[0] == obj1 and epi[1] == obj2) or (epi[0] == obj2 and epi[1] == obj1):
                    added=1
                    start[epi[3]] = epi
                    end[epi[4]] = epi
            if added == 1:
                st=start.keys()
                st.sort()
                E_s.append(start[st[0]])

                en=end.keys()
                en.sort()
                E_f.append(end[en[-1]])

        return (E_s, E_f)


    def get_activity_graph(self, episodes):
        """
        Generates the qstag for a set of input episode QSRs.
        """


        temporal_map = {'after': 'before',
                        'metby' : 'meets',
                        'overlapped_by': 'overlaps',
                        'started_by': 'starts',
                        'contains':'during',
                        'finished_by': 'finishes'
                        }

        objects = {}
        data    = []
        types   = []
        spatial_rels = {}
        vertex_count = 0
        graph = Graph(directed=True)

        #for (o1, o1t, o2, o2t, rel, intv_start, intv_end) in episodes:
        for i in episodes:
            print(i)

        for (objs, relation, (intv_start, intv_end)) in episodes:

            print(objs, relation, (intv_start, intv_end))
            print("\nNeed to fit the below code into the new episode structure :)\n")
            sys.exit(1)

            #############################################
            #   Object Nodes:                           #
            #############################################
            if o1 not in objects:
                graph.add_vertex(o1)
                objects[o1] = vertex_count
                if o1t == 'trajectory':
                    graph.vs()[vertex_count]['obj_type'] = 'trajectory'
                else:
                    if self.obj_type == 1: graph.vs()[vertex_count]['obj_type'] = o1
                    if self.obj_type == 2: graph.vs()[vertex_count]['obj_type'] = o1t
                    if self.obj_type == 3: graph.vs()[vertex_count]['obj_type'] = 'object'

                graph.vs()[vertex_count]['node_type'] = 'object'
                vertex_count += 1

            if o2 not in objects:
                graph.add_vertex(o2)
                objects[o2] = vertex_count
                if o2t == 'trajectory':
                    graph.vs()[vertex_count]['obj_type'] = 'trajectory'
                else:
                    if self.obj_type == 1: graph.vs()[vertex_count]['obj_type'] = o2
                    if self.obj_type == 2: graph.vs()[vertex_count]['obj_type'] = o2t
                    if self.obj_type == 3: graph.vs()[vertex_count]['obj_type'] = 'object'

                graph.vs()[vertex_count]['obj_type'] = o2
                #graph.vs()[vertex_count]['obj_type'] = o2t
                graph.vs()[vertex_count]['node_type'] = 'object'
                vertex_count += 1

            #############################################
            #   Spatial Nodes:                          #
            #############################################
            graph.add_vertex(rel)
            graph.vs()[vertex_count]['node_type'] = 'spatial_relation'
            # Add edges from spatial node to objects
            graph.add_edge(objects[o1], vertex_count)
            graph.add_edge(vertex_count, objects[o2])
            self.spatial_obj_edges.append((objects[o1], vertex_count))
            self.spatial_obj_edges.append((vertex_count, objects[o2]))
            data.append((objects[o1], objects[o2], vertex_count, intv_start, intv_end))
            vertex_count += 1


        #############################################
        #   Temporal Nodes:                         #
        #############################################
        (E_s, E_f) = self.get_E_set(objects, data)
        temporal_vertices = {}
        for ((o11, o12, rel1, s1, e1),(o21, o22, rel2, s2, e2)) in combinations(data,2):


            #No temporal relation between two starting episodes or between two final episodes
            if ((o11, o12, s1, e1) in E_s and (o21, o22, s2, e2) in E_s) or ((o11, o12, s1, e1) in E_f and (o21, o22, s2, e2) in E_f):
                continue

            temporal_rel = get_allen_relation(s1, e1, s2, e2)
            # If temporal_rel is in temporal_map get its value otherwise keep it the same
            # If the edges are directed, then we need to change the direction of the edges
            # if we change change the temporal relation to its inverse
            temporal_rel_new = temporal_map.get(temporal_rel, temporal_rel)
            # Add temporal node to the graph

            graph.add_vertex(temporal_rel_new)
            graph.vs()[vertex_count]['node_type'] = 'temporal_relation'
            temporal_rel_vertex_id          = vertex_count
            temporal_vertices[temporal_rel_new] = vertex_count
            vertex_count += 1

            if temporal_rel_new == temporal_rel:
                # Add edges from temporal node to the spatial nodes
                graph.add_edge(rel1, temporal_rel_vertex_id)
                graph.add_edge(temporal_rel_vertex_id, rel2)
                self.temp_spatial_edges.append((rel1, temporal_rel_vertex_id))
                self.temp_spatial_edges.append((temporal_rel_vertex_id, rel2))
            else:
                # PD: if an inverse temporal relation has been used, switch the edges around
                graph.add_edge(temporal_rel_vertex_id, rel1)
                graph.add_edge(rel2, temporal_rel_vertex_id)
                self.temp_spatial_edges.append((temporal_rel_vertex_id, rel1))
                self.temp_spatial_edges.append((rel2, temporal_rel_vertex_id))
        return graph



def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
    print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
          + " and received at " + str(qsrlib_response_message.req_received_at)
          + " and finished at " + str(qsrlib_response_message.req_finished_at))
    print("---")
    print(qsrlib_response_message.qsrs.get_sorted_timestamps())
    print("Response is:")
    for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                        qsrlib_response_message.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)


def compute_episodes(qsrlib_response_message):
    """
    Computes a long list of episodes with the format [(objects), {spatial relations}, (start, end)]
    
    Or a dictionary where the objects are the key?
    """

    episodes  = []
    #episodes  = {}
    obj_based_qsr_world = {}

    frames = qsrlib_response_message.qsrs.get_sorted_timestamps()
    for frame in frames:
        for objs, qsrs in qsrlib_response_message.qsrs.trace[frame].qsrs.items():
            if objs not in obj_based_qsr_world: obj_based_qsr_world[objs] = []
            obj_based_qsr_world[objs].append((frame, qsrs.qsr))

    for objs, frame_tuples in obj_based_qsr_world.items():
        #episodes[objs]  = []

        epi_start, epi_rel = frame_tuples[0]
        epi_end  = copy.copy(epi_start)

        objects = objs.split(',')
        for (frame, rel) in obj_based_qsr_world[objs]:
            if rel == epi_rel:
                epi_end = frame
            else:
                episodes.append( (objects, epi_rel, (epi_start, epi_end)) )
                #episodes[objs].append( (objects, epi_rel, (epi_start, epi_end)) )

                epi_start = epi_end = frame
                epi_rel = rel
        episodes.append( (objects, epi_rel, (epi_start, epi_end)) )
        #episodes[objs].append( (objects, epi_rel, (epi_start, epi_end)) )

    return episodes


if __name__ == "__main__":

    options = ["rcc2", "rcc3", "rcc8", "cardir", "qtcbs", "qtccs", "qtcbcs", "argd", "argprobd", "mos", "multiple"]
    #multiple = options[:]; multiple.remove("multiple"); multiple.remove("argd"); multiple.remove("argprobd")

    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str, default='qtcbs')
    #parser.add_argument("-i", "--input", help="file from which to read object states", type=str)
    #parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true", default=False)
    #parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states in qtc, or 's'-states in mos", type=float, default=0.01)
    #parser.add_argument("--no_collapse", help="does not collapse similar adjacent states. Only QTC", action="store_true", default=True)
    #parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    #parser.add_argument("-c", "--config", help="config file", type=str)

    args = parser.parse_args()
    args.validate = False
    args.quantisation_factor = 0.01
    args.no_collapse = True
    args.ros = False
    #args.distance_threshold = []
    qsrs_for = [("o1", "o2")]

    if args.qsr in options:
        which_qsr = args.qsr
    else:
        raise ValueError("qsr not found, keywords: %s" % options)

    world = World_Trace()

    dynamic_args = {which_qsr: {
            "quantisation_factor": args.quantisation_factor,
            #"distance_threshold": args.distance_threshold,
            "validate": args.validate,
            "no_collapse": args.no_collapse,
            "qsrs_for": qsrs_for
        }}


    o1 = [Object_State(name="o1", timestamp=0, x=2., y=2.),
          Object_State(name="o1", timestamp=1, x=2., y=2.),
          Object_State(name="o1", timestamp=2, x=2., y=2.),
          Object_State(name="o1", timestamp=3, x=14., y=11.),
          Object_State(name="o1", timestamp=4, x=14., y=11.),
          Object_State(name="o1", timestamp=5, x=14., y=11.)]

    o2 = [Object_State(name="o2", timestamp=0, x=4., y=1.),
          Object_State(name="o2", timestamp=1, x=4., y=1.),
          Object_State(name="o2", timestamp=2, x=4., y=1.),
          Object_State(name="o2", timestamp=3, x=14., y=11.),
          Object_State(name="o2", timestamp=4, x=14., y=11.),
          Object_State(name="o2", timestamp=5, x=13., y=11.)]

    world.add_object_state_series(o1)
    world.add_object_state_series(o2)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, dynamic_args=dynamic_args)

    qsrlib = QSRlib()
    qsrlib_response_message = qsrlib.request_qsrs(qsrlib_request_message)

    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)

    episodes = compute_episodes(qsrlib_response_message)

    #Or I use a dictionary with the objects as key?
    #episodes_list = list(itertools.chain.from_iterable(episodes.values()))
    #print("LIST: ", episodes_list)

    params={'MAX_ROWS':1, 'MIN_ROWS':None, 'MAX_EPI':4, 'num_cores':8}
    a = Activity_Graph(episodes, params)

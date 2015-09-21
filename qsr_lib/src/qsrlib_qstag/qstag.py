#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Qualitative Spatio-Temporal Activity Graph module

"""
from __future__ import print_function
import os, sys, copy
import argparse
from datetime import datetime
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import World_Trace, Object_State
from qsrlib_io.world_qsr_trace import World_QSR_Trace

from igraph import Graph
from itertools import combinations, permutations, product, combinations_with_replacement


def get_allen_relation(duration1, duration2):

    is1, ie1 = duration1
    is2, ie2 = duration2

    if is2-1 == ie1:
        return 'meets'
    elif is1-1 == ie2:
        return 'metby'

    elif is1 == is2 and ie1 == ie2:
        return 'equal'

    elif is2 > ie1:
        return 'before'
    elif is1 > ie2:
        return 'after'

    elif ie1 >= is2 and ie1 <= ie2 and is1 <= is2:
        return 'overlaps'
    elif ie2 >= is1 and ie2 <= ie1 and is2 <= is1:
        return 'overlapped_by'
    elif is1 >= is2 and ie1 <= ie2:
        return 'during'
    elif is1 <= is2 and ie1 >= ie2:
        return 'contains'
    elif is1 == is2 and ie1 < ie2:
        return 'starts'
    elif is1 == is2 and ie1 > ie2:
        return 'started_by'
    elif ie1 == ie2 and is2 < is1:
        return 'finishes'
    elif ie1 == ie2 and is2 > is1:
        return 'finished_by'


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

    def __init__(self, episodes, params, object_types={}):
        self.spatial_obj_edges  = []
        self.temp_spatial_edges = []
        self.episodes           = episodes
        self.object_types       = object_types
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



    def get_E_set(self, objects, spatial_data):
        """returns the Starting episode set (E_s) and the Endding episode set (E_s)
        See Sridar_AAAI_2010 for more details
        """
        objects_ids = objects.values()
        start, end = {}, {}
        E_s, E_f = [], []
        number_of_objects = len(spatial_data[0][0])

        for possible_ids in permutations(objects_ids, number_of_objects):
            added=0

            for epi in spatial_data:
                ep_objects =  epi[0]
                print("ep objects = ", type(ep_objects), ep_objects)
                print("combo objects = ", type(possible_ids), possible_ids)
                frame_window = epi[2]

                #if (objects[0] == obj1 and objects[1] == obj2):
                if list(possible_ids) == ep_objects:
                    print("YES?")
                    start[frame_window[0]] = epi
                    end[frame_window[1]] = epi
                    added=1
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

        spatial_nodes_edges = {"rcc2": 2, "rcc3": 2, "rcc8": 2, "cardir": 2,
                    "qtcbs": 2, "qtccs": 2, "qtcbcs": 2, "argd": 2, "argprobd": 2, "mos": 1}

        objects = {}
        spatial_data    = []
        vertex_count = 0
        graph = Graph(directed=True)

        print("Looping through the episodes...")
        #for (o1, o1t, o2, o2t, rel, intv_start, intv_end) in episodes:
        for (objs, relations, (intv_start, intv_end)) in episodes:
            print(objs, relations, (intv_start, intv_end))


            #############################################
            #   Object Nodes:                           #
            #############################################
            number_of_edges = set([])
            for rel in relations.keys():
                number_of_edges.add(spatial_nodes_edges[rel])
            if len(number_of_edges) != 1:
                print("QSRs WITH DIFFENENT SPATIAL NODE EDGES SELECTED!")
                sys.exit(1)
            else:
                spatial_edges = number_of_edges.pop()
            for o in objs:
                if o in objects: continue

                graph.add_vertex(o)
                objects[o] = vertex_count

                if o in self.object_types:
                    graph.vs()[vertex_count]['obj_type'] = self.object_types[o]

                graph.vs()[vertex_count]['node_type'] = 'object'
                vertex_count += 1

            object_ids = []
            for o in objs:
                object_ids.append(objects[o])
            #print("   OBJECT IDS ", object_ids)
            #############################################
            #   Spatial Nodes:                          #
            #############################################
            graph.add_vertex(relations)
            graph.vs()[vertex_count]['node_type'] = 'spatial_relation'

            # Add edges from spatial node to objects
            edge_from_object = objects[objs[0]]

            print("adding edge: : ", edge_from_object, vertex_count)
            graph.add_edge(edge_from_object, vertex_count)
            self.spatial_obj_edges.append( (edge_from_object, vertex_count) )

            if spatial_edges is 2:
                print("TWO OBJECTS -- ")
                edge_to_object = objects[objs[1]]
                graph.add_edge(vertex_count, edge_to_object)
                self.spatial_obj_edges.append( (vertex_count, edge_to_object) )

            elif spatial_edges is 3:
                print("THREE OBJECTS -- ")
                edge_from_object_2 = objects[objs[1]]
                edge_from_object_3  = objects[objs[2]]
                graph.add_edge(edge_from_object_2, vertex_count)
                graph.add_edge(edge_from_object_3, vertex_count)
                self.spatial_obj_edges.append( (edge_from_object_2, vertex_count) )
                self.spatial_obj_edges.append( (edge_from_object_3, vertex_count) )

            spatial_data.append( (object_ids, vertex_count,  (intv_start, intv_end)) )
            vertex_count += 1

        #############################################
        #   Temporal Nodes:                         #
        #############################################
        print("data: \n", spatial_data)
        print("objects \n", objects)

        (E_s, E_f) = self.get_E_set(objects, spatial_data)
        print("E_s: ", E_s)
        print("E_f: ", E_f)

        temporal_vertices = {}

        #for ((o11, o12, rel1, s1, e1), (o21, o22, rel2, s2, e2)) in combinations(spatial_data, 2):
        for epi1, epi2 in combinations(spatial_data, 2):

            if ( epi1 in E_s and epi2 in E_s) or ( epi1 in E_f and epi2 in E_f): continue
            (objs1, rels1, frames1) = epi1
            (objs2, rels2, frames2) = epi2

            print (epi1, epi2)
            temporal_rel = get_allen_relation(frames1, frames2)

            # If temporal_rel is in temporal_map get its value otherwise keep it the same
            # If the edges are directed, then we need to change the direction of the edges
            # if we change change the temporal relation to its inverse
            temporal_rel_new = temporal_map.get(temporal_rel, temporal_rel)
            print(temporal_rel_new)
            # Add temporal node to the graph

            graph.add_vertex(temporal_rel_new)
            graph.vs()[vertex_count]['node_type'] = 'temporal_relation'
            temporal_rel_vertex_id                = vertex_count
            temporal_vertices[temporal_rel_new]  = vertex_count
            vertex_count += 1

            if temporal_rel_new == temporal_rel:
                # Add edges from spatial node to temporal node
                graph.add_edge(rels1, temporal_rel_vertex_id)
                graph.add_edge(temporal_rel_vertex_id, rels2)
                self.temp_spatial_edges.append((rels1, temporal_rel_vertex_id))
                self.temp_spatial_edges.append((temporal_rel_vertex_id, rels2))
            else:
                # PD: if an inverse temporal relation has been used, switch the edges around
                graph.add_edge(temporal_rel_vertex_id, rels1)
                graph.add_edge(rels2, temporal_rel_vertex_id)
                self.temp_spatial_edges.append((temporal_rel_vertex_id, rels1))
                self.temp_spatial_edges.append((rels2, temporal_rel_vertex_id))

        print(graph)
        print("NODES:")
        for node in graph.vs():
            print(node)
        print("EDGES:")
        for edge in graph.es():
            print(edge)
        return graph



    def graph2dot(self, out_dot_file, COLLAPSE_TEMPORAL_NODES=False):
        # Write the graph to dot file
        # Can generate a graph figure from this .dot file using the 'dot' command
        # dot -Tpng input.dot -o output.png
        dot_file = open(out_dot_file, 'w')
        dot_file.write('digraph activity_graph {\n')
        dot_file.write('    size = "45,45";\n')
        dot_file.write('    node [fontsize = "18", shape = "box", style="filled", fillcolor="aquamarine"];\n')
        dot_file.write('    ranksep=5;\n')
        # Create temporal nodes
        dot_file.write('    subgraph _1 {\n')
        dot_file.write('    rank="source";\n')

        #print(self.temporal_nodes)
        #print(self.spatial_nodes)
        #print(self.object_nodes)

        if COLLAPSE_TEMPORAL_NODES:
            temporal_names = {'before'  :0,
                              'meets'   :0,
                              'overlaps':0,
                              'starts'  :0,
                              'during'  :0,
                              'finishes':0,
                              'equal'   :0
                             }
            for tnode in self.temporal_nodes:
                temporal_names[tnode['name']] = tnode.index
            for tnode in temporal_names:
                dot_file.write('    %s [fillcolor="white", label="%s", shape=ellipse];\n' \
                               %(temporal_names[tnode], tnode))
        else:
            for tnode in self.temporal_nodes:
                dot_file.write('    %s [fillcolor="white", label="%s", shape=ellipse];\n' %(tnode.index, tnode['name']))

        dot_file.write('}\n')

        # Create spatial nodes
        dot_file.write('    subgraph _2 {\n')
        dot_file.write('    rank="same";\n')
        for rnode in self.spatial_nodes:
            dot_file.write('    %s [fillcolor="lightblue", label="%s"];\n' %(rnode.index, rnode['name']))
        dot_file.write('}\n')

        # Create object nodes
        dot_file.write('    subgraph _3 {\n')
        dot_file.write('    rank="sink";\n')
        for onode in self.object_nodes:
            dot_file.write('%s [fillcolor="tan1", label="%s"];\n' %(onode.index, onode['name']))
        dot_file.write('}\n')

        # Create temporal to spatial edges
        if COLLAPSE_TEMPORAL_NODES:
            for (source, target) in self.temp_spatial_edges:
                if self.graph.vs[source]['node_type'] == 'temporal_relation':
                    source = temporal_names[self.graph.vs[source]['name']]
                elif self.graph.vs[target]['node_type'] == 'temporal_relation':
                    target = temporal_names[self.graph.vs[target]['name']]
                dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(source, target))
        else:
            for t_edge in self.temp_spatial_edges:
                dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(t_edge[0], t_edge[1]))

        # Create spatial to object edges
        for r_edge in self.spatial_obj_edges:
            dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(r_edge[0], r_edge[1]))
        dot_file.write('}\n')
        dot_file.close()


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
    qsrs_for = [("o1", "o2"), ("o2", "o1"), ("o1"), ("o2")]
    #qsrs_for = [("o1", "o2"), ("o1")]
    object_types = {"o1": "Human", "o2": "Chair"}


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
        },
        "object_types" : object_types}


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
          Object_State(name="o2", timestamp=5, x=14., y=11.)]

    world.add_object_state_series(o1)
    world.add_object_state_series(o2)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, dynamic_args=dynamic_args)

    qsrlib = QSRlib()
    qsrlib_response_message = qsrlib.request_qsrs(qsrlib_request_message)

    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)

    episodes = compute_episodes(qsrlib_response_message)

    print("Episodes...")
    for i in episodes:
        print(i)

    #Or I use a dictionary with the objects as key?
    #episodes_list = list(itertools.chain.from_iterable(episodes.values()))
    #print("LIST: ", episodes_list)

    params={'MAX_ROWS':1, 'MIN_ROWS':None, 'MAX_EPI':4, 'num_cores':8}
    a = Activity_Graph(episodes, params, dynamic_args["object_types"])

    a.graph2dot('/tmp/act_gr.dot')
    os.system('dot -Tpng /tmp/act_gr.dot -o /tmp/act_gr.png')

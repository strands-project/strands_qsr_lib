# -*- coding: utf-8 -*-
"""Qualitative Spatio-Temporal Activity Graph module

"""
from __future__ import print_function
import argparse
from datetime import datetime
# from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
# from qsrlib_io.world_trace import World_Trace, Object_State
# from qsrlib_io.world_qsr_trace import World_QSR_Trace

from igraph import Graph
from itertools import combinations, permutations, product, combinations_with_replacement
from qsrlib_qstag.qsr_episodes import compute_episodes


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

    #Protect the variables that are not needed from the outside
    def __init__(self, world, world_qsr, object_types={}):
        """Constructor.

        :param world: The World Trace object
        :type world: :class:`World_Trace <qsrlib_io.world_trace>`
        :param world_qsr: The QSR_World_Trace object (QSRlib_Response_Message)
        :type world_qsr: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace>`
        :param object_types: dictionary of object name to a generic object type
        :type object_types: dict
        """

        self.__episodes = compute_episodes(world_qsr)
        """list: The list of QSR Episodes used to generate the QSTAG."""
        self.__spatial_obj_edges  = []
        """list: A list of edges connecting the spatial nodes to the object nodes."""
        self.__temp_spatial_edges = []
        """list: A list of edges connecting the spatial nodes to the temporal nodes."""
        self.__object_types = self.get_objects_types(object_types, world)
        """dict: A dictionary containing the object names, and the generic object typese."""
        self.graph = self.get_activity_graph(self.__episodes)
        """igraph.Graph: An igraph graph object containing all the object, spatial and temporal nodes."""

    @property
    def episodes(self):
        """Getter.

        :return: `self.__episodes`
        :rtype: list
        """
        return self.__episodes


    @property
    def abstract_graph(self):
        """Getter.

        :return: `self.abstract_graph`
        :rtype: igraph.Graph
        """
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
        """Getter.

        :return: `object_nodes`
        :rtype: list
        """
        # Get object nodes from graph
        object_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'object':
                object_nodes.append(node)
        return object_nodes

    @property
    def abstract_object_nodes(self):
        """Getter.

        :return: `object_nodes` from abstract_graph
        :rtype: list
        """
        # Get object nodes from abstract graph
        object_nodes = []
        for node in self.abstract_graph.vs():
            if node['node_type'] == 'object':
                object_nodes.append(node)
        return object_nodes

    @property
    def spatial_nodes(self):
        """Getter.

        :return: `spatial_nodes`
        :rtype: list
        """
        # Get spatial relation nodes from graph
        spatial_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'spatial_relation':
                spatial_nodes.append(node)
        return spatial_nodes

    @property
    def temporal_nodes(self):
        """Getter.

        :return: `temporal_nodes`
        :rtype: list
        """
        # Get temporal relation nodes from graph
        temporal_nodes = []
        for node in self.graph.vs():
            if node['node_type'] == 'temporal_relation':
                temporal_nodes.append(node)
        return temporal_nodes

    def get_objects_types(self, objects_types, world):
        """Generates a dictionary of object name and object type pairs
        Using both the dynamic_args dictionary where key = `objects_types`, and the
        **kwargs value [object_type] in the World Trace object

        :param objects_types: Uses the dynamic_args dictionary  where key = `objects_types` if provided
        :type objects_types: dictionary
        :param world: Otherwise, looks at the **kwargs value [object_type] in the World Trace object
        :type world: :class:`World_Trace <qsrlib_io.world_trace>`
        :return: A dictionary with the object name as keys and the generic object type as value.
        :rtype: dict
        """
        for t, state in world.trace.items():
            for oname, odata in state.objects.items():
                if oname not in objects_types:
                    try:
                        objects_types[oname] = odata.kwargs["object_type"]
                    except KeyError:
                        objects_types[oname] = "unknown"
        return objects_types

    def get_E_set(self, objects, spatial_data):
        """Returns the Starting episode set (E_s) and the Endding episode set (E_s)
        See Sridar_AAAI_2010 for more details

        :param objects: object dictionary with name as key, and node ID as value
        :type objects: dictionary
        :param spatial_data: A list of tuples, where a tuple contains a list of objects, a spatial relation node ID, and a duration of time.
        :type spatial_data: list
        :return: A tuple containing two sets of QSR Episodes, where a temporal node does not hold beteen Episodes in the same set.
        :rtype: tuple
        """
        objects_ids = objects.values()
        start, end = {}, {}
        E_s, E_f = [], []
        number_of_objects = len(spatial_data[0][0])

        for possible_ids in permutations(objects_ids, number_of_objects):
            added=0
            for epi in spatial_data:
                ep_objects =  epi[0]
                frame_window = epi[2]

                #if (objects[0] == obj1 and objects[1] == obj2):
                if list(possible_ids) == ep_objects:
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
        """Generates the qstag for a set of input episode QSRs.

        :param episodes: list of QSR Epiosdes generated using `compute_episodes(world_qsr)``
        :type episodes: list
        :return: igraph.Graph: An igraph graph object containing all the object, spatial and temporal nodes.
        :rtype: igraph.Graph
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

        #print("Looping through the episodes...")
        for (objs, relations, (intv_start, intv_end)) in episodes:
            #print(objs, relations, (intv_start, intv_end))


            #############################################
            #   Object Nodes:                           #
            #############################################
            number_of_edges = set([])
            for rel in relations.keys():
                number_of_edges.add(spatial_nodes_edges[rel])
            if len(number_of_edges) != 1:
                raise ValueError("QSRs with different spatial node edges selected.")
            else:
                spatial_edges = number_of_edges.pop()

            for o in objs:
                if o in objects: continue

                graph.add_vertex(o)
                objects[o] = vertex_count

                if o in self.__object_types:
                    graph.vs()[vertex_count]['obj_type'] = self.__object_types[o]

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

            #print("adding edge: : ", edge_from_object, vertex_count)
            graph.add_edge(edge_from_object, vertex_count)
            self.__spatial_obj_edges.append( (edge_from_object, vertex_count) )

            if spatial_edges is 2:
                #print("TWO OBJECTS -- ")
                edge_to_object = objects[objs[1]]
                graph.add_edge(vertex_count, edge_to_object)
                self.__spatial_obj_edges.append( (vertex_count, edge_to_object) )

            elif spatial_edges is 3:
                #print("THREE OBJECTS -- ")
                edge_from_object_2 = objects[objs[1]]
                edge_from_object_3  = objects[objs[2]]
                graph.add_edge(edge_from_object_2, vertex_count)
                graph.add_edge(edge_from_object_3, vertex_count)
                self.__spatial_obj_edges.append( (edge_from_object_2, vertex_count) )
                self.__spatial_obj_edges.append( (edge_from_object_3, vertex_count) )

            spatial_data.append( (object_ids, vertex_count,  (intv_start, intv_end)) )
            vertex_count += 1

        #############################################
        #   Temporal Nodes:                         #
        #############################################
        #print("data: \n", spatial_data)
        #print("objects \n", objects)

        (E_s, E_f) = self.get_E_set(objects, spatial_data)
        #print("E_s: ", E_s)
        #print("E_f: ", E_f)

        temporal_vertices = {}

        #for ((o11, o12, rel1, s1, e1), (o21, o22, rel2, s2, e2)) in combinations(spatial_data, 2):
        for epi1, epi2 in combinations(spatial_data, 2):

            if ( epi1 in E_s and epi2 in E_s) or ( epi1 in E_f and epi2 in E_f): continue
            (objs1, rels1, frames1) = epi1
            (objs2, rels2, frames2) = epi2

            #print (epi1, epi2)
            temporal_rel = get_allen_relation(frames1, frames2)

            # If temporal_rel is in temporal_map get its value otherwise keep it the same
            # If the edges are directed, then we need to change the direction of the edges
            # if we change change the temporal relation to its inverse
            temporal_rel_new = temporal_map.get(temporal_rel, temporal_rel)
            #print(temporal_rel_new)

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
                self.__temp_spatial_edges.append((rels1, temporal_rel_vertex_id))
                self.__temp_spatial_edges.append((temporal_rel_vertex_id, rels2))
            else:
                # If an inverse temporal relation has been used, switch the edges around
                graph.add_edge(temporal_rel_vertex_id, rels1)
                graph.add_edge(rels2, temporal_rel_vertex_id)
                self.__temp_spatial_edges.append((temporal_rel_vertex_id, rels1))
                self.__temp_spatial_edges.append((rels2, temporal_rel_vertex_id))
        return graph


    def graph2dot(self, out_dot_file):
        """To visualize the iGraph graph, this prints a dot file to the file location given

        :param out_dot_file: file location to save the dot file
        :type out_dot_file: string
        """

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
        for t_edge in self.__temp_spatial_edges:
            dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(t_edge[0], t_edge[1]))

        # Create spatial to object edges
        for r_edge in self.__spatial_obj_edges:
            dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(r_edge[0], r_edge[1]))
        dot_file.write('}\n')
        dot_file.close()


def get_allen_relation(duration1, duration2):
    """Generates an Allen interval algebra relation between two discrete durations of time

    :param duration1: First duration of time (start_frame, end_frame)
    :type duration1: tuple
    :param duration2: Second duration of time (start_frame, end_frame)
    :type duration2: tuple
    """

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

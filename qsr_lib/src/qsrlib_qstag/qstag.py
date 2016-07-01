#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Qualitative Spatio-Temporal Activity Graph module
"""
from __future__ import print_function
import sys
from igraph import Graph as iGraph
from itertools import combinations, chain
import qsrlib_qstag.utils as utils


class Activity_Graph:
    """
    Activity Graph class:
    Lower level is a set of only nodes of type 'object'. Middle level nodes are only of
    type 'spatial_relation'. Top level nodes are only of type 'temporal_relation'.

    Accepts a QSRLib.World_Trace and QSRLib.QSR_World_Trace as input.
    """
    def __init__(self, world, world_qsr, object_types={}, params={}):
        """Constructor.

        :param world: The World Trace object
        :type world: :class:`World_Trace <qsrlib_io.world_trace>`
        :param world_qsr: The QSR_World_Trace object (QSRlib_Response_Message)
        :type world_qsr: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace>`
        :param object_types: dictionary of object name to a generic object type
        :type object_types: dict
        """
        self.__episodes = utils.compute_episodes(world_qsr)
        """list: The list of QSR Episodes used to generate the QSTAG."""

        self.__object_types = self.get_objects_types(object_types, world)
        """dict: A dictionary of object names and types."""

        self.graph, self.__spatial_obj_edges, self.__temp_spatial_edges = get_graph(self.__episodes, self.__object_types)
        """igraph.Graph: An igraph graph object containing all the object, spatial and temporal nodes.
        list: A list of edges connecting the spatial nodes to the object nodes.
        list: A list of edges connecting the spatial nodes to the temporal nodes."""

        try:
            if not isinstance(params["max_eps"], int):
                raise RuntimeError("params needs to contain a dictionary of Graphlet parameters. i.e. max_eps, and min/max_rows.")
        except KeyError:
            print("dynamic args needs something like this:  {qstag {params: {min_rows:1, max_rows:1, max_eps:3}}" )
            sys.exit(1)

        self.graphlets = Graphlets(self.__episodes, params, self.__object_types)
        """Creates a Graphlets object containing lists of, unique graphlets, hashes and histogram of graphlets."""


    @property
    def episodes(self):
        """Getter.

        :return: `self.__episodes`
        :rtype: list
        """
        return self.__episodes

    @property
    def spatial_obj_edges(self):
        """Getter.

        :return: `self.__spatial_obj_edges`
        :rtype: list
        """
        return self.__spatial_obj_edges

    @property
    def temp_spatial_edges(self):
        """Getter.

        :return: `self.__temp_spatial_edges`
        :rtype: list
        """
        return self.__temp_spatial_edges

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

    @property
    def histogram(self):
        """Getter.

        :return: `self.__histogram`
        :rtype: dict
        """
        return {k: v for k, v in zip(self.__hashes, self.__histogram)}

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


    @staticmethod
    def get_objects_types(objects_types, world):
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

class Graphlets:
    '''
    Graphlet class:
    Minimal subgraphs of the same structure as the Activity Graph.
    '''

    def __init__(self, episodes, params, object_types):
        """Constructor.

        :param episodes: list of QSR episodes, each a tuple of the form: ([objects], {epi_rel}, (epi_start, epi_end))
        :type episodes: list
        :param params: dictionary of parameters: minimum rows, maximum rows, maximum episodes used for graphlets
        :type params: dict
        :param object_types: dictionary of object name to a generic object type. Default is set to "unknown" type
        :type object_types: dict
        """
        try:
            max_eps = params["max_eps"]
        except KeyError:
            params = {"min_rows":1, "max_rows":1, "max_eps":3}

        all_graphlets, hashes = get_graphlet_selections(episodes, params, object_types, vis=False)
        """lists: Two lists of all graphlets and hashes in Activity_Graph."""

        self.histogram = []
        """list: The list of graphlet counts (zip with codebook for a hash of each, and check graphlets for the iGraph)."""
        self.code_book = []
        """list: The list of graphlet hashes (zip with histogram for count of each)."""
        self.graphlets = {}
        """dict: dictionary of the graphlet hash as key, and the iGraph object as value."""

        for h, g in zip(hashes, all_graphlets):

            if h not in self.code_book:
                self.code_book.append(h)
                self.histogram.append(1)
                self.graphlets[h] = g
            else:
                ind = self.code_book.index(h)
                self.histogram[ind] += 1


def get_graphlet_selections(episodes, params, object_types, vis=False):
    """ This function implements Sridar's validity criteria to select all valid
    graphlets from an activity graph: see Sridar_AAAI_2010 for more details.

    :param episodes: list of episodes, where one episode = [[obj_list], {QSR dict}, (start, end_tuple)]
    :type episodes: list
    :param params: a dictionary containing parameters for the generation of the QSTAG. i.e.  "min_rows", "max_rows", "max_eps"
    :type params: dict

    :return list_of_graphlets: a list of iGraph graphlet objects
    :rtype: list
    :return list_of_graphlet_hashes: a list of hashes, relating to the graphlets
    :rtype: list
    """

    if vis: print("num of episodes:", len(episodes))
    if vis: print("all episodes: ", episodes)
    episode_ids = {}
    intervals = {}

    # Gather the episodes and interval data
    # Use ID codes for the episodes throughout the function
    # At the end we replace the codes with episodes
    for ep_id, ep in enumerate(episodes):
        if vis: print("\nID:", ep_id, ep)
        ep_start = ep[2][0]
        ep_end = ep[2][1]

        objs = tuple([ob for ob in ep[0]])
        episode_ids[ep_id] = ep

        temporal_info = [ep_start, ep_end, ep_id]
        if vis: print("temp info:", temporal_info, "obj info:", objs)

        if objs not in intervals: intervals[objs] = [temporal_info]
        else: intervals[objs].append(temporal_info)
    if vis: print("\nepisode_ids: ", episode_ids)
    if vis: print("\nintervals: ", intervals)

    hashed_IDs = {}
    range_of_rows = range(params["min_rows"], params["max_rows"]+1)
    for r in range_of_rows:
        # Once we select the number of rows, find all combinations of rows of r.
        for obj_pair_comb in combinations(intervals.keys(), r):
            if vis: print("\nobject_rows: ", obj_pair_comb)

            # Collect intervals from episodes of relevant rows
            relevant_episodes=[]
            for obj_pair in obj_pair_comb:
                relevant_episodes.extend(intervals[obj_pair])

            if vis: print("relevant_episodes", relevant_episodes)

            if relevant_episodes != []:
                interval_breaks = utils.get_temporal_chords_from_episodes(relevant_episodes)
            else:
                #Covers the case where no episodes are added. i.e. episodes that do not start at frame 1.
                interval_breaks = []
            if vis: print("interval_breaks: ", interval_breaks)

            num_of_interval_breaks = len(interval_breaks)
            if vis: print('num of intervals:', num_of_interval_breaks)

            # Loop through this broken timeline and find all
            # combinations (r is from 1 to num_of_intervals)
            # of consecutive intervals (intervals in a stretch).
            hashed_IDs[obj_pair_comb] = {}
            for len_ in xrange(1, num_of_interval_breaks+1):
                #print("len_=",len_)
                for pos in xrange(num_of_interval_breaks):
                    # Find the combined interval of this combination of intervals
                    selected_intervals = interval_breaks[pos:pos+len_]
                    # Get the relations active in this active interval
                    selected_ids = [epi[2] for epi in selected_intervals]
                    # Some episodes are repeated as they are active in two or more intervals.
                    # So remove the duplicates .
                    selected_ids_set = tuple(set(chain.from_iterable(selected_ids)))

                    ##IF selected_ids_set IS TOO LARGE - DON'T BOTHER HASHING IT.
                        #Only allow Graphlets of the specified number of Rows. Not all rows.
                    if len(selected_ids_set) <= params["max_eps"]:
                        if hash(selected_ids_set) not in hashed_IDs[obj_pair_comb]:
                            hashed_IDs[obj_pair_comb][hash(selected_ids_set)] = selected_ids_set

                    #print("pos=",pos)
                    #print("selected_intervals", selected_intervals)
                    #print("selected_ids", selected_ids)
                    #print("eps:", selected_ids_set)
    if vis: print("\nepisode keys:", hashed_IDs.keys())
    if vis: print("\nepisode combinations:", hashed_IDs)

    # Replace the ID codes with the episodes
    #episodes_selection = {}
    list_of_graphlets = []
    list_of_graphlet_hashes = []

    for objs, IDs in hashed_IDs.items():
        #episodes_selection[objs] = []

        for hash_, id_codes in IDs.items():
            eps = [episode_ids[epi_code] for epi_code in id_codes]

            ## If Object Types exist for an object, use it in the graphlet.
                #Replace the objects with object types - will remain the same if no types entered
            eps = [ (tuple([object_types[ob] if object_types[ob] != "unknown" else ob for ob in e[0]]),
                      e[1], e[2]) for e in eps ]

            # Remove graphlets that have only one spatial relation - i.e. no temporal relation
            #if len(eps) <= 1: continue
            if len(eps) <= params["max_eps"]:
                #episodes_selection[objs].append(eps)
                graph, spatial_obj_edges, temp_spatial_edges = get_graph(eps)
                list_of_graphlets.append(graph)
                list_of_graphlet_hashes.append(utils.graph_hash(graph))

                if vis:
                    print("\nEPS= ", eps)
                    print(graph)
                    print("HASH:", utils.graph_hash(graph))

    return list_of_graphlets, list_of_graphlet_hashes


def get_graph(episodes, object_types={}):
    """Generates a graph from a set of input episode QSRs.

    :param episodes: list of episodes, where one episode = [[obj_list], {QSR dict}, (start, end_tuple)]
    :type episodes: list
    :param object_types: a dictionary of object ID and object type.
    :type object_types: dict

    :return: igraph.Graph: An igraph graph object containing all the object, spatial and temporal nodes.
    :rtype: igraph.Graph
    """

    temporal_map = {'>': '<',
                    'mi' : 'm',
                    'oi': 'o',
                    'si': 's',
                    'di':'d',
                    'fi': 'f'
                    }

    spatial_nodes_edges = {"rcc2": 2, "rcc3": 2, "rcc8": 2, "cardir": 2,
                "qtcbs": 2, "qtccs": 2, "qtcbcs": 2, "argd": 2, "argprobd": 2, "mos": 1,
                "tpcc": 2, "rcc4": 2, "rcc5": 2}

    objects = {}
    spatial_data = []
    spatial_obj_edges, temp_spatial_edges = [], []
    vertex_count = 0
    graph = iGraph(directed=True)

    if episodes == []:
        return graph, spatial_obj_edges, temp_spatial_edges

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

            if o in object_types:
                graph.vs()[vertex_count]['obj_type'] = object_types[o]

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
        spatial_obj_edges.append((edge_from_object, vertex_count))

        if spatial_edges is 2:
            #print("TWO OBJECTS -- ")
            edge_to_object = objects[objs[1]]
            graph.add_edge(vertex_count, edge_to_object)
            spatial_obj_edges.append( (vertex_count, edge_to_object) )

        elif spatial_edges is 3:
            #print("THREE OBJECTS -- ")
            edge_from_object_2 = objects[objs[1]]
            edge_from_object_3  = objects[objs[2]]
            graph.add_edge(edge_from_object_2, vertex_count)
            graph.add_edge(edge_from_object_3, vertex_count)
            spatial_obj_edges.append( (edge_from_object_2, vertex_count) )
            spatial_obj_edges.append( (edge_from_object_3, vertex_count) )

        spatial_data.append( (object_ids, vertex_count,  (intv_start, intv_end)) )
        vertex_count += 1

    #############################################
    #   Temporal Nodes:                         #
    #############################################
    #print("data: \n", spatial_data)
    #print("objects \n", objects)

    E_s, E_f = utils.get_E_set(objects, spatial_data)
    #print("E_s: ", E_s)
    #print("E_f: ", E_f)

    temporal_vertices = {}
    for epi1, epi2 in combinations(spatial_data, 2):

        # don't create a temporal relation between two episodes in the start set, or two in the end set.
        if ( epi1 in E_s and epi2 in E_s) or ( epi1 in E_f and epi2 in E_f): continue
        (objs1, rels1, frames1) = epi1
        (objs2, rels2, frames2) = epi2

        #print (epi1, epi2)
        temporal_rel = utils.get_allen_relation(frames1, frames2)

        # If temporal_rel is in temporal_map get its value otherwise keep it the same
        # If the edges are directed, then we need to change the direction of the edges
        # if we change change the temporal relation to its inverse
        temporal_rel_new = temporal_map.get(temporal_rel, temporal_rel)
        #print(temporal_rel_new)

        # Add temporal node to the graph
        graph.add_vertex(temporal_rel_new)
        graph.vs()[vertex_count]['node_type'] = 'temporal_relation'
        temporal_rel_vertex_id = vertex_count
        temporal_vertices[temporal_rel_new] = vertex_count
        vertex_count += 1

        if temporal_rel_new == temporal_rel:
            # Add edges from spatial node to temporal node
            graph.add_edge(rels1, temporal_rel_vertex_id)
            graph.add_edge(temporal_rel_vertex_id, rels2)
            temp_spatial_edges.append((rels1, temporal_rel_vertex_id))
            temp_spatial_edges.append((temporal_rel_vertex_id, rels2))
        else:
            # If an inverse temporal relation has been used, switch the edges around
            graph.add_edge(temporal_rel_vertex_id, rels1)
            graph.add_edge(rels2, temporal_rel_vertex_id)
            temp_spatial_edges.append((temporal_rel_vertex_id, rels1))
            temp_spatial_edges.append((rels2, temporal_rel_vertex_id))
    return graph, spatial_obj_edges, temp_spatial_edges

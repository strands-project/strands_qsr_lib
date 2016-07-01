#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""QSTAR Utilities
"""
from __future__ import print_function
from itertools import combinations, permutations
import copy, sys
from igraph import Graph as iGraph
import numpy as np
import warnings



def compute_episodes(world_qsr):
	"""
	Compute QSR Episodes from a QSRLib.QSR_World_Trace object.
	QSR Episodes compresses repeating QSRs into a temporal interval over which they hold.
	Returns: a long list of episodes with the format, `[(objects), {spatial relations}, (start_frame, end_frame)]`.

	FILTERS: if any of the qsr values == Ignore, the entire episode will be ignored.

	Example content:
	----------------
	o1,mug,o2,hand,sur,3,7
	o1,mug,o3,head,con,4,9
	o2,hand,o3,head,dis,1,9
	----------------
	..seealso:: For further details about QSR Episodes, refer to its :doc:`description. <../handwritten/qsrs/qstag/>`

	:param world_qsr: The QSR_World_Trace object (QSRlib_Response_Message)
	:type world_qsr: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace>`
	"""

	episodes = []
	obj_based_qsr_world = {}
	frames = world_qsr.get_sorted_timestamps()

	"""remove the first frame which cannot contain a qtcb relation"""
	if "qtcbs" in world_qsr.qsr_type:
		if frames[0] == 1.0: frames.pop(0)

	for frame in frames:
		for objs, qsrs in world_qsr.trace[frame].qsrs.items():
			my_qsrs = {}
			#print("h", objs, qsrs.qsr)

			for qsr_key, qsr_val in qsrs.qsr.items():
				#print("  ", qsr_key, qsr_val)
				if qsr_key is "tpcc":
					origin,relatum,datum = objs.split(',')
					new_key=("%s-%s,%s") % (origin,relatum,datum)
					try:
						obj_based_qsr_world[new_key].append((frame, {"tpcc": qsrs.qsr["tpcc"]}))
					except KeyError:
						obj_based_qsr_world[new_key] = [(frame, {"tpcc": qsrs.qsr["tpcc"]})]
				else:
					my_qsrs[qsr_key] = qsr_val

			if my_qsrs != {}:
				try:
					obj_based_qsr_world[objs].append((frame, my_qsrs))
				except KeyError:
					obj_based_qsr_world[objs] = [(frame, my_qsrs)]

	#print("s", obj_based_qsr_world[objs])
	for objs, frame_tuples in obj_based_qsr_world.items():
		epi_start, epi_rel = frame_tuples[0]
		epi_end  = copy.copy(epi_start)

		objects = objs.split(',')
		for (frame, rel) in frame_tuples:
			if rel == epi_rel:
				epi_end = frame
			else:
				episodes.append( (objects, epi_rel, (epi_start, epi_end)) )
				epi_start = epi_end = frame
				epi_rel = rel
		episodes.append((objects, epi_rel, (epi_start, epi_end)))

	"""If any of the qsr values == ignore. Remove that episode entirely. """
	filtered_out_ignore = []
	for ep in episodes:
		ignore_flag = 0
		for qsr, val in ep[1].items():
			if val == "Ignore":	ignore_flag = 1
		if ignore_flag == 0: filtered_out_ignore.append(ep)

	print("number of eps:", len(filtered_out_ignore))

	return filtered_out_ignore

def get_E_set(objects, spatial_data):
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
	return E_s, E_f

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
		return 'm'
	elif is1-1 == ie2:
		return 'mi'

	elif is1 == is2 and ie1 == ie2:
		return '='

	elif is2 > ie1:
		return '<'
	elif is1 > ie2:
		return '>'

	elif ie1 >= is2 and ie1 < ie2 and is1 < is2:
		return 'o'
	elif ie2 >= is1 and ie2 < ie1 and is2 < is1:
		return 'oi'
	elif is1 > is2 and ie1 < ie2:
		return 'd'
	elif is1 < is2 and ie1 > ie2:
		return 'di'
	elif is1 == is2 and ie1 < ie2:
		return 's'
	elif is1 == is2 and ie1 > ie2:
		return 'si'
	elif ie1 == ie2 and is2 < is1:
		return 'f'
	elif ie1 == ie2 and is2 > is1:
		return 'fi'


def graph_hash(G, node_name_attribute='name', edge_name_attribute=None):
	"""
	See Figure 4 in 'kLog: A Language for Logical and Relational Learning with Kernels'
	for the algorithm.

	Takes an igraph graph, node_name attribute and edge_name_attribute. Note that
	edge_name_attribute is optional i.e. for graphs without edge labels or to ignore edge labels,
	edge_name_attribute is None.
	"""

	# suppress Runtime Warnings regarding not being able to find a path through the graphs
	warnings.filterwarnings('ignore')

	for node in G.vs:
		paths = G.get_shortest_paths(node)
		node_hashes = []
		for path in paths:
			if len(path) != 0:
				node_name = G.vs[path[-1]][node_name_attribute]
				if node_name == None:
					node_name = repr(None)
				node_hashes.append((len(path), node_name))
		node_hashes.sort()
		node_hashes_string = ':'.join([repr(i) for i in node_hashes])
		node['hash_name'] = hash(node_hashes_string)
	warnings.filterwarnings('always')
	if edge_name_attribute:
		edge_hashes = [(G.vs[edge.source]['hash_name'], G.vs[edge.target]['hash_name'],\
								   edge[edge_name_attribute]) for edge in G.es]
	else:
		edge_hashes = [(G.vs[edge.source]['hash_name'], G.vs[edge.target]['hash_name'])\
					   for edge in G.es]
	edge_hashes.sort()
	edge_hashes_string = ':'.join([repr(i) for i in edge_hashes])
	return hash(edge_hashes_string)

def get_temporal_chords_from_episodes(episodes):
	"""
	Function returns temporal chords from a subset of episodes

	:param episodes: a list of episodes, where one epiode has the format (start_frame, end_frame, id)
	:type episodes: list
	:return: list of chords
	:rtype: list
	"""
	interval_data = {}
	interval_breaks = []
	# For each time point in the combined interval, get the state of the
	# system which is just a list of relations active in that time point.

	#todo: can this work with floats? Not unless there is a measure of unit.
	for (s, e, id_) in episodes:
		for i in range(int(s), int(e+1)):
			if i not in interval_data:
				interval_data[i] = []
			interval_data[i].append(id_)

	keys = interval_data.keys()
	keys.sort()

	# Now based on the state changes, break the combined interval
	# whenever there is a change in the state
	start = keys[0]
	interval_value = interval_data[start]
	for i in keys:
		if interval_value == interval_data[i]:
			end = i
			continue
		else:
			interval_breaks.append([start, end, interval_value])
			start = i
			end = i
			interval_value = interval_data[start]
	else:
		# Adding the final interval
		interval_breaks.append([start, end, interval_value])
	return interval_breaks




def graph2dot(graph, out_dot_file):
	"""To visualize the iGraph graph, this prints a dot file to the file location given

	:param graph: Activity Graph object (QSTAG)
	:type graph: Activity_Graph type
	:param out_dot_file: file location to save the dot file
	:type out_dot_file: string
	"""

	# Write the graph to dot file
	# Can generate a graph figure from this .dot file using the 'dot' command
	# dot -Tpng input.dot -o output.png
	dot_file = open(out_dot_file, 'w')
	dot_file.write('digraph activity_graph {\n')
	dot_file.write('	size = "45,45";\n')
	dot_file.write('	node [fontsize = "18", shape = "box", style="filled", fillcolor="aquamarine"];\n')
	dot_file.write('	ranksep=5;\n')
	# Create temporal nodes
	dot_file.write('	subgraph _1 {\n')
	dot_file.write('	rank="source";\n')

	#print(graph.temporal_nodes)
	#print(graph.spatial_nodes)
	#print(graph.object_nodes)

	for tnode in graph.temporal_nodes:
		dot_file.write('	%s [fillcolor="white", label="%s", shape=ellipse];\n' %(tnode.index, tnode['name']))

	dot_file.write('}\n')

	# Create spatial nodes
	dot_file.write('	subgraph _2 {\n')
	dot_file.write('	rank="same";\n')
	for rnode in graph.spatial_nodes:
		dot_file.write('	%s [fillcolor="lightblue", label="%s"];\n' %(rnode.index, rnode['name']))
	dot_file.write('}\n')

	# Create object nodes
	dot_file.write('	subgraph _3 {\n')
	dot_file.write('	rank="sink";\n')
	for onode in graph.object_nodes:
		dot_file.write('%s [fillcolor="tan1", label="%s"];\n' %(onode.index, onode['name']))
	dot_file.write('}\n')

	# Create temporal to spatial edges
	for t_edge in graph.temp_spatial_edges:
		dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(t_edge[0], t_edge[1]))

	# Create spatial to object edges
	for r_edge in graph.spatial_obj_edges:
		dot_file.write('%s -> %s [arrowhead = "normal", color="red"];\n' %(r_edge[0], r_edge[1]))
	dot_file.write('}\n')
	dot_file.close()

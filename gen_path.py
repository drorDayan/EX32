from arr2_epec_seg_ex import *
from collision_detection import *
from conversions import *
import sample_polygons
import queue
import random
from math import pi

def rootp(s, parent):
	if (parent[s] == s):
		return s
	parent[s] = rootp(parent[s], parent)
	return parent[s]

def connected(a, b, parent):
	return rootp(a, parent) == rootp(b, parent)

def connect(a, b, parent):
	a = rootp(a, parent)
	b = rootp(b, parent)
	parent[a] = b

def generate_milestones(p1, p2, l, polygons, epsilon, n, max_x, max_y, min_x, min_y, length):
	if not is_position_valid(p1[0], p1[1], p1[2], l, polygons, epsilon):
		return []
	if not is_position_valid(p2[0], p2[1], p2[2], l, polygons, epsilon):
		return []
	v = []

	for theta in range(0, int(2 * pi * length.to_double()), int(epsilon.to_double())):
		theta = FT(theta)
		for x in range(min_x, max_x, int(epsilon.to_double())):
			x = FT(x)
			for y in range(min_y, max_y, int(epsilon.to_double())):
				y = FT(y)
				if is_position_valid(x, y, theta / length, l, polygons, epsilon):
					if len(v) and v[-1].x() == x and v[-1].y().to_double() + int(epsilon.to_double()) == y.to_double():
						v.remove(v[-1])
					v.append(Point_3(x, y, theta))
					if len(v) > n:
						return
		for y in range(min_y, max_y, int(epsilon.to_double())):
			y = FT(y)
			for x in range(min_x, max_x, int(epsilon.to_double())):
				x = FT(x)
				if is_position_valid(x, y, theta / length, l, polygons, epsilon):
					if len(v) and v[-1].y() == y and v[-1].x().to_double() + int(epsilon.to_double()) == x.to_double():
						v.remove(v[-1])
					v.append(Point_3(x, y, theta))
					if len(v) > n:
						return
		print(str(theta) + " " + str(len(v)))
	# while len(v) < n:
	# 	x = FT(random.randint(min_x, max_x))
	# 	y = FT(random.randint(min_y, max_y))
	# 	theta = FT(random.uniform(0, 2 * pi * length.to_double()))
	# 	if is_position_valid(x, y, theta / length, l, polygons, epsilon):
	# 		v.append(Point_3(x, y, theta))
	# 	if not len(v) % 10000:
	# 		print(len(v))
	# print(v)
	return v

def bfs(v, g, s, t, path, length):
	q = queue.Queue()
	parent = [-1 for a in g]
	edge = [[] for a in g]
	parent[s] = s
	q.put(s)
	while not q.empty():
		s = q.get()
		if s == t:
			break
		for a, e in g[s]:
			if parent[a] == -1:
				parent[a] = s
				edge[a] = e
				q.put(a)
	if parent[t] == -1:
		return False
	path.append([v[t].x(), v[t].y(), v[t].z(), True])
	while parent[t] != t:
		path.append([v[parent[t]].x(), v[parent[t]].y(), v[parent[t]].z() / length, not edge[t]])
		t = parent[t]
	return True


def get_min_max(length, obstacles, origin, destination):
	extra = 10
	max_x = max(max([max([x for x, y in obs]) for obs in obstacles]), int(origin[0]), int(destination[0])) + length + extra
	max_y = max(max([max([y for x, y in obs]) for obs in obstacles]), int(origin[1]), int(destination[1])) + length + extra
	min_x = min(min([min([x for x, y in obs]) for obs in obstacles]), int(origin[0]), int(destination[0])) - length - extra
	min_y = min(min([min([y for x, y in obs]) for obs in obstacles]), int(origin[1]), int(destination[1])) - length - extra
	return max_x, max_y, min_x, min_y


def k_nn(tree, k, query, eps):
	search_nearest = True
	sort_neighbors = True
	search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
	lst = []
	search.k_neighbors(lst)
	return lst

def is_valid(p1, p2, epsilon, length):
	# path_polygons = []
	# sample_polygons.sample_polygons(segment[0], segment[1], length, path_polygons)

	# path_set = Polygon_set_2()
	# path_set.join_polygons(path_polygons)
	# obstacles_set = Polygon_set_2()
	# obstacles_set.join_polygons(obstacle_polygons)

	# #check that there are no collisions
	# return True if not path_set.do_intersect(obstacles_set) else False
	x = p1.x().to_double() - p2.x().to_double()
	y = p1.y().to_double() - p2.y().to_double()
	z = p1.z().to_double() - p2.z().to_double()
	if (z < 0):
		z += 2 * pi
	z = min(z, 2 * pi - z)
	z *= length.to_double()
	return (x ** 2 + y ** 2 + z ** 2) ** 0.5 < 2 * epsilon.to_double()

def phash(point):
	return str([point.x(), point.y(), point.z()])

def make_graph(milestones, obstacles, epsilon, length):
	parent = [i for i in range(len(milestones))]
	obstacle_polygons = []
	for obs in obstacles:
	  p = tuples_list_to_polygon_2(obs)
	  obstacle_polygons.append(p)
	indexing = {}
	for i in range(len(milestones)):
		indexing[phash(milestones[i])] = i

	g = [[] for milestone in milestones]
	tree = Kd_tree(milestones)
	number_of_neighbors = 5
	for i, milestone in enumerate(milestones):
		if not i % 10000:
			print(i)
		temp = []
		nn = k_nn(tree, number_of_neighbors + 1, milestone, FT(Gmpq(0.0)))  # the + 1 to number_of_neighbors is to count for count v as it's neighbor
		for neighbor in nn[1:]:  # first point is self and no need for edge from v to itself
			if neighbor[1] > FT(Gmpq(0)):  # no need for edge from v to itself (just making sure)
				j = indexing[phash(neighbor[0])]
				if connected(i, j, parent):
					continue
				#print(str(neighbor[1]) + " " + str(milestones[i]) + " " + str(milestones[j]))
				# segment = [[milestone.x(), milestone.y(), milestone.z(), True], [neighbor[0].x(), neighbor[0].y(), neighbor[0].z(), True]]
				# if is_valid(segment, obstacle_polygons, epsilon):
				# 	connect(i, j, parent)
				# 	g[i].append((j, True))
				# 	g[j].append((i, False))
				# else:
				# 	segment[1][3] = False
				# 	if is_valid(segment, obstacle_polygons, epsilon):
				# 		connect(i, j, parent)
				# 		g[i].append((j, False))
				# 		g[j].append((i, True))
				if neighbor[1].to_double() < (2 * epsilon.to_double()) ** 2:
					z = milestones[i].z().to_double() - milestones[j].z().to_double()
					connect(i, j, parent)
					if (z < 0):
						z += 2 * pi * length.to_double()
					if (z < pi * length.to_double()):
						g[i].append((j, True))
						g[j].append((i, False))
					else:
						g[i].append((j, False))
						g[j].append((i, True))
	print(connected(len(g) - 1, len(g) - 2, parent))
	return g


def generate_path(path, length, obstacles, origin, destination):
	max_x, max_y, min_x, min_y = get_min_max(int(str(length)), obstacles, origin, destination)
	poly_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
	origin = [FT(Gmpq(x)) for x in origin]
	destination = [FT(Gmpq(x)) for x in destination]
	my_eps = FT(5)
	number_of_points_to_find = 1000000

	milestones = generate_milestones(origin, destination, length, poly_obstacles, my_eps, number_of_points_to_find, max_x, max_y, min_x, min_y, length)
	if len(milestones) == 0:
		return
	print("hello")
	# add origin and destination
	milestones.append(Point_3(origin[0], origin[1], origin[2] * length))
	milestones.append(Point_3(destination[0], destination[1], destination[2] * length))
	print(milestones[:min(10000, len(milestones))])
	g = make_graph(milestones, obstacles, my_eps, length)
	temp = []
	success = bfs(milestones, g, len(milestones) - 1, len(milestones) - 2, temp, length)
	if success:
		for t in temp:
			path.append(t)

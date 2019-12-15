from arr2_epec_seg_ex import *
from collision_detection import *
import queue
import random
from math import pi


def generate_milestones(p1, p2, l, polygons, epsilon, n, max_x, max_y, min_x, min_y):
	if not is_position_valid(p1[0], p1[1], p1[2], l, polygons, epsilon):
		return []
	if not is_position_valid(p2[0], p2[1], p2[2], l, polygons, epsilon):
		return []
	v = []
	while len(v) < n:
		x = FT(random.randint(min_x, max_x))
		y = FT(random.randint(min_y, max_y))
		theta = FT(random.uniform(0, 2 * pi))
		if is_position_valid(x, y, theta, l, polygons, epsilon):
			v.append(Point_3(x, y, theta))
	# print(v)
	return v


def bfs(v, g, s, t, path):
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
				q.put(s)
	if parent[t] == -1:
		return False
	path.append(v[t] + [True])
	while parent[t] != -1:
		path.append(v[parent[t]] + [not edge[t][3]])
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


def make_graph(milestones):
	g = []
	tree = Kd_tree(milestones)
	number_of_neighbors = 3
	for milestone in milestones:
		temp = []
		nn = k_nn(tree, number_of_neighbors + 1, milestone, FT(Gmpq(0.0)))  # the + 1 to number_of_neighbors is to count for count v as it's neighbor
		for neighbor in nn[1:]:  # first point is self and no need for edge from v to itself
			if neighbor[1] > FT(Gmpq(0)):  # no need for edge from v to itself (just making sure)
				# TODO make sure this is a valid edge
				# notice: the check here should be for 2 different edges (the clockwise and counter-clockwise)
				clockwise = True
				temp.append((neighbor[0], clockwise))
		g.append((milestone, temp))
	return g


def generate_path(path, length, obstacles, origin, destination):
	print(path)
	print(length)
	print(obstacles)
	print(origin)
	print(destination)
	# path.append((FT(Gmpq(200)), FT(Gmpq(500)), FT(Gmpq("0/3")), True))
	# path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("2/1")), True))
	# path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("1/1")), False))
	max_x, max_y, min_x, min_y = get_min_max(int(str(length)), obstacles, origin, destination)
	poly_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
	origin = [FT(Gmpq(x)) for x in origin]
	destination = [FT(Gmpq(x)) for x in destination]
	my_eps = FT(0)
	number_of_points_to_find = 1000

	milestones = generate_milestones(origin, destination, length, poly_obstacles, my_eps, number_of_points_to_find, max_x, max_y, min_x, min_y)
	if len(milestones) == 0:
		return
	# add origin and destination
	milestones.append(Point_3(origin[0], origin[1], origin[2]))
	milestones.append(Point_3(destination[0], destination[1], destination[2]))

	print(milestones)
	g = make_graph(milestones)
	print(g)
	temp = []
	success = bfs(milestones, g, 1, 0, temp)
	if success:
		for t in temp:
			path.append(t)

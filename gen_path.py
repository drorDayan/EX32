from arr2_epec_seg_ex import *
from collision_detection import *
import queue
import random
from math import pi
import time


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


def is_valid(p1, p2, l, polygons, epsilon, clockwise):
	p1x, p1y, p1z = p1.x(), p1.y(), p1.z()
	p2x, p2y, p2z = p2.x(), p2.y(), p2.z()
	curr = [p1x, p1y, p1z]
	x_diff = p2x - p1x
	y_diff = p2y - p1y
	if (clockwise and p1z >= p2z) or (not clockwise and p1z < p2z):
		z_diff = p2z - p1z
	elif clockwise and p1z < p2z:
		z_diff = FT(-1)*(p1z+(FT(2*pi)-p2z))
	else:
		z_diff = p2z+(FT(2*pi)-p1z)
	length = math.sqrt(float(str(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff)))
	# print(length)
	num_of_steps = math.ceil(length/(float(str(epsilon))/2))
	diff_vec = [x_diff/FT(num_of_steps), y_diff/FT(num_of_steps), z_diff/FT(num_of_steps)]
	# print(num_of_steps)
	for i in range(num_of_steps):
		curr[0] += diff_vec[0]
		curr[1] += diff_vec[1]
		curr[2] = FT(float(str((curr[2] + diff_vec[2] + FT(2*pi)))) % (2*pi))
		# print(curr)
		if not is_position_valid(curr[0], curr[1], curr[2], l, polygons, epsilon):
			return False
	# print(p1)
	# print(p2)
	return True


def get_nn(milestone, l, polygons, epsilon, tree, groups, number_of_neighbors):
	temp = []
	nn = k_nn(tree, number_of_neighbors + 1, milestone, FT(Gmpq(0.0)))  # the + 1 to number_of_neighbors is to count for count v as it's neighbor
	for neighbor in nn[1:]:  # first point is self and no need for edge from v to itself
		if groups[str(milestone)].count(str(neighbor[0])) == 0:
			# notice: the check here should be for 2 different edges (the clockwise and counter-clockwise)
			if is_valid(milestone, neighbor[0], l, polygons, epsilon, True):
				temp.append((neighbor[0], True))
				groups[str(milestone)] = groups[str(milestone)] + groups[str(neighbor[0])]
				groups[str(neighbor[0])] = groups[str(milestone)]
			elif is_valid(milestone, neighbor[0], l, polygons, epsilon, False):
				temp.append((neighbor[0], False))
				groups[str(milestone)] = groups[str(milestone)] + groups[str(neighbor[0])]
				groups[str(neighbor[0])] = groups[str(milestone)]
	return temp


def make_graph(milestones, l, polygons, epsilon):
	g = []
	tree = Kd_tree(milestones)
	groups = {}
	for milestone in milestones:
		groups[str(milestone)] = [str(milestone)]
	for milestone in milestones:
		for number_of_neighbors in [15, 30, 50]:
			temp = get_nn(milestone, l, polygons, epsilon, tree, groups, number_of_neighbors)
			if len(temp) > 0:
				break
			else:
				print("found a point with no valid neighbors no in its group for number_of_neighbors = "+str(number_of_neighbors))
		g.append((milestone, temp))
	return g


def generate_path(path, length, obstacles, origin, destination):
	print(path)
	print(length)
	print(obstacles)
	print(origin)
	print(destination)
	start = time.time()
	max_x, max_y, min_x, min_y = get_min_max(int(str(length)), obstacles, origin, destination)
	poly_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
	origin = [FT(Gmpq(x)) for x in origin]
	destination = [FT(Gmpq(x)) for x in destination]
	my_eps = FT(5)
	for number_of_points_to_find in [5000 * i for i in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192]]:
		print("number_of_points_to_find")
		print(number_of_points_to_find)
		milestones = generate_milestones(origin, destination, length, poly_obstacles, my_eps, number_of_points_to_find, max_x, max_y, min_x, min_y)
		if len(milestones) == 0:
			return
		# add origin and destination
		milestones.append(Point_3(origin[0], origin[1], origin[2]))
		milestones.append(Point_3(destination[0], destination[1], destination[2]))

		print(milestones)
		g = make_graph(milestones, length, poly_obstacles, my_eps)
		print(g)

		done = time.time()
		elapsed = done - start
		print(elapsed)

		temp = []
		success = bfs(milestones, g, len(milestones) - 1, len(milestones) - 2, temp)
		if success:
			for t in temp:
				path.append(t)
			break  # no need to retry with more points

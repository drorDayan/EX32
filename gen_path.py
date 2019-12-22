from arr2_epec_seg_ex import *
from collision_detection import *
import queue
import random
from math import pi
import time

# dist
pole_l = 0


# The following function returns the transformed distance between two points
# (for Euclidean distance the transformed distance is the squared distance)
def transformed_distance(p1, p2):
	global pole_l
	x_diff = p2.x() - p1.x()
	y_diff = p2.y() - p1.y()
	z_diff = p2.z() - p1.z()  # min(FT(FT.to_double((p2.z() - p1.z()+FT(2*pi))) % 2*pi), FT(FT.to_double((p1.z() - p2.z()+FT(2*pi))) % 2*pi))
	return FT(math.sqrt(FT.to_double((x_diff * x_diff + y_diff * y_diff + z_diff * (pole_l) * z_diff * (pole_l)))))


# The following function returns the transformed distance between the query
# point q and the point on the boundary of the rectangle r closest to q.
def min_distance_to_rectangle(q, r):
	global pole_l
	min_coord_x = r.min_coord(0)
	min_coord_y = r.min_coord(1)
	min_coord_z = r.min_coord(2)
	max_coord_x = r.max_coord(0)
	max_coord_y = r.max_coord(1)
	max_coord_z = r.max_coord(2)
	qx = q.x()
	qy = q.y()
	qz = q.z()
	if qx < min_coord_x:
		x_diff = min_coord_x - qx
	elif qx > max_coord_x:
		x_diff = qx - max_coord_x
	else:
		x_diff = FT(0)
	if qy < min_coord_y:
		y_diff = min_coord_y - qy
	elif qy > max_coord_y:
		y_diff = qy - max_coord_y
	else:
		y_diff = FT(0)
	if qz < min_coord_z:
		z_diff = (min_coord_z - qz)
	elif qz > max_coord_z:
		z_diff = (qz - max_coord_z)
	else:
		z_diff = FT(0)
	return FT(math.sqrt(FT.to_double((x_diff * x_diff + y_diff * y_diff + z_diff * (pole_l) * z_diff * (pole_l)))))


# The following function returns the transformed distance between the query
# point q and the point on the boundary of the rectangle r furthest to q.
def max_distance_to_rectangle(q, r):
	global pole_l
	min_coord_x = r.min_coord(0)
	min_coord_y = r.min_coord(1)
	min_coord_z = r.min_coord(2)
	max_coord_x = r.max_coord(0)
	max_coord_y = r.max_coord(1)
	max_coord_z = r.max_coord(2)
	qx = q.x()
	qy = q.y()
	qz = q.z()
	if qx < min_coord_x:
		x_diff = max_coord_x - qx
	elif qx > max_coord_x:
		x_diff = qx - min_coord_x
	else:
		x_diff = FT(0)
	if qy < min_coord_y:
		y_diff = max_coord_y - qy
	elif qy > max_coord_y:
		y_diff = qy - min_coord_y
	else:
		y_diff = FT(0)
	if qz < min_coord_z:
		z_diff = (max_coord_z - qz)
	elif qz > max_coord_z:
		z_diff = (qz - min_coord_z)
	else:
		z_diff = FT(0)
	return FT(math.sqrt(FT.to_double((x_diff * x_diff + y_diff * y_diff + z_diff * (pole_l) * z_diff * (pole_l)))))


# The following function returns the transformed distance for a value d
# Fo example, if d is a value computed using the Euclidean distance, the transformed distance should be d*d
def transformed_distance_for_value(d):
	return d*d


# The following function returns the inverse of the transformed distance for a value d
# Fo example, if d is a sqaured distance value then its inverse should be sqrt(d)
def inverse_of_transformed_distance_for_value(d):
	return FT(math.sqrt(FT.to_double(d)))


distance = Distance_python(transformed_distance, min_distance_to_rectangle, max_distance_to_rectangle, transformed_distance_for_value, inverse_of_transformed_distance_for_value)
# code


def generate_milestones(l, polygons, epsilon, n, max_x, max_y, min_x, min_y):
	v = []
	while len(v) < n:
		x = FT(random.randint(min_x, max_x))
		y = FT(random.randint(min_y, max_y))
		theta = FT(random.uniform(0, 2 * pi))
		if is_position_valid(x, y, theta, l, polygons, epsilon):
			v.append(Point_3(x, y, theta))
	return v


def bfs(v, graph, s, t, path):
	g = [[] for x in v]
	numbering = {}
	for i in range(len(v)):
		numbering[str(v[i])] = i
	for u, adj in graph:
		u = numbering[str(u)]
		for x, d in adj:
			g[u].append((numbering[str(x)], d))
			g[numbering[str(x)]].append((u,not d))
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
		path.append([v[parent[t]].x(), v[parent[t]].y(), v[parent[t]].z(), not edge[t]])
		t = parent[t]
	return True


def get_min_max(obstacles, origin, destination):
	max_x = max(max([max([x for x, y in obs]) for obs in obstacles]), float(origin[0]), float(destination[0]))
	max_y = max(max([max([y for x, y in obs]) for obs in obstacles]), float(origin[1]), float(destination[1]))
	min_x = min(min([min([x for x, y in obs]) for obs in obstacles]), float(origin[0]), float(destination[0]))
	min_y = min(min([min([y for x, y in obs]) for obs in obstacles]), float(origin[1]), float(destination[1]))
	return max_x, max_y, min_x, min_y


def k_nn(tree, k, query, eps):
	search_nearest = True
	sort_neighbors = True
	search = K_neighbor_search_python(tree, query, k, eps, search_nearest, distance, sort_neighbors)
	lst = []
	search.k_neighbors(lst)
	return lst


def segment_valid(p1, l, polygons, epsilon, x_diff, y_diff, z_diff):
	if FT(math.sqrt(FT.to_double(x_diff * x_diff + y_diff * y_diff + z_diff*l * z_diff*l))) <= epsilon:
		return True
	mid = [p1.x()+x_diff/FT(2), p1.y()+y_diff/FT(2), FT(FT.to_double((p1.z() + z_diff/FT(2) + FT(2*pi))) % (2*pi))]
	diff = [x_diff/FT(2), y_diff/FT(2), z_diff/FT(2)]
	if not is_position_valid(mid[0], mid[1], mid[2], l, polygons, epsilon):
		return False
	if not segment_valid(p1, l, polygons, epsilon, diff[0], diff[1], diff[2]):
		return False
	return segment_valid(Point_3(mid[0], mid[1], mid[2]), l, polygons, epsilon, diff[0], diff[1], diff[2])


def is_valid(p1, p2, l, polygons, epsilon, clockwise):
	p1x, p1y, p1z = p1.x(), p1.y(), p1.z()
	p2x, p2y, p2z = p2.x(), p2.y(), p2.z()
	x_diff = p2x - p1x
	y_diff = p2y - p1y
	if (clockwise and p1z >= p2z) or (not clockwise and p1z < p2z):
		z_diff = p2z - p1z
	elif clockwise and p1z < p2z:
		z_diff = FT(-1)*(p1z+(FT(2*pi)-p2z))
	else:
		z_diff = p2z+(FT(2*pi)-p1z)
	return segment_valid(p1,  l, polygons, epsilon, x_diff, y_diff, z_diff)


class DisjointSet:
	def __init__(self, vertices, parent):
		self.vertices = vertices
		self.parent = parent

	def find(self, item):
		if self.parent[item] == item:
			return item
		else:
			return self.find(self.parent[item])

	def union(self, set1, set2):
		root1 = self.find(set1)
		root2 = self.find(set2)
		self.parent[root1] = root2


def make_graph(milestones, l, polygons, epsilon):
	g = []
	tree = Kd_tree(milestones)
	groups = {}
	number_of_neighbors = 15
	for milestone in milestones:
		groups[str(milestone)] = str(milestone)
	ds = DisjointSet(milestones, groups)
	for milestone in milestones:
		temp = []
		nn = k_nn(tree, number_of_neighbors + 1, milestone, FT(0))  # the + 1 to number_of_neighbors is to count for count v as it's neighbor
		for neighbor in nn[1:]:  # first point is self and no need for edge from v to itself
			if ds.find(str(milestone)) != ds.find(str(neighbor[0])):
				# notice: the check here should be for 2 different edges (the clockwise and counter-clockwise)
				if is_valid(milestone, neighbor[0], l, polygons, epsilon, True):
					temp.append((neighbor[0], True))
					ds.union(str(milestone), str(neighbor[0]))
				elif is_valid(milestone, neighbor[0], l, polygons, epsilon, False):
					temp.append((neighbor[0], False))
					ds.union(str(milestone), str(neighbor[0]))
		g.append((milestone, temp))
	return g


def generate_path(path, length, obstacles, origin, destination):
	global pole_l
	pole_l = length
	start = time.time()
	max_x, max_y, min_x, min_y = get_min_max(obstacles, origin, destination)
	poly_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
	origin = [FT(Gmpq(x)) for x in origin]
	destination = [FT(Gmpq(x)) for x in destination]
	my_eps = FT(5)
	if not is_position_valid(origin[0], origin[1], origin[2], length, poly_obstacles, my_eps) or not is_position_valid(destination[0], destination[1], destination[2], length, poly_obstacles, my_eps):
		print("invalid input")
		return False

	for number_of_points_to_find in [1000 * i for i in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192]]:
		print("number_of_points_to_find: " + str(number_of_points_to_find))
		milestones = generate_milestones(length, poly_obstacles, my_eps, number_of_points_to_find, max_x, max_y, min_x, min_y)
		print("generated milestones, "+"time= "+str(time.time()-start))
		# add origin and destination
		milestones.append(Point_3(origin[0], origin[1], origin[2]))
		milestones.append(Point_3(destination[0], destination[1], destination[2]))

		g = make_graph(milestones, length, poly_obstacles, my_eps)
		print("finish making the graph, "+"time= "+str(time.time()-start))

		temp = []
		success = bfs(milestones, g, len(milestones) - 1, len(milestones) - 2, temp)
		if success:
			for t in temp:
				path.append(t)
			break  # no need to retry with more points

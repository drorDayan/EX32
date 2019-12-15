from arr2_epec_seg_ex import *
import queue
import random

def generate_milestones(p1, p2, l, polygons, epsilon, n):
	if not is_position_valid(p1[0], p1[1], p1[2], l, polygons, epsilon):
		return []
	if not is_position_valid(p2[0], p2[1], p2[2], l, polygons, epsilon):
		return []
	v = [p1, p2]
	while len(v) < n:
		x = random.randint(0, 10000)
		y = random.randint(0, 10000)

def bfs(v, g, s, t, path):
	q = queue.Queue()
	parent = [-1 for a in g]
	edge = [[] for a in g]
	parent[s] = s
	q.put(s)
	while not q.empty():
		s = q.get()
		if (s == t):
			break
		for a, e in g[s]:
			if (parent[a] == -1):
				parent[a] = s
				edge[a] = e
				q.put(s)
	if (parent[t] == -1):
		return False
	path.append(v[t] + [True])
	while parent[t] != -1:
		path.append(v[parent[t]] + [not edge[t][3]])
		t = parent[t]
	return True

def generate_path(path, length, obstacles, origin, destination):
	print(path)
	print(length)
	print(obstacles)
	print(origin)
	print(destination)
	path.append((FT(Gmpq(200)), FT(Gmpq(500)), FT(Gmpq("0/3")), True))
	path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("2/1")), True))
	path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("1/1")), False))

	milestones = generate_milestones(origin, destination, obstacles, length, 0, 100)
	if len(milestones) == 0:
		return
	g = make_graph(milestones)
	temp = []
	success = bfs(milestones, g, 1, 0, temp)
	if success:
		for t in temp:
			path.append(t)

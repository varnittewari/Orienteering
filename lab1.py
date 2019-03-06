from PIL import Image
from collections import defaultdict
from heapq import heappush, heappop
import math
import time
from collections import deque

class Orienteering:
    __slots__ = ["elevation", "terrain", "terrain_speed", "course",
                 "output_points", "season", "winter", "fall", "spring"]

    def __init__(self, elevation_file, color_file, course_file, season="summer"):
        self.terrain_speed = {(248, 148, 18, 255): 9, (255, 192, 0, 255): 5.5, (255, 255, 255, 255): 6.5,
                              (2, 208, 60, 255): 5, (2, 136, 40, 255): 3, (5, 73, 24, 255): 0.5,
                              (0, 0, 255, 255): 0.2, (71, 51, 3, 255): 10, (0, 0, 0, 255): 9.5, (205, 0, 101, 255): 0,
                              (255, 255, 0, 255): 6.5, (121, 76, 19, 255): 2, (0, 191, 255, 255): 4}
        self.elevation = {}
        self.read_elevation(elevation_file)
        self.terrain = []
        self.read_terrain(color_file)
        self.course = []
        self.read_course(course_file)
        self.output_points = set()
        self.season = season
        if season == "winter":
            self.winter = set()
            self.find_boundary((0, 0, 255, 255), (0, 191, 255, 255))
            self.make_winter()
        if season == "fall":
            self.fall = set()
            self.find_boundary((255, 255, 255, 255), (255, 255, 0, 255))
            self.make_fall()
        if season == "spring":
            self.spring = set()
            self.find_boundary((0, 0, 255, 255), (121, 76, 19, 255))
            self.make_spring()
        self.find_path()
        self.create_output()

    def find_boundary(self, old_color, new_color):
        for i in range(len(self.terrain)):
            x = i % 395
            y = i // 395
            node = (x, y)
            current = self.terrain[self.terrain_index(node)]
            if current == old_color:
                neighbors = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
                for neighbor in neighbors:
                    i, j = neighbor
                    cond1 = 0 <= i < 395
                    cond2 = 0 <= j < 500
                    if cond1 and cond2:
                        neighbor_value = self.terrain[self.terrain_index(neighbor)]
                        if neighbor_value != current:
                                if self.season == "winter":
                                    self.winter.add(neighbor)
                                    self.terrain[self.terrain_index(neighbor)] = new_color
                                if self.season == "fall":
                                    self.fall.add(neighbor)
                                if self.season == "spring":
                                    self.spring.add(neighbor)

    def make_winter(self):
        queue = deque(list(self.winter))
        visited = defaultdict(lambda: 0)
        depth = defaultdict(lambda: math.inf)
        for x in queue:
            depth[x] = 1
            visited[x] = 1
        while queue:
            current = queue.popleft()
            self.winter.add(current)
            self.terrain[self.terrain_index(current)] = (0, 191, 255, 255)
            if depth[current] > 7:
                break
            for neighbor in self.get_neighbors(current):
                neighbor_value = self.terrain[self.terrain_index(neighbor)]
                if neighbor_value == (0, 0, 255, 255):
                    if visited[neighbor] == 0:
                        visited[neighbor] = 1
                        depth[neighbor] = depth[current] + 1
                        queue.append(neighbor)

    def make_spring(self):
        queue = deque(list(self.spring))
        visited = defaultdict(lambda: 0)
        depth = defaultdict(lambda: math.inf)
        delta_height = defaultdict(lambda: math.inf)
        for x in queue:
            depth[x] = 0
            visited[x] = 0
            delta_height[x] = 0
        while queue:
            current = queue.popleft()
            self.terrain[self.terrain_index(current)] = (121, 76, 19, 255)
            if depth[current] > 15:
                break
            for neighbor in self.get_neighbors(current):
                neighbor_value = self.terrain[self.terrain_index(neighbor)]
                current_elevation = self.elevation[self.elevation_ind(current)]
                neighbor_elevation = self.elevation[self.elevation_ind(neighbor)]
                elevation_difference = (neighbor_elevation - current_elevation)
                delta_height[neighbor] = delta_height[current] + elevation_difference
                if neighbor_value != (0, 0, 255, 255):
                    if delta_height[neighbor] > 1:
                        break
                    else:
                        if visited[neighbor] == 0:
                            visited[neighbor] = 1
                            depth[neighbor] = depth[current] + 1
                            queue.append(neighbor)

    def make_fall(self):
        for point in self.fall:
            for neighbor in self.get_neighbors(point):
                neighbor_value = self.terrain[self.terrain_index(neighbor)]
                cond1 = (neighbor_value == (71, 51, 3, 255))
                cond2 = (neighbor_value == (0, 0, 0, 255))
                if cond1 or cond2:
                    self.terrain[self.terrain_index(neighbor)] = (255, 255, 0, 255)

    def elevation_ind(self, point):
        x, y = point
        t = (y, x)
        return t

    def read_elevation(self, filename):
        with open(filename) as elevation:
            i = 0
            for line in elevation:
                line = line.strip()
                k = line.split()
                for j in range(395):
                    t = (i, j)
                    self.elevation[t] = float(k[j])
                i = i+1

    def read_terrain(self, filename):
        im = Image.open(filename)
        self.terrain = list(im.getdata())

    def terrain_index(self, point):
        x, y = point
        return y * 395 + x

    def speed(self, point):
        x, y = point
        return self.terrain_speed[self.terrain[self.terrain_index((x, y))]]

    def read_course(self, filename):
        with open(filename) as course:
            for line in course:
                line = line.strip()
                x, y = line.split()
                x, y = int(x), int(y)
                checkpoint = (x, y)
                self.course.append(tuple(checkpoint))

    def get_neighbors(self, point):
        x, y = point
        neighbors = []
        for i in (x - 1, x, x + 1):
            for j in (y - 1, y, y + 1):
                cond1 = not(i == x and j == y)
                cond2 = 0 <= i < 395
                cond3 = 0 <= j < 500
                if cond1 and cond2 and cond3:
                    neighbors.append((i, j))
        return neighbors

    def cost(self, current, neighbor):
        x = 10.29
        y = 7.55
        x1, y1 = current
        x2, y2 = neighbor
        xy_cost = math.sqrt(((y2-y1)*y)**2 + ((x2-x1)*x)**2)
        neighbor_speed = self.speed(neighbor)
        current_elevation = self.elevation[(y1, x1)]
        neighbor_elevation = self.elevation[(y2, x2)]
        elevation_difference = abs(neighbor_elevation - current_elevation)
        if not(x1 == x2 and y1 == y2):
            if neighbor_speed == 0:
                return float("inf")
            xyz_distance = math.sqrt(xy_cost**2 + elevation_difference**2)
            return xyz_distance/neighbor_speed
        else:
            return 0

    def heuristic(self, current, target):
        x = 10.29
        y = 7.55
        x1, y1 = current
        x2, y2 = target
        speed = 10
        euclidean_distance = math.sqrt(((y2-y1)*y)**2 + ((x2-x1)*x)**2)
        if not(x1 == x2 and y1 == y2):
            return euclidean_distance/speed
        else:
            return 0

    def find_path(self):
        cost = 0
        for i in range(len(self.course)-1):
            start = self.course[i]
            goal = self.course[i+1]
            cost += self.a_star(start, goal)
        print("Total cost for "+self.season+" is "+str(cost)+" seconds")

    def create_output(self):
        image = Image.new("RGBA", (395, 500), "white")
        im = image.load()
        for i in range(len(self.terrain)):
            x = i % 395
            y = i // 395
            node = (x, y)
            if node not in self.output_points:
                im[node] = self.terrain[i]
        for point in self.output_points:
            im[point] = (255, 0, 0, 255)
        for point in self.course:
            im[point] = (138, 43, 226, 255)
        image.save(self.season+".png")

    def is_in(self, node, list_of_list):
        for i in list_of_list:
            if i[1] == node:
                return True
        return False

    def a_star(self, start, goal):
        open = []
        closed = set()
        predecessor = {}
        g = defaultdict(lambda: math.inf)
        g[start] = 0
        f = defaultdict(lambda: math.inf)
        f[start] = self.heuristic(start, goal)
        heappush(open, [f[start], start])
        while open:
            current = heappop(open)[1]
            if current == goal:
                path = [current]
                return_value = g[current]
                self.output_points.add(current)
                while current in predecessor.keys():
                    current = predecessor[current]
                    path.append(current)
                    self.output_points.add(current)
                return return_value
            closed.add(current)
            for neighbor in self.get_neighbors(current):
                if self.speed(neighbor) != 0:
                    if neighbor in closed:
                        continue
                    if self.is_in(neighbor, open):
                        new_g = g[current] + self.cost(current, neighbor)
                        if new_g < g[neighbor]:
                            predecessor[neighbor] = current
                            g[neighbor] = new_g
                            f[neighbor] = g[neighbor] + self.heuristic(neighbor, goal)
                            heappush(open, [f[neighbor], neighbor])
                    else:
                        predecessor[neighbor] = current
                        g[neighbor] = g[current] + self.cost(current, neighbor)
                        f[neighbor] = g[neighbor] + self.heuristic(neighbor, goal)
                        heappush(open, [f[neighbor], neighbor])
        return False


if __name__ == '__main__':
    p0 = time.time()
    for season in ["summer", "fall", "winter", "spring"]:
        t0 = time.time()
        obj = Orienteering("mpp.txt", "terrain.png", "red.txt", season)
        t1 = time.time()
        print("Execution time for "+season+" is "+str(t1-t0)+"\n")
    p1 = time.time()
    print("Total time:", (p1-p0))
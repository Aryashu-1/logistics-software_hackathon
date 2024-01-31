from flask import Flask, render_template, request, jsonify
import math
import heapq
import random

app = Flask(__name__)

class Edge:
    def __init__(self, dest, weight=1, net_demand=0):
        self.dest = dest
        self.weight = weight
        self.net_demand = net_demand

class Graph:
    def __init__(self):
        self.adjList = {}
        self.net_demand = {}

    def addVertex(self, vertex, demand=0):
        if vertex not in self.adjList:
            self.adjList[vertex] = []
            self.net_demand[vertex] = demand
        else:
            self.net_demand[vertex] += demand

    def addEdge(self, src, dest, dist, cost, time):
        net_demand_src = self.net_demand[src]
        net_demand_dest = self.net_demand[dest]
        weight = dist * cost * time * abs(net_demand_src - net_demand_dest)
        edge = Edge(dest, self.edgeWeight(weight), net_demand_src - net_demand_dest)
        self.adjList[src].append(edge)
        edge = Edge(src, self.edgeWeight(weight), net_demand_dest - net_demand_src)
        self.adjList[dest].append(edge)

    def edgeWeight(self, edge_number):
        sigmoid_output = 1 / (1 + math.exp(-edge_number*0.01))
        return int(sigmoid_output*100)

    def eagerDijkstra(self, start, end):
        distances = {vertex: math.inf for vertex in self.adjList}
        distances[start] = 0
        visited = set()
        pq = []  # Indexed Priority Queue [(distance, vertex)]

        def update_distance(vertex, new_distance):
            for i, (dist, v) in enumerate(pq):
                if v == vertex:
                    pq[i] = (new_distance, vertex)
                    heapq.heapify(pq)
                    break

        heapq.heappush(pq, (0, start))

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)

            if current_vertex == end:
                break

            if current_vertex in visited:
                continue

            visited.add(current_vertex)

            for edge in self.adjList[current_vertex]:
                weight = edge.weight
                neighbor = edge.dest
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    if neighbor not in visited:
                        heapq.heappush(pq, (distance, neighbor))
                    else:
                        update_distance(neighbor, distance)

        return distances[end]

    def prim(self, start):
        visited = set()
        edges_heap = []  # Min Heap for edges [(weight, source, destination)]

        for edge in self.adjList[start]:
            heapq.heappush(edges_heap, (edge.weight, start, edge.dest))

        visited.add(start)
        min_spanning_tree = []

        while edges_heap:
            weight, src, dest = heapq.heappop(edges_heap)

            if dest not in visited:
                visited.add(dest)
                min_spanning_tree.append((src, dest, weight))

                for edge in self.adjList[dest]:
                    if edge.dest not in visited:
                        heapq.heappush(edges_heap, (edge.weight, dest, edge.dest))

        return min_spanning_tree

# Example usage:
g = Graph()

# Adding seven random vertices
for i in range(7):
    net_demand = random.randint(-10, 10)
    g.addVertex(i, net_demand)

# Adding edges with random parameters
for i in range(7):
    for j in range(i + 1, 7):
        dist = random.randint(5, 20)
        cost = random.uniform(1, 5)
        time = random.uniform(1, 5)
        g.addEdge(i, j, dist, cost, time)

# Flask routes
@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        start_node = int(request.form.get('start_node'))
        end_node = int(request.form.get('end_node'))

        # Running Eager Dijkstra's algorithm from specified start to end nodes
        shortest_distance = g.eagerDijkstra(start_node, end_node)

        return render_template('index.html', shortest_distance=shortest_distance, start_node=start_node, end_node=end_node, vertices=g.net_demand.items())

    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True)

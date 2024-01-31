class Edge {
    constructor(dest, weight = 1, netDemand = 0) {
        this.dest = dest;
        this.weight = weight;
        this.netDemand = netDemand;
    }
}

class Graph {
    constructor() {
        this.adjList = {};
        this.netDemand = {};
    }

    addVertex(vertex, demand = 0) {
        if (!(vertex in this.adjList)) {
            this.adjList[vertex] = [];
            this.netDemand[vertex] = demand;
        } else {
            this.netDemand[vertex] += demand;
        }
    }

    addEdge(src, dest, dist, cost, time) {
        const netDemandSrc = this.netDemand[src];
        const netDemandDest = this.netDemand[dest];
        const weight = dist * cost * time * Math.abs(netDemandSrc - netDemandDest);
        const edge = new Edge(dest, this.edgeWeight(weight), netDemandSrc - netDemandDest);
        this.adjList[src].push(edge);
        const reverseEdge = new Edge(src, this.edgeWeight(weight), netDemandDest - netDemandSrc);
        this.adjList[dest].push(reverseEdge);
    }

    printGraph() {
        const result = [];
        for (const vertex in this.adjList) {
            const edges = this.adjList[vertex].map(edge => `(${edge.dest}, Weight: ${edge.weight}, NetDemand: ${edge.netDemand})`);
            result.push(`Vertex ${vertex} connects to: ${edges.join(', ')}`);
        }
        result.push("\nList of all vertices along with their net demands:");
        for (const [vertex, demand] of Object.entries(this.netDemand)) {
            result.push(`Vertex ${vertex}: Net Demand: ${demand}`);
        }
        return result;
    }

    edgeWeight(edgeNumber) {
        const sigmoidOutput = 1 / (1 + Math.exp(-edgeNumber * 0.01));
        return Math.floor(sigmoidOutput * 100);
    }

    eagerDijkstra(start, end) {
        const distances = {};
        for (const vertex in this.adjList) {
            distances[vertex] = Infinity;
        }
        distances[start] = 0;
        const visited = new Set();
        const pq = new PriorityQueue();  // Use the implemented priority queue

        const updateDistance = (vertex, newDistance) => {
            pq.enqueue([newDistance, vertex], newDistance);
        };

        pq.enqueue([0, start], 0);

        while (pq.size() > 0) {
            const [currentDistance, currentVertex] = pq.dequeue();

            if (currentVertex == end) {
                break;
            }

            if (visited.has(currentVertex)) {
                continue;
            }

            visited.add(currentVertex);

            for (const edge of this.adjList[currentVertex]) {
                const weight = edge.weight;
                const neighbor = edge.dest;
                const distance = currentDistance + weight;

                if (distance < distances[neighbor]) {
                    distances[neighbor] = distance;
                    if (!visited.has(neighbor)) {
                        pq.enqueue([distance, neighbor], distance);
                    } else {
                        updateDistance(neighbor, distance);
                    }
                }
            }
        }

        return distances[end];
    }

    prim(start) {
        const visited = new Set([start]);
        const edgesHeap = new PriorityQueue(); // Use the implemented priority queue

        // Initialize with the edges of the starting node
        for (const edge of this.adjList[start]) {
            edgesHeap.enqueue([edge.weight, start, edge.dest], edge.weight);
        }

        const minSpanningTree = [];

        while (edgesHeap.size() > 0) {
            const [weight, src, dest] = edgesHeap.dequeue();

            if (!visited.has(dest)) {
                visited.add(dest);
                minSpanningTree.push([src, dest, weight]);

                // Add new edges to the priority queue
                for (const edge of this.adjList[dest]) {
                    if (!visited.has(edge.dest)) {
                        edgesHeap.enqueue([edge.weight, dest, edge.dest], edge.weight);
                    }
                }
            }
        }

        return minSpanningTree;
    }
}

class PriorityQueue {
    constructor() {
        this.heap = [];
    }

    size() {
        return this.heap.length;
    }

    enqueue(element, priority) {
        this.heap.push({ element, priority });
        this.heap.sort((a, b) => a.priority - b.priority);
    }

    dequeue() {
        if (this.size() === 0) {
            return null;
        }
        return this.heap.shift().element;
    }
}

// Example usage:

const g = new Graph();

// Adding seven random vertices
for (let i = 0; i < 7; i++) {
    const netDemand = Math.floor(Math.random() * 21) - 10;
    g.addVertex(i, netDemand);
}

// Adding edges with random parameters
for (let i = 0; i < 7; i++) {
    for (let j = i + 1; j < 7; j++) {
        const dist = Math.floor(Math.random() * 16) + 5;
        const cost = Math.random() * 4 + 1;
        const time = Math.random() * 4 + 1;
        g.addEdge(i, j, dist, cost, time);
    }
}

// Print the graph
const graphOutput = g.printGraph();
console.log('Graph:', graphOutput);

// Running Prim's algorithm to find the minimum spanning tree from specified start node
const startNodePrim = 0;
const minimumSpanningTree = g.prim(startNodePrim);
console.log("\nMinimum Spanning Tree edges:");
for (const edge of minimumSpanningTree) {
    console.log(`Edge: ${edge[0]} - ${edge[1]}, Weight: ${edge[2]}`);
}

// Running Eager Dijkstra's algorithm from specified start to end nodes
const startNodeDijkstra = 0;
const endNodeDijkstra = 6;
const shortestDistance = g.eagerDijkstra(startNodeDijkstra, endNodeDijkstra);
console.log(`\nShortest distance from Node ${startNodeDijkstra} to Node ${endNodeDijkstra} using Eager Dijkstra's algorithm:`);
console.log(`Distance: ${shortestDistance}`);

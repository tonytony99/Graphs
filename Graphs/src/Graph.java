import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

/* A representation of an undirected graph.
 * Example : the graph below would be represented as 
 *	     (1)
 *	    /
 *	   3
 *	  /
 *	(0)
 *	  \
 *	   5
 *	    \
 *	     (2)
 * nodes = [0,1,2]
 * adjacencyList = [[1,2],[0],[0]]
 * weights = [[3,5],[3],[5]]
 */

public class Graph {
	ArrayList<ArrayList<Integer>> adjacencyList;
	ArrayList<ArrayList<Integer>> weights;
	ArrayList<Integer> nodes;
	int highestNode;
	
	// Add a node to this graph with number = number of nodes
	// in the graph before this one was added
	int addNode(){
		highestNode ++;
		nodes.add(highestNode);
		// Holds all of the neighbours of the new node
		adjacencyList.add(new ArrayList<Integer>());
		// Holds all of the weights between the node and its neighbours
		weights.add(new ArrayList<Integer>());
		return highestNode;
	}	
	
	// Add multiple nodes to this graph
	void addNodes(int n){
		for (int i = 0; i < n; i++){
			addNode();
		}
		return;
	}
	
	// Graphs are undirected so adding an edge from node a to node b
	// means also adding an edge from node b to node a (with the same weight)
	void addEdge(int from, int to, int weight){
		if (adjacencyList.get(from).contains(to) || adjacencyList.get(from).contains(from)) {
			return;
		}
		adjacencyList.get(from).add(to);
		adjacencyList.get(to).add(from);
		weights.get(from).add(weight);
		weights.get(to).add(weight);
		return;
	}
	
	// If no weight is specified then a weight of 0 is used for that edge
	void addEdge(int from, int to){
		if (adjacencyList.get(from).contains(to) || adjacencyList.get(from).contains(from)) {
			return;
		}
		adjacencyList.get(from).add(to);
		adjacencyList.get(to).add(from);
		weights.get(from).add(0);
		weights.get(to).add(0);
		return;
	}
	
	// Get the nodes which node has edges to
	ArrayList<Integer> getNeighbours(int node){
		return adjacencyList.get(node);
	}
	
	// Get the weight on the edge between node from and node to
	int getWeight(int from, int to){
		int weightIndex = adjacencyList.get(from).indexOf(to);
		return weights.get(from).get(weightIndex);		
	}
	
	// Get the list of nodes in this graph
	ArrayList<Integer> getNodes() {
		return nodes;
	}
	
	// Get the path visited by DFS through this graph starting from startNode
	// given the path that's already been followed so far
	ArrayList<Integer> depthFirstSearch(int startNode, ArrayList<Integer> path){
		if (path.contains(startNode)) {
			return path;
		}
		path.add(startNode);
		ArrayList<Integer> neighbours = getNeighbours(startNode);
		for (Iterator<Integer> i = neighbours.iterator(); i.hasNext();) {
		    int neighbour = i.next();
		    if (! path.contains(neighbour)) {
		    	path = depthFirstSearch(neighbour, path);
		    }
		}
		return path;		
	}
	
	// Get the path visited by DFS through this graph starting from startNode
	ArrayList<Integer> depthFirstSearch(int startNode){
		ArrayList<Integer> path = new ArrayList<Integer>();
		path = depthFirstSearch(startNode, path);
		return path;
	}
	
	// Get the path visited by BFS through this graph starting from startNode
	ArrayList<Integer> breadthFirstSearch(int startNode){
		ArrayList<Integer> path = new ArrayList<Integer>();
		Queue<Integer> queue = new LinkedList<Integer>();
		path = breadthFirstSearch(startNode, path, queue);
		return path;
	}
	
	// Get the path visited by BFS through this graph starting from startNode
	// given the path that's already been followed so far
	ArrayList<Integer> breadthFirstSearch(int startNode, ArrayList<Integer> path, Queue<Integer> queue){
		path.add(startNode);
		queue.add(startNode);
		int currentNode;
		ArrayList<Integer> neighbours;
		while (! queue.isEmpty()) {
			currentNode = queue.remove();
			neighbours = getNeighbours(currentNode);
			for (Iterator<Integer> i = neighbours.iterator(); i.hasNext();) {
			    int neighbour = i.next();
			    if (! path.contains(neighbour)) {
			    	path.add(neighbour);
			    	queue.add(neighbour);
			    }
			}
		}
		return path;
	}
	
	// Use BFS with a priority queue ordered on distance from start node to
	// that node to get the shortest path between nodes from and to
	ArrayList<Integer> shortestPath(int from, int to){
		// Used to implement the priority queue of nodes (ordered on 
		// distance to that node). Each node is stored in the queue
		// as a pair (node number, known distance to this node)
		Comparator<int[]> pairComparator = new Comparator<int[]>() {
			public int compare(int[] l, int[] r) {
				return l[1] - r[1];
			}
		};
		
		PriorityQueue<int[]> pq = new PriorityQueue<int[]>(1, pairComparator);
		ArrayList<Integer> path = new ArrayList<Integer>();
		
		// Holds the node before this one on the shortest known path to this one
		int[] prev = new int[nodes.size()];
		int[] dist = new int[nodes.size()];
		int currentNode;
		dist[from] = 0;
		
		// Initially set the distance to all nodes (except from) to be infinite
		for (Iterator<Integer> i = nodes.iterator(); i.hasNext();) {
			int node = i.next();
			if (node != from){
				dist[node] = Integer.MAX_VALUE;
			}
			prev[node] = -1;
			pq.add(new int[]{node,dist[node]});
		}
		
		ArrayList<Integer> neighbours = new ArrayList<Integer>();
		int altDist;
		while (! pq.isEmpty()){
			// Get the node with the shortest known distance from start node 
			currentNode = pq.poll()[0];
			neighbours = getNeighbours(currentNode);
			
			for (Iterator<Integer> i = neighbours.iterator(); i.hasNext();) {
				int neighbour = i.next();
				altDist = dist[currentNode] + getWeight(currentNode, neighbour);
				// If the new path to this node is shorter than the path already
				// found to this node, then update the known path to this node
				if (altDist < dist[neighbour]) {
					pq.remove(new int[]{neighbour,dist[neighbour]});
					pq.add(new int[]{neighbour,altDist});
					dist[neighbour] = altDist;
					prev[neighbour] = currentNode;
				}
			}
		}
		
		// Now follow the chain of previous nodes from the final node back to
		// the start node to get the path from final node to start node
		currentNode = to;
		while (currentNode != from) {
			path.add(currentNode);
			currentNode = prev[currentNode];
		}
		path.add(from);
		
		// The path so far is in the wrong order (final to start) so reverse it
		Collections.reverse(path);
		return path;
	}
	
	// Create a graph with no nodes (hence no edges) in it
	Graph() {
		highestNode = -1;
		nodes = new ArrayList<Integer>();
		adjacencyList = new ArrayList<ArrayList<Integer>>();
		weights = new ArrayList<ArrayList<Integer>>();
	}
	
}
import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.Arrays;
import org.junit.Before;
import org.junit.Test;


public class GraphTest {
	Graph g;
	
	// Every test involves creating a graph
	@Before public void initialise(){
		g = new Graph();
	}	

	@Test
	public void testGraph() {
		assertNotNull(g.adjacencyList);
		assertNotNull(g.weights);
		assertNotNull(g.highestNode);
	}
	
	@Test
	public void testAddNode() {
		g.addNode();
		g.addNode();
		assertNotNull(g.adjacencyList.get(0));
		assertNotNull(g.adjacencyList.get(1));
	}
	
	@Test
	public void testAddNodes() {
		g.addNodes(2);
		assertNotNull(g.adjacencyList.get(0));
		assertNotNull(g.adjacencyList.get(1));
		assertEquals(g.adjacencyList.size(),2);
	}

	@Test
	public void testAddEdge() {
		g.addNodes(2);
		g.addEdge(0, 1, 5);
		assertEquals(g.getWeight(0, 1),5);
		assertEquals(g.getWeight(1, 0),5);
	}

	@Test
	public void testGetNeighbours() {
		g.addNodes(2);
		g.addEdge(0, 1, 5);
		ArrayList<Integer> neighbours0 = new ArrayList<Integer>();
		neighbours0.add(1);
		assertEquals(g.getNeighbours(0), neighbours0);
		ArrayList<Integer> neighbours1 = new ArrayList<Integer>();
		neighbours1.add(0);
		assertEquals(g.getNeighbours(1), neighbours1);		
	}
	
	@Test
	public void testDepthFirstSearch(){
		g.addNodes(8);
		g.addEdge(0, 1, 0);
		g.addEdge(1, 2, 0);
		g.addEdge(1, 3, 0);
		g.addEdge(2, 4, 0);
		g.addEdge(2, 5, 0);
		g.addEdge(4, 6, 0);
		Integer [] path = {1,0,2,4,6,5,3};
		assertEquals(Arrays.asList(path), g.depthFirstSearch(1));
	}
	
	@Test
	public void testBreadthFirstSearch(){
		g.addNodes(10);
		g.addEdge(0,1);
		g.addEdge(2,1);
		g.addEdge(3,1);
		g.addEdge(7,1);
		g.addEdge(7,8);
		g.addEdge(7,9);
		g.addEdge(4,6);
		g.addEdge(2,4);
		g.addEdge(2,5);
		Integer [] path = {0,1,2,3,7,4,5,8,9,6};
		assertEquals(Arrays.asList(path), g.breadthFirstSearch(0));

	}
	
	@Test
	public void testShortestPath(){
		g.addNodes(7);
		g.addEdge(0,1,5);
		g.addEdge(0,4,9);
		g.addEdge(1,2,6);
		g.addEdge(2,4,1);
		g.addEdge(2,5,4);
		g.addEdge(2,6,3);
		g.addEdge(3,5,1);
		g.addEdge(3,6,9);
		g.addEdge(4,6,4);
		g.addEdge(5,6,2);
		Integer [] path = {0,4,2,5,3};
		assertEquals(Arrays.asList(path), g.shortestPath(0,3));
		System.out.println(g.shortestPath(0,3));

	}
}
package agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_RTAStar implements PlanningAgent{
	private ComputedPlan bestPath;
	private boolean planned = false;
	private ArrayList<SearchNode> frontier;
	private GridDomain map;
	private State goalState;
	private HashMap<State, Float> hashTable = null;
	private float alpha;
	private float secondBestCost;

	/* Cost identifiers */
	protected static int G = 0;
	protected static int H = 1;
	protected static int F = 2;

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		this.map = map;
		this.goalState = gState;

		if(hashTable == null) hashTable = new HashMap<State, Float>();

		System.out.println("##############a");
		/* Planning phase */
		SearchNode nextNode = lookahead(sState, 5);

		System.out.println("##############b");
		bestPath = new ComputedPlan();
		bestPath.appendStep(nextNode.getNode());

		System.out.println("##############c");
		/* Return next move to be executed */
		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else return null;
	}

	private SearchNode lookahead(State startState, int depth){
		SearchNode startNode = new SearchNode(startState);

		frontier = new ArrayList<SearchNode>();

		/* Expand the frontier */
		System.out.println("##############d");
		alpha = Float.POSITIVE_INFINITY;
		frontier = expandNodes(startNode, depth);

		if(startNode.getParent() != null) frontier.add(startNode.getParent()); // to allow backtracking

		SearchNode bestNode = startNode;
		bestNode = startNode;
		SearchNode secondBestNode = startNode;

		System.out.println("##############e");
		
		Collections.sort(frontier, new NodeComparator());
		if(frontier.size() >= 1) bestNode = frontier.remove(0);

		/* For second best node */
		frontier = expandNodes(startNode, 1);
		Collections.sort(frontier, new NodeComparator());
		if(frontier.size() >= 1) secondBestNode = frontier.remove(0);
		if(secondBestNode.getNode().equals(bestNode.getNode()) && frontier.size() >= 2) secondBestNode = frontier.remove(0);

		System.out.println("##############f");
		/* Get the next move */
		while(bestNode != null && bestNode.getParent() != null && !bestNode.getParent().equals(startNode)){
			bestNode = bestNode.getParent();
		}
		/* Get the second best move */
		while(secondBestNode != null && secondBestNode.getParent() != null && !secondBestNode.getParent().equals(startNode)){
			secondBestNode = secondBestNode.getParent();
		}
		hashTable.put(startState, map.cost(startState, secondBestNode.getNode()) + secondBestNode.get(H));

		System.out.println("##############g");
		return bestNode;
	}

	private ArrayList<SearchNode> expandNodes(SearchNode currentNode, int depthToFrontier){
		ArrayList<SearchNode> frontier = new ArrayList<SearchNode>();

		/* Expand the node */
		System.out.println("##############h");
		ArrayList<State> children = map.getSuccessors(currentNode.getNode());

		for(State child:children){
			SearchNode childNode = new SearchNode(child);
			childNode.setParent(currentNode);

			/* Any point in considering? */
			if(map.isBlocked(childNode.getNode())) continue; // blocked, so no point

			/* Compute costs */
			float h;
			if(hashTable.containsKey(childNode.getNode())) h = hashTable.get(childNode.getNode()); // if in hash table, get that
			else h = map.hCost(childNode.getNode(), goalState); // else compute fresh h
			System.out.println("##############i " + h);

			if(h > alpha) continue; // alpha pruning

			float g = map.cost(currentNode.getNode(), childNode.getNode()); // compute g = current edge cost only
			System.out.println("##############j " + g);

			float f = g + h;
			if(f == Float.POSITIVE_INFINITY) continue; // ignore dead-ends

			childNode.set(G, g);
			childNode.set(H, h);
			childNode.set(F, f);

			/* Add costs and add to frontier */
			if(depthToFrontier == 1){
				if(h < alpha){
					alpha = h;
				}

				frontier.add(childNode); // add to frontier
			}else frontier.addAll(expandNodes(childNode, depthToFrontier - 1)); // go one level down
		}

		return frontier;
	}

	// You can if you want implement the methods below

	// Do we want to show extra info? (e.g., close and open nodes, current path)
	@Override
	public Boolean showInfo(){
		return true;
	}

	@Override
	public ArrayList<GridCell> expandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		if(frontier==null) return cells;
		for(SearchNode node: frontier)
			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
//		if(hashTable==null) return cells;
//		for(SearchNode node: hashTable.keySet())
//			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ComputedPlan getPath(){
		return bestPath;
	}
}

class NodeComparator implements Comparator<SearchNode>{
	public int compare(SearchNode me, SearchNode them){
		float myH = me.get(MyCoolAgent_RTAStar.H);
		float theirH = them.get(MyCoolAgent_RTAStar.H);
		return Float.compare(myH, theirH);
	}
}

package agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_VanillaAStar implements PlanningAgent{
	private ComputedPlan bestPath;
	private boolean planned = false;
	private ArrayList<SearchNode> openList = new ArrayList<SearchNode>();
	private ArrayList<SearchNode> closedList = new ArrayList<SearchNode>();
	private GridDomain map;
	private State goalState;
	private float w = 0.5f; // weight on heuristic cost
	private float bestCostFound = Float.POSITIVE_INFINITY;
	private long duration; // of running one round of A*
	private int iteration = 1;
	
	/* Cost identifiers */
	protected static int G = 0;
	protected static int H = 1;
	protected static int F = 2;

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		this.map = map;

		if(planned && (!goalState.equals(gState))){// || !bestPath.contains(sState))){
			planned = false;
			bestPath = null;
		}

		if(planned && timeLeft > duration * 2.5){
			planned = false;
		}

		if(!planned){
			planned = true;
			goalState = gState;
			long startTime = System.currentTimeMillis();
			w = timeLeft;
			bestPath = aStar(sState, System.currentTimeMillis() + timeLeft);
			long endTime = System.currentTimeMillis();
			duration = endTime - startTime;
System.out.println("#### time taken: " + duration);
			iteration++;
		}

		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else return null;
	}

	private ComputedPlan aStar(State startState, long deadline){
		/* Initialize node lists */
		ComputedPlan path = new ComputedPlan();

		SearchNode goalNode = null;

		/* Add the start node into open list */
		SearchNode startNode = new SearchNode(startState);
		startNode.setParent(null);
		openList.add(startNode);

		/* Iterate over each node from openList */
		while(!openList.isEmpty() && System.currentTimeMillis() < deadline){
			SearchNode consideredNode = openList.remove(0);

			if(closedList.contains(consideredNode)) continue; // already visited
			if(map.isBlocked(consideredNode.getNode())) continue; // blocked, so no point

			if(consideredNode.getNode().equals(goalState) && consideredNode.get(F) < bestCostFound){
				goalNode = consideredNode;
				bestCostFound = goalNode.get(F);
				continue; // goal reached?
			}

			/* Expand */

			if(consideredNode.get(F) < bestCostFound){ // only if it is a cheaper route
				ArrayList<State> children = map.getSuccessors(consideredNode.getNode());
				for(State child:children){
					SearchNode childNode = new SearchNode(child);

					if(closedList.contains(childNode)) continue; // already visited
					if(map.isBlocked(child)) continue; // blocked, so no point

					float g = map.cost(consideredNode.getNode(), child) + consideredNode.get(G); // compute g = current edge cost +
																									// previously
																									// recorded cost to get to parent
					float h = map.hCost(child, goalState); // compute h
					float f = g + w * h;

					if(f == Float.POSITIVE_INFINITY) continue;

					childNode.set(F, f); // set f

					childNode.setParent(consideredNode); // to track the path of reaching the node
					openList.add(childNode); // add to open list
				}
			}
			/* Mark as visited */
			closedList.add(consideredNode);
			/* Sort by f */
			Collections.sort(openList, new CostComparator());
		}

		while(goalNode != null){
			path.prependStep(goalNode.getNode());
			goalNode = goalNode.getParent();
		}

		return path;
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
		if(closedList == null) return cells;
		for(SearchNode node:closedList)
			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		if(openList == null) return cells;
		for(SearchNode node:openList)
			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ComputedPlan getPath(){
		return bestPath;
	}

}

class CostComparator implements Comparator<SearchNode>{
	public int compare(SearchNode me, SearchNode them){
		float myF = me.get(MyCoolAgent_VanillaAStar.F);
		float theirF = them.get(MyCoolAgent_VanillaAStar.F);
		return Float.compare(myF, theirF);
	}
}

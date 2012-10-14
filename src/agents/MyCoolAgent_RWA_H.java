package agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_RWA_H implements PlanningAgent{
	private ComputedPlan bestPath = new ComputedPlan();
	private boolean planned = false;
	private LinkedList<SearchNode> openList = new LinkedList<SearchNode>();
	private LinkedList<SearchNode> closedList = new LinkedList<SearchNode>();
	private GridDomain map;
	private State goalState;
	private float w0 = 5;
	private float wInc = 0.5f;
	private float bestCostFound = Float.POSITIVE_INFINITY;
	private boolean firstRun = true;

	/* Cost identifiers */
	protected static int G = 0;
	protected static int H = 1;
	protected static int F = 2;
	protected static int seen = 3;

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		this.map = map;

		if(planned && (!goalState.equals(gState) || !bestPath.contains(sState))){
			planned = false;
			bestPath = new ComputedPlan();
			openList = new LinkedList<SearchNode>();
			closedList = new LinkedList<SearchNode>();
		}

		if(!planned){
			goalState = gState;
			rwa(sState, w0, wInc, System.currentTimeMillis() + (long)(timeLeft * 0.75));
			planned = true;
		}

		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else return null;
	}

	private void rwa(State startState, float weight, float increment, long deadline){
		float bound = Float.POSITIVE_INFINITY;

		SearchNode startNode = new SearchNode(startState);
		startNode.set(G, 0);
		startNode.set(F, 0 + map.hCost(startState, goalState));
		startNode.setParent(null);
		boolean goalFound = false;
		SearchNode goalableNode = null;

		openList.add(startNode);
		while(!openList.isEmpty() && System.currentTimeMillis() < deadline){
			/* Get smallest costing node */
			// Collections.sort(openList, new FComparer());
			float minF = Float.POSITIVE_INFINITY;
			int nodeIndex = 0;
			for(int i = 0; i < openList.size(); i++){
				float f = openList.get(i).get(F);
				if(f < minF){
					minF = f;
					nodeIndex = i;
				}
			}

			SearchNode s = openList.remove(nodeIndex);

			ArrayList<State> children = map.getSuccessors(s.getNode());
			for(State child:children){
				float currH = map.hCost(child, goalState); // + s.get(G);
				if(currH >= bound) continue;

				SearchNode childNode = new SearchNode(child);
				if(!openList.contains(childNode) && !closedList.contains(childNode)){
					childNode.set(H, currH);
					childNode.setParent(s);
					float g = map.cost(s.getNode(), child) + s.get(G);
					float f = currH * weight + g;
					childNode.set(G, g);
					childNode.set(F, f);
					openList.add(childNode);
				}else if(openList.contains(childNode) && childNode.get(seen) == 1){
					if(currH < childNode.get(H)){

						childNode.set(H, currH);
						childNode.setParent(s);
						childNode.set(seen, 0);
						openList.add(childNode);
					}
				}else if(currH < childNode.get(H)){
					childNode.set(H, currH);
					childNode.setParent(s);
					if(openList.contains(childNode)) openList.add(childNode);
					else{
						openList.add(childNode);
						closedList.remove(childNode);
					}
				}

				if(child.equals(goalState)){
					goalFound = true;
					goalableNode = childNode;
					break;
				}

				s.set(seen, 1);
				closedList.add(s);
			}

			if(goalFound){
				bound = goalableNode.get(H);
				weight = Math.max(1, weight * increment);

				SearchNode node;
				for(int i = 0; i < openList.size(); i++){
					node = openList.get(i);
					node.set(seen, 1);
					openList.set(i, node);
				}

				for(int i = 0; i < closedList.size(); i++){
					node = closedList.get(i);
					node.set(seen, 1);
					closedList.set(i, node);
				}
				// break;
			}
		}

		while(goalableNode != null){
			bestPath.prependStep(goalableNode.getNode());
			goalableNode = goalableNode.getParent();
		}
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

class FComparer2 implements Comparator<SearchNode>{
	public int compare(SearchNode me, SearchNode them){
		float myF = me.get(MyCoolAgent_RWA_H.F);
		float theirF = them.get(MyCoolAgent_RWA_H.F);
		return Float.compare(myF, theirF);
	}
}

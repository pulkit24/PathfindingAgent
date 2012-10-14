package agents;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridCoord;
import pplanning.simviewer.model.GridDomain;
import agents.pulkit.Debug;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_DeadlineAware implements PlanningAgent{
	private FSorter fsorter = new FSorter();
	private PriorityQueue<SearchNode> openList = new PriorityQueue<SearchNode>(100, fsorter);
	private PriorityQueue<SearchNode> prunedList = new PriorityQueue<SearchNode>(100, fsorter);
	private int eCurr = 0;
	private ArrayList<Integer> deltaE = new ArrayList<Integer>();
	private int windowSize = 5;
	private long tCurr = 0;
	private ArrayList<Long> deltaT = new ArrayList<Long>();
	private GridDomain map;
	private boolean planned = false;
	private ComputedPlan bestPath = null;
	private State goalState = null;
	private int settlingTime = 10;
	private boolean connect4 = false;
	private boolean goalFound = false;
	private SearchNode backupBest = null;

	/* Cost identifiers */
	protected static int G = 0;
	protected static int H = 1;
	protected static int F = 2;
	protected static int E = 3;

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		this.map = map;
		Debug.MODE = true;

		if(planned && (!goalState.equals(gState) || !bestPath.contains(sState) || (!goalFound && sState.equals(backupBest.getNode())))){
			planned = false;
			bestPath = null;
		}

		if(!planned){
			goalState = gState;

			openList = new PriorityQueue<SearchNode>(100, fsorter);
			prunedList = new PriorityQueue<SearchNode>(100, fsorter);
			eCurr = 0;
			deltaE = new ArrayList<Integer>();
			tCurr = 0;
			deltaT = new ArrayList<Long>();
			settlingTime = 10;
			connect4 = false;
			goalFound = false;
			backupBest = null;

			bestPath = deadlineAwareSearch(sState, System.currentTimeMillis() + timeLeft * 0.95);
			planned = true;
		}

		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else return null;
	}

	private ComputedPlan deadlineAwareSearch(State startState, double deadline){
		SearchNode startNode = new SearchNode(startState);
		startNode.set(G, 0);
		float hStart = map.hCost(startState, goalState);
		startNode.set(H, hStart);
		startNode.set(F, 0 + hStart);
		startNode.set(E, 0); // extra
		eCurr = 0; // extra

		/* 1 */
		openList.add(startNode);

		/* 2 */

		/* 3 */
		ComputedPlan path = new ComputedPlan();
		SearchNode incumbent = null;

		/* 4 */
		while(System.currentTimeMillis() - deadline < 0){
			/* 5 */
			if(!openList.isEmpty()){
				/* 6 */
				float dmax = calculateDBound(deadline);

				/* 7 */
				SearchNode s = openList.remove();

				/* 8 */
				if(s.getNode().equals(goalState) && (incumbent == null || fsorter.compare(s, incumbent) < 0)){
					/* 9 */
					incumbent = s;
					goalFound = true; // extra
				}
				/* 10 */
				else if(dCheapest(s.getNode(), goalState) < dmax){
					/* 11 */
					ArrayList<State> children = map.getSuccessors(s.getNode());

					/* Hack for grid connectivity */
					if(!connect4 && children.size() == 4) connect4 = true;

					eCurr++; // extra
					if(deltaE.size() == windowSize) deltaE.remove(0); // extra
					deltaE.add(eCurr - (int)s.get(E)); // extra
					if(deltaT.size() == windowSize) deltaT.remove(0); // extra
					deltaT.add(System.currentTimeMillis() - tCurr); // extra
					tCurr = System.currentTimeMillis(); // extra

					for(State child:children){
						/* 12 */
						SearchNode childNode = new SearchNode(child);
						childNode.set(E, eCurr); // extra
						childNode.setParent(s); // extra
						float gChild = map.cost(s.getNode(), child) + s.get(G);
						float hChild = map.hCost(child, goalState);
						childNode.set(G, gChild);
						childNode.set(H, hChild);
						childNode.set(F, gChild + hChild);
						openList.add(childNode);
					}
				}
				/* 13 */
				else{
					/* 14 */
					prunedList.add(s);
				}
			}
			/* 15 */
			else{ // openList is empty
				/* 16 */
				recoverPrunedStates(deadline);
			}
		}
		/* 17 */
		while(incumbent != null){
			path.prependStep(incumbent.getNode());
			incumbent = incumbent.getParent();
		}
		path.prependStep(startState);

		return path;
	}

	private float calculateDBound(double deadline){
		if(settlingTime > 0){
			settlingTime--;
			return Float.POSITIVE_INFINITY;
		}else return ((float)(deadline - System.currentTimeMillis())) / (computeRInverse() * computeAverageDeltaE());
	}

	private float computeRInverse(){
		/* Compute deltaT average */
		float averageDeltaT = 0; // extra
		for(long entry:deltaT)
			averageDeltaT += entry; // extra
		averageDeltaT /= windowSize; // extra
		return averageDeltaT;
	}

	private float computeAverageDeltaE(){
		/* Compute deltaE average */
		float averageDeltaE = 0; // extra
		for(int entry:deltaE)
			averageDeltaE += entry; // extra
		averageDeltaE /= windowSize; // extra
		return averageDeltaE;
	}

	private void recoverPrunedStates(double deadline){
		/* 18 */
		double exp = (deadline - System.currentTimeMillis()) / computeRInverse(); // = estimatedExpansionsRemaining;
		/* 19 */
		while(exp > 0 && !prunedList.isEmpty()){
			/* 20 */
			// Collections.sort(prunedList, fsorter);
			SearchNode s = prunedList.remove();
			openList.add(s);
			exp -= dCheapest(s.getNode(), goalState);
		}
	}

	private float dCheapest(State currentState, State goalState){
		float straightLineDistance = 0;
		GridCoord from = ((GridCell)currentState).getCoord();
		GridCoord to = ((GridCell)goalState).getCoord();
		if(connect4) straightLineDistance = from.getManhattenDistance(to);
		else straightLineDistance = from.getEuclideanDistance(to);

		return straightLineDistance;
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
		if(prunedList == null) return cells;
		for(SearchNode node:prunedList)
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

class FSorter implements Comparator<SearchNode>{
	public int compare(SearchNode me, SearchNode them){
		float myCost = me.get(MyCoolAgent_DeadlineAware.F);
		float theirCost = them.get(MyCoolAgent_DeadlineAware.F);
		return Float.compare(myCost, theirCost);
	}
}

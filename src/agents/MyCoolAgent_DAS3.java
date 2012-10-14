package agents;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import com.sun.net.httpserver.Authenticator.Success;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;
import agents.pulkit.Debug;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_DAS3 implements PlanningAgent{
	private ComputedPlan bestPath = new ComputedPlan();
	private PriorityQueue<SearchNode> openList = new PriorityQueue<SearchNode>(100, new OpenListComparator());
	private PriorityQueue<SearchNode> prunedList = new PriorityQueue<SearchNode>(100, new OpenListComparator());
	private PriorityQueue<SearchNode> closedList = new PriorityQueue<SearchNode>(100, new ClosedListComparator());
	// private ArrayList<SearchNode> closedList = new ArrayList<SearchNode>();
	private GridDomain map;
	private boolean planned = false;
	private State goalState = null;
	private boolean firstRun = true;
	private long moveTimeReserved = 20; // in ms - time reserved for moving
	private float startHeuristic = 0;

	/* Weight parameters */
	private float weightG = 1.0f;
	private float weightH = 1.0f;

	/* Cost identifiers */
	protected static int G = 0;
	protected static int H = 1;
	protected static int F = 2;

	/* DAS-specific parameters */
	private int eCurr = 0;
	private long tPreExpansion = 0;
	private int windowSize = 100;
	private int settlingTime = 0;
	private int settlingTimeLeft;
	private int[] eDelayWindow = new int[windowSize];
	private int eDelayWindowCurrPosition = 0;
	private long[] tDelayWindow = new long[windowSize];
	private int tDelayWindowCurrPosition = 0;
	protected static int E = 3; // expansion number
	protected static long T = 4; // time at expansion

	private SearchNode incumbent = null;

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		Debug.MODE = false;
		Debug.allowOnly = "weight, goal";
		this.map = map;

		if(planned && (!goalState.equals(gState) || map.getChangedEdges().size() > 0 || !bestPath.contains(sState))){
			planned = false;
			firstRun = true;
		}

		if(!planned && firstRun){
			firstRun = false;
			goalState = gState;
			settlingTimeLeft = settlingTime;

			bestPath = new ComputedPlan();
			OpenListComparator fSorter = new OpenListComparator();
			ClosedListComparator hSorter = new ClosedListComparator();
			openList = new PriorityQueue<SearchNode>(100, fSorter);
			prunedList = new PriorityQueue<SearchNode>(100, fSorter);
			closedList = new PriorityQueue<SearchNode>(100, hSorter);
			// closedList = new ArrayList<SearchNode>();

			for(int i = 0; i < windowSize; i++)
				eDelayWindow[i] = 0;
			for(int i = 0; i < windowSize; i++)
				tDelayWindow[i] = 0;

			/* Start Node */
			SearchNode startNode = new SearchNode(sState);
			float g = 0; // Float.POSITIVE_INFINITY;
			startNode.set(G, g);
			startHeuristic = map.hCost(sState, gState);// * weight;
			startNode.set(H, startHeuristic);
			startNode.set(F, g + startHeuristic);
			startNode.setParent(null);
			openList.add(startNode);

			incumbent = null; // temp
		}

		if(!planned && !firstRun){
			if(timeLeft > moveTimeReserved){
				SearchNode goalNodePossible = das(System.currentTimeMillis() + timeLeft - moveTimeReserved);

				if(goalNodePossible != null){
					Debug.log("Agent", "node returned " + goalNodePossible.getNode());
					/* Backtrack from this possible goal node */
				}else{
					Debug.log("Agent", "null incumbent");
					// SearchNode bestNodeInOpenList = openList.peek(); // closed to goal node
					SearchNode bestNodeInClosedList = closedList.peek(); // closed to goal node
					// if(bestNodeInOpenList == null && bestNodeInClosedList == null){
					// goalNodePossible = null;
					// Debug.log("Agent", "woe! no list has nodes!");
					// }else if(bestNodeInOpenList == null || bestNodeInClosedList.get(F) < bestNodeInOpenList.get(F)){
					goalNodePossible = bestNodeInClosedList;
					Debug.log("Agent", "closed list has closest node " + goalNodePossible.getNode());
					// }else{
					// goalNodePossible = bestNodeInOpenList;
					// Debug.log("Agent", "open list has closest node " + goalNodePossible.getNode());
					// }
				}

				/* Construct path from goal node */
				while(goalNodePossible != null){
					Debug.log("Agent", "prepending in path state " + goalNodePossible.getNode().toString());
					bestPath.prependStep(goalNodePossible.getNode());
					goalNodePossible = goalNodePossible.getParent();
				}
				bestPath.prependStep(sState);
			}else{
				/* Construct greedy path from current best node known */
				Debug.log("Agent", "No time left, skipping...");
			}

			planned = true;
		}

		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else return null;
	}

	private SearchNode das(double deadline){
		Debug.log("DAS", "deadline", deadline + "");

		while(deadline - System.currentTimeMillis() > 0){

			Debug.log("DAS", "Iteration started");

			if(!openList.isEmpty()){
				/* How many levels ahead can we go at this rate? */
				double dmax = calculateDBound(deadline);
				Debug.log("DAS", "dmax", dmax + "");

				/* Get the next node from the open list */
				SearchNode currentNode = openList.poll();
				Debug.log("DAS", "now considering node " + currentNode.getNode().toString());
				closedList.add(currentNode); // add it to visited

				/* Is it a goal (and cheaper than incumbent)? */
				if(currentNode.getNode().equals(goalState) && (incumbent == null || currentNode.get(F) < incumbent.get(F))){
					Debug.log("DAS", "goal", "this state is a good goal state! Incumbent set.");
					incumbent = currentNode;

					/* Reset weights for next run */
					weightH = 1f;
					weightG = 1f;

				}else if(currentNode.get(H) < dmax){
					Debug.log("DAS", "dCheapest", currentNode.get(H) + " seems smaller than dmax");

					/* Expand children */
					ArrayList<State> children = map.getSuccessors(currentNode.getNode());
					eCurr++; // increment the expansion count

					/* Calculate expansion delay */
					Debug.log("DAS", "eDelay", "current position: " + eDelayWindowCurrPosition);
					eDelayWindow[eDelayWindowCurrPosition++] = eCurr - (int)currentNode.get(E);
					Debug.log("DAS", "eDelay", "current expansion " + eCurr + ", with delay " + eDelayWindow[eDelayWindowCurrPosition - 1]);
					eDelayWindowCurrPosition %= windowSize;

					tPreExpansion = System.nanoTime(); // start tracking the time for expansion

					float gParent = currentNode.get(G) * weightG;
					if(gParent == Float.POSITIVE_INFINITY) gParent = 0;

					boolean firstChild = true;
					Debug.log("DAS", "children", "expanding parent\n" + currentNode.getNode().toString());
					for(State child:children){
						// State child;
						// SuccessorIterator i = map.getNextSuccessor(currentNode.getNode());
						// while((child = i.next()) != null){
						SearchNode childNode = new SearchNode(child);
						if(closedList.contains(childNode) || openList.contains(childNode)){
							Debug.log("DAS", "children", "this child already visited/queued in open list " + child.toString());
							continue; // skip if already visited/added to open list
						}
						Debug.log("DAS", "children", "found an unvisited child\n" + child.toString() + ". Generating...");

						/* Compute costs for each child */
						float g = map.cost(currentNode.getNode(), child) + gParent;
						Debug.log("DAS", "children", "this child has g " + g);
						childNode.set(G, g);
						// float h = ((GridCell)child).getCoord().getManhattenDistance(((GridCell)goalState).getCoord());
						float h = map.hCost(child, goalState);// * weightH;
						childNode.set(H, h);
						Debug.log("DAS", "children", "this child has h " + h);
						float f = g + h * weightH;
						childNode.set(F, f);
						Debug.log("DAS", "children", "this child has total cost: f = " + f);
						childNode.set(E, eCurr);
						childNode.setParent(currentNode);
						openList.add(childNode);

						/* Update parent's H to better estimate using the child's F */
						if(firstChild){
							currentNode.set(H, f);
							currentNode.set(F, currentNode.get(G) + f);
							firstChild = false;
						}else if(currentNode.get(H) > f){
							currentNode.set(H, f);
							currentNode.set(F, currentNode.get(G) + f);
						}
					}

					/* Calculate time delay */
					tDelayWindow[tDelayWindowCurrPosition++] = System.nanoTime() - tPreExpansion;
					Debug.log("DAS", "tDelay", "time delay " + tDelayWindow[tDelayWindowCurrPosition - 1]);
					tDelayWindowCurrPosition %= windowSize;

					// closedList.add(currentNode); // add it to visited
				}else{
					Debug.log("DAS", "nope, pruned");
					prunedList.add(currentNode);
					// closedList.add(currentNode); // add it to visited
				}

			}else{
				/* Recover pruned list */
				Debug.log("DAS", "open list is now empty, depruning...");
				// System.exit(0);
				deprune(deadline);
			}

			Debug.log("DAS", "loop end, current time is " + System.nanoTime());
		}

		return incumbent;
	}

	private double calculateDBound(double deadline){
		/* Used to calculate dmax */
		if(settlingTimeLeft > 0){
			settlingTimeLeft--;
			return Double.POSITIVE_INFINITY;
		}else{
			double eDelaySum = 0, tDelaySum = 0;
			for(int i = 0; i < windowSize; i++){
				eDelaySum += eDelayWindow[i];
				tDelaySum += tDelayWindow[i];
			}
			tDelaySum /= 1000000; // convert to ms
			double exp = ((deadline - System.currentTimeMillis()) / tDelaySum) * windowSize;
			return (exp / eDelaySum) * windowSize;
		}
	}

	private void deprune(double deadline){
		/* Used to recover pruned nodes */
		double exp;
		if(settlingTimeLeft > 0){
			exp = startHeuristic;
		}else{
			double tDelaySum = 0;
			for(int i = 0; i < windowSize; i++){
				tDelaySum += tDelayWindow[i];
			}
			tDelaySum /= 1000000; // convert to ms
			double timeLeft = deadline - System.currentTimeMillis();
			exp = (timeLeft / tDelaySum) * windowSize;
		}

		/* Improve weight - inc. H and dec. G over time */
		if(exp != 0) weightH = (float)(startHeuristic / exp);
		if(weightH < 1) weightH = 1;
		Debug.log("DAS", "weight", "\tweightH " + weightH);

		weightG = (float)(1 - exp / startHeuristic);
		if(weightG <= 0.1f) weightG = 0.1f;
		if(weightG > 1) weightG = 1;
		Debug.log("DAS", "weight", "weightG " + weightG);

		while(exp > 0 && !prunedList.isEmpty()){
			SearchNode s = prunedList.poll();
			closedList.remove(s); // allow re-visit
			// float g = s.get(G);
			// float h = s.get(H) * weightH;
			// s.set(H, h);
			// s.set(F, g + h);

			s.set(E, eCurr); // reset expansion counter

			openList.add(s);
			exp -= s.get(H);
		}
		/* Reset delay window and settling time */
		eDelayWindow = new int[windowSize];
		for(int i = 0; i < windowSize; i++)
			eDelayWindow[i] = 0;
		settlingTimeLeft = settlingTime;
	}

	// You can if you want implement the methods below

	// Do we want to show extra info? (e.g., close and open nodes, current path)
	@Override
	public Boolean showInfo(){
		return planned;
	}

	@Override
	public ArrayList<GridCell> expandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		for(SearchNode node:closedList)
			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		for(SearchNode node:openList)
			cells.add((GridCell)node.getNode());
		for(SearchNode node:prunedList)
			cells.add((GridCell)node.getNode());
		return cells;
	}

	@Override
	public ComputedPlan getPath(){
		return bestPath;
	}

}

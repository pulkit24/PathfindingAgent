package agents;

import java.util.ArrayList;
import java.util.PriorityQueue;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;
import agents.pulkit.Debug;
import agents.pulkit.GridCellInformed;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;

public class MyCoolAgent_DAS4 implements PlanningAgent{
	/* Node lists */
	private PriorityQueue<GridCell> openList;
	private PriorityQueue<GridCell> prunedList;
	private GridCell[][] closedList;
	private double[][] g;
	private double[][] h;
	private double[][] f;
	private double[][] e;

	/* Solution stores */
	private ComputedPlan bestPath;
	private GridCellInformed incumbent = null;

	/* Map parameters */
	private GridDomain map;
	private GridCell goalState = null;
	private float startHeuristic = 0;

	/* Agent state parameters */
	private boolean planned = false;
	private boolean firstRun = true;

	/* Agent configuration */
	private long moveTimeReserved = 0; // in ms - time reserved for moving
	private float weightG = 1.0f; // weight given to g
	private float weightH = 1.0f; // weight given to h

	/* DAS-specific parameters */
	private int windowSize = 100;
	private int settlingTime = 100;
	private int settlingTimeLeft;
	/* Expansion counters */
	private int eCurr = 0;
	private int[] eDelayWindow = new int[windowSize];
	private int eDelayWindowCurrPosition = 0;
	/* Time counters */
	private long tPreExpansion = 0;
	private long[] tDelayWindow = new long[windowSize];
	private int tDelayWindowCurrPosition = 0;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		/* Debug configuration */
		Debug.MODE = true;
		Debug.allowOnly = "goal, weight";

		/* Store latest map */
		this.map = map;

		/* Do we need to replan? - we haven't done so, goal has moved, map has changed, or agent has been teleported */
		if(planned && (!goalState.equals(gState) || map.getChangedEdges().size() > 0 || !bestPath.contains(sState))){
			planned = false;
			firstRun = true;
		}

		/* Reset all parameters */
		if(!planned && firstRun){

			/* Node lists */
			openList = new PriorityQueue<GridCellInformed>(100);
			prunedList = new PriorityQueue<GridCellInformed>(100);// , new Sorter(Sorter.F));
			closedList = new GridCell[map.getWidth()][map.getHeight()];
			for(int i = 0; i < map.getHeight(); i++){
				for(int j = 0; j < map.getWidth(); j++)
					closedList[i][j] = null; // set closedList to null initially
			}
			/* Solution stores */
			bestPath = new ComputedPlan();
			incumbent = null;
			/* Map parameters */
			goalState = gState;
			startHeuristic = map.hCost(sState, gState);
			/* Agent state parameters */
			firstRun = false;
			/* Agent configuration */
			/* DAS-specific parameters */
			settlingTimeLeft = settlingTime;
			/* Expansion counters */
			for(int i = 0; i < windowSize; i++)
				eDelayWindow[i] = 0;
			/* Time counters */
			for(int i = 0; i < windowSize; i++)
				tDelayWindow[i] = 0;

			/* Initialize start node */
			GridCellInformed startCell = new GridCellInformed(sState, 0, startHeuristic, 0, weightG, weightH);
			openList.offer(startCell);
		}

		/* Now on to make a plan */
		if(!planned && !firstRun){
			if(timeLeft > moveTimeReserved){
				/* Execute DAS */
				GridCellInformed goalNodePossible = das(System.currentTimeMillis() + timeLeft - moveTimeReserved);

				if(goalNodePossible != null){
					Debug.log("Agent", "node returned " + goalNodePossible);
					/* Backtrack from this possible goal node */
				}else{
					Debug.log("Agent", "null incumbent");
					GridCellInformed bestNodeInPrunedList = prunedList.peek(); // closed to goal node
					goalNodePossible = bestNodeInPrunedList;
					Debug.log("Agent", "closed list has closest node " + goalNodePossible);
				}

				/* Construct path from goal node */
				while(goalNodePossible != null){
					Debug.log("Agent", "prepending in path state " + goalNodePossible.toString());
					bestPath.prependStep(goalNodePossible.cell);
					goalNodePossible = goalNodePossible.parent;
				}
				bestPath.prependStep(sState);

			}else{
				/* TODO Construct greedy path from current best node known */
				Debug.log("Agent", "No time left, skipping...");
			}

			planned = true;
		}

		/* Do we have a next step in the path? */
		if(bestPath != null && bestPath.getCurrentStepNo() < bestPath.getLength()) return (GridCell)bestPath.getNextStep();
		else{
			/* TODO Try to move blindly to goal */
			return null;
		}
	}

	private GridCellInformed das(double deadline){
		Debug.log("DAS", "deadline", deadline + "");

		/* As long as we have time ... */
		while(deadline - System.currentTimeMillis() > 0){

			Debug.log("DAS", "Iteration started");

			/* While the open list is empty */
			if(!openList.isEmpty()){
				/* How many levels ahead can we go at this rate? */
				double dmax = calculateDBound(deadline);
				Debug.log("DAS", "dmax", dmax + "");

				/* Get the next node from the open list */
				GridCellInformed currentCell = openList.poll();
				Debug.log("DAS", "now considering node " + currentCell.toString());
				markClosed(currentCell); // add it to visited

				/* Is it a goal (and cheaper than incumbent)? */
				if(currentCell.cell.equals(goalState) && (incumbent == null || currentCell.f < incumbent.f)){
					Debug.log("DAS", "goal", "this state is a good goal state! Incumbent set.");
					incumbent = currentCell;

					/* Reset weights for next run */
					weightH = 1f;
					weightG = 1f;

				}else if(dCheapest(currentCell) < dmax){
					Debug.log("DAS", "dCheapest", dCheapest(currentCell) + " seems smaller than dmax");
					Debug.log("DAS", "hCost", currentCell.h + "");

					/* Expand children */
					SuccessorIterator childIterator = map.getNextSuccessor(currentCell.cell);
					eCurr++; // increment the expansion count

					/* Calculate expansion delay */
					Debug.log("DAS", "eDelay", "current position: " + eDelayWindowCurrPosition);
					eDelayWindow[eDelayWindowCurrPosition++] = eCurr - (int)currentCell.e;
					Debug.log("DAS", "eDelay", "current expansion " + eCurr + ", with delay " + eDelayWindow[eDelayWindowCurrPosition - 1]);
					eDelayWindowCurrPosition %= windowSize;

					tPreExpansion = System.nanoTime(); // start tracking the time for expansion

					/* Iterate over each child */
					Debug.log("DAS", "children", "expanding parent\n" + currentCell.toString());
					GridCell child;
					boolean firstChild = true;
					while((child = childIterator.next()) != null){

						/* Have we already seen this node? */
						if(isClosed(child)){
							Debug.log("DAS", "children", "this child already visited/queued in open list " + child.toString());
							continue; // skip if already visited/added to open list
						}
						Debug.log("DAS", "children", "found an unvisited child\n" + child.toString() + ". Generating...");

						/* Add child to open list */
						GridCellInformed childCell = new GridCellInformed(child, currentCell, map.cost(currentCell.cell, child), weightH
								* map.hCost(child, goalState), eCurr, weightG, weightH);
						openList.add(childCell);

						/* Update parent's H to better estimate using the child's F */
//						double f = childCell.f;
//						if(firstChild){
//							currentCell.h = f;
//							currentCell.f = currentCell.g + f;
//							firstChild = false;
//						}else if(currentCell.h > f){
//							currentCell.h = f;
//							currentCell.f = currentCell.g + f;
//						}
					}

					/* Calculate time delay */
					tDelayWindow[tDelayWindowCurrPosition++] = System.nanoTime() - tPreExpansion;
					Debug.log("DAS", "tDelay", "time delay " + tDelayWindow[tDelayWindowCurrPosition - 1]);
					tDelayWindowCurrPosition %= windowSize;

					// closedList.add(currentNode); // add it to visited
				}else{
					/* I don't expect to be able to reach the goal from here */
					Debug.log("DAS", "nope, pruned");
					prunedList.offer(currentCell);
				}

			}else{
				/* Time left but open list is empty - Recover pruned list! */
				Debug.log("DAS", "open list is now empty, depruning...");
				deprune(deadline);
			}

			Debug.log("DAS", "loop end");
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

	private double dCheapest(GridCellInformed cell){
		/* Used to compute dCheapest - for now simply the Manhattan distance */
		return map.hCost(cell.cell, goalState);
//		return cell.cell.getCoord().getManhattenDistance(goalState.getCoord());
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
//		if(exp != 0) weightH = (float)(startHeuristic / exp);
//		if(weightH < 1) weightH = 1;
//		Debug.log("DAS", "weight", "\tweightH " + weightH);
//
//		weightG = (float)(1 - exp / startHeuristic);
//		if(weightG <= 0.1f) weightG = 0.1f;
//		if(weightG > 1) weightG = 1;
//		Debug.log("DAS", "weight", "weightG " + weightG);

		while(exp > 0 && !prunedList.isEmpty()){
			GridCellInformed s = prunedList.poll();
			unmarkClosed(s); // allow re-visit

			s.e = eCurr; // reset expansion counter

			openList.offer(s);
			exp -= dCheapest(s);
		}
		/* Reset delay window and settling time */
		eDelayWindow = new int[windowSize];
		for(int i = 0; i < windowSize; i++)
			eDelayWindow[i] = 0;
		settlingTimeLeft = settlingTime;
	}

	private void markClosed(GridCellInformed cell){
		closedList[cell.cell.getCoord().getX()][cell.cell.getCoord().getY()] = cell.cell;
	}

	private void unmarkClosed(GridCellInformed cell){
		closedList[cell.cell.getCoord().getX()][cell.cell.getCoord().getY()] = null;
	}

	private boolean isClosed(GridCellInformed cell){
		return isClosed(cell.cell);
	}

	private boolean isClosed(GridCell cell){
		return closedList[cell.getCoord().getX()][cell.getCoord().getY()] != null;
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
		for(int i = 0; i < map.getHeight(); i++){
			for(int j = 0; j < map.getWidth(); j++)
				if(closedList[i][j] != null) cells.add(closedList[i][j]);
		}
		return cells;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		if(openList == null) return cells;
		for(GridCellInformed node:openList)
			cells.add(node.cell);
		return cells;
	}

	@Override
	public ComputedPlan getPath(){
		return bestPath;
	}
}
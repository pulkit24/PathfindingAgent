package agents;

import java.util.ArrayList;
import java.util.PriorityQueue;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;
import agents.pulkit.GridCellInformed;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent_DAS2 implements PlanningAgent{
	/* Node lists */
	private PriorityQueue<GridCellInformed> openList;
	private PriorityQueue<GridCellInformed> prunedList;
	private GridCellInformed[][] closedList;

	/* Solution stores */
	private ComputedPlan bestPath;
	private GridCellInformed incumbent = null;
	private boolean goalFound = false;
	private int pathStepNo = 0;

	/* Map parameters */
	private GridDomain map;
	private GridCell goalState = null;
	private float startHeuristic = 0;
	private long startTime = 0;
	private GridCellInformed startCell = null;
	private float minCost = 0;
	private int[] operatorPreference;
	private int connectivity = 4; // manhattan or euclidean, or anything else!

	/* Agent state parameters */
	private boolean planned = false; // track whether a final plan has been prepared
	private boolean firstRun = true; // to run greedy first
	private boolean secondRun = false; // to run DAS next
	private long lastTimeNotice = 0; // to track the last notice of timeleft, and hence if more time has been allotted
	/* Partial replan parameters */
	boolean partialReplan = false;
	State partialStartState = null;

	/* Agent configuration */
	private long moveTimeReserved = 0; // in ms - time reserved for moving
	private float weightG = 1.0f; // weight given to g
	private float weightH = 1.0f; // weight given to h

	/* DAS-specific parameters */
	private int windowSize = 20;
	private int settlingTime = 20;
	private int settlingTimeLeft;
	/* Expansion counters */
	private int eCurr = 0;
	private int[] eDelayWindow = new int[windowSize];
	private int eDelayWindowCurrPosition = 0;
	/* Time counters */
	/* Expansion time counter for DAS */
	private long tPreExpansion = 0;
	private long[] tDelayWindow = new long[windowSize];
	private int tDelayWindowCurrPosition = 0;
	private double averageTimeTaken = 0;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		/* Which run is this? */
		if(!firstRun && !secondRun) secondRun = true;
		/* Some partial replan parameter declarations for later use */

		/* Store latest map */
		this.map = map;

		/* Dynamism - do we need to replan? */
		if(planned){
			/* 1. Goal moved */
			if(!goalState.equals(gState)){
				// /* Complete replan since all our heuristics are waste */
				// planned = false;
				// firstRun = true;
				// secondRun = false;
			}

			/* 2. Map has changed */
			else if(map.getChangedEdges().size() > 0){
				// /* Did anything change on our path? */
				// for(Edge edge:map.getChangedEdges()){
				// /* Iterate over the rest of the path left to find out */
				// for(int i = pathStepNo; i < bestPath.getLength(); i++){
				// State pivotState = bestPath.getStep(i);
				// if(pivotState.equals(edge.getStart()) || pivotState.equals(edge.getEnd())){
				// /* Yes! Set flags to start partial replan when we arrive at the pivotal cell */
				// partialReplan = true;
				// partialStartState = pivotState;
				// }
				// }
				// }
			}

			/* 3. Time was generously increased with none of the above accompanying causes */
			else if(lastTimeNotice < timeLeft){
				/* Resume DAS */
				planned = false;
				secondRun = true;
				/* For correct comparison, make previous goal's cost reflect only the remaining path */
				GridCellInformed currentCell = closedList[sState.getCoord().getX()][sState.getCoord().getY()];
				incumbent.fUnweighted -= currentCell.fUnweighted;
				currentCell.g = 0;
				currentCell.f = currentCell.h;
				currentCell.gUnweighted = 0;
				currentCell.fUnweighted = currentCell.h;
				openList.offer(currentCell); // for DAS to pick up

				/* Node lists */
				// openList = new PriorityQueue<GridCellInformed>(100);
				deprune(timeLeft + System.currentTimeMillis() - moveTimeReserved);
				prunedList = new PriorityQueue<GridCellInformed>(100);
				closedList = new GridCellInformed[map.getWidth()][map.getHeight()];
				/* Solution stores */
				// bestPath = new ComputedPlan();
				// incumbent = null;
				goalFound = false;
				/* Map parameters */
				// goalState = gState;
				startHeuristic = map.hCost(sState, gState);
				/* Agent state parameters */
				// firstRun = false;
				/* Partial replan parameters */
				// partialReplan = false;
				// partialStartState = null;
				/* Agent configuration */
				weightG = 1f;
				weightH = 1f;
				/* DAS-specific parameters */
				settlingTimeLeft = settlingTime;
				/* Expansion and time counters */
				for(int i = 0; i < windowSize; i++){
					eDelayWindow[i] = 0;
					tDelayWindow[i] = 0;
				}

			}

			/* 4. Agent was teleported or anything else */
			else{
				// /* Get a complete replan */
				// planned = false;
				// firstRun = true;
				// secondRun = false;
			}
		}

		/* Record the time allotted */
		lastTimeNotice = timeLeft;

		/* Reset all parameters */
		if(!planned && firstRun){

			/* Node lists */
			openList = new PriorityQueue<GridCellInformed>(100);
			prunedList = new PriorityQueue<GridCellInformed>(100);
			closedList = new GridCellInformed[map.getWidth()][map.getHeight()];
			/* Solution stores */
			bestPath = new ComputedPlan();
			incumbent = null;
			goalFound = false;
			/* Map parameters */
			goalState = gState;
			startHeuristic = map.hCost(sState, gState);
			startTime = timeLeft;
			minCost = map.getMinCost();
			/* Get goal direction */
			operatorPreference = new int[2]; // North & East
			setOperatorPreference(sState, gState);
			/* Agent state parameters */
			firstRun = false;
			/* Partial replan parameters */
			partialReplan = false;
			partialStartState = null;
			/* Agent configuration */
			weightG = 1f;
			weightH = 1f;
			/* DAS-specific parameters */
			settlingTimeLeft = settlingTime;
			/* Expansion and time counters */
			for(int i = 0; i < windowSize; i++){
				eDelayWindow[i] = 0;
				tDelayWindow[i] = 0;
			}

			/* Initialize start node */
			startCell = new GridCellInformed(sState, 0, startHeuristic, 0, weightG, weightH);
			openList.offer(startCell);

			/* Now on to make a plan */
			/* Compute greedy solution first */
			incumbent = greedy(startCell);

			long timeBeforePlan = System.nanoTime();
			extractPlan(sState, incumbent);
			moveTimeReserved = System.nanoTime() - timeBeforePlan;
			moveTimeReserved /= 1E6; // convert to ms

			return null;
		}

		if(!planned && secondRun){
			if(timeLeft > moveTimeReserved){
				/* Clear closed list */
				closedList = new GridCellInformed[map.getWidth()][map.getHeight()];

				/* Execute DAS */
				das(startCell, System.currentTimeMillis() + timeLeft - moveTimeReserved);

				/* Regenerate plan only if a new goal found */
				if(goalFound) extractPlan(sState, incumbent);

			}else{
				/* TODO Construct greedy path from current best node known */
			}

			planned = true;
		}

		/* Do we have a next step in the path? */
		if(pathStepNo >= bestPath.getLength()){
			return null;
		}
		return (GridCell)bestPath.getStep(pathStepNo++);
	}

	private void setOperatorPreference(GridCell sState, GridCell gState){
		float xdiff = gState.getCoord().getX() - sState.getCoord().getX();
		float ydiff = gState.getCoord().getY() - sState.getCoord().getY();
		operatorPreference[0] = ydiff > 0 ? 1 : (ydiff < 0 ? -1 : 0); // North
		operatorPreference[1] = xdiff > 0 ? 1 : (xdiff < 0 ? -1 : 0); // East
	}

	private void extractPlan(GridCell sState, GridCellInformed goalNodePossible){
		/* Construct path from goal node */
		while(goalNodePossible != null){
			bestPath.prependStep(goalNodePossible.cell);
			goalNodePossible = goalNodePossible.parent;
		}
		bestPath.prependStep(sState);
		pathStepNo = 0; // reset step counter
	}

	private GridCellInformed greedy(GridCellInformed currentCell){
		/* Get a failsafe greedy path */
		/* Iterate over all cells */
		boolean firstIteration = true;

		while(!currentCell.cell.equals(goalState)){
			markClosed(currentCell);

			SuccessorIterator childIterator = map.getNextSuccessor(currentCell.cell);

			/* Track the child that's closest to the goal */
			float minH = Float.POSITIVE_INFINITY;
			GridCell nextCell = null;
			GridCell child;

			while((child = childIterator.next()) != null){
				if(firstIteration) connectivity++; // take this opportunity to determine map connectivity

				if(isClosed(child) || map.isBlocked(child) || child.equals(currentCell.cell)) continue;

				/* Is this the closest child? */
				float thisH = (float)dCheapest(child);
				if(thisH < minH){
					minH = thisH;
					nextCell = child;
				}
			}

			if(firstIteration) firstIteration = false;

			/* Dead end? */
			if(nextCell == null) return null;

			currentCell = new GridCellInformed(nextCell, currentCell, map.cost(currentCell.cell, nextCell), hCost(nextCell, currentCell),
					0, weightG, weightH);
		}

		System.out.println("Greedy goal f: " + currentCell.fUnweighted);

		return currentCell;
	}

	private GridCellInformed das(GridCellInformed startCell, double deadline){
		/* As long as we have time ... */
		while(deadline - System.currentTimeMillis() > averageTimeTaken){

			/* While the open list is empty */
			if(startCell != null || !openList.isEmpty()){

				/* How many levels ahead can we go at this rate? */
				double dmax = calculateDBound(deadline);

				/* Get the next node from the open list or the supplied start node */
				GridCellInformed currentCell;
				if(startCell != null){
					currentCell = startCell;
					startCell = null; // disable for next iteration
				}else currentCell = openList.poll();
				markClosed(currentCell); // add it to visited

				/* Is it worse than the incumbent solution found? */
				if(incumbent != null && currentCell.fUnweighted > incumbent.fUnweighted){
					continue;
				}

				/* Is it a goal (and cheaper than incumbent)? */
				if((currentCell.cell != null && currentCell.cell.equals(goalState))
						&& (incumbent == null || currentCell.fUnweighted < incumbent.fUnweighted)){
					incumbent = currentCell;

					/* Reset weights for next run */
					weightH = 1f;
					weightG = 1f;

					goalFound = true;

					System.out.println("DAS goal f: " + currentCell.fUnweighted);

				}else if(dCheapest(currentCell) < dmax){

					/* Expand children */
					SuccessorIterator childIterator = map.getNextSuccessor(currentCell.cell);
					eCurr++; // increment the expansion count

					/* Calculate expansion delay */
					eDelayWindow[eDelayWindowCurrPosition++] = eCurr - (int)currentCell.e;
					eDelayWindowCurrPosition %= windowSize;

					tPreExpansion = System.nanoTime(); // start tracking the time for expansion

					/* Iterate over each child */
					GridCell child;
					while((child = childIterator.next()) != null){

						/* Have we already seen this node? */
						if(isClosed(child)){
							if(!goalFound){	// only do it once
								/* Is this a better path? */
								GridCellInformed childCell = closedList[child.getCoord().getX()][child.getCoord().getY()];
								if(childCell.parent != null && currentCell.fUnweighted < childCell.parent.fUnweighted){
									childCell.updateCosts(map.cost(currentCell.cell, child), childCell.h, weightG, weightH);
									childCell.parent = currentCell;
								}
							}
							continue;
						}else if(map.isBlocked(child) || child.equals(currentCell.cell)){
							continue; // skip
						}

						/* Add child to open list */
						GridCellInformed childCell = new GridCellInformed(child, currentCell, map.cost(currentCell.cell, child), hCost(
								child, currentCell), eCurr, weightG, weightH);
						openList.add(childCell);
					}

					/* Calculate time delay */
					tDelayWindow[tDelayWindowCurrPosition++] = System.nanoTime() - tPreExpansion;
					tDelayWindowCurrPosition %= windowSize;

				}else{
					/* I don't expect to be able to reach the goal from here */
					prunedList.offer(currentCell);
				}

			}else{
				/* Time left but open list is empty - Recover pruned list! */
				deprune(deadline);
			}

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
			double timeLeft = deadline - System.currentTimeMillis();
			double exp = (timeLeft * windowSize) / tDelaySum;
			averageTimeTaken = tDelaySum / windowSize;
			return (exp * windowSize) / eDelaySum;
		}
	}

	private double dCheapest(GridCellInformed cell){
		/* Used to compute dCheapest - for now simply the Manhattan distance */
		return cell.cell.getCoord().getManhattenDistance(goalState.getCoord());
	}

	private double dCheapest(GridCell cell){
		/* Used to compute dCheapest - for now simply the Manhattan distance */
		return cell.getCoord().getManhattenDistance(goalState.getCoord());
	}

	private double hCost(GridCell cell, GridCellInformed parent){
		setOperatorPreference(cell, goalState);

		float xdiff = cell.getCoord().getX() - parent.cell.getCoord().getX();
		float ydiff = cell.getCoord().getY() - parent.cell.getCoord().getY();
		int north = ydiff > 0 ? 1 : (ydiff < 0 ? -1 : 0); // North
		north *= operatorPreference[0];
		// north = north == 0 ? -1 : north;
		int east = xdiff > 0 ? 1 : (xdiff < 0 ? -1 : 0); // East
		east *= operatorPreference[1];
		// east = east == 0 ? -1 : east;

		double h = parent.h - (north * minCost) - (east * minCost);
		// System.out.println(cell+" h: "+h+"\n\tparent h: "+parent.h);
		return h;
	}

	private void deprune(double deadline){
		/* Used to recover pruned nodes */
		double exp, timeLeft = deadline - System.currentTimeMillis();
		if(settlingTimeLeft > 0){
			exp = startHeuristic;
		}else{
			double tDelaySum = 0;
			for(int i = 0; i < windowSize; i++){
				tDelaySum += tDelayWindow[i];
			}
			tDelaySum /= 1000000; // convert to ms
			exp = (timeLeft * windowSize) / tDelaySum;

			/* Improve weight - inc. H and dec. G over time */
			weightG = (float)(1 - timeLeft / startTime);
			if(weightG < 0.1f) weightG = 0.1f;
			if(weightG > 1) weightG = 1;

			weightH = (float)(startTime / timeLeft);

			if(!goalFound && (weightH + weightG <= 2)) weightH = weightG;
			else weightG = 1f;
			// System.out.println("wG: " + weightG + "\n\twH: " + weightH);

		}

		while(exp > 0 && !prunedList.isEmpty()){
			GridCellInformed s = prunedList.poll();
			unmarkClosed(s); // allow re-visit

			s.e = eCurr; // reset expansion counter

			openList.offer(s);
			exp -= dCheapest(s);
		}
		settlingTimeLeft = settlingTime;
	}

	private void markClosed(GridCellInformed cell){
		closedList[cell.cell.getCoord().getX()][cell.cell.getCoord().getY()] = cell;
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
				if(closedList[i][j] != null) cells.add(closedList[i][j].cell);
		}
		return cells;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes(){
		ArrayList<GridCell> cells = new ArrayList<GridCell>();
		for(GridCellInformed node:openList)
			cells.add(node.cell);
		for(GridCellInformed node:prunedList)
			cells.add(node.cell);
		return cells;
	}

	@Override
	public ComputedPlan getPath(){
		return bestPath;
	}
}
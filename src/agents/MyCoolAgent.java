/**@Pulkit - 3360413
 * RMIT University
 * 
 * Agent implementation for pathfinding on Apparate
 * using Deadline-Aware Search (DAS), with modifications.
 * Operation:
 *  1. On first call to getNextMove(), an incumbent greedy output is generated
 *  2. On the next call, a full DAS output is generated (if possible)
 *  3. Third call onwards, the next node is returned for the agent to move
 */
package agents;

import java.util.ArrayList;
import java.util.PriorityQueue;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;
import agents.pulkit.GridCellInformed;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.Edge;
import au.rmit.ract.planning.pathplanning.entity.State;

public class MyCoolAgent implements PlanningAgent{
	/* Node lists */
	private PriorityQueue<GridCellInformed> openList;
	private PriorityQueue<GridCellInformed> prunedList;
	private GridCellInformed[][] closedList;	// easily retrievable record of visited nodes

	/* Solution stores and information */
	private ComputedPlan bestPath;
	private GridCellInformed incumbent = null;	// best known solution so far
	private boolean goalFound = false;			// handy flag for toggling anything you want when a goal is found
	private int pathStepNo = 0;					// index pointer to the path step in bestPath

	/* Map parameters */
	private GridDomain map;						// stores the full map for further use across functions
	private GridCell goalState = null;			// stores the final goal - use it to check if goal has moved
	private float startHeuristic = 0;			// if you want to compare current situation against start (e.g., dynamic weighting)
	private long startTime = 0;					// same as above, but with time
	private GridCellInformed startCell = null;	// initial node for the current search algorithm
	private float minCost = 0;					// min cost of any edge on this map - used for calculating your own heuristic
	private int[] operatorPreference;			// [North, East], each direction +1 for preferred, -1 not, 0 don't care
	private int connectivity = 0; 				// manhattan or euclidean, or anything else (3D?)!

	/* Agent state parameters */
	private boolean planned = false; 			// track whether a final plan has been prepared
	private boolean firstRun = true; 			// to run greedy first (or any backup pathfinder)
	private boolean secondRun = false; 			// to run DAS next (or whatever's the main pathfinder)
	private long lastTimeNotice = 0; 			// to track the last notice of timeleft, and hence if more time has been allotted

	/* Agent configuration */
	private long timeReserved = 0; 				// in ms - time reserved for generating ComputedPlan from the resultant goal node
	private float weightG = 1.0f; 				// weight given to g, if any
	private float weightH = 1.0f; 				// weight given to h, if any

	/* DAS-specific parameters */
	private int windowSize = 20;				// magic number! but inescapable
	private int settlingTime = 20;				// >= window size
	private int settlingTimeLeft;				// to track the time left before dmax is calculated
	/* Expansion counters */
	private int eCurr = 0;
	private int[] eDelayWindow = new int[windowSize];
	private int eDelayWindowCurrPosition = 0;
	/* Time counters */
	private long tPreExpansion = 0;
	private long[] tDelayWindow = new long[windowSize];
	private int tDelayWindowCurrPosition = 0;
	/* Time counter for full DAS iteration */
	private double averageTimeTaken = 0;		// so we can avoid entering another iteration if time left is less than average time taken

	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft, long stepTime, long timeLeft){
		/* Which run is this? */
		if(!firstRun && !secondRun) secondRun = true;
		
		/* Store latest map */
		this.map = map;

		/* Dynamism - do we need to replan? */
		if(planned){
			
			/* 1. Goal moved */
			if(!goalState.equals(gState)){
				
				/* Did it move away? */
				double oldDistance = sState.getCoord().getManhattenDistance(goalState.getCoord());
				double newDistance = sState.getCoord().getManhattenDistance(gState.getCoord());
				if(oldDistance < newDistance){
					
					/* Plan a path between those goal positions */
					startCell = closedList[goalState.getCoord().getX()][goalState.getCoord().getY()];
					startCell.parent = null;
					GridCell oldGoalCell = goalState;
					initializeParameters(map, goalState, gState, timeLeft);
					startCell.updateCosts(0, startHeuristic, 1, 1);
					ComputedPlan goalPath = executeDAS(map, oldGoalCell, startCell, timeLeft, 0);
					
					/* Join paths */
					for(int i = 0; i < goalPath.getLength(); i++)
						bestPath.appendStep(goalPath.getStep(i));
					
				}else{
					/* Worthless, re-plan */
					planned = false;
					firstRun = true;
					secondRun = false;
				}
			}

			/* 2. Map has changed */
			else if(map.getChangedEdges().size() > 0){
				
				/* Did anything change on our path? */
				for(Edge edge:map.getChangedEdges()){
					
					/* Iterate over the rest of the path left to find out */
					for(int i = pathStepNo; i < bestPath.getLength(); i++){
						State pivotState = bestPath.getStep(i);
						
						if(pivotState.equals(edge.getStart()) || pivotState.equals(edge.getEnd())){
							/* Yes! Replan! */
							planned = false;
							firstRun = true;
							secondRun = false;
							break;
						}
					}
					if(!planned) break;
				}
			}

			/* 3. Time was generously increased with none of the above accompanying causes */
			else if(lastTimeNotice < timeLeft){
				/* Resume DAS */

				/* Find the current f cost ahead to the goal */
				startCell = closedList[sState.getCoord().getX()][sState.getCoord().getY()];
				double currentGoalF = incumbent.gUnweighted - startCell.gUnweighted;
				startCell.parent = null;
				initializeParameters(map, sState, gState, timeLeft);
				startCell.updateCosts(0, startHeuristic, 1, 1);

				/* Do DAS */
				bestPath = executeDAS(map, sState, startCell, timeLeft, currentGoalF);
				pathStepNo = 0; // reset step counter
			}
		}

		/* Record the time allotted */
		lastTimeNotice = timeLeft;

		/* First run - do a basic pathfinder for contingency */
		if(!planned && firstRun){
			/* Reset all parameters */
			/* Agent state parameters */
			firstRun = false;

			/* Initialize everything */
			initializeParameters(map, sState, gState, timeLeft);

			/* Initialize start node */
			startCell = new GridCellInformed(sState, 0, startHeuristic, 0, weightG, weightH);
			openList.offer(startCell);

			/* Now on to make a plan */
			/* Compute greedy solution first - or any incumbent solution finder */
			incumbent = greedy(startCell);

			/* Generate plan, all the while assessing the time taken to do so */
			long timeBeforePlan = System.nanoTime();
			bestPath = extractPlan(sState, incumbent);
			timeReserved = System.nanoTime() - timeBeforePlan; // this time will be reserved when doing DAS
			timeReserved /= 1E6; // convert to ms

			return null;
		}

		/* Second run - do full pathfinder - DAS in our case */
		if(!planned && secondRun){
			/* Run DAS */
			bestPath = executeDAS(map, sState, null, timeLeft, 0);
			pathStepNo = 0; 		// init step counter

			planned = true;
		}

		/* Do we have a next step in the path? */
		if(pathStepNo >= bestPath.getLength()){
			return null;
		}
		return (GridCell)bestPath.getStep(pathStepNo++);
	}

	/****************************************************************
	 * 		Contingency search - Greedy
	 * 		Replace with any pathfinding algorithm as needed
	 * 
	 * Performs greedy search and returns a goal node. Path can be traced back 
	 * from the goal node using extractPlan()
	 * 
	 * @param currentCell - start cell for the greedy algorithm
	 * @return incumbent goal node
	 */
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

		return currentCell;
	}

	/****************************************************************
	 * 
	 * 				Main search algorithm: DAS in our case
	 * 
	 * Executes DAS and returns a full computed plan.
	 * 
	 * @param map - game board
	 * @param sState - start cell
	 * @param startCell - (optional) start cell with costs already recorded
	 * @param timeLeft - for establishing deadline for DAS
	 * @param maxF - for subsequent DAS runs, you can tell it to find solutions of no more than maxF cost
	 * @return full computed plan
	 */
	private ComputedPlan executeDAS(GridDomain map, GridCell sState, GridCellInformed startCell, long timeLeft, double maxF){
		if(timeLeft > timeReserved){
			/* Clear closed list */
			closedList = new GridCellInformed[map.getWidth()][map.getHeight()];
	
			/* Execute DAS */
			das(startCell, System.currentTimeMillis() + timeLeft - timeReserved);
	
			/* Regenerate plan only if a new goal found */
			if(goalFound && (maxF <= 0 || incumbent.fUnweighted < maxF)) return extractPlan(sState, incumbent);
		}
		return bestPath;
	}

	/** Core DAS algorithm. Solution path can be traced back from 
	 * returned goal node using extractPlan()
	 * 
	 * @param startCell - start cell with initial costs set
	 * @param deadline - time in millis since Jan 1, 1970, until DAS can run
	 * @return returns the goal node
	 */
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

					// System.out.println("DAS goal f: " + currentCell.fUnweighted);

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
							/* Is this a better path? */
							GridCellInformed childCell = closedList[child.getCoord().getX()][child.getCoord().getY()];
							if(childCell.parent != null && currentCell.fUnweighted < childCell.parent.fUnweighted){
								unmarkClosed(childCell);
							}else continue;
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

	/** Function for computing dmax - will not compute dmax (return infinity)
	 * for the duration of settlingTime
	 * @param deadline - allotted to DAS
	 * @return dmax value
	 */
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

	/** Computes dCheapest. So far, doesn't get into the whole 
	 * dCheapest_hat and dCheapest_error factors, for simplicity.
	 * 
	 * @param cell - cell for which dCheapest must be calculated
	 * @return dCheapest, i.e., min estimated cost to goal
	 */
	private double dCheapest(GridCellInformed cell){
		return dCheapest(cell.cell);
	}
	private double dCheapest(GridCell cell){
		/* Used to compute dCheapest - for now simply the Manhattan distance */
		double x = Math.abs(cell.getCoord().getX() - goalState.getCoord().getX());
		double y = Math.abs(cell.getCoord().getY() - goalState.getCoord().getY());
		return Math.max(x, y) + (Math.sqrt(2) - 1) * Math.min(x, y);
	}

	/** DAS-specific function to recover pruned nodes. 
	 * As a result, the open list will now have new nodes to consider.
	 * 
	 * @param deadline - allotted to DAS
	 */
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
			weightH = (float)(startTime / timeLeft);
			if(weightH > startHeuristic) weightH = startHeuristic;

			/* Weight on G disabled
			weightG = (float)(1 - timeLeft / startTime);
			if(weightG < 0.1f) weightG = 0.1f;
			if(weightG > 1) weightG = 1;

			if(!goalFound && (weightH + weightG <= 2)) weightH = weightG;
			else weightG = 1f;
			*/
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

	
	/****************************************************************
	 * 
	 * 				Heuristic computation
	 * 
	 * Sets operator preference - [North, East] where both directions are
	 * +1 if that direction is favoured
	 * -1 if not, and 0 if that direction is irrelevant.
	 * Direction is considered favoured if the goal is in that direction.
	 * 
	 * Use this function in the hCost() function.
	 * 
	 * @param sState - start state
	 * @param gState - goal state
	 */
	private void setOperatorPreference(GridCell sState, GridCell gState){
		float xdiff = gState.getCoord().getX() - sState.getCoord().getX();
		float ydiff = gState.getCoord().getY() - sState.getCoord().getY();
	
		if(Math.abs(xdiff) > Math.abs(ydiff)){
			operatorPreference[0] = 0; // North
			operatorPreference[1] = xdiff > 0 ? 1 : (xdiff < 0 ? -1 : 0); // East
		}else if(Math.abs(ydiff) > Math.abs(xdiff)){
			operatorPreference[0] = ydiff > 0 ? 1 : (ydiff < 0 ? -1 : 0); // North
			operatorPreference[1] = 0; // East
		}else{
			operatorPreference[0] = ydiff > 0 ? 1 : (ydiff < 0 ? -1 : 0); // North
			operatorPreference[1] = xdiff > 0 ? 1 : (xdiff < 0 ? -1 : 0); // East
		}
	}

	/** Compute heuristic cost, overriding map.hCost provided by Apparate.
	 * Uses incremental heuristic computation. I.e., heuristic child = heuristic parent + 1, or -1, or same.
	 * based on if the motion was towards the goal, away, etc.
	 * 
	 * @param cell - cell whose h is needed
	 * @param parent - cell's parent, for quick incremental heuristic computation
	 * @return heuristic value
	 */
	private double hCost(GridCell cell, GridCellInformed parent){
			float xdiff = cell.getCoord().getX() - parent.cell.getCoord().getX();
			float ydiff = cell.getCoord().getY() - parent.cell.getCoord().getY();
	
			setOperatorPreference(parent.cell, goalState);
	
			int north = ydiff > 0 ? 1 : (ydiff < 0 ? -1 : 0); // North
			north *= operatorPreference[0];
			int east = xdiff > 0 ? 1 : (xdiff < 0 ? -1 : 0); // East
			east *= operatorPreference[1];
	
			double change = (north * minCost) + (east * minCost);
	//		if(Math.abs(change) >= 1) change = Math.signum(change);
			double h = parent.h - change;
			return h;
		}

	/****************************************************************
	 * 
	 * 				Extra utility functions for agent operation
	 * 
	 * Reset all parameters to initial/default values. 
	 * Handy functions to reset whenever you want.
	 * 
	 * @param map - game map
	 * @param sState - start state
	 * @param gState - goal state
	 * @param timeLeft - time left, what else?
	 */
	private void initializeParameters(GridDomain map, GridCell sState, GridCell gState, long timeLeft){
		/* Node lists */
		openList = new PriorityQueue<GridCellInformed>(100);
		prunedList = new PriorityQueue<GridCellInformed>(100);
		closedList = new GridCellInformed[map.getWidth()][map.getHeight()];
		/* Solution stores */
		incumbent = null;
		goalFound = false;
		/* Map parameters */
		goalState = gState;
		startHeuristic = map.hCost(sState, gState);
		startTime = timeLeft;
		minCost = map.getMinCost();
		/* Get goal direction */
		operatorPreference = new int[2]; // North & East
		setOperatorPreference(sState, goalState);
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

	/****************************************************************
	 * 				Closed list functions 
	 * 
	 * Set closed 
	 * 
	 * @param cell 
	 */
	private void markClosed(GridCellInformed cell){
		closedList[cell.cell.getCoord().getX()][cell.cell.getCoord().getY()] = cell;
	}

	/** Set as open
	 * 
	 * @param cell
	 */
	private void unmarkClosed(GridCellInformed cell){
		closedList[cell.cell.getCoord().getX()][cell.cell.getCoord().getY()] = null;
	}

	/**	Check if cell is closed
	 * 
	 * @param cell
	 * @return true if closed, false if not
	 */
	private boolean isClosed(GridCellInformed cell){
		return isClosed(cell.cell);
	}
	private boolean isClosed(GridCell cell){
		return closedList[cell.getCoord().getX()][cell.getCoord().getY()] != null;
	}

	/****************************************************************
	 * 
	 * 			Extract computed plan path from the goal found
	 * 
	 * Handy function to trace back the path from the goal node.
	 * Make sure as you search, you set parent nodes for each child.
	 * 
	 * @param sState - start cell
	 * @param goalNode - goal node found
	 * @return full plan path
	 */
	private ComputedPlan extractPlan(GridCell sState, GridCellInformed goalNode){
		/* Construct path from goal node */
		ComputedPlan path = new ComputedPlan();
		while(goalNode != null){
			path.prependStep(goalNode.cell);
			goalNode = goalNode.parent;
		}
		path.prependStep(sState);
		return path;
	}

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
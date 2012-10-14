package agents.ramirez.astar;

import java.util.ArrayList;
import java.util.PriorityQueue;

import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import agents.ramirez.astar.Node;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

public class PathPlanner implements PlanningAgent {

	final ThreadMXBean threadMX = ManagementFactory.getThreadMXBean();
	
	private PriorityQueue<Node> 		OPEN = new PriorityQueue<Node>();
	private Node[][] 					CLOSED;
	private GridDomain 					problem = null;
	private	 ComputedPlan				solution = null;
	private int						planStep = 0;
	private GridCell 					goal = null;
	private Node 						root = null;
	private float 						h0;
	private long  						expanded = 0;
	private long						generated = 0;
	private boolean					done = false;
		
	final private double 				MS_TO_NS_CONV_FACT = 1E6;
	
	@Override
	public ArrayList<GridCell> expandedNodes() {
		ArrayList<GridCell> nodes = new ArrayList<GridCell>();
		for ( int i = 0; i < problem.getHeight(); i++ ) {
			for ( int j = 0; j < problem.getWidth(); j++ )
				if ( CLOSED[i][j] != null )
					nodes.add(CLOSED[i][j].state);
		}
		return nodes;
	}
	
	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		ArrayList<GridCell> nodes = new ArrayList<GridCell>();
		while (!OPEN.isEmpty()) {
			nodes.add( OPEN.poll().state );
		}
		return nodes;
	}
	
	private double getElapsed() {
		//return System.nanoTime() / MS_TO_NS_CONV_FACT;
		//return System.nanoTime();
		return threadMX.getCurrentThreadCpuTime();
	}
	
	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {
		//System.out.println("Time left: " + timeLeft + " Step Time: " + stepTime );
		if ( solution == null) {
			planStep = 0;
			System.out.println("Initializing search engine");
			initialize( map, start, goal );
			
			double searchTime = (double)timeLeft * MS_TO_NS_CONV_FACT * 0.8; //- ((double)stepTime * h0);
			System.out.println("Search time: " + (searchTime/MS_TO_NS_CONV_FACT) + " ms");
			
			double deadline = getElapsed() + searchTime;
			Node p = doSearch(deadline);
			if ( p == null) {
				System.out.println("No solution found!");
				return null;
			}
			extractSolution(p);
			System.out.println("Plan length: " + solution.getLength() + " Cost: " + solution.getCost());
		}
		// Check if path has been exhausted.
		if (planStep >= solution.getLength()) {
			System.out.println("Solution has been executed");
			return null;
		}

		// Return the next step in the path.
		//System.out.println("Returning plan step #" + planStep + ", " + (solution.getLength() - planStep) + " to go");
		return (GridCell) solution.getStep(planStep++);
	}

	private void extractSolution(Node n) {
		solution = new ComputedPlan();
		
		Node current = n;
		while ( current != null ) {
			solution.prependStep(current.state);
			current = current.parent;
		}
		solution.setCost( n.gn );
	}

	private Node doSearch(double deadline) {
		System.out.println("Starting search");
		Node incumbent = null;
		OPEN.offer(root);
		
		while ( !OPEN.isEmpty() ) {
			Node n = OPEN.poll();
			/*
			System.out.println(	"Expansion: " + expanded + "s=(" 
								+ n.state.getCoord().getX() + ", " 
								+ n.state.getCoord().getY()  
								+ "), g(n) = " + n.gn +  "h(n) = " + n.hn + ", f(n) = " + n.fn );
			*/
			if ( isGoal( n )  ) {
				System.out.println("Goal Reached!");
				incumbent = n;
				break;
			}
			expand(n);
			
		}
		
		System.out.println("Nodes expanded: " + expanded + " generated: " + generated );
		System.out.println( "Time left: " + (deadline - getElapsed()) + " ns"  );
		done = true;
		return incumbent;
	}

	private boolean isClosed(GridCell s) {
		return CLOSED[s.getCoord().getX()][s.getCoord().getY()] != null;
	}

	private void expand(Node n) {
		expanded++;

		for (State absSucc : problem.getSuccessors(n.state) ) {

			if ( problem.isBlocked(absSucc) ) continue;
			
			GridCell succ = (GridCell)absSucc;
			
			if ( isClosed( succ ) ) continue;
			
			float h = manhattan(succ);
			
			Node n2 = new Node( succ, n, problem.cost(n.state, succ), h );
			OPEN.offer( n2 );
			generated++;
		}
		close(n);
	}

	private void close(Node n) {
		CLOSED[n.state.getCoord().getX()][n.state.getCoord().getY()] = n;
	}

	private boolean isGoal(Node n) {
		return n.state.getCoord().getX() == goal.getCoord().getX() 
				&& n.state.getCoord().getY() == goal.getCoord().getY();
	}

	@Override
	public ComputedPlan getPath() {
		return solution;
	}

	@Override
	public Boolean showInfo() {
		return done;
	}

	private	 void	initialize( GridDomain prob, GridCell s0, GridCell g ) {
		problem = prob;
		System.out.println( "Map dimensions: " + problem.getWidth() + "x" + problem.getHeight() );
		goal = g;
		h0 = manhattan(s0);
		expanded = 0;
		CLOSED = new Node[problem.getWidth()][problem.getHeight()];
		System.out.println("Closed has been initialized");
		root = new Node( s0, 0.0f, h0 );
	}

	private float manhattan(GridCell s) {
		return Math.abs(s.getCoord().getX() - goal.getCoord().getX()) +
			       Math.abs(s.getCoord().getY() - goal.getCoord().getY());
	}

	private float euclidean(GridCell s) {
		float dx = s.getCoord().getX() - goal.getCoord().getX();
		dx *= dx;
		float dy = s.getCoord().getY() - goal.getCoord().getY();
		dy *= dy;
		return (float)Math.sqrt( dx + dy );
	}
	
}
package agents.ramirez.astar;

import pplanning.simviewer.model.GridCell;

public class Node implements Comparable<Node> {

	public	GridCell	state; 			// state
	public long		depth = 0;		// depth
	public float		gn	  = 0.0f;	// g(n)
	public float		hn;				// h(n)
	public	float		fn;				// f(n)
	public Node			parent = null;
	
	public Node( GridCell s, float cost, float h ) {
		state = s;
		init( cost, h );
	}
	
	public Node( GridCell s, Node p, float cost, float h) {
		state = s;
		parent = p;
		init( cost, h );
	}
	
	private void init( float cost, float h ) {
		if ( parent != null ) {
			gn = cost + parent.gn;
			depth = parent.depth + 1;
		}
			
		hn = h;
		fn = hn + gn;			
	}
	
	public int compareTo(Node other) {
		// Check if the values are "the same" 
		if (this.fn < other.fn)
			return -1;
		
		if ( this.fn > other.fn )
			return 1;
		
		if ( this.hn < other.hn)
			return -1;
		
		if (this.hn > other.hn )
			return 1;
		
		if ( this.gn > other.gn )
			return -1;
		
		return 1;
	}

}
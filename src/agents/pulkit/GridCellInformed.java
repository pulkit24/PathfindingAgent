package agents.pulkit;

import java.util.Random;
import pplanning.simviewer.model.GridCell;

public class GridCellInformed implements Comparable<GridCellInformed>{
	/* Me and my kin */
	public GridCell cell;
	public GridCellInformed parent = null;

	/* Costs */
	public double g = 0;
	public double h = 0;
	public double f = 0;
	public double gUnweighted = 0;
	public double fUnweighted = 0;

	/* Vacillation tracker */
	public int e = 0;

	public GridCellInformed(GridCell cell, double edgeCost, double h, int e, double weightG, double weightH){
		setup(cell, null, edgeCost, h, e, weightG, weightH);
	}

	public GridCellInformed(GridCell cell, GridCellInformed parent, double edgeCost, double h, int e, double weightG, double weightH){
		setup(cell, parent, edgeCost, h, e, weightG, weightH);
	}

	/* Common initialization */
	public void setup(GridCell cell, GridCellInformed parent, double edgeCost, double h, int e, double weightG, double weightH){
		this.cell = cell;
		this.parent = parent;
		this.g = edgeCost + weightG * (parent == null ? 0 : parent.g);
		this.h = h;
		this.f = this.g + weightH * this.h;
		this.e = e;
		this.gUnweighted = edgeCost + (parent == null ? 0 : parent.gUnweighted);
		this.fUnweighted = this.gUnweighted + this.h;
	}

	/* For changing costs */
	public void updateCosts(double edgeCost, double h, double weightG, double weightH){
		this.g = edgeCost + weightG * (parent == null ? 0 : parent.g);
		this.h = h;
		this.f = this.g + weightH * this.h;
		this.gUnweighted = edgeCost + (parent == null ? 0 : parent.gUnweighted);
		this.fUnweighted = this.gUnweighted + this.h;
	}

	/* For sorting */
	public int compareTo(GridCellInformed otherCell){
		/* First compare by f */
		int comparison = Double.compare(this.f, otherCell.f);
		if(comparison == 0){
			/* Break ties by h */
			comparison = Double.compare(this.h, otherCell.h);
			if(comparison == 0){
				/* Break ties by g */
				comparison = Double.compare(this.g, otherCell.g);
				if(comparison == 0){
					/* Break ties by unweighted f */
					comparison = Double.compare(this.fUnweighted, otherCell.fUnweighted);
					if(comparison == 0){
						/* Break ties by unweighted g */
						comparison = Double.compare(this.gUnweighted, otherCell.gUnweighted);
						if(comparison == 0){
							/* Break ties by expansion time */
							comparison = Double.compare(this.e, otherCell.e);
//							if(comparison == 0){
//								/* Oh chuck it, go random! */
//								comparison = (new Random()).nextInt(2) - 1;
//							}
						}
					}
				}
			}
		}
		return comparison;
	}

	public String toString(){
		return this.cell.toString() + " f = " + f + " g = " + g + " h = " + h + " e = " + e;
	}
}

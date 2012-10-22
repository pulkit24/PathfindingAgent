/**@Pulkit - 3360413
 * RMIT University
 * 
 * Handy container for GridCell with inbuilt support for parameters:
 * 	f, g, h
 * 	unweighted f and g (automatically calculated)
 * 	parent cell reference
 * 	e (expansion count specific to DAS)
 * 
 * All data is publicly accessible.
 * 
 * Post-initialization, use updateCosts() to update costs if needed.
 * 
 * Inbuilt comparator that breaks ties in the following order:
 * 1. f cost
 * 2. h cost
 * 3. g cost (higher is probably closer/more accurate)
 * 4. Unweighted f, in case it is different from f
 * 5. Unweighted g, in case it is different from g
 * 6. e, the expansion count, so as to favour newer nodes (higher expansion count)
 * 
 */
package agents.pulkit;

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
	private void setup(GridCell cell, GridCellInformed parent, double edgeCost, double h, int e, double weightG, double weightH){
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
				/* Break ties by g - higher g is better (more accurate) */
				comparison = Double.compare(otherCell.g, this.g);
				if(comparison == 0){
					/* Break ties by unweighted f */
					comparison = Double.compare(this.fUnweighted, otherCell.fUnweighted);
					if(comparison == 0){
						/* Break ties by unweighted g - higher g is better (more accurate) */
						comparison = Double.compare(otherCell.gUnweighted, this.gUnweighted);
						if(comparison == 0){
							/* Break ties by expansion time */
							comparison = Double.compare(this.e, otherCell.e);
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

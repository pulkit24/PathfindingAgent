package agents.pulkit;

import java.util.Comparator;

public class Sorter implements Comparator<GridCellInformed>{
	private int sortBy = 0;

	public static int H = 2;
	public static int F = 3;

	public Sorter(int sortBy){
		this.sortBy = sortBy;
	}

	public int compare(GridCellInformed me, GridCellInformed them){
		int comparison = 0;

		if(sortBy == F){
			/* First compare by f */
			comparison = Double.compare(me.f, them.f);
			if(comparison == 0){
				/* Break ties by h */
				comparison = Double.compare(me.h, them.h);
				if(comparison == 0){
					/* Break ties by g */
					comparison = Double.compare(me.g, them.g);
					if(comparison == 0){
						/* Oh chuck it, favour the newer item */
						comparison = 1;
					}
				}
			}

		}else if(sortBy == H){
			/* First compare by h */
			comparison = Double.compare(me.h, them.h);
			if(comparison == 0){
				/* Break ties by f */
				comparison = Double.compare(me.f, them.f);
				if(comparison == 0){
					/* Break ties by g */
					comparison = Double.compare(me.g, them.g);
					if(comparison == 0){
						/* Oh chuck it, favour the newer item */
						comparison = 1;
					}
				}
			}
		}
		return comparison;
	}
}

// 
// 
// 

#include "HookLoad.h"

void HookLoad::setup()
{
	for (uint lc_idx = 0; lc_idx < SPACIAL_DIMENSIONS; lc_idx++) {
		lc[lc_idx].setup();
	}
}

void HookLoad::detect()
{
	{
		for (uint dim_idx = 0; dim_idx < SPACIAL_DIMENSIONS; dim_idx++) {
			_raw[dim_idx] = 0;
			for (uint lc_idx = 0; lc_idx < NUM_LOADCELLS; lc_idx++) {
				_raw[dim_idx] += lc[lc_idx].getRaw(dim_idx);
			}
		}
	}
}

#pragma once

#include "CoreMinimal.h"

namespace PathSmoothing {

	// run the smoothing algorithm that I came up with years ago
	// ray trace from prev point (or in case of first, from second point) to intended move point
	// if bad trace, move invalid

	void smooth_path(
		UWorld* world,
		FVector* waypoints,
		FVector** path,
		FVector** normals,
		int path_len,
		int pass_ct=5,
		float strength=0.8f
	);
}
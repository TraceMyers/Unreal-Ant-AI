#include "PathSmoothing.h"

#include "AINav.h"
#include "Cling.h"

void PathSmoothing::smooth_path(
	UWorld* world,
	FVector* waypoints,
	FVector** path,
	FVector** normals,
	int path_len,
	int pass_ct,
	float strength
) {
	FVector normal_buf[AINav::SMOOTHED_PATH_MAX_LEN];
	const int end_i = path_len - 1;
	const float tr_offset = 30.0f;
	const float tr_offset_sq = tr_offset * tr_offset;
	const float hit_epsilon_factor = 0.99f;
	for (int i = 0; i < end_i; i++) {
		const int wp_i = 3 * i;
		FVector& path_pos = *(path[i]);
		FVector diff = *(path[i + 1]) - path_pos;
		waypoints[wp_i] = path_pos;
		waypoints[wp_i + 1] = path_pos + diff * 0.3333f;
		waypoints[wp_i + 2] = path_pos + diff * 0.6666f;
		const FVector& normal = *(normals[i]);
		normal_buf[wp_i] = normal;
		normal_buf[wp_i + 1] = normal;
		normal_buf[wp_i + 2] = normal;
	}

	const int wp_end_i = end_i * 3;
	const int wp_end_i_m1 = wp_end_i - 1;
	waypoints[wp_end_i] = *(path[end_i]);
	
	for (int i = 0; i < pass_ct; i++) {
		for (int j = 1; j < wp_end_i_m1; j++) {
			FVector& prev = waypoints[j - 1];
			FVector& cur = waypoints[j];
			FVector& next = waypoints[j + 1];

			const FVector midpoint = prev + (next - prev) * 0.5f;
			const FVector new_cur = cur + (midpoint - cur) * strength;
			const FVector new_diff = new_cur - prev;
			const float new_dist_sq = new_diff.SizeSquared();

			FVector tr_start = prev + normal_buf[j] * tr_offset; // TODO: un-magic this number
			FVector tr_end = prev + new_diff * 1.1f;
			const float tr_min_dist = FMath::Sqrt(new_dist_sq + tr_offset_sq) * hit_epsilon_factor;
			
			FHitResult hit;
			world->LineTraceSingleByChannel(
				hit,
				tr_start,
				tr_end,
				ECC_WorldStatic
			);
			if (hit.Distance < tr_min_dist) {
				continue;	
			}

			cur = new_cur;
		}	
	}	
}

#include "OrientationComponent.h"
#include "Common.h"
#include "TriGrid.h"

UOrientationComponent::UOrientationComponent() {
	PrimaryComponentTick.bCanEverTick = false;
}


void UOrientationComponent::BeginPlay() {
	Super::BeginPlay();
}


void UOrientationComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UOrientationComponent::init(TriGrid* _tri_grid, float _capsule_radius, float _capsule_half_height) {
	tri_grid = _tri_grid;
	capsule_radius = _capsule_radius;
	capsule_half_height = _capsule_half_height;
	head_on_hit_max = capsule_half_height * 1.8f;
	head_on_hit_normalizer = 1.0f / (head_on_hit_max - capsule_radius);
}

void UOrientationComponent::get_new_orientation(const FVector& loc, const FVector& up, FQuat& ori, const FVector& move) {
	if (!tri_grid) {
		// comm::print("ERROR UOrientationComponent::get_new_orientation(): failure at !tri_grid");
		return;
	}
	if (move.Size() != 0.0f) {
		move_cached = move;
	}

	const FVector tr_start = loc + up * capsule_radius;
	const FVector head_on_dir = move_cached * head_on_hit_max;
	const FVector tr_end = tr_start + head_on_dir;
	const float head_on_hit_dist = comm::trace_hit_ground(tr_start, tr_end, GetOwner());
	if (head_on_hit_dist > 0.0f) {
		const FVector head_on_hit_loc = comm::get_hit_loc();
		find_near_tris(loc, head_on_hit_loc);
		if (head_on_hit_tri == nullptr) {
			if (near_tri != nullptr) {
				set_orientation(near_tri->normal, up, ori);
			}
		}
		else if (near_tri == nullptr) {
			set_orientation(head_on_hit_tri->normal, up, ori);	
		}
		else {
			set_weighted_orientation(up, head_on_hit_dist, ori);
		}
	}
	else {
		find_near_tri(loc);
		if (near_tri != nullptr) {
			set_orientation(near_tri->normal, up, ori);
		}
	}	
}

void UOrientationComponent::find_near_tri(const FVector& loc) {
	uint32 box_ct;
	TArray<Tri>** grid_boxes = tri_grid->get_nearby_grid_boxes(loc, box_ct);
	near_tri = nullptr;
	for (uint32 i = 0; i < box_ct; i++) {
		auto& grid_box = *grid_boxes[i];
		for (int j = 0; j < grid_box.Num(); j++) {
			Tri& tri = grid_box[j];
			if (tri.point_near(loc)) {
				near_tri = &tri;

#ifdef ORIENTATION_DEBUG
				DrawDebugCircle(
					GetWorld(),
					tri.center,
					3.0f,
					2,
					FColor::Red,
					false,
					-1,
					0,
					3.0f
				);
#endif

				return;
			}
		}	
	}
	if (near_tri == nullptr) {
		// comm::print("WARNING UOrientationComponent::find_near_tri(): failure at near_tri == nullptr");
	}
}

void UOrientationComponent::find_near_tris(const FVector& loc, const FVector& head_on_hit_loc) {
	uint32 box_ct;
	TArray<Tri>** grid_boxes = tri_grid->get_nearby_grid_boxes(loc, box_ct);
	bool found_tri[2] {false, false};
	near_tri = nullptr;
	head_on_hit_tri = nullptr;
	for (uint32 i = 0; i < box_ct; i++) {
		auto& grid_box = *grid_boxes[i];
		for (int j = 0; j < grid_box.Num(); j++) {
			Tri& tri = grid_box[j];
			if (!found_tri[0] && tri.point_near(loc)) {
				near_tri = &tri;

#ifdef ORIENTATION_DEBUG
				DrawDebugCircle(
					GetWorld(),
					tri.center,
					3.0f,
					2,
					FColor::Red,
					false,
					-1,
					0,
					3.0f
				);
#endif

				if (found_tri[1]) {
					return;
				}
				found_tri[0] = true;
			}
			if (!found_tri[1] && tri.point_near(head_on_hit_loc)) {
				head_on_hit_tri = &tri;
				if (found_tri[0]) {
					return;
				}
				found_tri[1] = true;
			}	
		}	
	}
	if (!found_tri[0] || !found_tri[1]) {
		// comm::print("WARNING UOrientationComponent::find_near_tris(): failure at near_tris == nullptr");
	}
}

void UOrientationComponent::set_orientation(const FVector& to_normal, const FVector& up, FQuat& ori) {
	const float half_theta = FMath::Acos(FVector::DotProduct(up, to_normal)) * 0.5f;
	const FVector rot_axis_adj = FVector::CrossProduct(up, to_normal).GetSafeNormal() * FMath::Sin(half_theta);
	const FQuat rot(
		rot_axis_adj.X,
		rot_axis_adj.Y,
		rot_axis_adj.Z,
		FMath::Cos(half_theta)
	);
	ori = rot * ori;
}

void UOrientationComponent::set_weighted_orientation(const FVector& up, float hit_dist, FQuat& ori) const {
	float forward_weight = (head_on_hit_max - hit_dist) * head_on_hit_normalizer;
	if (forward_weight > 1.0f) {
		forward_weight = 1.0f;
	}
	const float near_weight = 1.0f - forward_weight;
	const FVector weighted_normal = (near_tri->normal * near_weight + head_on_hit_tri->normal * forward_weight).GetSafeNormal();
	set_orientation(weighted_normal, up, ori);	
}

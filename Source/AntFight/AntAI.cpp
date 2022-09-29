#include "AntAI.h"
#include "AntAIDispatch.h"
#include "Components/BoxComponent.h"
#include "Common.h"

// TODO: add some sounds
// TODO: add basic menus
// TODO: code cleanup

void AAntAI::on_collis_box_overlap_begin(
	UPrimitiveComponent* overlapped_comp,
	AActor* other_actor,
	UPrimitiveComponent* other_comp,
	int32 other_body_index,
	bool from_sweep,
	const FHitResult& sweep_result
) {
	AAnt* other_ant = Cast<AAnt>(other_actor);
	if (other_ant && other_ant != this) {
		if (find_nearby_ant(other_ant) >= 0) {
    		return;
    	}	
		bool avoid_responsibility; // :)
		if (status == AI_STATUS_RETRACING) {
			avoid_responsibility = false;
		}
		else {
			avoid_responsibility = !other_ant->has_avoid_responsibility(this);
		}
		add_nearby_ant(other_ant, avoid_responsibility);
	}
}

void AAntAI::on_collis_box_overlap_end(
	UPrimitiveComponent* overlapped_comp,
	AActor* other_actor,
	UPrimitiveComponent* other_comp,
	int32 other_body_index
) {
	AAnt* other_ant = Cast<AAnt>(other_actor);
	if (other_ant) {
		remove_nearby_ant(other_ant);
	}
}

bool AAntAI::deal_with_jams(float delta_time) {
	jammed_time += delta_time;
	if (jammed_time > JAMMED_MAX) {
		avoidance_yaw_half_theta = 0.0f;
		unjam = true;
		no_accel = false;
		jammed_time = 0.0f;
		return true;
	}
	return false;
}

void AAntAI::add_nearby_ant(AAnt* ant, bool collis) {
	if (nearby_i < MAX_NEARBY) {
		auto& nearby_ant = nearby_ants[nearby_i];
		nearby_ant.ant = ant;
		nearby_ant.collision_responsibility = collis;
		while (nearby_i < MAX_NEARBY && nearby_ants[nearby_i].ant != nullptr) {
			nearby_i++;
		}
		if (nearby_i > nearby_top) {
			nearby_top = nearby_i;
		}
	}
}

void AAntAI::remove_nearby_ant(AAnt* ant) {
	for (int i = 0; i < nearby_top; i++) {
		if (nearby_ants[i].ant == ant) {
			nearby_ants[i].ant = nullptr;
			if (i < nearby_i) {
				nearby_i = i;
			}
			if (nearby_top == i + 1) {
				do {
					nearby_top--;
				}
				while (nearby_top > 0 && nearby_ants[nearby_top - 1].ant == nullptr);
			}
		}
	}	
}

int AAntAI::find_nearby_ant(AAnt* ant) const {
	for (int i = 0; i < nearby_top; i++) {
		const auto& nearby_ant = nearby_ants[i];
		if (nearby_ant.ant == ant) {
			return i;
		}
	}
	return -1;
}

AAntAI::AAntAI() {
	dispatch = nullptr;
	collis_box = CreateDefaultSubobject<UBoxComponent>("collis box");
	collis_box->SetupAttachment(skelly_mesh);
	status = AI_STATUS_INIT_PATH;
	path_incomplete = false;
	breadcrumb_ctr = 0.0f;
	prev_waypoint_breadcrumb_ct = 0;
	ant_ahead = false;
	stuck_check_i = 0;
	stuck_check_ctr = 0.0f;
}

void AAntAI::Tick(float delta_time) {
	set_true_move(delta_time);
	Super::Tick(delta_time);
	cling_smooth_rotate(delta_time);
}

void AAntAI::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) {
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

// does *this* ant have avoid responsibility in potential collision with other_ant?
bool AAntAI::has_avoid_responsibility(AAnt* other_ant) {
	const int i = find_nearby_ant(other_ant);
	if (i >= 0) {
		return nearby_ants[i].collision_responsibility;
	}
	return false;
}

const FVector& AAntAI::get_destination() const {
	return destination;
}

void AAntAI::BeginPlay() {
	Super::BeginPlay();
	collis_box->OnComponentBeginOverlap.AddDynamic(this, &AAntAI::on_collis_box_overlap_begin);
	collis_box->OnComponentEndOverlap.AddDynamic(this, &AAntAI::on_collis_box_overlap_end);
	sq_capsule_full_height = capsule_full_height * capsule_full_height;
	avoid_tick_ctr = FMath::RandRange(0, AVOID_TICK_MAX - 1);
	avoidance_yaw_half_theta = 0.0f;
	jammed_time = 0.0f;
	unjam_time = 0.0f;
	prev_yaw = 0.0f;
	acceptance_radius = capsule_full_height * 1.5f;
	breadcrumb_reset();
}

void AAntAI::set_true_move(float delta_time) {
	const FVector& prev_true_move = true_move;
	true_move = FVector::ZeroVector;
	if (status == AI_STATUS_INIT_PATH) {
		// comm::print("%s init path", TCHAR_TO_ANSI(*this->GetName()));
		if (!path_incomplete) {
			destination = dispatch->test_get_destination();
		}	
		bool pathing = false;
		if (dispatch->get_path(this, destination, pathing)) {
			path_incomplete = false;
			if (pathing) {
				status = AI_STATUS_PATHING;
				cur_waypoint = GetActorLocation();
				stuck_check_ctr = 0.0f;
				stuck_check_i = 0;
			}
			else {
				status = AI_STATUS_WAITING;
			}
		}
	}
	else if (status == AI_STATUS_WAITING) {
		// TODO: frozen ants are not waiting
		DISPATCH_STATUS dstatus = dispatch->get_pathfinding_status(this, path_incomplete);
		if (dstatus == DISPATCH_FAILED) {
			status = AI_STATUS_INIT_PATH;
		}
		else if (dstatus == DISPATCH_READY) {
			status = AI_STATUS_PATHING;
			cur_waypoint = GetActorLocation();
			stuck_check_ctr = 0.0f;
			stuck_check_i = 0;
		}
	}
	else if (status == AI_STATUS_PATHING) {
		const FVector cur_loc = forward_positions[0];
		if (FVector::Distance(cur_loc, cur_waypoint) < acceptance_radius) {
			const FVector offset = get_mesh_up();
			const FVector tr_start = cur_loc + offset;
			const FVector tr_end = cur_waypoint + offset;
			if (
				comm::trace_hit_static_actor(tr_start, tr_end) > 0.0f
				|| comm::trace_hit_static_actor(tr_end, tr_start) > 0.0f
			) {
				goto PATH_KEEP_MOVING;
			}
			breadcrumb_reset();
			const FVector* next_waypoint = dispatch->get_next_waypoint(this);
			if (next_waypoint == nullptr) {
				status = AI_STATUS_INIT_PATH;
			}
			else {
				cur_waypoint = *next_waypoint;
				goto PATH_KEEP_MOVING;
			}
		}
		else {
			PATH_KEEP_MOVING:

			// stuck_check_ctr += delta_time;
			// if (stuck_check_ctr >= STUCK_CHECK_TIME) {
			// 	stuck_check_ctr = 0;
			// 	if (stuck_check_i < 0 || stuck_check_i >= STUCK_CHECK_CT) {
			// 		// BUG: only occasionally running into this issue where stuck_check_i *appears* to be
			// 		// erroneously overwritten with what *appears* to be a float value near zero. With the given
			// 		// compiler, stuck_check_i comes after stuck_check_cache and before stuck_check_ctr. It doesn't
			// 		// seem like a buffer overflow from stuck_check_cache, since none of the position values are
			// 		// near zero in this test. I don't know why stuck_check_ctr would be the culprit; it's possible
			// 		// both are being overwritten and stuck_check_ctr >= STUCK_CHECK_TIME still functions semi-correctly
			// 		stuck_check_i = 0;
			// 		comm::print("stuck check i fixed");
			// 	}
			// 	stuck_check_cache[stuck_check_i++] = cur_loc;
			// 	if (stuck_check_i == STUCK_CHECK_CT) {
			// 		stuck_check_i = 0;
			// 		const FVector start_loc = stuck_check_cache[0];
			// 		float avg_dist = 0.0f;
			// 		for (int i = 1; i < STUCK_CHECK_CT; i++) {
			// 			avg_dist += FVector::Distance(start_loc, stuck_check_cache[i]);
			// 		}
			// 		avg_dist *= STUCK_CHECK_AVG_CONST;
			// 		if (avg_dist <= STUCK_AVG_DIST) {
			// 			comm::print("stuck, repathing");
			// 			status = AI_STATUS_INIT_PATH;
			// 			path_incomplete = false;
			// 			return;
			// 		}
			// 	}
			// }

			const FVector intended_move = calculate_intended_move(cur_loc);
			if (unjam) {
				true_move = intended_move;
				unjam_time += delta_time;
				if (unjam_time >= JAMMED_MAX) {
					unjam_time = 0.0f;
					unjam = false;
				}
			}
			else {
				avoid_tick_ctr++;
				if (avoid_tick_ctr == AVOID_TICK_MAX) {
					avoid_collisions(delta_time);
					avoid_tick_ctr = 0;
				}
				if (!ant_ahead) {
					if (avoidance_yaw_half_theta != 0.0f) {
						true_move = avoidance_rot.RotateVector(intended_move);
					}
					else {
						true_move = intended_move;
					}
				}
			}
			calculate_rot_speed_penalty(prev_true_move);
			drop_breadcrumbs(delta_time);	
		}
	}
	else { // RETRACING STEPS due to nearly colliding with a static mesh
		retrace_time += delta_time;
		// wait at the head of each step retrace to appear to be thinking about how to retrace steps
		if (retrace_time > RETRACE_WAIT_TIME) {
			const FVector cur_loc = forward_positions[0];
			if (FVector::Distance(cur_loc, cur_waypoint) < acceptance_radius) {
				const FVector offset = get_mesh_up();
				const FVector tr_start = cur_loc + offset;
				const FVector tr_end = cur_waypoint + offset;
				if (
					comm::trace_hit_static_actor(tr_start, tr_end) > 0.0f
					|| comm::trace_hit_static_actor(tr_end, tr_start) > 0.0f
				) {
					goto RETRACE_KEEP_MOVING;
				}
				step_retrace_i++;
				if (step_retrace_i >= step_retrace_top) {
					status = AI_STATUS_PATHING;
					cur_waypoint = save_waypoint;
				}
				else {
					cur_waypoint = step_retrace[step_retrace_i];
					goto RETRACE_KEEP_MOVING;
				}
			}
			else {
				RETRACE_KEEP_MOVING:
				true_move = calculate_intended_move(cur_loc);	
			}
			calculate_rot_speed_penalty(prev_true_move);
		}
	}
	move_mag = true_move.Size();
}

void AAntAI::cling_smooth_rotate(float delta_time) {
	Super::cling_smooth_rotate(delta_time);
}

void AAntAI::reset_nav() {
	status = AI_STATUS_INIT_PATH;
	path_incomplete = false;
}

void AAntAI::avoid_collisions(float delta_time) {
	// *try* to avoid collisions. just do your best, little ant.

	const FVector& cur_loc = forward_positions[0];
	no_accel = false;
	bool added_steering = false;

	FVector tr_start = cur_loc + get_mesh_up();
	FVector tr_end = tr_start + get_mesh_forward() * capsule_half_height * 2.0f;
	if (comm::trace_hit_nonground_static_actor(tr_start, tr_end) >= 0.0f) {
		set_step_retrace();
		save_waypoint = cur_waypoint;
		cur_waypoint = step_retrace[0];
		unjam_time = 0.0f;
		retrace_time = 0.0f;
		avoidance_yaw_half_theta = 0.0f;
		no_accel = true;
		status = AI_STATUS_RETRACING;
		return;
	}
	tr_start += get_mesh_forward() * capsule_half_height * 0.98f;
	tr_end = tr_start + get_mesh_forward() * capsule_half_height * 0.1f;
	if (comm::trace_hit_ant(tr_start, tr_end, this) >= 0.0f) {
		const bool jammed = deal_with_jams(delta_time);
		if (jammed) {
			ant_ahead = false;
		}
		else {
			no_accel = true;
			ant_ahead = true;
		}
		return;
	}
	ant_ahead = false;
	
	set_steering_tr_starts();
	for (int i = 0; i < nearby_top; i++) {
		const NearbyAnt& nearby_ant = nearby_ants[i];
		if (nearby_ant.ant == nullptr) {
			continue;
		}
		if (nearby_ant.collision_responsibility) {
			AAnt* other_ant = nearby_ant.ant;
			const FVector oa_loc = other_ant->GetActorLocation();
			const int collis_val = possible_collision(other_ant);
			if (!collis_val) {
				continue;
			}
			const FVector ant_dir = (oa_loc - cur_loc).GetSafeNormal();
			const float dir_theta = FMath::Acos(FVector::DotProduct(get_mesh_right(), ant_dir));
			no_accel = true;
			
			if (collis_val == 1 && ground_speed > THREE_QUARTER_MAX_GROUND_SPEED) {
				ground_speed = THREE_QUARTER_MAX_GROUND_SPEED;
			}
			if (dir_theta > HALF_PI) {
				// to the right
				if (avoidance_yaw_half_theta >= 0.0f && avoidance_yaw_half_theta < AYHT_MAX) {
					avoidance_yaw_half_theta += AYHT_STEP * delta_time;
					added_steering = true;
				}
				// else if (ground_speed > THREE_QUARTER_MAX_GROUND_SPEED) {
				// 	ground_speed = THREE_QUARTER_MAX_GROUND_SPEED;
				// }
			}
			else {
				// to the left, to the left
				if (avoidance_yaw_half_theta <= 0.0f && avoidance_yaw_half_theta > AYHT_MIN) {
					avoidance_yaw_half_theta -= AYHT_STEP * delta_time;
					added_steering = true;
				}
				// else if (ground_speed > THREE_QUARTER_MAX_GROUND_SPEED) {
				// 	ground_speed = THREE_QUARTER_MAX_GROUND_SPEED;
				// }
			}
		}
		else if (possible_collision(nearby_ant.ant) == 1 && ground_speed > THREE_QUARTER_MAX_GROUND_SPEED) {
			no_accel = true;
			ground_speed = THREE_QUARTER_MAX_GROUND_SPEED;
		}
	}
	if (no_accel) {
		const bool jammed = deal_with_jams(delta_time);
		if (jammed) {
			return;
		}
	}
	else {
		jammed_time = 0.0f;
	}

	bool set_quat = true;
	if (!added_steering) {
		set_quat = center_correct_ayht(delta_time);	
	}
	if (set_quat) {
		set_avoidance_rot();	
	}
}


int AAntAI::possible_collision(AAnt* other_ant) const {
	// dead simple occasionally correct future collision check
	// avoiding sphere intersection math
	const FVector* oa_forward_positions = other_ant->get_forward_positions();
	for (int i = 0; i < FORWARD_POSITION_CT; i++) {
		for (int j = 0; j < FORWARD_POSITION_CT; j++) {
			if (FVector::DistSquared(oa_forward_positions[j], forward_positions[i]) < sq_capsule_full_height) {
				return i + 1;
			}
		}
	}
	return 0;
}

void AAntAI::set_steering_tr_starts() {
	const FVector& cur_loc = forward_positions[0];
	const FVector true_move_right_scaled = FVector::CrossProduct(true_move, get_mesh_up()) * capsule_radius;
	const FVector true_move_scaled = true_move * capsule_half_height;
	steering_tr_starts[0] = cur_loc + true_move_right_scaled + true_move_scaled;
	steering_tr_starts[1] = cur_loc - true_move_right_scaled + true_move_scaled;
}

bool AAntAI::center_correct_ayht(float delta_time) {
	bool set_quat = true;
	if (avoidance_yaw_half_theta > 0.0f) {
		avoidance_yaw_half_theta -= AYHT_STEP * delta_time;
	}
	else if (avoidance_yaw_half_theta < 0.0f) {
		avoidance_yaw_half_theta += AYHT_STEP * delta_time;
	}
	
	if (avoidance_yaw_half_theta == 0.0f) {
		set_quat = false;
	}
	else if( FMath::Abs(avoidance_yaw_half_theta) < AYHT_ZERO_EPSILON * delta_time) {
		avoidance_yaw_half_theta = 0.0f;
		set_quat = false;
	}
	return set_quat;
}

void AAntAI::set_avoidance_rot() {
	const FVector rot_axis_adj = get_mesh_up() * FMath::Sin(avoidance_yaw_half_theta);
	avoidance_rot.X = rot_axis_adj.X;
	avoidance_rot.Y = rot_axis_adj.Y;
	avoidance_rot.Z = rot_axis_adj.Z;
	avoidance_rot.W = FMath::Cos(avoidance_yaw_half_theta);
}

void AAntAI::nonground_collis_check() {
	
}

void AAntAI::set_step_retrace() {
	int j = breadcrumb_i == 0 ? BREADCRUMB_CT - 1 : breadcrumb_i - 1;
	step_retrace_top = prev_waypoint_breadcrumb_ct < BREADCRUMB_CT ? prev_waypoint_breadcrumb_ct : BREADCRUMB_CT;
	for (int i = 0; i < prev_waypoint_breadcrumb_ct; i++, j++) {
		if (j == BREADCRUMB_CT) {
			j = 0;
		}
		step_retrace[i] = breadcrumbs[j];
	}
	step_retrace_i = 0;
}

void AAntAI::calculate_rot_speed_penalty(const FVector& prev_true_move) {
	// speed penalty for faster rotation
	const float true_move_theta_delta = FMath::Acos(FVector::DotProduct(true_move, prev_true_move));
	if (true_move_theta_delta > 0.0f) {
		const float delta_ratio = TMTD_MAX / true_move_theta_delta;
		if (delta_ratio < 1.0f) {
			rot_speed_multiplier = ROT_SPEED_MULTIPLIER_MIN;
		}
		else {
			// sigmoid
			rot_speed_multiplier = 1.0f / (1.0f + FMath::Exp(-(delta_ratio-2.0f)));
		}
	}
	else {
		rot_speed_multiplier = 1.0f;
	}
}

void AAntAI::drop_breadcrumbs(float delta_time) {
	breadcrumb_ctr += delta_time;
	if (breadcrumb_ctr > BREADCRUMB_DT) {
		breadcrumb_ctr = 0.0f;
		prev_waypoint_breadcrumb_ct++;
		breadcrumbs[breadcrumb_i++] = GetActorLocation();
		if (breadcrumb_i >= BREADCRUMB_CT) {
			breadcrumb_i = 0;
		}
	}
}

void AAntAI::breadcrumb_reset() {
	breadcrumb_ctr = 0.0f;
	prev_waypoint_breadcrumb_ct = 1;
	breadcrumbs[breadcrumb_i++] = GetActorLocation();
	if (breadcrumb_i >= BREADCRUMB_CT) {
		breadcrumb_i = 0;
	}
}

FVector AAntAI::calculate_intended_move(const FVector& cur_loc) const {
	const FVector intended_move = (cur_waypoint - cur_loc).GetSafeNormal();
	const FVector mesh_up = get_mesh_up();
	const float half_mesh_plane_theta =
		(HALF_PI - FMath::Acos(FVector::DotProduct(mesh_up, intended_move))) * 0.5f;
	const float angle_sin = FMath::Sin(half_mesh_plane_theta);
	const FVector rot_axis = FVector::CrossProduct(mesh_up, intended_move).GetSafeNormal();
	const FVector rot_axis_and_angle = rot_axis * angle_sin;
	FQuat rot_quat(
		rot_axis_and_angle.X,
		rot_axis_and_angle.Y,
		rot_axis_and_angle.Z,
		FMath::Cos(half_mesh_plane_theta)
	);
	rot_quat.Normalize();
	return rot_quat.RotateVector(intended_move);
}

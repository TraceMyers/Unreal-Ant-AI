#include "Ant.h"
#include "AntAI.h"
#include "AntGI.h"
#include "AntAIDispatch.h"
#include "AntGMB.h"
#include "CollisionDebugDrawingPublic.h"
#include "Common.h"
#include "DrawDebugHelpers.h"
#include "OrientationComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Engine/StaticMeshActor.h"

// TODO: propagate rotations of blueprint actor in world down to proper components and un-rotate actor

AAnt::AAnt() {
	PrimaryActorTick.bCanEverTick = true;
	USceneComponent* root = CreateDefaultSubobject<USceneComponent>(FName("root"));
	SetRootComponent(root);
	skelly_mesh = CreateDefaultSubobject<USkeletalMeshComponent>(FName("skeletal mesh"));
	skelly_mesh->SetupAttachment(root);
	mesh_cling = CreateDefaultSubobject<USceneComponent>(FName("cling component"));
	mesh_cling->SetupAttachment(skelly_mesh);
	cling_box = CreateDefaultSubobject<UBoxComponent>(FName("cling box"));
	cling_box->SetupAttachment(skelly_mesh);
	orientation_cmp = CreateDefaultSubobject<UOrientationComponent>(FName("orientation component"));
	AddOwnedComponent(orientation_cmp);
	GetCapsuleComponent()->SetupAttachment(skelly_mesh);

	rot_speed_multiplier = 1.0f;
	no_accel = false;
	fall_velocity = FVector::ZeroVector;
	fall_init();
	GetCapsuleComponent()->SetSimulatePhysics(false);
	skelly_mesh->SetSimulatePhysics(false);
	cling_box->SetSimulatePhysics(false);
}

AAnt::~AAnt() {}

void AAnt::BeginPlay() {
	Super::BeginPlay();
	GetMovementComponent()->PrimaryComponentTick.bCanEverTick = false;
	game_instance = Cast<UAntGI>(GetGameInstance());
	AAntGMB* ant_gmb = Cast<AAntGMB>(GetWorld()->GetAuthGameMode());
	capsule_radius = GetCapsuleComponent()->GetScaledCapsuleRadius();
	on_ground_tolerance = capsule_radius * 0.05;
	capsule_radius_with_tolerance = capsule_radius + on_ground_tolerance;
	capsule_radius_less_tolerance = capsule_radius - on_ground_tolerance;
	capsule_half_height = GetCapsuleComponent()->GetScaledCapsuleHalfHeight();
	mesh_cling->SetWorldRotation(skelly_mesh->GetComponentRotation());
	orientation_cmp->init(ant_gmb->get_tri_grid(), capsule_radius, capsule_half_height);
	capsule_full_height = capsule_half_height * 2.0f;
	
	GetCapsuleComponent()->OnComponentBeginOverlap.AddDynamic(this, &AAnt::on_capsule_overlap_begin);
	GetCapsuleComponent()->OnComponentEndOverlap.AddDynamic(this, &AAnt::on_capsule_overlap_end);
}

void AAnt::set_true_move(float delta_time) {
	true_move = FVector::ZeroVector;
}

FVector AAnt::get_mesh_right() const {
	return -skelly_mesh->GetForwardVector();
}

FVector AAnt::get_mesh_up() const {
	return skelly_mesh->GetUpVector();	
}

FVector AAnt::get_mesh_forward() const {
	return skelly_mesh->GetRightVector();	
}


void AAnt::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	resolve_overlaps(DeltaTime);	
	if (on_ground) {
		move(DeltaTime);
	}
	else {
		fall(DeltaTime);
	}
	fix_fall_through();
}

void AAnt::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) {
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void AAnt::move(float delta_time) {
	if (move_mag > 0.0f) {
		if (move_mag < 0.6f) {
			move_mag = 0.6f;
		}
		else if (move_mag > 1.0f) {
			move_mag = 1.0f;
		}
		prev_move_mag = move_mag;
		
		const FVector move = true_move * move_mag * ground_speed * delta_time;
		if (try_move_on_ground(move, delta_time)) {
			rotate_while_moving(delta_time, true_move);
		}
	}
	else if (ground_speed > 0.0f) {
		const FVector move = true_move * prev_move_mag * ground_speed * delta_time;
		if (try_move_on_ground(move, delta_time, false)) {
			rotate_while_moving(delta_time, true_move);
			if (ground_speed == 0.0f) {
				direction = 0.0f;
			}
		}
	}
}

void AAnt::rotate_while_moving(float delta_time, const FVector& move) {
	const FVector mesh_forward = get_mesh_forward();
	const FVector mesh_right = get_mesh_right();
	const float sign = FMath::Sign(FVector::DotProduct(move, mesh_right));
	const float dir_theta = FVector::DotProduct(move, mesh_forward);
	const float yaw_theta_adj = FMath::Acos(dir_theta) * sign * 0.5 * BODY_ROT_SPEED;
	if (FMath::Abs(yaw_theta_adj) > FLOAT_EPSILON) {
		direction = dir_theta * sign * 180.0f / PI;
		const FVector rot_axis_and_angle = get_mesh_up() * FMath::Sin(yaw_theta_adj);
		FQuat yaw_rot(
			rot_axis_and_angle.X,
			rot_axis_and_angle.Y,
			rot_axis_and_angle.Z,
			FMath::Cos(yaw_theta_adj)
		);
		yaw_rot.Normalize();	
		skelly_mesh->SetRelativeRotation(yaw_rot * skelly_mesh->GetRelativeRotation().Quaternion());
	}

	const FVector ant_up = mesh_cling->GetUpVector();
	FQuat cling_rot = mesh_cling->GetComponentQuat();
	orientation_cmp->get_new_orientation(GetActorLocation(), ant_up, cling_rot, true_move);
	mesh_cling->SetWorldRotation(cling_rot);
}

void AAnt::set_on_ground() {
	const FVector cur_loc = GetActorLocation();
	const float hit_dist = comm::trace_hit_ground(
		cur_loc, cur_loc - get_mesh_up() * capsule_radius_with_tolerance, this
	);
	if (hit_dist <= capsule_radius_less_tolerance) {
		in_ground = true;
		on_ground = false;
	}
	else if (hit_dist > 0.0f) {
		in_ground = false;
		on_ground = true;
	}
	else {
		fall_init();
	}
}

// Gravity currently unnecessary since ants only detach from mesh surface by error. So, just using this to
// push ant away from their mesh's up vector for convenient use with fix_fall_through()
void AAnt::fall(float delta_time) {
	// fall_velocity.Z -= GRAVITY_ACCEL * delta_time;
	// const FVector fall_delta = fall_velocity * delta_time;
	// const FVector cur_loc = GetActorLocation();
	// const float hit_dist = comm::trace_hit_ground(cur_loc, cur_loc + fall_delta, this);
	// if (hit_dist > 0.0f) {
	// 	AddActorWorldOffset(-FVector::UpVector * hit_dist);
	// 	on_ground = true;
	// }
	// else {
	// 	AddActorWorldOffset(fall_delta);
	// }
	const FVector mesh_up = get_mesh_up();
	fall_velocity.Z -= GRAVITY_ACCEL * delta_time;
	const FVector fall_delta = mesh_up * (fall_velocity.Z * delta_time);
	const FVector cur_loc = GetActorLocation();
	const float hit_dist = comm::trace_hit_ground(cur_loc, cur_loc + fall_delta, this);
	if (hit_dist > 0.0f) {
		AddActorWorldOffset(-mesh_up * hit_dist);
		on_ground = true;
	}
	else {
		AddActorWorldOffset(fall_delta);
	}
}

void AAnt::fall_init() {
	fall_velocity.Z = -capsule_radius_with_tolerance;
	on_ground = false;
	in_ground = false;
}

bool AAnt::try_move_on_ground(const FVector& move, float delta_time, bool accel) {
	const FVector cur_loc = GetActorLocation();
	const FVector mesh_up = get_mesh_up();
	FVector tr_start = cur_loc + mesh_up * capsule_radius;
	const FVector move_dir_radius = true_move * capsule_radius * 0.5f;
	// const float forward_hit_dist = comm::trace_hit_ground(tr_start, tr_start + move_dir_radius, this);
	const float forward_hit_dist = -1.0f;
	if (forward_hit_dist == -1.0f) {
		const FVector vertical_disp = mesh_up * capsule_radius * 3.0f;
		const FVector vertical_tolerance = -vertical_disp * 3.0f;
		tr_start = cur_loc + move * rot_speed_multiplier + vertical_disp;
		const float drop_hit_dist = comm::trace_hit_ground(tr_start, tr_start + vertical_tolerance);
		if (drop_hit_dist > 0.0f) {
			if (accel) {
				if (ground_speed < MAX_GROUND_SPEED && !no_accel) {
					ground_speed += GROUND_ACCEL * delta_time;
					if (ground_speed > MAX_GROUND_SPEED) {
						ground_speed = MAX_GROUND_SPEED;
					}
				}
			}
			else { // de-accel
				ground_speed -= GROUND_ACCEL * delta_time;
				if (ground_speed < 0.0f) {
					ground_speed = 0.0f;
				}
			}
			SetActorLocation(tr_start - mesh_up * drop_hit_dist);
			return true;
		}
	}
	ground_speed = 0.0f;
	return false;
}

void AAnt::cling_smooth_rotate(float delta_time) {
	const FVector mesh_up = get_mesh_up();
	const FVector mesh_cling_up = mesh_cling->GetUpVector();
	smooth_rot_half_theta = FMath::Acos(FVector::DotProduct(mesh_up, mesh_cling_up)) * 0.5f;
	if (smooth_rot_half_theta > ROT_EPSILON) {
		float rot_delta = CLING_ROT_SPEED * 3.0f * delta_time * smooth_rot_half_theta;
		if (rot_delta > smooth_rot_half_theta) {
			rot_delta = smooth_rot_half_theta;
		}
		const float cos_half_socket_rot_delta = FMath::Cos(rot_delta);
		const float sin_half_socket_rot_delta = FMath::Sin(rot_delta);
		const FVector mesh_rot_axis_adj =
			FVector::CrossProduct(mesh_up, mesh_cling_up) * sin_half_socket_rot_delta;
		FQuat rot_quat (
			mesh_rot_axis_adj.X,
			mesh_rot_axis_adj.Y,
			mesh_rot_axis_adj.Z,
			cos_half_socket_rot_delta
		);
		skelly_mesh->SetWorldRotation(rot_quat * skelly_mesh->GetComponentQuat());
		const FVector cling_rot_axis_adj =
			FVector::CrossProduct(mesh_cling_up, mesh_up) * sin_half_socket_rot_delta;
		rot_quat = FQuat(
			cling_rot_axis_adj.X,
			cling_rot_axis_adj.Y,
			cling_rot_axis_adj.Z,
			cos_half_socket_rot_delta
		);
		mesh_cling->SetWorldRotation(rot_quat * mesh_cling->GetComponentQuat());
	}
}

void AAnt::dbg_draw_true_move() {
	const FVector start_loc = GetActorLocation() + get_mesh_up() * capsule_radius * 2.0f + 1.0f;
	const FVector end_loc = start_loc + true_move * 8.0f;	
	DrawDebugLine(GetWorld(), start_loc, end_loc, FColor::Red, false, -1, 0, 1.0f);
}

void AAnt::fix_fall_through() {
	if (forward_positions[0].Z <= 0.0f) {
		SetActorLocation(FVector(1500.0f, -1200.0f, 300.0f));
		skelly_mesh->SetWorldRotation(FRotator(180.0f, 0.0f, 0.0f));
		fall_init();
		AAntAI* self_ai = Cast<AAntAI>(this);
		if (self_ai) {
			self_ai->reset_nav();
		}
	}	
}

float AAnt::get_ground_speed() const {
	return ground_speed;
}

const FVector& AAnt::get_true_move() {
	return true_move;
}

// called by AntAIDispatch; as of 9/11/2022, only if we are an AAntAI or AAntPlayer
// NOTE: would be better to incorporate waypoints if collisions start to look bad; maybe have AAntAI override
void AAnt::update_forward_positions() {
	const FVector cur_loc = GetActorLocation();
	const FVector move_vec = true_move;
	for (int i = 0; i < FORWARD_POSITION_CT; i++) {
		forward_positions[i] = cur_loc + move_vec * capsule_full_height * i;
	}
}

const FVector* AAnt::get_forward_positions() {
	return forward_positions;
}

bool AAnt::has_avoid_responsibility(AAnt* other_ant) {
	return false;
}

void AAnt::set_dispatch(AAntAIDispatch* _dispatch) {
	dispatch = _dispatch;
	// making sure dispatch's tick is called before ours so update_forward_positions() is called before AI collis checks
	AddTickPrerequisiteActor(dispatch); 
}

void AAnt::resolve_overlaps(float delta_time) {
	const FVector mesh_up = get_mesh_up();
	const FVector vertical_disp = mesh_up * capsule_radius * 3.0f;
	const FVector vertical_tolerance = -vertical_disp * 3.0f;
	const float nudge_dist = ground_speed * delta_time;
	const float fault_speed_val = ground_speed / MAX_GROUND_SPEED;
	
	for (int i = 0; i < overlapping_ants.Num(); i++) {
		AAnt* a = overlapping_ants[i];
		const bool still_overlapping = GetCapsuleComponent()->IsOverlappingComponent(Cast<AAnt>(a)->GetCapsuleComponent());
		if (still_overlapping) {
			const FVector cur_loc = GetActorLocation();
			const FVector diff_dir = (cur_loc - a->GetActorLocation()).GetSafeNormal();
			
			// resolving ant-ant collisions by pushing them outside of each other, dependent on velocity vectors
			const float fault_theta_val = FMath::Abs( HALF_PI - FMath::Acos(FVector::DotProduct(diff_dir, get_mesh_forward()))) / HALF_PI;
			const FVector tr_start = cur_loc + diff_dir * nudge_dist * fault_theta_val * fault_speed_val + vertical_disp;
			
			const float hit_dist = comm::trace_hit_ground(tr_start, tr_start + vertical_tolerance, this);
			if (hit_dist > 0.0f) {
				SetActorLocation(tr_start - mesh_up * hit_dist);
			}
		}
	}

	for (int i = 0; i < overlapping_meshes.Num(); i++) {
		AStaticMeshActor* m = overlapping_meshes[i];
		for (int j = 0; j < 2; j++) {
			const bool still_overlapping = GetCapsuleComponent()->IsOverlappingActor(m);
			if (still_overlapping) {
				const FVector cur_loc = GetActorLocation();
				const FVector diff_dir = (cur_loc - m->GetActorLocation()).GetSafeNormal();
				const FVector tr_start = cur_loc + diff_dir * nudge_dist + vertical_disp;
				const float hit_dist = comm::trace_hit_ground(tr_start, tr_start + vertical_tolerance, this);
				if (hit_dist > 0.0f) {
					SetActorLocation(tr_start - mesh_up * hit_dist);
				}
			}
		}
	}
}

void AAnt::on_capsule_overlap_begin(
	UPrimitiveComponent* overlapped_comp,
	AActor* other_actor,
	UPrimitiveComponent* other_comp,
	int32 other_body_index,
	bool from_sweep,
	const FHitResult& sweep_result
) {
	AStaticMeshActor* static_mesh = Cast<AStaticMeshActor>(other_actor);
	if (static_mesh) {
		if (!other_actor->ActorHasTag(FName("ground"))) {
			if (overlapping_meshes.Find(static_mesh) >= 0) {
				return;
			}
			overlapping_meshes.Add(static_mesh);
		}
		return;
	}
	AAnt* ant = Cast<AAnt>(other_actor);
	if (ant) {
		if (overlapping_ants.Find(ant) >= 0 || ant == this) {
			return;
		}
		overlapping_ants.Add(ant);
	}
}

void AAnt::on_capsule_overlap_end(
	UPrimitiveComponent* overlapped_comp,
	AActor* other_actor,
	UPrimitiveComponent* other_comp,
	int32 other_body_index
) {
	AStaticMeshActor* static_mesh = Cast<AStaticMeshActor>(other_actor);
	if (static_mesh) {
		if (!other_actor->ActorHasTag(FName("ground"))) {
			overlapping_meshes.Remove(static_mesh);
		}
		return;
	}
	
	AAnt* ant = Cast<AAnt>(other_actor);
	if (ant) {
		overlapping_ants.Remove(ant);
	}
}

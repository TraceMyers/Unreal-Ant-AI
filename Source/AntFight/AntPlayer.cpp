#include "AntPlayer.h"

#include "AntGMB.h"
#include "Line.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Kismet/GameplayStatics.h"

// TODO: propagate rotations of blueprint actor in world down to proper components and un-rotate actor (??)

AAntPlayer::AAntPlayer() {
	PrimaryActorTick.bCanEverTick = true;
	USceneComponent* root = GetRootComponent();
	spring_arm_socket = CreateDefaultSubobject<USceneComponent>(FName("spring arm socket"));
	spring_arm_socket->SetupAttachment(root);
	spring_arm = CreateDefaultSubobject<USpringArmComponent>(FName("spring arm"));
	spring_arm->SetupAttachment(spring_arm_socket);
	cam_follow = CreateDefaultSubobject<USceneComponent>(FName("cam follow"));
	cam_follow->SetupAttachment(spring_arm);
	camera = CreateDefaultSubobject<UCameraComponent>(FName("camera"));
	camera->SetupAttachment(root);
	
	pitch_correction = false;
	arm_correction = false;
}

void AAntPlayer::BeginPlay() {
	Super::BeginPlay();
	arm_default_tilt = spring_arm->GetRelativeRotation().Pitch;
	arm_tilt_upper_max_rel = ARM_TILT_UPPER_MAX + arm_default_tilt;
	arm_tilt_lower_max_rel = ARM_TILT_LOWER_MAX + arm_default_tilt;
	spring_arm_default_length = spring_arm->TargetArmLength / 2.0f;
	cam_default_height = camera->GetRelativeLocation().Z;
	spring_arm->TargetArmLength = SPRING_ARM_TARGET_LEN;
}

void AAntPlayer::set_true_move(float delta_time) {
	move_mag = move_buf.Size2D();
	if (move_mag > 0.0f) {
		// finding the camera's forward vector rotated onto the mesh's plane
		const FVector cam_forward = camera->GetForwardVector();
		const FVector mesh_up = get_mesh_up();
		const float cam_tilt_mesh_rel = HALF_PI - FMath::Acos(FVector::DotProduct(mesh_up, cam_forward));
		const float cam_adj = cam_tilt_mesh_rel * 0.5f;
		const float angle_sin = FMath::Sin(cam_adj);
		// const FVector rot_axis = FVector::CrossProduct(mesh_up, cam_forward).GetSafeNormal();
		const FVector rot_axis = camera->GetRightVector();
		FVector rot_axis_and_angle = rot_axis * angle_sin;
		FQuat rot_quat(
			rot_axis_and_angle.X,
			rot_axis_and_angle.Y,
			rot_axis_and_angle.Z,
			FMath::Cos(cam_adj)
		);
		rot_quat.Normalize();
		const FVector cam_forward_mesh_rel = rot_quat.RotateVector(cam_forward);

		// rotating vector with movement input
		const float move_theta_adj = FMath::Atan2(move_buf.Y, move_buf.X) * 0.5f;
		rot_axis_and_angle = mesh_up * FMath::Sin(move_theta_adj);
		rot_quat = FQuat(
			rot_axis_and_angle.X,
			rot_axis_and_angle.Y,
			rot_axis_and_angle.Z,
			FMath::Cos(move_theta_adj)
		);
		rot_quat.Normalize();
		true_move = rot_quat.RotateVector(cam_forward_mesh_rel);
	}
}

void AAntPlayer::Tick(float DeltaTime) {
	set_true_move(DeltaTime);
	Super::Tick(DeltaTime);
	free_look(DeltaTime);
	cling_smooth_rotate(DeltaTime);
	// AAntGMB* ant_gmb = Cast<AAntGMB>(GetWorld()->GetAuthGameMode());
	// ant_gmb->draw_navmesh(GetActorLocation());
}

void AAntPlayer::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) {
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	PlayerInputComponent->BindAction("Jump", IE_Pressed, this, &AAntPlayer::jump);
	PlayerInputComponent->BindAction("Attack", IE_Pressed, this, &AAntPlayer::attack);
	PlayerInputComponent->BindAxis("MoveFB", this, &AAntPlayer::move_fb);
	PlayerInputComponent->BindAxis("MoveLR", this, &AAntPlayer::move_lr);
	PlayerInputComponent->BindAxis("LookPan", this, &AAntPlayer::look_pan);
	PlayerInputComponent->BindAxis("LookTilt", this, &AAntPlayer::look_tilt);
}

void AAntPlayer::jump() {
	
}

void AAntPlayer::attack() {
	
}

void AAntPlayer::move_fb(float throttle) {
	move_buf.X = throttle;
}

void AAntPlayer::move_lr(float throttle) {
	move_buf.Y = throttle;
}

void AAntPlayer::look_pan(float throttle) {
	look_buf.X += throttle;
}

void AAntPlayer::look_tilt(float throttle) {
	look_buf.Y += throttle;
}

// TODO: redo pitch math so that no multiplication by 2 and 0.5 is necessary on pan & pitch
void AAntPlayer::free_look(float delta_time) {
	look_buf *= CAM_ROT_SPEED * delta_time;

	// TODO find common rotation axis and do pan & rot at same time
	FRotator new_spring_arm_rot = spring_arm->GetRelativeRotation();
	new_spring_arm_rot.Roll = 0.0f;
	
	// pan
	const float pan_vel = look_buf.X;
	if (pan_vel != 0.0f) {
		new_spring_arm_rot.Yaw += pan_vel;
	}
	
	// tilt
	const float tilt_vel = look_buf.Y;
	if (tilt_vel != 0.0f) {
		float new_pitch = new_spring_arm_rot.Pitch + tilt_vel;
		if (new_pitch < ARM_TILT_LOWER_MAX) {
			new_pitch = ARM_TILT_LOWER_MAX;
		}
		else if (new_pitch > ARM_TILT_UPPER_MAX) {
			new_pitch = ARM_TILT_UPPER_MAX;
		}
		new_spring_arm_rot.Pitch = new_pitch;
		cam_freelook_center_timer = 1.6f;
		pitch_correction = true;
	}
	else if (pitch_correction) {
		cam_freelook_center_timer -= delta_time;
		const float pitch_deviation = new_spring_arm_rot.Pitch - arm_default_tilt;
		if (cam_freelook_center_timer <= 0.0f) {
			 if (pitch_deviation * pitch_deviation > FLT_EPSILON) {
			 	const float sigmoid_val = 1.0f / (1.0f + FMath::Exp(-(cam_freelook_center_timer * 4.0f + 10.0f)));
			 	new_spring_arm_rot.Pitch = arm_default_tilt + pitch_deviation * sigmoid_val;
			}
			else {
				pitch_correction = false;
			}
		}
	}
	spring_arm->SetRelativeRotation(new_spring_arm_rot);
	move_camera();
	look_buf = FVector::ZeroVector;
}

// TODO: smooth in/out and better finish
void AAntPlayer::cling_smooth_rotate(float delta_time) {
	Super::cling_smooth_rotate(delta_time);

	const FVector mesh_up = get_mesh_up();
	const FVector spring_arm_sock_up = spring_arm_socket->GetUpVector();
	smooth_rot_half_theta = FMath::Acos(FVector::DotProduct(mesh_up, spring_arm_sock_up)) * 0.5f;
	if (smooth_rot_half_theta > SPRING_ARM_ROT_EPSILON) {
		float rot_delta = CLING_ROT_SPEED * 2.0f * delta_time * smooth_rot_half_theta;
		if (rot_delta > smooth_rot_half_theta) {
			rot_delta = smooth_rot_half_theta;
		}
		const float cos_half_socket_rot_delta = FMath::Cos(rot_delta);
		const float sin_half_socket_rot_delta = FMath::Sin(rot_delta);
		const FVector spring_arm_rot_axis_adj =
			FVector::CrossProduct(spring_arm_sock_up, mesh_up) * sin_half_socket_rot_delta;
		FQuat rot_quat (
			spring_arm_rot_axis_adj.X,
			spring_arm_rot_axis_adj.Y,
			spring_arm_rot_axis_adj.Z,
			cos_half_socket_rot_delta
		);
		spring_arm_socket->SetRelativeRotation(rot_quat * spring_arm_socket->GetRelativeRotation().Quaternion());
	}
}

void AAntPlayer::move_camera() {
	camera->SetWorldRotation(cam_follow->GetComponentRotation());
	camera->SetWorldLocation(cam_follow->GetComponentLocation());
}
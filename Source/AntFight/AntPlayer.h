#pragma once

#include "CoreMinimal.h"
#include "Ant.h"
#include "GameFramework/Character.h"
#include "AntPlayer.generated.h"

UCLASS()
class ANTFIGHT_API AAntPlayer : public AAnt {
	GENERATED_BODY()

public:
	
	AAntPlayer();
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void jump();
	void attack();
	void move_fb(float throttle);
	void move_lr(float throttle);
	void look_pan(float throttle);
	void look_tilt(float throttle);
	void free_look(float delta_time);
	void move_camera();
	
	UPROPERTY()
	USceneComponent* spring_arm_socket;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category="cpp")
	class UCameraComponent* camera;	
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category="cpp")
	class USpringArmComponent* spring_arm;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly)
	USceneComponent* cam_follow;
	
protected:
	
	virtual void BeginPlay() override;
	virtual void set_true_move(float delta_time) override;
	virtual void cling_smooth_rotate(float delta_time) override;

private:


	FVector move_buf;
	FVector look_buf;
	float cam_freelook_center_timer;
	float cam_freelook_centering_from_pitch_diff;
	float up_tilt_space;
	float down_tilt_space;
	float spring_arm_default_length;
	float arm_default_tilt;
	float cam_default_height;
	float arm_tilt_upper_max_rel;
    float arm_tilt_lower_max_rel;
	bool pitch_correction;
	bool arm_correction;

	static constexpr float ARM_TILT_UPPER_MAX = 89.0f;
	static constexpr float ARM_TILT_LOWER_MAX = -89.0f;
	static constexpr float CAM_ROT_SPEED = 25.0f;
	static constexpr float SPRING_ARM_ROT_EPSILON = 1e-3;
	static constexpr float SPRING_ARM_TARGET_LEN = 60.0f;
	static constexpr float CAMERA_COLLIS_TOLERANCE = 50.0f;

};

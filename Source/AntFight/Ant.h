#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Ant.generated.h"

UCLASS()
class ANTFIGHT_API AAnt : public ACharacter {
	GENERATED_BODY()

public:
	
	static constexpr int FORWARD_POSITION_CT = 3;
	
	AAnt();
	~AAnt();
	
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void move(float delta_time);
	void rotate_while_moving(float delta_time, const FVector& move_vec);
	void set_on_ground();
	void fall(float delta_time);
	void fall_init();
	bool try_move_on_ground(const FVector& move, float delta_time, bool accel=true);
    virtual bool has_avoid_responsibility(AAnt* other_ant);
	virtual float get_ground_speed() const;
    virtual const FVector& get_true_move();
	void update_forward_positions();
	const FVector* get_forward_positions();
	void set_dispatch(class AAntAIDispatch* dispatch);
	void resolve_overlaps(float delta_time);
	FVector get_mesh_right() const;
	FVector get_mesh_up() const;
	FVector get_mesh_forward() const;
	
	UFUNCTION()
	void on_capsule_overlap_begin(
		UPrimitiveComponent* overlapped_comp,
		AActor* other_actor,
		UPrimitiveComponent* other_comp,
		int32 other_body_index,
		bool from_sweep,
		const FHitResult& sweep_result
	);
	UFUNCTION()
	void on_capsule_overlap_end(
		class UPrimitiveComponent* overlapped_comp,
		class AActor* other_actor,
		class UPrimitiveComponent* other_comp,
		int32 other_body_index
	);
	
	UPROPERTY(BlueprintReadOnly)
	USceneComponent* mesh_cling;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly)
	USkeletalMeshComponent* skelly_mesh;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly)
	class UBoxComponent* cling_box;
	UPROPERTY()
	class UOrientationComponent* orientation_cmp;
	UPROPERTY(BlueprintReadWrite, VisibleInstanceOnly)
	float ground_speed;
	UPROPERTY(BlueprintReadWrite, VisibleInstanceOnly)
	float direction;
	
	class AAntAIDispatch* dispatch;
	class UAntGI* game_instance;
	
protected:
	
	static constexpr float BODY_ROT_SPEED = PI/40.0f;
	static constexpr float FLOAT_EPSILON = 1e-3;
	static constexpr float SLOWFALL_SPEED = 100.0f;
	static constexpr float GRAVITY_ACCEL = 980.0f;
	static constexpr float MAX_GROUND_SPEED = 110.0f;
	static constexpr float GROUND_ACCEL = 180.0f;
	static constexpr float CLING_ROT_SPEED = 6.0f;
	static constexpr float ROT_EPSILON = 1e-2;
	static constexpr float THREE_QUARTER_MAX_GROUND_SPEED = MAX_GROUND_SPEED * 0.75f;
    	
	FVector true_move;
	FVector fall_velocity;
	FVector forward_positions[FORWARD_POSITION_CT];
	bool on_ground;
	bool in_ground;
	bool no_accel;
	bool forward_positions_updated;
	float move_mag;
	float prev_move_mag;
	float capsule_radius;
	float capsule_radius_with_tolerance;
	float capsule_radius_less_tolerance;
	float on_ground_tolerance;
	float capsule_half_height;
	float smooth_rot_half_theta;
	float rot_speed_multiplier;
	float capsule_full_height;

	virtual void BeginPlay() override;
	virtual void set_true_move(float delta_time);
	virtual void cling_smooth_rotate(float delta_time);
	void dbg_draw_true_move();
	void fix_fall_through();
	
private:

	TArray<AAnt*> overlapping_ants;
	TArray<class AStaticMeshActor*> overlapping_meshes;

};


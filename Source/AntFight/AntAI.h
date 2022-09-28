#pragma once

#include "CoreMinimal.h"
#include "Ant.h"
#include "AntAI.generated.h"

UCLASS()
class ANTFIGHT_API AAntAI : public AAnt {
	GENERATED_BODY()

public:

	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly)
	UBoxComponent* collis_box;
	
	UFUNCTION()
	void on_collis_box_overlap_begin(
		UPrimitiveComponent* overlapped_comp,
		AActor* other_actor,
		UPrimitiveComponent* other_comp,
		int32 other_body_index,
		bool from_sweep,
		const FHitResult& sweep_result
	);
	UFUNCTION()
	void on_collis_box_overlap_end(
		class UPrimitiveComponent* overlapped_comp,
		class AActor* other_actor,
		class UPrimitiveComponent* other_comp,
		int32 other_body_index
	);
	
	
	enum AI_STATUS {
		AI_STATUS_INIT_PATH,
		AI_STATUS_WAITING,
		AI_STATUS_PATHING,
		AI_STATUS_RETRACING
	} status;

	AAntAI();
	virtual void Tick(float delta_time) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual bool has_avoid_responsibility(AAnt* other_ant) override;
	const FVector& get_destination() const;
	// as of 9/28/2022, only called by AAnt::fix_fall_through()
	void reset_nav();

protected:

	virtual void BeginPlay() override;
	virtual void set_true_move(float delta_time) override;
	virtual void cling_smooth_rotate(float delta_time) override;

private:
	
	struct NearbyAnt {

		NearbyAnt() : ant(nullptr), collision_responsibility(false) {}
		
    	AAnt* ant;
		bool collision_responsibility;

		NearbyAnt(AAnt* _ant, bool _collision_responsibility) {
			ant = _ant;
			collision_responsibility = _collision_responsibility;
		}

		bool operator == (const NearbyAnt& b) const {
			return (*this).ant == b.ant;
		}

    };

	static constexpr int MDC_LEN = 5;
	static constexpr int BREADCRUMB_CT = 10;
	static constexpr int AVOID_TICK_MAX = 3;
	static constexpr int MAX_NEARBY = 7;
	static constexpr int STUCK_CHECK_CT = 8;
	static constexpr float STUCK_CHECK_TIME = 0.1f;
	static constexpr float AYHT_MAX = PI * 0.5f * 0.5f;
	static constexpr float AYHT_MIN = -AYHT_MAX; // avoidance yaw half theta
	static constexpr float AYHT_STEP = AYHT_MAX * 1.8f;
	static constexpr float MDC_EPSILON = 1e-1;
	static constexpr float INV_MDC_LEN = 1.0f / MDC_LEN;
	static constexpr float NUDGE_SPEED = MAX_GROUND_SPEED * 2.0f;
	static constexpr float JAMMED_MAX = 0.4f;
	static constexpr float AYHT_ZERO_EPSILON = AYHT_STEP * 0.9f;
	static constexpr float QUARTER_MAX_GROUND_SPEED = 0.25f * MAX_GROUND_SPEED;
	static constexpr float HALF_MAX_GROUND_SPEED = 0.5f * MAX_GROUND_SPEED;
	static constexpr float TMTD_MAX = PI / 1600.0f;
	static constexpr float ROT_SPEED_MULTIPLIER_MIN = 0.4f;
	static constexpr float BREADCRUMB_DT = 0.3f;
	static constexpr float RETRACE_WAIT_TIME = 0.3f;
	static constexpr float STUCK_CHECK_AVG_CONST = 1.0f / (float)(STUCK_CHECK_CT - 1);
	static constexpr float STUCK_AVG_DIST = 1.0f;
	
	NearbyAnt nearby_ants[MAX_NEARBY];
	int nearby_top = 0;
	int nearby_i = 0;
	FVector breadcrumbs[BREADCRUMB_CT];
	float breadcrumb_ctr;
	int breadcrumb_i;
	int prev_waypoint_breadcrumb_ct;
	FVector step_retrace[BREADCRUMB_CT];
	FVector save_waypoint;
	int step_retrace_i;
	int step_retrace_top;
	float retrace_time;
	FVector destination;
	float avoidance_yaw_half_theta;
	float sq_capsule_full_height;
	FQuat avoidance_rot;
	int avoid_tick_ctr;
	float jammed_time;
	bool unjam;
	float unjam_time;
	FVector steering_tr_starts[2];
	float prev_yaw;
	bool path_incomplete;
	FVector cur_waypoint;
	float acceptance_radius;
	bool ant_ahead;
	FVector stuck_check_cache[STUCK_CHECK_CT];
	int stuck_check_i;
	float stuck_check_ctr;

	void avoid_collisions(float delta_time);
	int possible_collision(AAnt* other_ant) const;
	void set_steering_tr_starts();
	bool center_correct_ayht(float delta_time);
	void set_avoidance_rot();
	void nonground_collis_check();
	void set_step_retrace();
	void calculate_rot_speed_penalty(const FVector& prev_true_move);
	void drop_breadcrumbs(float delta_time);
	void breadcrumb_reset();
	FVector calculate_intended_move(const FVector& cur_loc) const;
	bool deal_with_jams(float delta_time);
	void add_nearby_ant(AAnt* ant, bool collis);
	void remove_nearby_ant(AAnt* ant);
	int find_nearby_ant(AAnt* ant) const;
	
};


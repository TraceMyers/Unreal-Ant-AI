#pragma once

#include "CoreMinimal.h"
#include "Ant.h"
#include "AntAI.generated.h"

UCLASS()
class ANTFIGHT_API AAntAI : public AAnt {
	GENERATED_BODY()

public:
	
	enum AI_STATUS {
		AI_STATUS_INIT_PATH,
		AI_STATUS_WAITING,
		AI_STATUS_PATHING,
		AI_STATUS_RETRACING
	} status;
	
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly)
	UBoxComponent* collis_box;
	
	

	AAntAI();
	virtual void Tick(float delta_time) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual bool has_avoid_responsibility(AAnt* other_ant) override;
	const FVector& get_destination() const;
	// as of 9/28/2022, only called by AAnt::fix_fall_through()
	void reset_nav();
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

protected:

	virtual void BeginPlay() override;
	virtual void set_true_move(float delta_time) override;
	virtual void cling_smooth_rotate(float delta_time) override;

private:
	
	struct NearbyAnt {
    	AAnt* ant;
		bool collision_responsibility;
		
		NearbyAnt() : ant(nullptr), collision_responsibility(false) {}

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
	static constexpr int STUCK_CHECK_TIME = 0.5f;
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
	bool first_waypoint_found;
	FVector stuck_check_loc;
	float stuck_check_ctr;
	bool stuck;
	int waypoint_ctr;
	
	void avoid_collisions(float delta_time);
	int possible_collision(AAnt* other_ant) const;
	bool center_correct_ayht(float delta_time);
	void set_avoidance_rot();
	bool deal_with_jams(float delta_time);

	// breadcrumbs are used when an ant runs into a static mesh that is not part of the navmesh in order to retrace
	// their steps to the last known good location.
	void drop_breadcrumbs(float delta_time);
	void breadcrumb_reset();
	
	void set_steering_tr_starts();
	void set_step_retrace();
	void calculate_rot_speed_penalty(const FVector& prev_true_move);
	void rotate_move_vec_onto_mesh_plane(FVector& move_vec, const FVector& cur_loc) const;
	
	void add_nearby_ant(AAnt* ant, bool collis);
	void remove_nearby_ant(AAnt* ant);
	int find_nearby_ant(AAnt* ant) const;
	
};


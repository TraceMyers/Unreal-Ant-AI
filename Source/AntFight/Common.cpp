#include "Common.h"
#include "Ant.h"
#include "Engine/StaticMeshActor.h"
#include <chrono>

namespace comm {

	UWorld* world;
	FVector hit_loc;
	AActor* hit_actor;
	double times[MAX_TIMES];
	int t_i;
	double start_time;

	void set_world(UWorld* _world) {
		world = _world;
		t_i = 0;
	}

	const FVector& get_hit_loc() {
		return hit_loc;
	}

	const AActor* get_hit_actor() {
		return hit_actor;
	}
	
	float trace_hit_ground(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor) {
		FHitResult hit;
		FCollisionQueryParams collis_params;
		if (ignored_actor != nullptr) {
			collis_params.AddIgnoredActor(ignored_actor);
		}
		world->LineTraceSingleByChannel(
			hit,
			tr_start,
			tr_end,
			ECC_STATIC_MESHES,
			collis_params
		);
		if (hit.Actor != nullptr) {
			TWeakObjectPtr<AActor> actor = hit.Actor;
			if (actor.IsValid() && actor.Get()->ActorHasTag(FName("ground"))) {
				hit_loc = hit.Location;
				hit_actor = hit.GetActor();
				return hit.Distance;
			}
		}
		return -1.0f;
	}
	
	float trace_hit_static_actor(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor) {
		FHitResult hit;
		FCollisionQueryParams collis_params;
		if (ignored_actor != nullptr) {
			collis_params.AddIgnoredActor(ignored_actor);
		}
		world->LineTraceSingleByChannel(
			hit,
			tr_start,
			tr_end,
			ECC_STATIC_MESHES,
			collis_params
		);
		if (hit.Actor != nullptr) {
			TWeakObjectPtr<AActor> actor = hit.Actor;
			if (actor.IsValid() && Cast<AStaticMeshActor>(actor.Get())) {
				hit_loc = hit.Location;
				hit_actor = hit.GetActor();
				return hit.Distance;
			}
		}
		return -1.0f;
	}
	
	float trace_hit_nonground_static_actor(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor) {
		FHitResult hit;
		FCollisionQueryParams collis_params;
		if (ignored_actor != nullptr) {
			collis_params.AddIgnoredActor(ignored_actor);
		}
		world->LineTraceSingleByChannel(
			hit,
			tr_start,
			tr_end,
			ECC_STATIC_MESHES,
			collis_params
		);
		if (hit.Actor != nullptr) {
			TWeakObjectPtr<AActor> actor = hit.Actor;
			if (actor.IsValid()) {
				AStaticMeshActor* sm = Cast<AStaticMeshActor>(actor.Get());
				if (sm && !sm->ActorHasTag("Ground")) {
					hit_loc = hit.Location;
					hit_actor = hit.GetActor();
					return hit.Distance;
				}
			}
		}
		return -1.0f;
	}

	float trace_hit_ant(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor) {
		FHitResult hit;
		FCollisionQueryParams collis_params;
		if (ignored_actor != nullptr) {
			collis_params.AddIgnoredActor(ignored_actor);
		}
		world->LineTraceSingleByChannel(
			hit,
			tr_start,
			tr_end,
			ECC_Visibility,
			collis_params
		);
		if (hit.Actor != nullptr) {
			TWeakObjectPtr<AActor> actor = hit.Actor;
			if (actor.IsValid() && Cast<AAnt>(actor.Get())) {
				hit_loc = hit.Location;
				hit_actor = hit.GetActor();
				return hit.Distance;
			}
		}
		return -1.0f;
	}
	
	void log_start_time() {
		if (t_i < MAX_TIMES) {
			start_time = FPlatformTime::Seconds();
		}
	}
	
	void log_end_time() {
		if (t_i < MAX_TIMES) {
			times[t_i++] = FPlatformTime::Seconds() - start_time;
		}
	}
	
	double get_time_avg() {
		double t_avg = 0.0f;
		for (int i = 0; i < t_i; i++) {
			t_avg += times[i];	
		}
		t_avg *= 1.0f / t_i;
		return t_avg;
	}
	
	double get_time_max() {
		double t_max = -DBL_MAX;
		for (int i = 0; i < t_i; i++) {
			const double& t = times[i];
			if (t > t_max) {
				t_max = t;	
			}
		}
		return t_max;
	}
	
	int get_time_ct() {
		return t_i;	
	}
	
	void clear_times() {
		t_i = 0;	
	}
	
	bool times_full() {
		return t_i == MAX_TIMES;	
	}
	
	int get_times_capacity() {
		return MAX_TIMES;
	}
}

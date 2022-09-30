#include "Common.h"

#include "Ant.h"
#include "Engine/StaticMeshActor.h"

namespace comm {

	UWorld* world;
	FVector hit_loc;
	AActor* hit_actor;

	void set_world(UWorld* _world) {
		world = _world;
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
}

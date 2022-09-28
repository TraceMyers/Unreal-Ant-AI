#include "AntAIDispatch.h"
#include "AntAI.h"
#include "AntPlayer.h"
#include "Common.h"
#include "Kismet/GameplayStatics.h"
#include "AINav.h"
#include "DrawDebugHelpers.h"

AAntAIDispatch::AAntAIDispatch() {
	PrimaryActorTick.bCanEverTick = true;
}

void AAntAIDispatch::BeginPlay() {
	Super::BeginPlay();
	TArray<AActor*> actors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAnt::StaticClass(), actors);
	for (int i = 0; i < actors.Num(); i++) {
		AAntAI* ant = Cast<AAntAI>(actors[i]);
		if (ant) {
			ant->set_dispatch(this);
			ants.Add(ant);
			// comm::print("ant: %s", TCHAR_TO_ANSI(*ant->GetName()));
		}
		else {
			AAntPlayer* ant_player = Cast<AAntPlayer>(actors[i]);
			if (ant_player) {
				ant_player->set_dispatch(this);
				ant_players.Add(ant_player);
				// comm::print("ant player: %s", TCHAR_TO_ANSI(*ant_player->GetName()));
			}
		}
	}
	for (int i = 0; i < task_markers.Num(); i++) {
		const auto task_marker = task_markers[i];
		const float radius = task_marker->mesh->GetStaticMesh()->GetBounds().SphereRadius;
		const FVector scale = task_marker->mesh->GetComponentScale();
		float sq_acceptance_radius = radius * scale.X; // assuming uniform scale
		sq_acceptance_radius *= sq_acceptance_radius;
		comm::print("%.2f", sq_acceptance_radius);
		task_marker_sq_radii.Add(sq_acceptance_radius);
	}
	
}

void AAntAIDispatch::dbg_draw_paths() const {
	const FVector two_up = FVector::UpVector * 2.0f;
	for (int i = 0; i < main_calls.pc_top; i++) {
		const auto& call = main_calls.pathing[i];
		if (call.caller == nullptr) {
			continue;
		}
		const FVector* path = call.path;
		for (int j = 0; j < call.path_len - 1; j++) {
			DrawDebugLine(
				GetWorld(),
				path[j] + two_up,
				path[j+1] + two_up,
				FColor::Green,
				false,
				-1,
				0,
				1.5f
			);
		}
	}
}

FVector* AAntAIDispatch::get_valid_path(int key, int& path_len, const FVector& caller_loc, bool copy_backward) const {
	FVector* path = ai_nav->get_path(key, path_len);
	if (path == nullptr || path_len < 1) {
		return nullptr;
	}
	float first_node_sq_dist;
	if (copy_backward) {
		first_node_sq_dist = FVector::DistSquared(path[path_len-1], caller_loc);
	}
	else {
		first_node_sq_dist = FVector::DistSquared(path[0], caller_loc);
	}
	// // TODO: change to whatever sq_radius was originally called for
	if (first_node_sq_dist > AINav::MIN_SQ_RADIUS) {
		return nullptr;
	}
	return path;
}


void AAntAIDispatch::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	dbg_draw_paths();
	
	for (int i = 0; i < ants.Num(); i++) {
		ants[i]->update_forward_positions();
	}
	for (int i = 0; i < ant_players.Num(); i++) {
		ant_players[i]->update_forward_positions();
	}

	const int waiting_calls_len = main_calls.wc_top;
	for (int i = 0; i < waiting_calls_len; i++) {
		const WaitingCall& call = main_calls.waiting[i];
		if (call.caller == nullptr) {
			continue;
		}
		const int key = call.path_key;
		const AINav::AI_PATH_STATUS status = ai_nav->get_path_status(key);
		if (status == AINav::AI_PATH_READY) {
			int path_len;
			FVector* path = get_valid_path(key, path_len, call.caller->GetActorLocation(), call.copy_backward);
			if (!path) {
				main_calls.drop_waiting_call(i);
				continue;
			}
			const bool path_complete = ai_nav->path_is_complete(key);
			main_calls.transfer_waiting_to_pathing(call.caller, i, path, path_len, path_complete);	
		}
		else if (status == AINav::AI_PATH_FREE) {
			main_calls.drop_waiting_call(i);
		}
	}
}

bool AAntAIDispatch::get_path(AAntAI* caller, const FVector& dest, bool& cached, float sq_radius) {
	const FVector start = caller->GetActorLocation();
	return get_path(caller, start, dest, cached, sq_radius);	
}

bool AAntAIDispatch::get_path(
	AAntAI* caller,
	const FVector& start,
	const FVector& dest,
	bool& cached,
	float sq_radius
) {
	bool copy_backward = false;
	const int key = ai_nav->find_path(
		start,
		dest,
		cached,
		copy_backward,
		sq_radius,
		sq_radius
	);
	if (key < 0) {
		return false;
	}
	if (cached) {
		int path_len;
		FVector* path = get_valid_path(key, path_len, caller->GetActorLocation(), copy_backward);
		if (!path) {
			return false;
		}
		return main_calls.add_pathing_call(caller, path, path_len, copy_backward) >= 0;
	}
	return main_calls.add_waiting_call(caller, key, copy_backward) >= 0;
}

// are we will waiting or have we been transferred to pathing?
DISPATCH_STATUS AAntAIDispatch::get_pathfinding_status(const AAntAI* caller, bool &path_incomplete) const {
	for (int i = 0; i < main_calls.wc_top; i++) {
		const auto& call = main_calls.waiting[i];
		if (caller == call.caller) {
			return DISPATCH_WAITING;
		}
	}
	for (int i = 0; i < main_calls.pc_top; i++) {
		const auto& call = main_calls.pathing[i];
		if (caller == call.caller) {
			path_incomplete = !call.path_complete;		
			return DISPATCH_READY;
		}
	}
	return DISPATCH_FAILED;
}

FVector* AAntAIDispatch::get_next_waypoint(const AAntAI* caller) {
	for (int i = 0; i < main_calls.pc_top; i++) {
		PathingCall& call = main_calls.pathing[i];
		if (call.caller == caller) {
			if (call.path_i == call.path_len) {
				main_calls.drop_pathing_call(i);
				return nullptr;
			}
			if (call.path[call.path_i] == FVector::ZeroVector) {
				comm::print("hello");
			}
			return &call.path[call.path_i++];
		}
	}
	return nullptr;
}

void AAntAIDispatch::set_ai_nav(AINav* _ai_nav) {
	ai_nav = _ai_nav;
}

FVector AAntAIDispatch::test_get_destination() {
	const int task_ct = task_markers.Num();
	const int marker_i = FMath::RandRange(0, task_ct - 1);
	const FVector dest = task_markers[marker_i]->GetActorLocation();
	return dest;
}

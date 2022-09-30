#pragma once

#include "CoreMinimal.h"
#include "AINav.h"
#include "TaskMarker.h"
#include "GameFramework/Actor.h"
#include "AntAIDispatch.generated.h"

enum DISPATCH_STATUS {
	DISPATCH_WAITING,
	DISPATCH_FAILED,
	DISPATCH_READY
};

UCLASS()
class ANTFIGHT_API AAntAIDispatch : public AActor {
	GENERATED_BODY()
	
public:

	FVector PATH_INCOMPLETE = FVector(FLT_MAX, FLT_MAX, FLT_MAX);

	UPROPERTY(BlueprintReadWrite, EditInstanceOnly)
	TArray<ATaskMarker*> task_markers;
	TArray<float> task_marker_sq_radii;
    
	AAntAIDispatch();
	virtual void Tick(float DeltaTime) override;
	void set_ai_nav(AINav* ai_nav);

	FVector test_get_destination(int& test_key);
	bool get_path(
		class AAntAI* caller,
		const FVector& destination,
		bool& pathing_immediately,
		float sq_radius=8000.0f
	);
	DISPATCH_STATUS get_pathfinding_status(const class AAntAI* caller, bool& path_incomplete) const;
	FVector* get_next_waypoint(const class AAntAI* caller);

protected:
    
	virtual void BeginPlay() override;

private:
	
	struct WaitingCall {
    	AAntAI* caller;
    	int path_key;
		bool copy_backward;
    };
    
    struct PathingCall {
    	AAntAI* caller;
    	FVector path[AINav::SMOOTHED_PATH_MAX_LEN];
    	int path_len;
    	int path_i;
    	bool path_complete;
    };

	struct CallContainer {
    	static constexpr int MAX_CALLS = 512;
    	
    	WaitingCall waiting[MAX_CALLS];
    	PathingCall pathing[MAX_CALLS];
    	int wc_i;
    	int pc_i;
    	int wc_top;
    	int pc_top;

		
		CallContainer() {
			for (int i = 0; i < MAX_CALLS; i++) {
				waiting[i].caller = nullptr;
				pathing[i].caller = nullptr;
			}
			wc_i = 0;
			pc_i = 0;
			wc_top = 0;
			pc_top = 0;
		} 
    	
    	int add_waiting_call(AAntAI* caller, int path_key, bool copy_backward) {
    		if (wc_i == MAX_CALLS) {
    			return -1;
    		}
    		auto& call = waiting[wc_i];
    		call.caller = caller;
    		call.path_key = path_key;
			call.copy_backward = copy_backward;
			const int waiting_key = wc_i;
    		for ( ; wc_i < MAX_CALLS && waiting[wc_i].caller != nullptr; wc_i++) {
    			;
    		}
    		if (wc_i > wc_top) {
    			wc_top = wc_i;
    		}
    		return waiting_key;
    	}
    
    	bool drop_waiting_call(AAntAI* caller) {
    		for (int i = 0; i < wc_top; i++) {
    			auto& call = waiting[i];
    			if (call.caller == caller) {
    				call.caller = nullptr;
    				if (i == wc_top - 1) {
    					do {
							wc_top--;
						}
    					while (wc_top > 0 && waiting[wc_top - 1].caller == nullptr);
    				}
    				if (i < wc_i) {
    					wc_i = i;
    				}
    				return true;
    			}
    		}
    		return false;
    	}
    
    	bool drop_waiting_call(int index) {
    		if (index >= 0 && index < MAX_CALLS) {
    			auto& call = waiting[index];
    			if (call.caller != nullptr) {
    				call.caller = nullptr;
    				if (index == wc_top - 1) {
    					do {
    						wc_top--;
    					}
    					while (wc_top > 0 && waiting[wc_top - 1].caller == nullptr);
    				}
    				if (index < wc_i) {
    					wc_i = index;
    				}
    				return true;
    			}
    		}
    		return false;
    	}
    
    	bool transfer_waiting_to_pathing(
    		AAntAI* caller,
    		int waiting_index,
    		FVector* path,
    		int path_len,
    		bool path_complete
		) {
    		if (pc_i == MAX_CALLS) {
    			return false;
    		}
			const bool copy_backward = waiting[waiting_index].copy_backward;
    		if (!drop_waiting_call(waiting_index)) {
    			return false;
    		}
    		auto& call = pathing[pc_i];
    		call.caller = caller;
    		call.path_len = path_len;
    		call.path_i = 0;
			call.path_complete = path_complete;
			if (copy_backward) {
				for (int i = path_len - 1, cpi = 0; i >= 0; i--, cpi++) {
					call.path[cpi] = path[i];
				}
			}
			else for (int i = 0 ; i < path_len; i++) {
    			call.path[i] = path[i];
    		}
    		for ( ; pc_i < MAX_CALLS && pathing[pc_i].caller != nullptr; pc_i++) {
    			;
    		}
    		if (pc_i > pc_top) {
    			pc_top = pc_i;
    		}
    		return true;
    	}

		int add_pathing_call(AAntAI* caller, FVector* path, int path_len, bool copy_backward) {
			if (pc_i == MAX_CALLS) {
				return -1;
			}
			auto& call = pathing[pc_i];
			call.caller = caller;
			call.path_len = path_len;
			call.path_i = 0;
			call.path_complete = true;
			const int pathing_key = pc_i;
			if (copy_backward) {
				for (int i = path_len - 1, cpi = 0; i >= 0; i--, cpi++) {
					call.path[cpi] = path[i];
				}
			}
			else for (int i = 0 ; i < path_len; i++) {
				call.path[i] = path[i];
			}
			for ( ; pc_i < MAX_CALLS && pathing[pc_i].caller != nullptr; pc_i++) {
				;
			}
			if (pc_i > pc_top) {
				pc_top = pc_i;
			}
			return pathing_key;
		}
    
    	bool drop_pathing_call(AAntAI* caller) {
    		for (int i = 0; i < pc_top; i++) {
    			auto& call = pathing[i];
    			if (call.caller == caller) {
    				call.caller = nullptr;
    				if (i == pc_top - 1) {
    					do {
    						pc_top--;
    					}
    					while (pc_top > 0 && pathing[pc_top - 1].caller == nullptr);
    				}
    				if (i < pc_i) {
    					pc_i = i;
    				}
    				return true;
    			}
    		}
    		return false;
    	}
    
    	bool drop_pathing_call(int index) {
    		if (index >= 0 && index < MAX_CALLS) {
    			auto& call = pathing[index];
    			if (call.caller != nullptr) {
    				call.caller = nullptr;
    				if (index == pc_top - 1) {
    					do {
    						pc_top--;
    					}
    					while (pc_top > 0 && pathing[pc_top - 1].caller == nullptr);
    				}
    				if (index < pc_i) {
    					pc_i = index;
    				}
    				return true;
    			}
    		}
    		return false;
    	}
    };

	TArray<AAntAI*> ants;
	TMap<AAntAI*, AINav::NavNode*> ant_nodes;
	TArray<class AAntPlayer*> ant_players;

	CallContainer main_calls;
	CallContainer subpath_calls;
	AINav* ai_nav;

	void dbg_draw_paths() const;
	FVector* get_valid_path(int key, int& path_len, const FVector& caller_loc, bool copy_backward) const;

};

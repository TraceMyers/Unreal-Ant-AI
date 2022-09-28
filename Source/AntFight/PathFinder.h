#pragma once

#include "CoreMinimal.h"
#include "AINav.h"

typedef AINav::NavNode* VNode;

class ANTFIGHT_API FPathFinder : public FRunnable {

public:

	FPathFinder();
	virtual ~FPathFinder() override;
	bool start_thread(
		VNode _start,
		VNode _end,
		VNode* path_buf,
		int* path_len,
		FThreadSafeBool* full_path_found,
		AINav* ai_nav
	);
	void stop_thread();
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Stop() override;
	

private:

	static constexpr int STEP_CT = 20;
	
	bool search_multistep();
	
	FRunnableThread* thread = nullptr;
	bool run_thread;
	
	FVector start;
	FVector end;
	
	VNode start_node;
	VNode end_node;
	FVector end_node_loc;
	
	TArray<VNode> search_stack;
	VNode search_vector;
	
	VNode* path_buf;
	int* path_len;
	FThreadSafeBool* full_path_found;
	AINav* ai_nav;
};

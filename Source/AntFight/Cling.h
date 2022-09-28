#pragma once

#include "CoreMinimal.h"
#include "AntGI.h"
#include "Engine/StaticMeshActor.h"

// A much better version of this would use a density-based mapping from world space to grid space
// to reduce high density boxes in high vertex ct areas
// would require a good packing algorithm for the different box sizes

// typedef struct VertexNode* VNode;
//
// struct VertexNode {
// 	// these are only written to at the start of the game
// 	FVector location;
// 	FVector normal;
// 	TArray<VNode> edges;
// 	TArray<float> weights;
//
// 	// only FPathFinder writes to these
// 	float back_weight;
// 	float forward_weight;
// 	VNode prev;
// 	
// 	bool operator == (const VertexNode& b) const {
// 		return this == &b;
// 	}
// };
//
// struct VertexGraphPopulatorBox {
// 	FVector position;
// 	FVector normal;
// 	FVector upper;
// 	FVector lower;
// };
//
// namespace Cling {
//
// 	enum DBG_DRAW {
// 		DBG_NONE,				// 0
// 		DBG_GRID,				// 1
// 		DBG_GRAPH,				// 2
// 		DBG_ANT_ORIENTATION,	// 3
// 		DBG_GRAPH_PROGRESSIVE,	// 4
// 		DBG_GRAPH_NODE_DIRS,	// 5
// 		DBG_GRID_PROGRESSIVE,	// 6
// 		DBG_NEAR_ANT,			// 7
// 		DBG_DRAW_PATHS,			// 8
// 	};
// 	
// 	enum CL_PATH_STATUS {
// 		CL_PATH_FREE,
// 		CL_PATH_READY,
// 		CL_PATH_FINDING
// 	};
//
// 	
// 	constexpr int PATH_MAX_LEN = 256;
// 	// NOTE: changing this value does not yet pass down the chain; changing it will break the program
// 	constexpr int PATH_SMOOTH_DIVISIONS = 3;
// 	constexpr int SMOOTHED_PATH_MAX_LEN = PATH_MAX_LEN * PATH_SMOOTH_DIVISIONS;
// 	constexpr float VGRAPH_MAX_EDGE_LEN = 140.0f;
// 	
// 	void init(UWorld* world, UAntGI* gi, const TArray<AStaticMeshActor*>& ground_mesh_actors, float actor_offset=60.0f);
// 	bool find_nearest_vertex(const FVector& location, const FVector& forward); // grid
// 	FQuat* get_cling_rotation(const FVector& forward); // grid
// 	void dbg_draw(float delta_time);
// 	void dbg_draw_switch(DBG_DRAW val);
// 	int find_path(const FVector& from, const FVector& to, float goal_acceptance_sq_radius=-1, bool radius_also_from=false);
// 	CL_PATH_STATUS get_path_status(int key);
// 	FVector* get_cached_path(int key, int& path_len);
// 	void kill_pathfinder();
//
// 	// for pathfinder
// 	VertexNode* get_nearest_pathable_vnode(const FVector& loc, float sq_radius=-1);
// 	FVector** get_path_buf();
// 	FVector** get_path_normals();
// 	void pathfinding_finished(bool success, int path_len);
// 	
// };
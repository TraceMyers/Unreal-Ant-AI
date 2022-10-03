#pragma once

#include "TriGrid.h"

/*
 * Navmesh & pathfinding interface
 */

class AINav {

public:

	enum AI_PATH_STATUS {
		AI_PATH_FREE,
		AI_PATH_READY,
		AI_PATH_FINDING
	};

	struct InitNode {
		Tri* tri;
		FVector loc;
		TArray<InitNode*> edges;
		TArray<int> cross_connections; // connections to other grid boxes, encoded
		FIntVector4 navnode_indices;
		bool culled;
		
		InitNode(Tri& _tri) {
			tri = &_tri;
			navnode_indices = FIntVector4(-1, -1, -1, -1);
			culled = false;
		}

		bool operator == (const InitNode& other) const {
			return this->tri == other.tri;
		}
	};
	
	struct NavNode {
		FVector location;
		FVector normal;
		TArray<NavNode*> edges;
		TArray<float> weights;

		// only FPathFinder writes to these
		float back_weight;
		float fore_weight;
		NavNode* link;
		
		NavNode(const InitNode& init_node) {
			const Tri& tri = *init_node.tri;
			location = init_node.loc;
			normal = tri.normal;
			back_weight = FLT_MAX;
			fore_weight = FLT_MAX;
			link = nullptr;
		}
		
		void nav_reset() {
			back_weight = FLT_MAX;
			fore_weight = FLT_MAX;
			link = nullptr;
		}
		
		bool operator == (const NavNode& b) const {
			return this == &b;
		}
	};

	// coming just over the top of the in-practice max path len keeps long paths from being fragmented,
	// reducing overwrites in the cache
	static constexpr int PATH_MAX_LEN = 130;
	static constexpr int PATH_SMOOTH_DIVISIONS = 2;
	static constexpr int INV_PATH_SMOOTH_DIVISIONS = 1.0f / (float)PATH_SMOOTH_DIVISIONS;
	static constexpr int SMOOTHED_PATH_MAX_LEN = (PATH_MAX_LEN - 1) * PATH_SMOOTH_DIVISIONS + 1;
	// 10/3/22: 90 gives best results - cache_hit: ~44%, avg find_path() response time: ~54ms
	static constexpr int PATH_CACHE_LEN = 90;
	static constexpr int MAX_NODES_PER_BOX = 25;
	static constexpr int GSPACE_SIDELEN = TriGrid::GSPACE_SIDELEN;
	static constexpr int GSPACE_SIDELEN_SQ = GSPACE_SIDELEN * GSPACE_SIDELEN;
	static constexpr int GSPACE_SIDELEN_M1 = GSPACE_SIDELEN - 1;
	static constexpr float NORMAL_MAX_ANGLE = 3.0f * PI / 8.0f;
	static constexpr float MIN_SQ_RADIUS = 8000.0f; // TODO: make dependent on whatever parameters normalize the graph
	static constexpr float KEEP_ALIVE_TIME = 20.0f;

	AINav();
	~AINav();
	void build_graph(UWorld* world, TriGrid* tri_grid);
	void tick(float delta_time);
	// earlier, currently unused but maybe future-ly usable implementation; problem: with a radius
	// around the start location, the path to the start point might be unclear. Also requires two nearby node searches
	// every call
	int find_path(
		const FVector& from,
		const FVector& to,
		bool& cached,
		bool& copy_backward,
		float sq_end_radius=MIN_SQ_RADIUS,
		float sq_start_radius=MIN_SQ_RADIUS
	);
	// current version. Makes far more sense; just keep track of what node the ant was last on
	// this will likely remain useful even when the version where end_node is known is implemented
	int find_path(
    	NavNode* start_node,
    	const FVector& to,
    	bool& cached,
    	bool& copy_backward,
    	float sq_end_radius=MIN_SQ_RADIUS
    );
	FVector* get_path(int key, int& path_len);
	AI_PATH_STATUS get_path_status(int key) const;
	bool path_is_complete(int key) const;
	NavNode* get_end_node(int key) const;
	NavNode* get_start_node(int key) const;
	NavNode* find_nearest_node(const FVector& loc);
	NavNode* find_nearby_node(const FVector& loc, float sq_radius=MIN_SQ_RADIUS);
	// called by pathfinder
	void pathfinding_finished();
	
private:
	
	enum TIME_TEST_CODE {
		TTC_FIND_PATH,
		TTC_CACHE_FIND,
		TTC_PATHFINDER_END
	};

	enum TIME_TEST_STATE {
		TTC_PATHFINDER_TIMING,
		TTC_CACHED_TIMING,
		TTC_FINISHED
	};

	enum PT_REF {
		TRI_1_A = 0x0001,
		TRI_1_B = 0x0002,
		TRI_1_C = 0x0004,
		TRI_2_A = 0x0010,
		TRI_2_B = 0x0020,
		TRI_2_C = 0x0040
	};

	enum EDGE_REF {
		TRI_1_AB = 0x0000,
		TRI_1_BC = 0x0001,
		TRI_1_CA = 0x0002,
		TRI_2_AB = 0x0000,
		TRI_2_BC = 0x0010,
		TRI_2_CA = 0x0020
	};

	const int EDGE_INDICES [3][2] {
		{0, 1}, {1, 2}, {2, 0}
	};

	TArray<NavNode> nav_graph[GSPACE_SIDELEN][GSPACE_SIDELEN][GSPACE_SIDELEN];

	class FPathFinder* pathfinder;
	NavNode* pathfinder_buf[PATH_MAX_LEN];
	int pathfinder_path_len;
	FThreadSafeBool full_path_found;
	FThreadSafeBool pathfinding;
	
	FVector smoothed_path_cache[PATH_CACHE_LEN][SMOOTHED_PATH_MAX_LEN];
	NavNode* end_nodes[PATH_CACHE_LEN];
	NavNode* start_nodes[PATH_CACHE_LEN];
	AI_PATH_STATUS path_statuses[PATH_CACHE_LEN];
	int path_lens[PATH_CACHE_LEN];
	bool path_complete[PATH_CACHE_LEN];
	float path_keep_alive[PATH_CACHE_LEN];
	int cache_write_i;
	
	UWorld* world;
	FVector inv_gbox_world_dims;
	FVector world_origin;
	TArray<NavNode>* graph_box_cache[27];

	TIME_TEST_STATE ttc_state;
	int ttc_cached_ct;
	int ttc_pathfinder_ct;
	double ttc_pathfinding_max;	
	double ttc_pathfinding_avg;
	bool ttc_reset;

	static inline TArray<InitNode>& get_node_box(void* init_nodes, int i, int j, int k);
	static bool get_alike_edge(const FVector* pts_a, const FVector* pts_b, int& indices);
	static int cross_connection_encoding(const FIntVector& grid_local, const FIntVector& grid_other);
	static void init_node_net(TriGrid* tri_grid, void* init_nodes);
	static void filter_node_net(void* init_nodes);
	void node_net_spatial_normalization(void* init_nodes);
	void set_nav_graph(void* init_nodes);
	
	bool get_grid_pos(const FVector& loc, FIntVector& gpos) const;
	TArray<NavNode>** get_nearby_graph_boxes(const FVector& loc, uint32& ct);

	inline int find_cached_path(const FVector& from, const FVector& to, bool& copy_backward) const;
	inline int find_cached_path_start_rad(
		const FVector& from,
		const FVector& to,
		bool& copy_backward,
		float start_rad
	) const;
	inline int find_cached_path_end_rad(
		const FVector& from,
		const FVector& to,
		bool& copy_backward,
		float end_rad
	) const;
	inline int find_cached_path_two_rad(
		const FVector& from,
		const FVector& to,
		bool& copy_backward,
		float start_rad,
		float end_rad
	) const;
	static inline bool path_match_two_rad(
		const FVector& from,
		const FVector& to,
		const FVector& path_a,
		const FVector& path_b,
		float start_radius,
		float end_radius
	);
	static inline bool path_match_start_rad(
		const FVector& from,
		const FVector& to,
		const FVector& path_a,
		const FVector& path_b,
		float start_radius
	);
	static inline bool path_match_end_rad(
		const FVector& from,
		const FVector& to,
		const FVector& path_a,
		const FVector& path_b,
		float end_radius
	);
	static inline bool path_match(
		const FVector& from,
		const FVector& to,
		const FVector& path_a,
		const FVector& path_b
	);
	void smooth_path(int key, int path_len);

	void dbg_draw_graph();
	// doesn't tank performance as much since it only draws near the location
	void dbg_draw_nav_graph(const FVector& near_loc);
	void dbg_draw_normals();
	
	void update_time_test(TIME_TEST_CODE c);
};



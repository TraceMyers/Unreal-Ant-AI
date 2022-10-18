#pragma once

#include "CoreMinimal.h"
#include "Tri.h"

// apologies, this is just too convenient
# define GSPACE_ITERATE_START \
	for (int i = 0; i < TriGrid::GSPACE_SIDELEN; i++) { \
	for (int j = 0; j < TriGrid::GSPACE_SIDELEN; j++) { \
	for (int k = 0; k < TriGrid::GSPACE_SIDELEN; k++) {
# define GSPACE_ITERATE_END }}}

/*
 * Takes provided static meshes, pulls their data from GPU buffers, and creates a single continuous mesh from
 * those meshes. This is used for ant movement/orientation and for building the navmesh in AINav.
 *
 * The single mesh (collection of adjacent tris) is divided into grid boxes so that finding them is faster.
 * In the future, the grid will become less useful. Objects using tri data shouldn't need to look up which
 * tri they're on at any given time; they should be able to store a protected reference than can be updated
 * to an adjacent tri when they move to it.
 *
 * This adjacent tri lookup will be further useful in making the navmesh more efficient (see
 * AINav::node_net_spatial_normalization())
 */

class TriGrid {

public:
	
	static constexpr float ONE_THIRD = 1.0f / 3.0f;
	static constexpr float TWO_THIRDS = 2.0f / 3.0f;
	static constexpr int EDGE_AB = 0x0001;
	static constexpr int EDGE_BC = 0x0002;
	static constexpr int EDGE_CA = 0x0004;
	// world space = (rectangular prism containing meshes), GSPACE = (grid cube of boxes containing tris)
	static constexpr int GSPACE_SIDELEN = 10;
	
	TriGrid();
	~TriGrid();
	bool init(
		UWorld* world,
		class UAntGI* gi,
		const TArray<class AStaticMeshActor*>& ground_actors,
		float character_space_buffer
	);
	void dbg_draw(float delta_time);
	TArray<Tri>** get_nearby_grid_boxes(const FVector& loc, uint32& ct);
	TArray<Tri>** get_nearby_grid_boxes(int i, int j, int k, uint32& ct, bool forward_only=false);
	void dbg_draw_edges();
	void dbg_draw_normals();
	void dbg_draw_edges_progressive(float delta_time);
	void dbg_draw_overlapping_tris();
	void dbg_toggle_ground_mesh_visibility();
	void dbg_draw_overlapping_tri_pts();
	void dbg_draw_overlapping_tri_polygons();
	void dbg_draw_new_tris();
	const FVector& get_inv_gbox_world_dims();
	const FVector& get_world_origin();

	TArray<Tri>& get_tribox(int i, int j, int k);

private:

	static constexpr int MAX_TRI_COLLECTION_CACHE_SIZE = 8192;

	UWorld* world;
	UAntGI* gi;
	FVector wmin;
	FVector wmax;
	FVector gbox_world_dims;
	FVector inv_gbox_world_dims;
	FVector world_origin;
	int mesh_ct;
	bool populated;
	bool ground_mesh_visibility;
	
	TArray<Tri> grid[GSPACE_SIDELEN][GSPACE_SIDELEN][GSPACE_SIDELEN];
	TArray<Tri>* tribox_cache[27];
	TArray<UStaticMeshComponent*> mesh_cmps;

	// stored if TRIGRID_DEBUG is defined, otherwise cleared after init
	TArray<Tri> overlapping_tris;
	TArray<FIntVector> overlap_tri_boxes;
	TArray<FVector> dbg_overlapping_tri_intersection_pts;
	TArray<Polygon> overlap_polygons;
	TArray<Tri> new_tris;

	FIntVector prog_ctr;
	float prog_outer;

	bool get_vertex_data(
		const TArray<class AStaticMeshActor*>& ground_actors,
		TArray<uint16*>& ind_buf,
		TArray<FVector*>& vert_buf,
		TArray<uint32>& cts
	) const;
	bool set_grid_dimensions(
		TArray<uint16*>& ind_buf,
		TArray<FVector*>& vert_buf,
		TArray<uint32>& cts,
		float character_space_buffer
	);
	bool init_set_grid_dimensions(
		TArray<uint16*>& ind_buf,
		TArray<FVector*>& vert_buf,
		TArray<uint32>& cts
	);
	bool populate_grid(
		TArray<uint16*>& ind_buf,
		TArray<FVector*>& vert_buf,
		TArray<uint32>& cts
	);
	
	// removing triangles inside other meshes and marking overlapping tris
	bool line_test_cull(const TArray<class AStaticMeshActor*>& ground_actors);
	
	// all functions below take overlapping mesh tris, determine intersections, generate new polygons
	// along the intersections, and then generate new tris within those polygons, which provides a
	// continuous mesh for orientation and for making the navmesh
	void generate_new_triangles_from_overlapping_tris();
	int get_intersections(
		TArray<TriEdge>& tri_edges,
		const Tri& overlapping_tri,
		PolyPoint* intersections
	);
	static bool set_point_of_intersection(
		const FVector& origin,
		const FVector& edge_norm,
		float edge_len,
		const Tri& tri,
		FVector& point_of_intersection
	);
	static void add_tri_edge_pending_connections(TArray<PolyPoint>& points, int pt_index, int pt_flags, int tri_flags);
	inline void add_pending_connections(TArray<PolyPoint>& points, PolyPoint& a, PolyPoint& b, int tri_flags);
	void find_polypoints(
    	TArray<TArray<TriEdge>>& edges,
    	TArray<TArray<PolyPoint>>& poly_points,
    	int overlapping_tri_ct
    );
	void generate_polygons(TArray<TArray<PolyPoint>>& poly_points, int overlapping_tri_ct);
	void generate_tris_from_polygons();
	void add_new_tris_to_grid();
	void clear_demo_data();

};






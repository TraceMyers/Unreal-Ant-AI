﻿#include "AINav.h"
#include "Common.h"
#include "DrawDebugHelpers.h"
#include "PathFinder.h"
#include "Tri.h"

// TODO hold onto path in smoothed cache for a little time or until dispatch says OK to freeing it up, to allow
// a little time for unfinished pathers to copy theirs (since they try to get theirs ahead of time)

AINav::AINav() :
	pathfinder(nullptr),
	pathfinder_path_len(-1),
	pathfinding(false),
	cache_write_i(0),
	world(nullptr)
{
	memset(smoothed_path_cache, 0, PATH_CACHE_LEN * SMOOTHED_PATH_MAX_LEN * sizeof(FVector));
	memset(pathfinder_buf, NULL, PATH_MAX_LEN * sizeof(FVector*));
	memset(path_statuses, AI_PATH_FREE, PATH_CACHE_LEN * sizeof(AI_PATH_STATUS));
	memset(path_lens, 0, PATH_CACHE_LEN * sizeof(int));
	memset(graph_box_cache, NULL, 27 * sizeof(TArray<NavNode>*));
}

AINav::~AINav() {
	if (pathfinder) {
		pathfinder->stop_thread();
	}
}

void AINav::build_graph(UWorld* _world, TriGrid* tri_grid) {
	world = _world;
	inv_gbox_world_dims = tri_grid->get_inv_gbox_world_dims();
	world_origin = tri_grid->get_world_origin();
	pathfinder = new FPathFinder();
	
	TArray<InitNode> init_nodes [GSPACE_SIDELEN][GSPACE_SIDELEN][GSPACE_SIDELEN];
	init_node_net(tri_grid, (void*)init_nodes);
	filter_node_net((void*)init_nodes);
	// TODO: node net spatial normalization
	// will be difficult, but I already have ideas of how to do it
	set_nav_graph((void*)init_nodes);
}

void AINav::dbg_draw(float delta_time) {
#ifdef AINAV_DEBUG
	// dbg_draw_normals();
	// dbg_draw_graph();
#endif 
}

int AINav::find_path(
	const FVector& from,
	const FVector& to,
	bool& cached,
	bool& copy_backward,
	float sq_end_radius,
	float sq_start_radius
) {
	const int cached_path_index = find_cached_path_two_rad(from, to, copy_backward, sq_start_radius, sq_end_radius);	
	if (cached_path_index >= 0) {
		cached = true;
		return cached_path_index;
	}
	if (!pathfinding) {
		NavNode* start_node = find_nearby_node(from, sq_start_radius);
		if (start_node == nullptr) {
			return -1;
		}
		NavNode* end_node = find_nearby_node(to, sq_end_radius);
		if (end_node == nullptr || start_node->location == end_node->location) {
			return -1;
		}
		// TODO: probably unnecessary - remove?
		for (int i = 0; i < PATH_MAX_LEN; i++) {
			pathfinder_buf[i] = nullptr;
		}
		GSPACE_ITERATE_START
		auto& graph_box = nav_graph[i][j][k];
		for (int m = 0; m < graph_box.Num(); m++) {
			NavNode& nav_node = graph_box[m];
			nav_node.nav_reset();
		}
		GSPACE_ITERATE_END
		pathfinder->stop_thread();
		path_statuses[cache_write_i] = AI_PATH_FINDING;
		pathfinding = true;
		const bool thread_started = pathfinder->start_thread(
			start_node,
			end_node,
			pathfinder_buf,
			&pathfinder_path_len,
			&full_path_found,
			this
		);
		if (!thread_started) {
			pathfinding = false;
			path_statuses[cache_write_i] = AI_PATH_FREE;
			return -1;
		}
		return cache_write_i;
	}
	return -1;
}

AINav::AI_PATH_STATUS AINav::get_path_status(int key) const {
#ifdef AINAV_DEBUG
	check(key >= 0 && key < PATH_CACHE_LEN);
#endif
	// comm::print("checking on %d, which is set to %d", key, path_statuses[key]);
	return path_statuses[key];
}

FVector* AINav::get_path(int key, int& path_len) {
#ifdef AINAV_DEBUG
	check(key >= 0 && key < PATH_CACHE_LEN);
#endif
	if (path_statuses[key] == AI_PATH_READY) {
		path_len = path_lens[key];
		return smoothed_path_cache[key];
	}
	return nullptr;
}

bool AINav::path_is_complete(int key) const {
	return path_complete[key];
}

void AINav::pathfinding_finished() {
	if (pathfinder_path_len > 0) { // if success
		// TODO: set smoothed path len
		// TODO: smooth path
		// for now, just copying path in straight
		path_lens[cache_write_i] = pathfinder_path_len;
		FVector* cache_slot = smoothed_path_cache[cache_write_i];
		for (int i = 0; i < pathfinder_path_len; i++) {
			cache_slot[i] = pathfinder_buf[i]->location;
		}
		path_complete[cache_write_i] = full_path_found;
		path_statuses[cache_write_i] = AI_PATH_READY;
	}
	else {
		path_statuses[cache_write_i] = AI_PATH_FREE;
	}

	cache_write_i = cache_write_i == PATH_CACHE_LEN - 1 ? 0 : cache_write_i + 1;
	pathfinding = false;
}

void AINav::dbg_draw_graph() {
	GSPACE_ITERATE_START
	auto& box_nodes = nav_graph[i][j][k];
	for (int m = 0; m < box_nodes.Num(); m++) {
		auto& node = box_nodes[m];
		DrawDebugCircle(world, node.location, 2.0f, 3, FColor::Red);
		for (int n = 0; n < node.edges.Num(); n++) {
			if (node.edges[n] != nullptr) {
				DrawDebugLine(world, node.location, node.edges[n]->location, FColor::Blue, false, -1, 0, 1.0f);
			}
		}
	}
	GSPACE_ITERATE_END
}

void AINav::dbg_draw_normals() {
	GSPACE_ITERATE_START
	auto& box_nodes = nav_graph[i][j][k];
	for (int m = 0; m < box_nodes.Num(); m++) {
		auto& node = box_nodes[m];
		DrawDebugLine(world, node.location, node.location + node.normal * 5.0f, FColor::Blue, false, -1, 0, 1.0f);
	}
	GSPACE_ITERATE_END
}


int AINav::find_cached_path(const FVector& from, const FVector& to, bool& copy_backward) const {
	for (int i = 0; i < PATH_CACHE_LEN; i++) {
		const FVector& path_start  = smoothed_path_cache[i][0];
		const FVector& path_end = smoothed_path_cache[i][path_lens[i] - 1];
		if (path_statuses[i] == AI_PATH_READY) {
			if (path_match(from, to, path_start, path_end)) {
				return i;
			}
			if (path_match(from, to, path_end, path_start)) {
				copy_backward = true;
				return i;
			}
		}
	}
	return -1;
}

int AINav::find_cached_path_start_rad(
	const FVector& from,
	const FVector& to,
	bool& copy_backward,
	float start_rad
) const {
	for (int i = 0; i < PATH_CACHE_LEN; i++) {
		const FVector& path_start  = smoothed_path_cache[i][0];
		const FVector& path_end = smoothed_path_cache[i][path_lens[i] - 1];
		if (path_statuses[i] == AI_PATH_READY) {
			if (path_match_start_rad(from, to, path_start, path_end, start_rad)) {
				return i;
			}
			if (path_match_start_rad(from, to, path_end, path_start, start_rad)) {
				copy_backward = true;
				return i;
			}
		}
	}
	return -1;
}

int AINav::find_cached_path_end_rad(
	const FVector& from,
	const FVector& to,
	bool& copy_backward,
	float end_rad
) const {
	for (int i = 0; i < PATH_CACHE_LEN; i++) {
		const FVector& path_start  = smoothed_path_cache[i][0];
		const FVector& path_end = smoothed_path_cache[i][path_lens[i] - 1];
		if (path_statuses[i] == AI_PATH_READY) {
			if (path_match_end_rad(from, to, path_start, path_end, end_rad)) {
				return i;
			}
			if (path_match_end_rad(from, to, path_end, path_start, end_rad)) {
				copy_backward = true;
				return i;
			}
		}
	}
	return -1;
}

int AINav::find_cached_path_two_rad(
	const FVector& from,
	const FVector& to,
	bool& copy_backward,
	float start_rad,
	float end_rad
) const {
	for (int i = 0; i < PATH_CACHE_LEN; i++) {
		const FVector& path_start  = smoothed_path_cache[i][0];
		const FVector& path_end = smoothed_path_cache[i][path_lens[i] - 1];
		if (path_statuses[i] == AI_PATH_READY) {
			if (path_match_two_rad(from, to, path_start, path_end, start_rad, end_rad)) {
				return i;
			}
			if (path_match_two_rad(from, to, path_end, path_start, start_rad, end_rad)) {
				copy_backward = true;
				return i;
			}
		}
	}
	return -1;
}

bool AINav::path_match_two_rad(
	const FVector& from,
	const FVector& to,
	const FVector& path_a,
	const FVector& path_b,
	float start_radius,
	float end_radius
) {
	return (
		(
			from == path_a
			|| (from - path_a).SizeSquared() < start_radius
		) && (
			to == path_b
			|| (to - path_b).SizeSquared() < end_radius
		)
	);
}

bool AINav::path_match_start_rad(
	const FVector& from,
	const FVector& to,
	const FVector& path_a,
	const FVector& path_b,
	float start_radius
) {
	return (
		(
			from == path_a
			|| (from - path_a).SizeSquared() < start_radius
		)
		&& to == path_b
	);
}

bool AINav::path_match_end_rad(
	const FVector& from,
	const FVector& to,
	const FVector& path_a,
	const FVector& path_b,
	float end_radius
) {
	return (
		from == path_a
		&& (
			to == path_b
			|| (to - path_b).SizeSquared() < end_radius
		)
	);
}

bool AINav::path_match(
	const FVector& from,
	const FVector& to,
	const FVector& path_a,
	const FVector& path_b
) {
	return from == path_a && to == path_b;	
}

void AINav::init_node_net(TriGrid* tri_grid, void* grid_nodes) {
	GSPACE_ITERATE_START
	TArray<Tri>& tribox = tri_grid->get_tribox(i, j, k);
	const int box_ct = tribox.Num();
	TArray<InitNode>& box_nodes = get_node_box(grid_nodes, i, j, k);
	for (int m = 0; m < box_ct; m++) {
		Tri& tri = tribox[m];
		if (tri.flags & Tri::OVERLAP) {
			continue;
		}
		box_nodes.Add(InitNode(tri));
	}
	GSPACE_ITERATE_END
	
	GSPACE_ITERATE_START
	TArray<InitNode>& box_nodes = get_node_box(grid_nodes, i, j, k);
	const int box_node_ct = box_nodes.Num();
	
	for (int m = 0; m < box_node_ct - 1; m++) {
		InitNode& node_m = box_nodes[m];
		const Tri& tri_m = *node_m.tri;
		const FVector m_pts[3] {
			tri_m.a,
			tri_m.b,
			tri_m.c
		};
		auto& node_m_edges = node_m.edges;
		for (int n = m + 1; n < box_node_ct; n++) {
			InitNode& node_n = box_nodes[n];
			const Tri& tri_n = *node_n.tri;
			const FVector n_pts[3] {
				tri_n.a,
				tri_n.b,
				tri_n.c
			};
			int edge_indices;
			const bool share_edge = get_alike_edge(m_pts, n_pts, edge_indices);
			if (share_edge) {
				node_m_edges.Add(&node_n);
				node_n.edges.Add(&node_m);	
				// const int tri_m_edge_i = edge_indices & 0x000f;
				// const int tri_n_edge_i = (edge_indices & 0x00f0) >> 8;
				// node_m.edges[tri_m_edge_i] = &node_n;
				// node_n.edges[tri_n_edge_i] = &node_m;
				if (node_m_edges.Num() == 3) {
					break;
				}
			}
		}
	}
	
	// forward only; backward connections are already made
	const int x_min = i;
	const int y_min = j;
	const int z_min = k;
	const int x_max = i == GSPACE_SIDELEN_M1 ? i : i + 1;
	const int y_max = j == GSPACE_SIDELEN_M1 ? j : j + 1;
	const int z_max = k == GSPACE_SIDELEN_M1 ? k : k + 1;
	const FIntVector m_gridpos(i, j, k);
	
	for (int m = 0; m < box_node_ct; m++) {
		InitNode& node_m = box_nodes[m];
		auto& node_m_edges = node_m.edges;
		if (node_m_edges.Num() == 3) {
			continue;
		}
		const Tri& tri_m = *node_m.tri;
		const FVector m_pts[3] {
			tri_m.a,
			tri_m.b,
			tri_m.c
		};
		for (int x = x_min; x <= x_max; x++) {
			for (int y = y_min; y <= y_max; y++) {
				for (int z = z_min; z <= z_max; z++) {
					if (x == i && y == j && z == k) {
						continue; 
					}
					const FIntVector n_gridpos(x, y, z);
					TArray<InitNode>& nearby_box_nodes = get_node_box(grid_nodes, x, y, z);
					const int nearby_box_node_ct = nearby_box_nodes.Num();
	
					for (int n = 0; n < nearby_box_node_ct; n++) {
						InitNode& node_n = nearby_box_nodes[n];
						const Tri& tri_n = *node_n.tri;
						const FVector n_pts[3] {
							tri_n.a,
							tri_n.b,
							tri_n.c
						};
						int edge_indices;
						const bool share_edge = get_alike_edge(m_pts, n_pts, edge_indices);
						if (share_edge) {
							node_m_edges.Add(&node_n);
							node_n.edges.Add(&node_m);	
							const int forward_encoding = cross_connection_encoding(m_gridpos, n_gridpos);
							node_m.cross_connections.Add(forward_encoding);
							node_n.cross_connections.Add(-forward_encoding);
							if (node_m_edges.Num() == 3) {
								goto XYZ_ITERATE_END;
							}
						}
					}
				}
			}
		}
		XYZ_ITERATE_END:
		;
	}
	GSPACE_ITERATE_END
}

void AINav::filter_node_net(void* init_nodes) {
	int remove_ct;
	int pass_ct = 0;
	int total_remove_ct = 0;
	do {
		remove_ct = 0;
		GSPACE_ITERATE_START
		TArray<InitNode>& box_nodes = get_node_box(init_nodes, i, j, k);
		for (int m = 0; m < box_nodes.Num(); m++) {
			InitNode& node = box_nodes[m];
			if (node.culled) {
				continue;
			}
			TArray<InitNode*>& node_edges = node.edges;
			int edge_ct = node_edges.Num();
			if (edge_ct < 2) {
				node.culled = true;
				remove_ct++;
			}
			else {
				const FVector& node_normal = node.tri->normal;
				for (int n = 0; n < edge_ct; n++) {
					const InitNode* edge = node_edges[n];
					if (edge->culled) {
						if (edge_ct < 3) {
							node.culled = true;
							remove_ct++;
							break;
						}
						edge_ct--;
					}
					else if (pass_ct == 0) {
						const FVector& edge_normal = edge->tri->normal;
						const float theta = FMath::Acos(FVector::DotProduct(node_normal, edge_normal));
						if (theta > NORMAL_MAX_ANGLE) {
							if (edge_ct < 3) {
								node.culled = true;
								remove_ct++;
								break;
							}
							edge_ct--;
						}
					}
				}
			}
		}
		GSPACE_ITERATE_END
		total_remove_ct += remove_ct;
		pass_ct++;
	} while(remove_ct > 0);
	comm::print("total remove ct: %d", total_remove_ct);
}

bool AINav::get_alike_edge(const FVector* pts_a, const FVector* pts_b, int& indices) {
	uint32 flags = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (pts_a[i] == pts_b[j]) {
				if (flags == 0) {
					flags |= 1 << i;
					flags |= 1 << (j + 8);
					break;
				}
				if (flags & TRI_1_A) {
					if (i == 1) {
						indices = TRI_1_AB;
					}
					else { 
						indices = TRI_1_CA;	
					}
				}
				else {
					indices	= TRI_1_BC;
				}
				if (flags & TRI_2_A) {
					if (j == 1) { 
						indices |= TRI_2_AB;
					}
					else { 
						indices |= TRI_2_CA;	
					}
				}
				else { 
					indices |= TRI_2_BC;	
				}
				return true;
			}	
		}
	}
	return false;
}

// there are up to 26 boxes surrounding a grid box; this encodes the other box's relative position as a single number
// (also encodes same position as 0)
// (... basically just linear storage position difference math)
int AINav::cross_connection_encoding(const FIntVector& grid_xyz, const FIntVector& grid_ijk) {
	return (
		9 * (grid_xyz.X - grid_ijk.X)
		+ 3 * (grid_xyz.Y - grid_ijk.Y)
		+ (grid_xyz.Z - grid_ijk.Z)
	);
}

inline TArray<AINav::InitNode>& AINav::get_node_box(void* init_nodes, int _i, int _j, int _k) {
	return *((TArray<InitNode>*)init_nodes + _i * GSPACE_SIDELEN_SQ + _j * GSPACE_SIDELEN + _k);
}

void AINav::set_nav_graph(void* init_nodes) {
	GSPACE_ITERATE_START
	TArray<InitNode>& box_nodes = get_node_box(init_nodes, i, j, k);
	TArray<NavNode>& nav_nodes = nav_graph[i][j][k];
	int nav_i = 0;
	for (int m = 0; m < box_nodes.Num(); m++) {
		auto& init_node = box_nodes[m];
		if (init_node.culled) {
			continue;
		}
		init_node.navnode_indices = FIntVector4(i, j, k, nav_i++);
		// TODO: change once initnode locs are set
		NavNode nav_node (init_node);
		nav_node.location = init_node.tri->center;	
		nav_nodes.Add(nav_node);
	}
	GSPACE_ITERATE_END
	
	GSPACE_ITERATE_START
	TArray<InitNode>& box_nodes = get_node_box(init_nodes, i, j, k);
	TArray<NavNode>& nav_nodes = nav_graph[i][j][k];
	int nav_i = 0;
	for (int m = 0; m < box_nodes.Num(); m++) {
		auto& init_node = box_nodes[m];
		if (init_node.culled) {
			continue;
		}
		auto& init_node_edges = init_node.edges;
		NavNode& nav_node = nav_nodes[nav_i++];
		for (int n = 0; n < init_node_edges.Num(); n++) {
			const auto edge = init_node_edges[n];
			if (edge->culled) {
				continue;
			}
			const FIntVector4& ni = edge->navnode_indices;
			NavNode& other = nav_graph[ni.X][ni.Y][ni.Z][ni.W];
			nav_node.edges.Add(&other);
			nav_node.weights.Add(FVector::Distance(nav_node.location, other.location));
		}
	}
	GSPACE_ITERATE_END
}

AINav::NavNode* AINav::find_nearby_node(const FVector& loc, float sq_radius) {
#ifdef AINAV_DEBUG
	check(sq_radius >= MIN_SQ_RADIUS);
#endif
	uint32 box_ct;
	const auto graph_boxes = get_nearby_graph_boxes(loc, box_ct);
	if (graph_boxes == nullptr) {
		return nullptr;
	}
	for (uint32 i = 0; i < box_ct; i++) {
		auto& graph_box = *graph_boxes[i];
		for (int j = 0; j < graph_box.Num(); j++) {
			NavNode& node = graph_box[j];
			const FVector diff = node.location - loc;
			const float sq_dist = diff.SizeSquared();
			if (sq_dist <= sq_radius) {
				const FVector offset = node.normal * 1.0f;
				const FVector tr_start = loc + offset;
				const FVector tr_end = node.location + offset;
				if (
					comm::trace_hit_static_actor(tr_start, tr_end) > 0.0f 
					|| comm::trace_hit_static_actor(tr_end, tr_start) > 0.0f
				) {
					continue;
				}
				return &node;
			}
		}
	}
	return nullptr;
}

bool AINav::get_grid_pos(const FVector& loc, FIntVector& gpos) const {
	gpos = FIntVector((loc - world_origin) * inv_gbox_world_dims);
	if (
		gpos.X >= 0 && gpos.X < GSPACE_SIDELEN
		&& gpos.Y >= 0 && gpos.Y < GSPACE_SIDELEN
		&& gpos.Z >= 0 && gpos.Z < GSPACE_SIDELEN
	) {
		return true;
	}
	return false;
}

TArray<AINav::NavNode>** AINav::get_nearby_graph_boxes(const FVector& loc, uint32& ct) {
	FIntVector graph_indices;
	if (!get_grid_pos(loc, graph_indices)) {
		return nullptr;	
	}
	const int gspace_x_min = graph_indices.X > 0 ? graph_indices.X - 1 : graph_indices.X;
	const int gspace_y_min = graph_indices.Y > 0 ? graph_indices.Y - 1 : graph_indices.Y;
	const int gspace_z_min = graph_indices.Z > 0 ? graph_indices.Z - 1 : graph_indices.Z;
	const int gspace_x_max = graph_indices.X < GSPACE_SIDELEN_M1 ? graph_indices.X + 1 : graph_indices.X;
	const int gspace_y_max = graph_indices.Y < GSPACE_SIDELEN_M1 ? graph_indices.Y + 1 : graph_indices.Y;
	const int gspace_z_max = graph_indices.Z < GSPACE_SIDELEN_M1 ? graph_indices.Z + 1 : graph_indices.Z;
	graph_box_cache[0] = &nav_graph[graph_indices.X][graph_indices.Y][graph_indices.Z];
	uint32 ctr = 1;
	for (int i = gspace_x_min; i <= gspace_x_max; i++) {
		for (int j = gspace_y_min; j <= gspace_y_max; j++) {
			for (int k = gspace_z_min; k <= gspace_z_max; k++) {
				if (i == graph_indices.X && j == graph_indices.Y && k == graph_indices.Z) {
					continue;
				}
				graph_box_cache[ctr++] = &nav_graph[i][j][k];			
			}
		}
	}
	ct = ctr;
	return graph_box_cache;
}

void AINav::smooth_path(int key) {
	
}

void AINav::node_net_spatial_normalization(void* init_nodes) {
	
}

#include "Cling.h"
#include "Common.h"
#include "DrawDebugHelpers.h"
#include "PathFinder.h"
#include "PathSmoothing.h"
#include "Rendering/PositionVertexBuffer.h"
#include "Rendering/StaticMeshVertexBuffer.h"

#define PACKAGED_DEBUG 1

// TODO: support rotated meshes
// TODO: read fbx data directly to create a more robust set of orientation and navmeshes

// namespace {
// 	
// 	constexpr int GRIDSPACE_SIDE_LEN = 10;
// 	constexpr int NODES_PER_BOX = 20;
// 	constexpr int VGRAPH_POPULATOR_BUF_SIZE = 1024;
// 	constexpr int VGRAPH_RADIAL_ALLOWANCE_DIVISIONS = 8;
// 	constexpr int PATH_CACHE_LEN = 100; // some x > 50 and x < 120 is best for original test map
// 	constexpr float max_vert_dist_sq = 100.0f * 100.0f;
// 	constexpr float ROT_EPSILON = 1e-4;
// 	constexpr float VGRAPH_POPULATOR_DEFAULT_SIDE_HALF_LEN = 50.0f;
// 	constexpr float VGRAPH_NORM_HEIGHT = 30.0f;
// 	constexpr float TWO_PI = 2 * PI;
// 	
// 	FPathFinder *pathfinder = nullptr;
//
// 	// -- grid -- (9/7/22: used for orienting pawns on ground meshes)
// 	TArray<FVector> vertex_grid[GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN];
// 	TArray<FVector> normal_grid[GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN];
//
// 	// actor cache TODO: (needs improvement)
// 	FQuat last_rotation;
// 	FVector vert_normal;
// 	FVector vert_pos;
// 	FVector ant_forward;
// 	FVector ant_right;
// 	FVector ant_loc;
//
// 	// -- graph -- (9/7/22: used as navmesh)
// 	VertexNode vertex_graph[GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN][NODES_PER_BOX];
// 	int vg_node_cts[GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN][GRIDSPACE_SIDE_LEN];
//
// 	// graph init
// 	FVector vgraph_pop_up_offset = FVector::OneVector * VGRAPH_POPULATOR_DEFAULT_SIDE_HALF_LEN;
// 	FVector vgraph_pop_lo_offset = -FVector::OneVector * VGRAPH_POPULATOR_DEFAULT_SIDE_HALF_LEN;
//
// 	// pathing & pathfinding
// 	FVector* path_buf[Cling::PATH_MAX_LEN];
// 	FVector* path_normals[Cling::PATH_MAX_LEN];
// 	FVector smoothed_path_cache[PATH_CACHE_LEN][Cling::SMOOTHED_PATH_MAX_LEN];
// 	int cache_write_i = 0;
// 	
// 	Cling::CL_PATH_STATUS path_statuses[PATH_CACHE_LEN];
// 	int path_lens[PATH_CACHE_LEN];
// 	FThreadSafeBool finding_path;
//
// 	// graph init vals that would otherwise have to be parameters
// 	VNode radial_edge_allowance[VGRAPH_RADIAL_ALLOWANCE_DIVISIONS];
// 	int dist_ok;
// 	int theta_ok;
// 	int trace_ok;
// 	float edge_min_theta;
// 	float edge_max_theta;
// 	float trace_len;
// 	float hit_epsilon_factor;
// 	float radial_edge_phi[VGRAPH_RADIAL_ALLOWANCE_DIVISIONS];
// 	float point_distances[VGRAPH_RADIAL_ALLOWANCE_DIVISIONS];
// 	FVector vnode_a_loc;
// 	FVector vnode_a_normal;
// 	FVector tr_start;
// 	FVector vnode_a_plane_right;
// 	FVector vnode_a_plane_up;
// 	TArray<float> thetas;
// 	
// 	// graph dbg
// 	TArray<FVector> starts;
// 	TArray<FVector> ups;
// 	TArray<FVector> rights;
// 	TArray<FVector> dbg_normals;
// 	TArray<FVector> tr_starts;
// 	TArray<FVector> tr_ends;
//
// 	// dimension of grid boxes in world space & the world space origin of the grid
// 	FVector grid_box_sz;
// 	FVector grid_origin;
// 	
// 	UWorld* world;
// 	UAntGI* game_instance;
// 	TArray<AStaticMeshActor*> mesh_actors;
// 	
// 	int dbg_i = 0;
// 	int dbg_j = 0;
// 	int dbg_k = 0;
// 	int dbg_ctr = 0;
// 	int dbg_step_ctr = 0;
//
// 	int ant_i;
// 	int ant_j;
// 	int ant_k;
// 	bool ant_ijk_changed;
// 	
// 	Cling::DBG_DRAW draw_setting = Cling::DBG_NONE;
//
// 	struct MeshDepth {
//
// 		MeshDepth(AStaticMeshActor* _sm) {
// 			sm = _sm;
// 		}
// 		
// 		AStaticMeshActor* sm;
// 		TArray<float> depths;	
// 	};
//
// 	void init_grid(TArray<AStaticMeshActor*> ground_mesh_actors, float actor_offset) {
//     	float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
//     	float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
//     	const int actor_ct = ground_mesh_actors.Num();
// 		TArray<FTransform*> actor_transforms;
// 		// TArray<FStaticMeshVertexBuffer> smv_buffers;
// 		// TArray<FPositionVertexBuffer> pos_buffers;
// 		
//     	for (int i = 0; i < actor_ct; i++) {
//     		const auto actor = ground_mesh_actors[i];
//     		// TODO: triangles
// 			const FStaticMeshLODResources& LOD = actor->GetStaticMeshComponent()->GetStaticMesh()->GetRenderData()->LODResources[0];
//     		const uint32 vertex_ct = LOD.GetNumVertices();
//     		// const uint32 vertex_ct = LOD.GetNumTriangles();
//     		AStaticMeshActor* ground_actor = ground_mesh_actors[i];
//     		// comm::print("cpu accessible: %d", actor->GetStaticMeshComponent()->GetStaticMesh()->bAllowCPUAccess);
//     		
//     		// TODO: just init them to the first values; it works
//     		for (uint32 j = 0; j < 2 && j < vertex_ct; j++) {
//     			FVector pos = ground_actor->GetTransform().TransformPosition(LOD.VertexBuffers.PositionVertexBuffer.VertexPosition(j));
//     			const float pos_x = pos.X;
//     			const float pos_y = pos.Y;
//     			const float pos_z = pos.Z;
//     			// TODO: switch to just doing this for the first actor
//     			if (pos_x < min_x) {
//     				min_x = pos_x;
//     			}
//     			if (pos_x > max_x) {
//     				max_x = pos_x;
//     			}
//     			if (pos_y < min_y) {
//     				min_y = pos_y;
//     			}
//     			if (pos_y > max_y) {
//     				max_y = pos_y;
//     			}
//     			if (pos_z < min_z) {
//     				min_z = pos_z;
//     			}
//     			if (pos_z > max_z) {
//     				max_z = pos_z;
//     			}
//     		}
//     		for (uint32 j = 2; j < vertex_ct; j++) {
//     			auto pos = ground_actor->GetTransform().TransformPosition(LOD.VertexBuffers.PositionVertexBuffer.VertexPosition(j));
//     			const float pos_x = pos.X;
//     			const float pos_y = pos.Y;
//     			const float pos_z = pos.Z;
//     			if (pos_x < min_x) {
//     				min_x = pos_x;
//     			}
//     			else if (pos_x > max_x) {
//     				max_x = pos_x;
//     			}
//     			if (pos_y < min_y) {
//     				min_y = pos_y;
//     			}
//     			else if (pos_y > max_y) {
//     				max_y = pos_y;
//     			}
//     			if (pos_z < min_z) {
//     				min_z = pos_z;
//     			}
//     			else if (pos_z > max_z) {
//     				max_z = pos_z;
//     			}
//     		}
//     	}
// 		
// 		min_x -= actor_offset;
// 		max_x += actor_offset;
// 		min_y -= actor_offset;
// 		max_y += actor_offset;
// 		min_z -= actor_offset;
// 		max_z += actor_offset;
// 		
//     	const FVector world_sidelens (max_x - min_x, max_y - min_y, max_z - min_z);
//     	grid_box_sz = world_sidelens / GRIDSPACE_SIDE_LEN;
//     	grid_origin = FVector(min_x, min_y, min_z);
//     	FVector inv_grid_box_sz (
//     		1.0f / grid_box_sz.X,
//     		1.0f / grid_box_sz.Y,
//     		1.0f / grid_box_sz.Z
//     	);
// 		const int JIC_BUF = 3000.0f;
// 		int total_vertex_ct = 0;
// 		TArray<int> dbl_cts;
// 		TArray<MeshDepth> mesh_depths;
// 		for (int k = 0; k < actor_ct; k++) {
// 			mesh_depths.Add(MeshDepth(ground_mesh_actors[k]));
// 		}
//
// 		for (int i = 0; i < actor_ct; i++) {
//     		dbl_cts.Add(0);
// 			auto ground_actor = ground_mesh_actors[i];
// 			const FStaticMeshLODResources& LOD =  ground_actor->GetStaticMeshComponent()->GetStaticMesh()->GetRenderData()->LODResources[0];
//     		const uint32 vertex_ct = LOD.GetNumVertices();
//     		total_vertex_ct += vertex_ct;
//
// 			for (uint32 j = 0; j < vertex_ct; j++) {
// 				// using the transform here after storing was leading to segfault
//     			auto pos = ground_actor->GetTransform().TransformPosition(LOD.VertexBuffers.PositionVertexBuffer.VertexPosition(j));
//     			auto normal = LOD.VertexBuffers.StaticMeshVertexBuffer.VertexTangentZ(j);
//     			FVector gridspace_pos = (pos - grid_origin) * inv_grid_box_sz;
//     			int32 gspace_x = FMath::Floor(gridspace_pos.X);
//     			int32 gspace_y = FMath::Floor(gridspace_pos.Y);
//     			int32 gspace_z = FMath::Floor(gridspace_pos.Z);
// 				auto& vbox = vertex_grid[gspace_x][gspace_y][gspace_z];
//     			
//     			bool double_vertex = false;
//     			for (int k = 0; k < vbox.Num(); k++) {
//     				if (vbox[k] == pos) {
// 						dbl_cts[i]++;
//     					double_vertex = true;
//     					break;
//     				}
//     			}
//     			if (double_vertex) {
//     				continue;
//     			}
//
//     			// line trace test to see if points are inside other meshes; if so, bad for business
// 				const FVector tr_high (pos.X, pos.Y, max_z + JIC_BUF);
//     			const FVector tr_low (pos.X, pos.Y, min_z - JIC_BUF);
//     			tr_start = tr_high;
// 				for (int k = 0; k < actor_ct; k++) {
// 					mesh_depths[k].depths.Empty();
// 				}
// 		
// 				// going down
//     // 			float depth = 0.0f;
//     // 			while (true) {
// 				// 	FHitResult hit;
//     // 				world->LineTraceSingleByChannel(
// 				// 		hit,
// 				// 		tr_start,
// 				// 		tr_low,
// 				// 		ECC_WorldStatic
// 				// 	);
// 			 //
//     // 				if (hit.Distance > 0.0f) {
//     // 					depth += hit.Distance;
//     // 					AStaticMeshActor* sm = Cast<AStaticMeshActor>(hit.Actor);
//     // 					if (sm) {
//     // 						int sm_index = ground_mesh_actors.Find(sm);
//     // 						if (sm_index >= 0) {
//     // 							mesh_depths[sm_index].depths.Add(depth);
//     // 						}
//     // 					}
// 				// 		depth += 0.5f;
// 				// 		tr_start.Z -= hit.Distance + 0.5f;
//     // 				}
//     // 				else {
//     // 					break;
//     // 				}
// 				// }
//     // 			
// 				// // going up
//     // 			tr_start = tr_low;
//     // 			depth = tr_high.Z - tr_low.Z;
//     // 			while (true) {
// 				// 	FHitResult hit;
// 				// 	world->LineTraceSingleByChannel(
// 				// 		hit,
// 				// 		tr_start,
// 				// 		tr_high,
// 				// 		ECC_WorldStatic
// 				// 	);
// 			 //
// 				// 	if (hit.Distance > 0.0f) {
// 				// 		depth -= hit.Distance;
// 				// 		AStaticMeshActor* sm = Cast<AStaticMeshActor>(hit.Actor);
// 				// 		if (sm) {
// 				// 			int sm_index = ground_mesh_actors.Find(sm);
// 				// 			if (sm_index >= 0) {
// 				// 				mesh_depths[sm_index].depths.Add(depth);
// 				// 			}
// 				// 		}
// 				// 		depth -= 0.5f;
// 				// 		tr_start.Z += hit.Distance + 0.5f;
// 				// 	}
// 				// 	else {
// 				// 		break;
// 				// 	}
// 				// }
// 			 //
// 				// float pos_depth = tr_high.Z - pos.Z;
// 				// bool inside = false;
// 				// for (int k = 0; k < actor_ct && !inside; k++) {
// 				// 	if (k == i) {
// 				// 		continue;
// 				// 	}
// 				// 	auto& depths = mesh_depths[i].depths;
// 				// 	depths.Sort();
// 				// 	int depth_ct = depths.Num();
// 				// 	if (depth_ct >= 2) {
// 				// 		// comm::print("depth_ct: %d", depth_ct);
// 				// 		// comm::print("pos depth: %.2f", pos_depth);
// 				// 	}
// 				// 	for (int m = 1; m < depth_ct; m += 2) {
// 				// 		// comm::print("other depths: %.2f, %.2f", depths[m-1], depths[m]);
// 				// 		if (pos_depth > depths[m - 1] && pos_depth < depths[m]) {
// 				// 			inside = true;
// 				// 			break;
// 				// 		}
// 				// 	}
// 				// }
// 				// if (inside) {
// 				// 	continue;
// 				// }
// 				
// 				vbox.Add(pos);
// 				normal_grid[gspace_x][gspace_y][gspace_z].Add(normal);
// 			}
// 		}
// 		// comm::print("vertex ct: %d", total_vertex_ct);
// 		for (int i = 0; i < dbl_cts.Num(); i++) {
// 			// comm::print("double ct on mesh %d: %d", i, dbl_cts[i]);
// 		}
//     }
//
// 	void init_graph_nodes() {
// 		for (int i = 0; i < GRIDSPACE_SIDE_LEN; i++) {
// 			for (int j = 0; j < GRIDSPACE_SIDE_LEN; j++) {
// 				for (int k = 0; k < GRIDSPACE_SIDE_LEN; k++) {
// 					VertexGraphPopulatorBox vgraph_buf[VGRAPH_POPULATOR_BUF_SIZE];
// 					int vgraph_buf_ct = 0;
// 					TArray<FVector>& box_vertices = vertex_grid[i][j][k];
// 					TArray<FVector>& box_normals = normal_grid[i][j][k];
// 					const int vertex_ct = box_vertices.Num();
// 					if (vertex_ct > VGRAPH_POPULATOR_BUF_SIZE) {
// 						// comm::print("ERROR Cling::init_graph() bad vgraph buf size");
// 						return;
// 					}
// 					for (int m = 0; m < vertex_ct; m++) {
// 						const FVector& vert_position = box_vertices[m];
// 						bool inside_vgraph_box = false;
// 						for (int n = 0; n < vgraph_buf_ct; n++) {
// 							const auto& vbox = vgraph_buf[n];
// 							if (
// 								vert_position.X > vbox.lower.X && vert_position.X < vbox.upper.X
// 								&& vert_position.Y > vbox.lower.Y && vert_position.Y < vbox.upper.Y
// 								&& vert_position.Z > vbox.lower.Z && vert_position.Z < vbox.upper.Z
// 							) {
// 								inside_vgraph_box = true;
// 								break;
// 							}
// 						}
// 						if (!inside_vgraph_box) {
// 							auto& vbox = vgraph_buf[vgraph_buf_ct++];
// 							vbox.position = vert_position;
// 							vbox.normal = box_normals[m];
// 							vbox.lower = vert_position + vgraph_pop_lo_offset;
// 							vbox.upper = vert_position + vgraph_pop_up_offset;
// 						}
// 					}
// 					// comm::print("reduced from %d to %d", box_index_ct, vgraph_buf_ct);
// 					if (vgraph_buf_ct == 0) {
// 						vg_node_cts[i][j][k] = 0;
// 						continue;
// 					}
// 					auto& vgraph_box = vertex_graph[i][j][k];
// 					if (vgraph_buf_ct <= NODES_PER_BOX) {
// 						vg_node_cts[i][j][k] = vgraph_buf_ct;
// 						for (int m = 0; m < vgraph_buf_ct; m++) {
// 							vgraph_box[m].location = vgraph_buf[m].position;
// 							vgraph_box[m].normal = vgraph_buf[m].normal;	
// 						}
// 					}
// 					else {
// 						vg_node_cts[i][j][k] = NODES_PER_BOX;
// 						for (int m = 0; m < NODES_PER_BOX; m++) {
// 							const int n = (int)((float)m / NODES_PER_BOX * vgraph_buf_ct);
// 							vgraph_box[m].location = vgraph_buf[n].position;
// 							vgraph_box[m].normal = vgraph_buf[n].normal;
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
//
// 	void init_graph_connect_edges(int skip, int node_ct, VertexNode* vgraph_box, const TArray<VNode>& edges) {
// 		int edge_ct = edges.Num();
// 		for (int n = 0; n < node_ct; n++) {
// 			if (n == skip) {
// 				continue;
// 			}
// 			
// 			// check for existing connection
// 			auto& vnode_b = vgraph_box[n];
// 			bool already_connected = false;
// 			for (int i = 0; i < edge_ct; i++) {
// 				if (&vnode_b == edges[i]) {
// 					already_connected = true;
// 					break;
// 				}	
// 			}
// 			if (already_connected) {
// 				continue;
// 			}
// 			
// 			const FVector& vnode_b_loc = vnode_b.location;
// 			
// 			// too far?
// 			const FVector vnode_b_vec = vnode_b_loc - vnode_a_loc;
// 			const float dist = vnode_b_vec.Size();
// 			if (dist > Cling::VGRAPH_MAX_EDGE_LEN) {
// 				continue;
// 			}
// 			dist_ok++;
// 			
// 			// within edge min and edge max theta?
// 			const FVector vnode_b_vec_norm = vnode_b_vec / dist;
// 			const float cos_theta = FVector::DotProduct(vnode_a_normal, vnode_b_vec_norm);
// 			const float theta = FMath::Acos(cos_theta);
// 			if (theta < edge_min_theta || theta > edge_max_theta) {
// 				continue;
// 			}
// 			theta_ok++;
// 			
// 			// line trace within hit epsilon allowance? (going both directions because a hit won't trigger from behind a mesh)
// 			float tr_min_dist = (tr_start - vnode_b_loc).Size();
// 			FVector tr_end = tr_start + (vnode_b_loc - tr_start).GetSafeNormal() * trace_len;
// 			FHitResult hit;
// 			world->LineTraceSingleByChannel(
// 				hit,
// 				tr_start,
// 				tr_end,
// 				ECC_WorldStatic
// 			);
// 			if (hit.Distance < tr_min_dist * hit_epsilon_factor) {
// 				tr_starts.Add(tr_start);
// 				tr_ends.Add(vnode_b_loc);
// 				continue;	
// 			}
// 			FVector backward_tr_start = vnode_b_loc + vnode_b.normal * VGRAPH_NORM_HEIGHT;
// 			FVector backward_tr_end = backward_tr_start + (vnode_a_loc - backward_tr_start).GetSafeNormal() * trace_len;
// 			tr_min_dist = (tr_start - vnode_a_loc).Size();
// 			world->LineTraceSingleByChannel(
// 				hit,
// 				backward_tr_start,
// 				backward_tr_end,
// 				ECC_WorldStatic
// 			);
// 			if (hit.Distance < tr_min_dist * hit_epsilon_factor) {
// 				tr_starts.Add(tr_start);
// 				tr_ends.Add(vnode_b_loc);
// 				continue;	
// 			}
// 			trace_ok++;
// 			
// 			// project onto normal plane
// 			FVector projected = vnode_b_vec - vnode_a_normal * FVector::DotProduct(vnode_b_vec, vnode_a_normal);
// 			projected = projected.GetUnsafeNormal();
// 			
// 			// dp with 'right' and 'up' (relatively, given they are on the plane and at a right angle), classify
// 			const float theta_1 = FMath::Acos(FVector::DotProduct(vnode_a_plane_right, projected));
// 			const float theta_2 = FMath::Acos(FVector::DotProduct(vnode_a_plane_up, projected));
// 			float phi;
// 			if (theta_2 < HALF_PI) {
// 				phi = theta_1;
// 			}
// 			else {
// 				phi = TWO_PI - theta_1;
// 			}
// 			thetas.Add(phi);
//
// 			for (int o = 0; o < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; o++) {
// 				if (phi < radial_edge_phi[o]) {
// 					if (dist < point_distances[o]) {
// 						radial_edge_allowance[o] = &vgraph_box[n];
// 						point_distances[o] = dist;
// 					}
// 					break;
// 				}
// 			}
// 		}
// 	}
//
// 	void init_graph_cull() {
// 		// culling degenerate nodes over 5 passes
// 		int self_removed = 0;
// 		int have_nodes = 0;
// 		for (int a = 0; a < 5; a++) {
// 			for (int i = 0; i < GRIDSPACE_SIDE_LEN; i++) {
// 				for (int j = 0; j < GRIDSPACE_SIDE_LEN; j++) {
// 					for (int k = 0; k < GRIDSPACE_SIDE_LEN; k++) {
// 						int vg_node_ct = vg_node_cts[i][j][k];
// 						if (vg_node_ct  > 0) {
// 							have_nodes++;
// 						}
// 						auto vgraph_box = vertex_graph[i][j][k];
// 		
// 						int m = 0;
// 						while (m < vg_node_ct) {
// 							auto& vnode = vgraph_box[m];
// 							bool remove = false;
//
// 							// some nodes get connected to themselves, which is not happening by having multiple
// 							// references of the same node within the same box in the graph. where it's happening,
// 							// I'm not sure, but it can be cleaned up here.
// 							int removed = vnode.edges.Remove(&vnode);
// 							if (removed > 0) {
// 								self_removed += removed;
// 							}
// 							const int vnode_edge_ct = vnode.edges.Num();
//
// 							// remove anything from the graph that almost certainly isn't contributing
// 							if (vnode_edge_ct < 2) {
// 								remove = true;
// 							}
// 							else {
// 								// test to see if the node is 3-dimensionally acute, which causes pathing issues
// 								FVector avg_edge_dir = FVector::ZeroVector;
// 								TArray<FVector> edge_dirs;
// 								for (int n = 0; n < vnode_edge_ct; n++) {
// 									const FVector edge_dir = (vnode.edges[n]->location - vnode.location).GetSafeNormal();
// 									edge_dirs.Add(edge_dir);
// 									avg_edge_dir += edge_dir;
// 								}
// 								avg_edge_dir *= 1.0f / vnode_edge_ct;
// 								remove = true;
// 								float theta_sum = 0.0f;
// 								for (int n = 0; n < vnode_edge_ct; n++) {
// 									const float theta = FMath::Acos(FVector::DotProduct(edge_dirs[n], avg_edge_dir));
// 									theta_sum += theta;
// 									if (theta_sum > HALF_PI * 1.1) {
// 										remove = false;
// 										break;
// 									}
// 								}
// 							}
//
// 							if (remove) {
// 								// for each node in edges, remove this node from its edges
// 								for (int n = 0; n < vnode_edge_ct; n++) {
// 									const VNode edge = vnode.edges[n];
// 									edge->edges.Remove(&vnode);
// 								}
// 								// copy back
// 								for (int n = m + 1; n < vg_node_ct; n++) {
// 									VertexNode& prev = vgraph_box[n-1];
// 									const VertexNode& cur  = vgraph_box[n];
// 									auto& cur_edges = cur.edges;
// 									for (int o = 0; o < cur.edges.Num(); o++) {
// 										VNode edge = cur.edges[o];
// 										for (int p = 0; p < edge->edges.Num(); p++) {
// 											if (edge->edges[p] == &cur) {
// 												edge->edges[p] = &prev;
// 												break;
// 											}
// 										}
// 									}
// 									prev = cur;
// 								}
// 								vg_node_ct--;
// 							}
// 							else {
// 								m++;
// 							}
// 						}
// 						vg_node_cts[i][j][k] = vg_node_ct;
// 					}
// 				}
// 			}
// 		}
// 		// comm::print("removed %d self refs, have nodes: %d", self_removed, have_nodes);
// 	}
//
// 	void init_graph() {
//     	// comm::print("init graph");
// 		
// 		init_graph_nodes();
//     	
//     	edge_min_theta = PI / 6.0f;
//     	edge_max_theta = PI - edge_min_theta;
//     	hit_epsilon_factor = 0.97f;
//      
//     	// comm::print("edge min: %.2f, edge max: %.2f", edge_min_theta * 180/PI, edge_max_theta*180/PI);
//      
//     	// Using SAS to determine trace len (and giving it some room)
//     	{
//     		const float edge_min_phi = PI - edge_min_theta;
//     		trace_len = FMath::Sqrt(
//     			Cling::VGRAPH_MAX_EDGE_LEN * Cling::VGRAPH_MAX_EDGE_LEN
//     			+ VGRAPH_NORM_HEIGHT * VGRAPH_NORM_HEIGHT
//     			+ 2 * Cling::VGRAPH_MAX_EDGE_LEN * VGRAPH_NORM_HEIGHT * FMath::Cos(edge_min_phi)
//     		) * 1.1f;
//     	}
//
// 		// dividing each vertex normal plane into a pie, one outgoing connection per slice
//     	const float div_len = TWO_PI / VGRAPH_RADIAL_ALLOWANCE_DIVISIONS;
//     	for (int i = 1; i < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; i++) {
//     		radial_edge_phi[i-1] = div_len * i;
//     	}
//     	radial_edge_phi[VGRAPH_RADIAL_ALLOWANCE_DIVISIONS-1] = 2 * PI;
//      
// 		int vert_ct = 0;	
//     	int assign_ct = 0;
// 		
//     	const float test_epsilon_a = 1e-1;
//     	const float test_epsilon_b = PI - test_epsilon_a;
// 		TArray<int> collected_m;
// 		
//     	for (int i = 0; i < GRIDSPACE_SIDE_LEN; i++) {
//     		for (int j = 0; j < GRIDSPACE_SIDE_LEN; j++) {
//     			for (int k = 0; k < GRIDSPACE_SIDE_LEN; k++) {
//     				
//     				int vg_node_ct = vg_node_cts[i][j][k];
//     				auto vgraph_box = vertex_graph[i][j][k];
//     				
//     				for (int m = 0; m < vg_node_ct; m++) {
//     					collected_m.Add(m);
//     					vert_ct++;
//     					auto& vnode_a = vgraph_box[m];
//     					vnode_a_loc = vnode_a.location;
//     					vnode_a_normal = vnode_a.normal;
//     					
//     					float test_theta = FMath::Acos(FVector::DotProduct(vnode_a_normal, FVector::UpVector));
//     					if (test_theta < test_epsilon_a || test_theta > test_epsilon_b) {
//     						vnode_a_plane_right = FVector::CrossProduct(vnode_a_normal, FVector::RightVector).GetUnsafeNormal();
//     					}
//     					else {
//     						vnode_a_plane_right = FVector::CrossProduct(vnode_a_normal, FVector::UpVector).GetUnsafeNormal();	
//     					}
//     					vnode_a_plane_up = FVector::CrossProduct(vnode_a_normal, vnode_a_plane_right).GetUnsafeNormal();
//     					tr_start = vnode_a_loc + vnode_a_normal * VGRAPH_NORM_HEIGHT;
//
//     					// dbg vals
//     					starts.Add(vnode_a_loc);
//     					rights.Add(vnode_a_plane_right);
//     					ups.Add(vnode_a_plane_up);
//     					dbg_normals.Add(vnode_a_normal);
//     					
//     					for (int n = 0; n < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; n++) {
//     						radial_edge_allowance[n] = nullptr;
//     					}
//     					for (int n = 0; n < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; n++) {
//     						point_distances[n] = FLT_MAX;
//     					}
//
//     					// make edges within the current grid box
//     					init_graph_connect_edges(m, vg_node_ct, vgraph_box, vnode_a.edges);
//     					
//     					// make edges to neighboring grid boxes
// 						const int i_low = i > 0 ? i - 1 : i;
//     					const int j_low = j > 0 ? j - 1 : j;
//     					const int k_low = k > 0 ? k - 1 : k;
// 						const int i_high = i < GRIDSPACE_SIDE_LEN-1 ? i + 1 : i;
//     					const int j_high = j < GRIDSPACE_SIDE_LEN-1 ? j + 1 : j;
//     					const int k_high = k < GRIDSPACE_SIDE_LEN-1 ? k + 1 : k;
//     					for (int x = i_low; x <= i_high; x++) {
//     						for (int y = j_low; y <= j_high; y++) {
//     							for (int z = k_low; z <= k_high; z++) {
//     								if (x == i && y == j && z == k) {
//     									continue;
//     								}
//     								const int inner_vg_node_ct = vg_node_cts[x][y][z];
//     								auto inner_vgraph_box = vertex_graph[x][y][z];
//     								init_graph_connect_edges(-1, inner_vg_node_ct, inner_vgraph_box, vnode_a.edges);	
//     							}
//     						}
//     					}
//     					for (int n = 0; n < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; n++) {
//     						VNode node = radial_edge_allowance[n];
//     						if (node == nullptr || node == &vnode_a) {
//     							continue;
//     						}
//     						assign_ct++;
//     						vnode_a.edges.Add(node);
//     						vnode_a.weights.Add(point_distances[n]);
//     						node->edges.Add(&vnode_a);
//     						node->weights.Add(point_distances[n]);
//     					}
//     				}
//     			}
//     		}
//     	}
// 		
// 		init_graph_cull();
//
//     	// comm::print("vert ct: %d, dist ok: %d, theta ok: %d\ntrace ok: %d, assign_ct: %d", vert_ct, dist_ok, theta_ok, trace_ok, assign_ct);
//     	int counts[VGRAPH_RADIAL_ALLOWANCE_DIVISIONS];
//     	for (int i = 0; i < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; i++) {
//     		counts[i] = 0;
//     	}
//     	// comm::print("thetas num: %d", thetas.Num());
//     	for (int i = 0; i < thetas.Num(); i++) {
//     		for (int j = 0; j < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; j++) {
//     			if (thetas[i] <= radial_edge_phi[j]) {
//     				counts[j] += 1;
//     				break;
//     			}
//     		}
//     	}
//     	for (int i = 0; i < VGRAPH_RADIAL_ALLOWANCE_DIVISIONS; i++) {
//     		float perc = (float)counts[i] / (float)thetas.Num();
//     		// comm::print("less than %.2f: %d", radial_edge_phi[i] * 180 / PI, counts[i]);
//     	}
//     }
//     
//     void dbg_draw_ant_orientation() {
//     	const FVector draw_start = ant_loc + FVector::UpVector * 32.0f;
//     	DrawDebugLine(world, draw_start, vert_pos, FColor::Blue, false, -1, 0, 2);
//     	DrawDebugLine(world, draw_start, draw_start + vert_normal * 48.0f, FColor::Red, false, -1, 0, 2);
//     	DrawDebugLine(world, draw_start, draw_start + ant_forward * 48.0f, FColor::Yellow, false, -1, 0, 2);
//     	DrawDebugLine(world, draw_start, draw_start + ant_right * 48.0f, FColor::Magenta, false, -1, 0, 2);
//     }
//     
//     void dbg_draw_graph() {
//     	for (int i = 0; i < GRIDSPACE_SIDE_LEN; i++) {
//     		for (int j = 0; j < GRIDSPACE_SIDE_LEN; j++) {
//     			for (int k = 0; k < GRIDSPACE_SIDE_LEN; k++) {
//     				for (int m = 0; m < vg_node_cts[i][j][k]; m++) {
//     					auto& vnode = vertex_graph[i][j][k][m];
//     					DrawDebugPoint(world, vnode.location, 15.0f, FColor::Red);
//     					for (int n = 0; n < vnode.edges.Num(); n++) {
//     						DrawDebugLine(world, vnode.location, vnode.edges[n]->location, FColor::Blue, false, -1, 0, 1);
//     					}
//     				}
//     			}
//     		}
//     	}
//     }
//     
//     void dbg_draw_graph_progressive() {
// 		const int _stop = GRIDSPACE_SIDE_LEN * GRIDSPACE_SIDE_LEN * GRIDSPACE_SIDE_LEN;
// 		TArray<FVector*> va;
// 		TArray<FVector*> vb;
// 		for (int i = 0; i < 60; i++) {
// 			int escape_ctr = 0;
// 			do {
// 				dbg_ctr++;
// 				if (dbg_ctr >= 10) {
// 					dbg_ctr = 0;
// 					dbg_k++;
// 					if (dbg_k >= GRIDSPACE_SIDE_LEN) {
// 						dbg_k = 0;
// 						dbg_j++;
// 						if (dbg_j >= GRIDSPACE_SIDE_LEN) {
// 							dbg_j = 0;
// 							dbg_i++;
// 							if (dbg_i >= GRIDSPACE_SIDE_LEN) {
// 								dbg_i = 0;
// 							}
// 						}
// 					}
// 				}
// 				if (escape_ctr++ >= _stop) {
// 					break;
// 				}
// 			} while (vg_node_cts[dbg_i][dbg_j][dbg_k] == 0);
// 			for (int m = 0; m < vg_node_cts[dbg_i][dbg_j][dbg_k]; m++) {
// 				auto& vnode = vertex_graph[dbg_i][dbg_j][dbg_k][m];
// 				for (int n = 0; n < vnode.edges.Num(); n++) {
// 					FVector& vector_a = vnode.location;
// 					FVector& vector_b = vnode.edges[n]->location;
// 					if (vb.Find(&vector_a) >= 0 || va.Find(&vector_b) >= 0) {
// 						continue;
// 					}
// 					game_instance->spawn_line(vnode.location, vnode.edges[n]->location);
// 					va.Add(&vector_a);
// 					vb.Add(&vector_b);
// 				}
// 			}
// 		}
//     }
// 	
// 	void dbg_draw_grid() {
// 		for (int i = 0; i < GRIDSPACE_SIDE_LEN; i++) {
// 			for (int j = 0; j < GRIDSPACE_SIDE_LEN; j++) {
// 				for (int k = 0; k < GRIDSPACE_SIDE_LEN; k++) {
// 					TArray<FVector>& grid_box = vertex_grid[i][j][k];
// 					TArray<FVector>& normals = normal_grid[i][j][k];
// 					for (int m = 0; m < grid_box.Num(); m++) {
// 						FVector& vertex_pos = grid_box[m];
// 						DrawDebugPoint(world, vertex_pos, 20.0f, FColor::Red);
// 						DrawDebugLine(world, vertex_pos, vertex_pos + normals[m] * VGRAPH_NORM_HEIGHT, FColor::Blue, false, -1, 0, 1.0f);
// 					}
// 				}
// 			}
// 		}	
// 	}
//
// 	void dbg_draw_grid_progressive() {
// 		const int _stop = GRIDSPACE_SIDE_LEN * GRIDSPACE_SIDE_LEN * GRIDSPACE_SIDE_LEN;
// 		for (int i = 0; i < 90; i++) {
// 			int escape_ctr = 0;
// 			do {
// 				dbg_ctr++;
// 				if (dbg_ctr >= 10) {
// 					dbg_ctr = 0;
// 					dbg_k++;
// 					if (dbg_k >= GRIDSPACE_SIDE_LEN) {
// 						dbg_k = 0;
// 						dbg_j++;
// 						if (dbg_j >= GRIDSPACE_SIDE_LEN) {
// 							dbg_j = 0;
// 							dbg_i++;
// 							if (dbg_i >= GRIDSPACE_SIDE_LEN) {
// 								dbg_i = 0;
// 							}
// 						}
// 					}
// 				}
// 				if (escape_ctr++ >= _stop) {
// 					break;
// 				}
// 			} while (vg_node_cts[dbg_i][dbg_j][dbg_k] == 0);
// 			auto& vbox = vertex_grid[dbg_i][dbg_j][dbg_k];
// 			auto& nbox = normal_grid[dbg_i][dbg_j][dbg_k];
// 			for (int m = 0; m < vg_node_cts[dbg_i][dbg_j][dbg_k]; m++) {
// 				auto& vnode = vbox[m];
// 				auto& normal = nbox[m];
// 				game_instance->spawn_line(vnode, vnode + normal * 50.0f);
// 			}
// 		}
// 	}
//
// 	void dbg_draw_graph_node_dirs() {
// 		for (int i = 0; i < starts.Num(); i++) {
// 			DrawDebugLine(world, starts[i], starts[i] + ups[i] * 10.0f, FColor::Blue, false, -1, 0, 2.0f);
// 			DrawDebugLine(world, starts[i], starts[i] + rights[i] * 10.0f, FColor::Red, false, -1, 0, 2.0f);
// 			DrawDebugLine(world, starts[i], starts[i] + dbg_normals[i] * 10.0f, FColor::Yellow, false, -1, 0, 2.0f);
// 		}
// 	}
//
// 	void dbg_draw_paths() {
// 		for (int i = 0; i < PATH_CACHE_LEN; i++) {
// 			Cling::CL_PATH_STATUS path_status = path_statuses[i];
// 			if (path_status == Cling::CL_PATH_READY) {
// 				int path_len = path_lens[i];
// 				if (i >= Cling::PATH_MAX_LEN) {
// 					// comm::print("dbg_draw_paths(): bad path len");
// 					continue;
// 				}
// 				FVector* path = smoothed_path_cache[i];
// 				for (int j = 0; j < path_len - 1; j++) {
// 					DrawDebugLine(world, *(path + j), *(path + j + 1), FColor::Blue, false, -1, 0, 2);	
// 				}
// 			}
// 		}
// 	}
//
// 	void dbg_draw_graph_near_ant() {
// 		if (ant_ijk_changed) {
// 			game_instance->destroy_line_actors();
// 			auto& vbox = vertex_graph[ant_i][ant_j][ant_k];
// 			for (int i = 0; i < vg_node_cts[ant_i][ant_j][ant_k]; i++) {
// 				auto& v = vbox[i];
// 				const FVector& vpos = v.location;
// 				auto& e = v.edges;
// 				for (int j = 0; j < e.Num(); j++) {
// 					auto edge = e[j];
// 					game_instance->spawn_line_actor(vpos, edge->location);
// 				}
// 			}
// 			ant_ijk_changed = false;
// 		}
// 	}
// }
//
// // ---------------------------------------------------------------------------------------------------------------------
// // -------------------------------------------------------------------------------------------------------------- Public
// // ---------------------------------------------------------------------------------------------------------------------
//
// void Cling::init(UWorld* _world, UAntGI* gi, const TArray<AStaticMeshActor*>& ground_mesh_actors, float actor_offset) {
// 	// comm::print("cling init start");
// 	world = _world;
// 	game_instance = gi;
// 	init_grid(ground_mesh_actors, actor_offset);
// 	init_graph();
// 	mesh_actors = ground_mesh_actors;
// 	finding_path = false;
// 	for (int i = 0; i < PATH_CACHE_LEN; i++) {
// 		path_statuses[i] = CL_PATH_FREE;
// 	}
// 	pathfinder = new FPathFinder();
// 	ant_ijk_changed = true;
// 	ant_i = 0;
// 	ant_j = 0;
// 	ant_k = 0;
// }
//
// bool Cling::find_nearest_vertex(const FVector& location, const FVector& forward) {
// 	ant_loc = location;
// 	
// 	const FVector inv_grid_box_sz (
// 		1.0f / grid_box_sz.X,
// 		1.0f / grid_box_sz.Y,
// 		1.0f / grid_box_sz.Z
// 	);
// 	const FVector gridspace_pos = (location - grid_origin) * inv_grid_box_sz;
// 	const int32 gspace_x = FMath::Floor(gridspace_pos.X);
// 	const int32 gspace_y = FMath::Floor(gridspace_pos.Y);
// 	const int32 gspace_z = FMath::Floor(gridspace_pos.Z);
// 	
// 	if (
// 		gspace_x >= GRIDSPACE_SIDE_LEN || gspace_y >= GRIDSPACE_SIDE_LEN || gspace_z >= GRIDSPACE_SIDE_LEN
// 		|| gspace_x < 0 || gspace_y < 0 || gspace_z < 0
// 	) {
// 		return false;
// 	}
//
// 	if (gspace_x != ant_i || gspace_y != ant_j || gspace_z != ant_k) {
// 		ant_ijk_changed = true;
// 		ant_i = gspace_x;
// 		ant_j = gspace_y;
// 		ant_k = gspace_z;
// 		// comm::print("updated");
// 	}
// 	
// 	float shortest_sq_dist = FLT_MAX;
// 	FVector shortest_dist_norm;
// 	auto& local_verts = vertex_grid[gspace_x][gspace_y][gspace_z];
// 	auto& local_normals = normal_grid[gspace_x][gspace_y][gspace_z];
// 	for (int i = 0; i < local_verts.Num(); i++) {
// 		const FVector& vert_position = local_verts[i];
// 		const float sq_dist = FVector::DistSquared(vert_position, ant_loc);
// 		if (sq_dist < max_vert_dist_sq && sq_dist < shortest_sq_dist) {
// 			shortest_sq_dist = sq_dist;
// 			shortest_dist_norm = local_normals[i];
// 			vert_pos = vert_position;
// 		}
// 	}
// 	if (shortest_sq_dist < FLT_MAX) {
// 		vert_normal = shortest_dist_norm;
// 		// some hacky math because my brain is mushy
// 		const FVector vert_intermediate = -(forward - vert_normal * (FMath::Abs(FVector::DotProduct(vert_normal, forward)))).GetSafeNormal();
// 		ant_forward = FVector::CrossProduct(vert_intermediate, vert_normal).GetSafeNormal();
// 		ant_right = FVector::CrossProduct(vert_normal, ant_forward).GetSafeNormal();
// 		return true;
// 	}
// 	return false;
// }
//
// void Cling::dbg_draw(float delta_time) {
//
// 	switch(draw_setting) {
// 	case DBG_NONE:
// 		break;
// 	case DBG_ANT_ORIENTATION:
// 		dbg_draw_ant_orientation();
// 		break;
// 	case DBG_GRAPH:
// 		dbg_draw_graph();
// 		break;
// 	case DBG_GRAPH_PROGRESSIVE:
// 		dbg_draw_graph_progressive();
// 		break;
// 	case DBG_GRID:
// 		dbg_draw_grid();
// 		break;
// 	case DBG_GRAPH_NODE_DIRS:
// 		dbg_draw_graph_node_dirs();
// 		break;
// 	case DBG_GRID_PROGRESSIVE:
// 		dbg_draw_grid_progressive();
// 		break;
// 	case DBG_NEAR_ANT:
// 		dbg_draw_graph_near_ant();
// 		break;
// 	case DBG_DRAW_PATHS:
// 		dbg_draw_paths();
// 	default:
// 		;
// 	}
// }
//
// FQuat* Cling::get_cling_rotation(const FVector& mesh_up) {
// 	const float rot_theta_adj = FMath::Acos(FVector::DotProduct(mesh_up, vert_normal)) * 0.5;
// 	if (rot_theta_adj > ROT_EPSILON) {
// 		const FVector rot_axis_adj = FVector::CrossProduct(mesh_up, vert_normal).GetSafeNormal() * FMath::Sin(rot_theta_adj);
// 		last_rotation = FQuat(
// 			rot_axis_adj.X,
// 			rot_axis_adj.Y,
// 			rot_axis_adj.Z,
// 			FMath::Cos(rot_theta_adj)	
// 		);
// 		return &last_rotation;
// 	}
// 	return nullptr;
// }
//
// void Cling::dbg_draw_switch(DBG_DRAW val) {
// 	if (val >= DBG_NONE && val <= DBG_DRAW_PATHS) {
// 		if (draw_setting == DBG_NONE && val > DBG_NONE && val != DBG_DRAW_PATHS && val != DBG_NEAR_ANT) {
// 			for (int i = 0; i < mesh_actors.Num(); i++) {
// 				mesh_actors[i]->GetStaticMeshComponent()->SetVisibility(false);	
// 			}
// 		}
// 		else if (draw_setting != DBG_NONE && val == DBG_NONE) {
// 			for (int i = 0; i < mesh_actors.Num(); i++) {
// 				mesh_actors[i]->GetStaticMeshComponent()->SetVisibility(true);	
// 			}
// 		}
// 		draw_setting = val;
// 	}
// }
//
//
// int Cling::find_path(const FVector& from, const FVector& to, float sq_radius, bool radius_for_start) {
// 	const bool using_radius = sq_radius != -1;
// 	const bool using_radius_for_start = using_radius && radius_for_start;
// 	// look in cache for same or similar path first
// 	for (int i = 0; i < PATH_CACHE_LEN; i++) {
// 		const FVector& path_start = smoothed_path_cache[i][0];
// 		const FVector& path_end = smoothed_path_cache[i][path_lens[i] - 1];
// 		// TODO: also check for end=from and start=to, then give an indication to copy backwards
// 		if (
// 			path_statuses[i] == CL_PATH_READY
// 			&& (
// 				from == path_start
// 				|| (
// 					using_radius_for_start
// 					&& (from - path_start).SizeSquared() < sq_radius
// 				)
// 			)
// 			&& (
// 				to == path_end
// 				|| (
// 					using_radius
// 					&& (to - path_end).SizeSquared() < sq_radius
// 				)
// 			)
// 		) {
// 			return i;
// 		}
// 	}
// 	// if we don't find a cached path and we're not currently pathfinding, start pathfinder
// 	if (!finding_path) {
// 		path_statuses[cache_write_i] = CL_PATH_FINDING;
// 		const VNode start_node = get_nearest_pathable_vnode(from, (radius_for_start ? sq_radius : -1));
// 		const VNode end_node = get_nearest_pathable_vnode(to, sq_radius);
// 		if (start_node == nullptr || end_node == nullptr || start_node->location == end_node->location) {
// 			return -1;
// 		}
// 		for (int j = 0; j < PATH_MAX_LEN; j++) {
// 			path_buf[j] = nullptr;	
// 		}
// 		for (int x = 0; x < GRIDSPACE_SIDE_LEN; x++) {
// 			for (int y = 0; y < GRIDSPACE_SIDE_LEN; y++) {
// 				for (int z = 0; z < GRIDSPACE_SIDE_LEN; z++) {
// 					auto& vbox = vertex_graph[x][y][z];
// 					int box_len = vg_node_cts[x][y][z];
// 					for (int m = 0; m < box_len; m++) {
// 						VertexNode& vnode = vbox[m];
// 						vnode.back_weight = FLT_MAX;
// 						vnode.forward_weight = FLT_MAX;
// 						vnode.prev = nullptr;
// 					}
// 				}
// 			}
// 		}
// 		finding_path = true;
// 		pathfinder->stop_thread();
// 		pathfinder->start_thread(start_node, end_node);
// 		return cache_write_i;
// 	}
// 	return -1;
// }
//
// Cling::CL_PATH_STATUS Cling::get_path_status(int key) {
// 	return path_statuses[key];
// }
//
// FVector* Cling::get_cached_path(int key, int& path_len) {
// 	if (path_statuses[key] == CL_PATH_READY) {
// 		FVector* path = smoothed_path_cache[key];
// 		path_len = path_lens[key];
// 		return path;
// 	}
// 	return nullptr;
// }
//
// void Cling::kill_pathfinder() {
// 	if (pathfinder) {
// 		pathfinder->stop_thread();
// 		delete pathfinder;
// 	}
// 	finding_path = false;
// 	draw_setting = DBG_NONE; // might stop weird cache/crash issue
// }
//
// VertexNode* Cling::get_nearest_pathable_vnode(const FVector& loc, float sq_radius) {
// 	const FVector inv_grid_box_sz (
// 		1.0f / grid_box_sz.X,
// 		1.0f / grid_box_sz.Y,
// 		1.0f / grid_box_sz.Z
// 	);
// 	const FVector gridspace_pos = (loc - grid_origin) * inv_grid_box_sz;
// 	int32 gspace_x = FMath::Floor(gridspace_pos.X);
// 	int32 gspace_y = FMath::Floor(gridspace_pos.Y);
// 	int32 gspace_z = FMath::Floor(gridspace_pos.Z);
//
// 	if (gspace_x < 0) { gspace_x = 0; }
// 	else if (gspace_x >= GRIDSPACE_SIDE_LEN) { gspace_x = GRIDSPACE_SIDE_LEN - 1; }
// 	if (gspace_y < 0) { gspace_y = 0; }
// 	else if (gspace_y >= GRIDSPACE_SIDE_LEN) { gspace_y = GRIDSPACE_SIDE_LEN - 1; }
// 	if (gspace_z < 0) { gspace_z = 0; }
// 	else if (gspace_z >= GRIDSPACE_SIDE_LEN) { gspace_z = GRIDSPACE_SIDE_LEN - 1; }
// 	int x_min = gspace_x;
// 	int x_max = gspace_x;
// 	int y_min = gspace_y;
// 	int y_max = gspace_y;
// 	int z_min = gspace_z;
// 	int z_max = gspace_z;
// 	bool expanded = false;
// 	float min_dist = FLT_MAX;
// 	
// 	if (sq_radius == -1) {
// 		VNode closest_node = nullptr;
// 		while (true) {
// 			for (int x = x_min; x <= x_max; x++) {
// 				for (int y = y_min; y <= y_max; y++) {
// 					for (int z = z_min; z <= z_max; z++) {
// 						if (x == gspace_x && y == gspace_y && z == gspace_z && expanded) {
// 							continue;	
// 						}
// 						auto& vgraph_box = vertex_graph[x][y][z];
// 						const int vg_node_ct = vg_node_cts[x][y][z];
// 						for (int i = 0; i < vg_node_ct; i++) {
// 							const FVector diff = vgraph_box[i].location - loc;
// 							const float dist = diff.Size();
// 							// TODO: fix trace
// 							// const FVector trace_end = loc + diff * 1.5f;
// 							// FHitResult hit;
// 							// world->LineTraceSingleByChannel(
// 							// 	hit,
// 							// 	loc,
// 							// 	trace_end,
// 							// 	ECC_WorldStatic
// 							// );
// 							if (/*hit.Distance >= dist * hit_epsilon_factor &&*/ dist < min_dist) {
// 								closest_node = &vgraph_box[i];
// 								min_dist = dist;
// 							}
// 						}
// 						if (min_dist < FLT_MAX) {
// 							return closest_node;
// 						}
// 					}
// 				}
// 			}
// 			if (!expanded) {
// 				x_min = x_min > 0 ? x_min - 1 : x_min;
// 				y_min = y_min > 0 ? y_min - 1 : y_min;
// 				z_min = z_min > 0 ? z_min - 1 : z_min;
// 				x_max = x_max < GRIDSPACE_SIDE_LEN - 1 ? x_max + 1 : x_max;
// 				y_max = y_max < GRIDSPACE_SIDE_LEN - 1 ? y_max + 1 : y_max;
// 				z_max = z_max < GRIDSPACE_SIDE_LEN - 1 ? z_max + 1 : z_max;
// 				expanded = true;
// 			}
// 			else {
// 				break;
// 			}
// 		}
// 	}
// 	else while (true) {
// 		for (int x = x_min; x <= x_max; x++) {
// 			for (int y = y_min; y <= y_max; y++) {
// 				for (int z = z_min; z <= z_max; z++) {
// 					if (x == gspace_x && y == gspace_y && z == gspace_z && expanded) {
// 						continue;	
// 					}
// 					auto& vgraph_box = vertex_graph[x][y][z];
// 					const int vg_node_ct = vg_node_cts[x][y][z];
// 					for (int i = 0; i < vg_node_ct; i++) {
// 						const FVector diff = vgraph_box[i].location - loc;
// 						const float sq_dist = diff.SizeSquared();
// 						// TODO: fix trace
// 						// const FVector trace_end = loc + diff * 1.1f;
// 						// FHitResult hit;
// 						// world->LineTraceSingleByChannel(
// 						// 	hit,
// 						// 	loc,
// 						// 	trace_end,
// 						// 	ECC_WorldStatic
// 						// );
// 						if (/*hit.Distance >= dist * hit_epsilon_factor &&*/ sq_dist <= sq_radius) {
// 							return &vgraph_box[i];
// 						}
// 					}
// 				}
// 			}
// 		}
// 		if (!expanded) {
// 			x_min = x_min > 0 ? x_min - 1 : x_min;
// 			y_min = y_min > 0 ? y_min - 1 : y_min;
// 			z_min = z_min > 0 ? z_min - 1 : z_min;
// 			x_max = x_max < GRIDSPACE_SIDE_LEN - 1 ? x_max + 1 : x_max;
// 			y_max = y_max < GRIDSPACE_SIDE_LEN - 1 ? y_max + 1 : y_max;
// 			z_max = z_max < GRIDSPACE_SIDE_LEN - 1 ? z_max + 1 : z_max;
// 			expanded = true;
// 		}
// 		else {
// 			break;
// 		}
// 	}
// 	return nullptr;
// }
//
// // ------------------------------------------------------------------------------------------- called only by pathfinder
//
// FVector** Cling::get_path_buf() {
// 	return path_buf;
// }
//
// FVector** Cling::get_path_normals() {
// 	return path_normals;
// }
//
// void Cling::pathfinding_finished(bool success, int path_len) {
// 	if (success) {
// 		path_lens[cache_write_i] = (path_len - 1) * PATH_SMOOTH_DIVISIONS + 1;
// 		path_statuses[cache_write_i] = CL_PATH_READY;
// 		PathSmoothing::smooth_path(world, smoothed_path_cache[cache_write_i], path_buf, path_normals, path_len);
// 	}
// 	else {
// 		// TODO: de-necessitate this overwrite
// 		path_statuses[cache_write_i] = CL_PATH_FREE;
// 	}
// 	// incrementing regardless of success to make sure we don't have a main thread race condition to check the
// 	// status of a path (uncertainty comes from not knowing the order of instances making pathing requests)
// 	cache_write_i = cache_write_i == PATH_CACHE_LEN - 1 ? 0 : cache_write_i + 1;
// 	finding_path = false;
// }

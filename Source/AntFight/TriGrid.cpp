#include "TriGrid.h"
#include "Common.h"
#include "Engine/StaticMeshActor.h"
#include "Rendering/PositionVertexBuffer.h"
#include "AntGI.h"
#include "DrawDebugHelpers.h"

// NOTE: triangles whose vertices are all inside other meshes are culled. However, this can lead to problem cases
// where portions of triangles are outside and are still needed
// TODO: for mesh intersection merging, need to do line test intersections on potential edges to handle 3 or more
// mesh overlaps in the same tri
// TODO: a much better implementation would line test along tri edges for other meshes, which combined with
// the current line test would provide a good way to recover any polygons from the tris, and to get rid of
// tris whos points are outside, but whose bodies are inside

TriGrid::TriGrid() {
	populated = false;
	prog_ctr = FIntVector::ZeroValue;
	prog_outer = 0;
	ground_mesh_visibility = true;
}

TriGrid::~TriGrid() {}

bool TriGrid::init(
		UWorld* _world,
		UAntGI* _gi,
		const TArray<class AStaticMeshActor*>& ground_actors,
		float character_space_buffer
) {
	gi = _gi;
	world = _world;
	mesh_ct = ground_actors.Num();
	for (int i = 0; i < mesh_ct; i++) {
		mesh_cmps.Add(ground_actors[i]->GetStaticMeshComponent());
	}

	if (mesh_ct == 0) {
		comm::print("ERROR TriGrid::init() failure at mesh_ct == 0");
		return false;
	}
	
	TArray<uint16*> tri_index_buf;
	TArray<FVector*> vertex_buf;
	TArray<uint32> cts;
	
	bool so_far_so_success = get_vertex_data(ground_actors, tri_index_buf, vertex_buf, cts);
	if (so_far_so_success) {
		so_far_so_success = set_grid_dimensions(tri_index_buf, vertex_buf, cts, character_space_buffer);
	}
	if (so_far_so_success) {
		so_far_so_success = populate_grid(tri_index_buf, vertex_buf, cts);
	}
	if (so_far_so_success) {
		so_far_so_success = line_test_cull(ground_actors);
	}

	for (int i = 0; i < mesh_ct; i++) {
		delete tri_index_buf[i];
		delete vertex_buf[i];
	}

	if (so_far_so_success) {
		generate_new_triangles_from_overlapping_tris();
	}
	populated = so_far_so_success;
	
	return populated;	
}

void TriGrid::dbg_draw(float delta_time) {
	if (!populated) {
		return;
	}
	
#ifdef TRIGRID_DEBUG
	dbg_draw_edges();
	// dbg_draw_normals();
	// dbg_draw_overlapping_tris();
	// dbg_draw_overlapping_tri_polygons();
	// dbg_draw_overlapping_tri_pts();
	// dbg_draw_new_tris();
#endif
	
	// TODO: also allow optionally turning them on at runtime
}

TArray<Tri>** TriGrid::get_nearby_grid_boxes(const FVector& loc, uint32& ct) {
	const FIntVector gspace_pos ((loc - world_origin) * inv_gbox_world_dims);
	if (
		gspace_pos.X < 0 || gspace_pos.X >= GSPACE_SIDELEN
		|| gspace_pos.Y < 0 || gspace_pos.Y >= GSPACE_SIDELEN
		|| gspace_pos.Z < 0 || gspace_pos.Z >= GSPACE_SIDELEN
	) {
		comm::print("WARNING TriGrid::get_nearby_tri_collection() failure at loc->gspace conversion");
		return nullptr;
	}
	constexpr int GSPACE_SIDELEN_M1 = GSPACE_SIDELEN - 1;
	const uint32 gspace_x_min = gspace_pos.X == 0 ? 0 : gspace_pos.X - 1;
	const uint32 gspace_x_max = gspace_pos.X == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.X + 1;	
	const uint32 gspace_y_min = gspace_pos.Y == 0 ? 0 : gspace_pos.Y - 1;
	const uint32 gspace_y_max = gspace_pos.Y == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.Y + 1;	
	const uint32 gspace_z_min = gspace_pos.Z == 0 ? 0 : gspace_pos.Z - 1;
	const uint32 gspace_z_max = gspace_pos.Z == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.Z + 1;

	tribox_cache[0] = &grid[gspace_pos.X][gspace_pos.Y][gspace_pos.Z];
	uint32 ctr = 1;
	for (uint32 i = gspace_x_min; i <= gspace_x_max; i++) {
		for (uint32 j = gspace_y_min; j <= gspace_y_max; j++) {
			for (uint32 k = gspace_z_min; k <= gspace_z_max; k++) {
				if (i == gspace_pos.X && j == gspace_pos.Y && k == gspace_pos.Z) {
					continue;
				}
				tribox_cache[ctr++] = &grid[i][j][k];			
			}
		}
	}
	ct = ctr;
	return tribox_cache;
}

TArray<Tri>** TriGrid::get_nearby_grid_boxes(int i, int j, int k, uint32& ct, bool forward_only) {
	const FIntVector gspace_pos (i, j, k);
	if (
		gspace_pos.X < 0 || gspace_pos.X >= GSPACE_SIDELEN
		|| gspace_pos.Y < 0 || gspace_pos.Y >= GSPACE_SIDELEN
		|| gspace_pos.Z < 0 || gspace_pos.Z >= GSPACE_SIDELEN
	) {
		comm::print("WARNING TriGrid::get_nearby_tri_collection() failure at loc->gspace conversion");
		return nullptr;
	}
	constexpr int GSPACE_SIDELEN_M1 = GSPACE_SIDELEN - 1;
	uint32 gspace_x_min;
	uint32 gspace_y_min;
	uint32 gspace_z_min;
	if (forward_only) {
		gspace_x_min = gspace_pos.X;	
		gspace_y_min = gspace_pos.Y;	
		gspace_z_min = gspace_pos.Z;	
	}
	else {
		gspace_x_min = gspace_pos.X == 0 ? 0 : gspace_pos.X - 1;
		gspace_y_min = gspace_pos.Y == 0 ? 0 : gspace_pos.Y - 1;
		gspace_z_min = gspace_pos.Z == 0 ? 0 : gspace_pos.Z - 1;
	}
	const uint32 gspace_x_max = gspace_pos.X == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.X + 1;	
	const uint32 gspace_y_max = gspace_pos.Y == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.Y + 1;	
	const uint32 gspace_z_max = gspace_pos.Z == GSPACE_SIDELEN_M1 ? GSPACE_SIDELEN_M1 : gspace_pos.Z + 1;

	tribox_cache[0] = &grid[gspace_pos.X][gspace_pos.Y][gspace_pos.Z];
	uint32 ctr = 1;
	for (uint32 m = gspace_x_min; m <= gspace_x_max; m++) {
		for (uint32 n = gspace_y_min; n <= gspace_y_max; n++) {
			for (uint32 o = gspace_z_min; o <= gspace_z_max; o++) {
				if (m == gspace_pos.X && n == gspace_pos.Y && o == gspace_pos.Z) {
					continue;
				}
				tribox_cache[ctr++] = &grid[m][n][o];
			}
		}
	}
	ct = ctr;
	return tribox_cache;
}

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------:DEBUG
// ---------------------------------------------------------------------------------------------------------------------

void TriGrid::dbg_draw_edges() {
	GSPACE_ITERATE_START
	auto& tribox = grid[i][j][k];
	const uint32 box_tri_ct = tribox.Num();
	FColor c;
	for (uint32 m = 0; m < box_tri_ct; m++) {
		auto& tri = tribox[m];
		if (tri.flags & Tri::OVERLAP) {
			c = FColor::Red;
		}
		else {
			c = FColor::Blue;
		}
		DrawDebugLine(
			world, 
			tri.a,
			tri.b,
			c,
			false,
			-1,
			0,
			1.0f
		);
		DrawDebugLine(
			world, 
			tri.a,
			tri.c,
			c,
			false,
			-1,
			0,
			1.0f
		);
		DrawDebugLine(
			world, 
			tri.b,
			tri.c,
			c,
			false,
			-1,
			0,
			1.0f
		);
	}
	GSPACE_ITERATE_END
}

void TriGrid::dbg_draw_normals() {
	GSPACE_ITERATE_START
	auto& tribox = grid[i][j][k];
	const uint32 box_tri_ct = tribox.Num();
	for (uint32 m = 0; m < box_tri_ct; m++) {
		auto& tri = tribox[m];
		DrawDebugLine(
			world, 
			tri.center,
			tri.center + tri.normal * 30.0f,
			FColor::Blue,
			false,
			-1,
			0,
			1.0f
		);
	}
	GSPACE_ITERATE_END
}

// WARNING: will probably incur massive slowdown
// DebugDraw commands aren't available in packaged build so I need another way to debug packaged.
// Spawns particle lines progressively to show tri edges. Faster than using meshes for lines but still pretty slow.
void TriGrid::dbg_draw_edges_progressive(float delta_time) {
	constexpr float prog_delta = 0.1f;
	constexpr uint32 LINE_SOFT_MAX = 1300;
	prog_outer += delta_time;
	if (prog_outer > prog_delta) {
		prog_outer = 0;
		constexpr uint32 STOP = GSPACE_SIDELEN * GSPACE_SIDELEN * GSPACE_SIDELEN;
		uint32 escape_ctr = 0;
		uint32 line_ctr = 0;
		TArray<FVector> lines;

		while (true) {
			const auto& tribox = grid[prog_ctr.X][prog_ctr.Y][prog_ctr.Z];
			for (int i = 0; i < tribox.Num(); i++) {
				auto& tri = tribox[i];
				const FVector line_0 = tri.a - tri.b;
				const FVector line_1 = tri.a - tri.c;
				const FVector line_2 = tri.c - tri.b;
				bool line_0_drawn = false;
				bool line_1_drawn = false;
				bool line_2_drawn = false;
				for (int j = 0; j < lines.Num(); j++) {
					const FVector& line = lines[j];
					if (!line_0_drawn && (line_0 == line || line_0 == -line)) {
						line_0_drawn = true;
					}
					if (!line_1_drawn && (line_1 == line || line_1 == -line)) {
						line_1_drawn = true;
					}
					if (!line_2_drawn && (line_2 == line || line_2 == -line)) {
						line_2_drawn = true;
					}
				}
				if (!line_0_drawn) {
					gi->spawn_line(tri.a, tri.b);
					lines.Add(line_0);
					line_ctr++;
				}
				if (!line_1_drawn) {
					gi->spawn_line(tri.a, tri.c);
					lines.Add(line_1);
					line_ctr++;
				}
				if (!line_2_drawn) {
					gi->spawn_line(tri.c, tri.b);
					lines.Add(line_2);
					line_ctr++;
				}
			}
			if (line_ctr > LINE_SOFT_MAX) {
				break;
			}
			if (++prog_ctr.Z >= GSPACE_SIDELEN) {
				prog_ctr.Z = 0;
				if (++prog_ctr.Y >= GSPACE_SIDELEN) {
					prog_ctr.Y = 0;
					if (++prog_ctr.X >= GSPACE_SIDELEN) {
						prog_ctr.X = 0;
					}
				}
			}
			if (++escape_ctr >= STOP) {
				break;
			}
		}
	}	
}

// If the tris that are marked to be culled have not yet been, this can be used to show them
void TriGrid::dbg_draw_overlapping_tris() {
	for (int i = 0; i < overlapping_tris.Num(); i++) {
		auto& tri = overlapping_tris[i];
		DrawDebugLine(
			world, 
			tri.a,
			tri.b,
			FColor::Red,
			false,
			-1,
			0,
			1.0f
		);
		DrawDebugLine(
			world, 
			tri.a,
			tri.c,
			FColor::Red,
			false,
			-1,
			0,
			1.0f
		);
		DrawDebugLine(
			world, 
			tri.b,
			tri.c,
			FColor::Red,
			false,
			-1,
			0,
			1.0f
		);
	}	
}

void TriGrid::dbg_toggle_ground_mesh_visibility() {
	if (ground_mesh_visibility) {
		for (int i = 0; i < mesh_cmps.Num(); i++) {
			mesh_cmps[i]->SetVisibility(false);
		}
		ground_mesh_visibility = false;
	}
	else {
		for (int i = 0; i < mesh_cmps.Num(); i++) {
			mesh_cmps[i]->SetVisibility(true);
		}
		ground_mesh_visibility = true;
	}
}

void TriGrid::dbg_draw_overlapping_tri_pts() {
	for (int i = 0; i < dbg_overlapping_tri_intersection_pts.Num(); i++) {
		DrawDebugCircle(world, dbg_overlapping_tri_intersection_pts[i], 3.0f, 2, FColor::Green);
	}
}

void TriGrid::dbg_draw_overlapping_tri_polygons() {
	for (int i = 0; i < overlap_polygons.Num(); i++) {
		Polygon& polygon = overlap_polygons[i];
		for (int j = 0; j < polygon.points.Num() - 1; j++) {
			DrawDebugLine(world, polygon.points[j], polygon.points[j+1], FColor::Blue, false, -1, 0, 1.5f);	
		}
		DrawDebugLine(world, polygon.points[0], polygon.points[polygon.points.Num()-1], FColor::Blue, false, -1, 0, 1.5f);	
	}
}

void TriGrid::dbg_draw_new_tris() {
	for (int i = 0; i < new_tris.Num(); i++) {
		Tri& tri = new_tris[i];
		DrawDebugLine(world, tri.a, tri.b, FColor::Blue, false, -1, 0, 1.0f);
		DrawDebugLine(world, tri.b, tri.c, FColor::Blue, false, -1, 0, 1.0f);
		DrawDebugLine(world, tri.a, tri.c, FColor::Blue, false, -1, 0, 1.0f);
	}	
}

const FVector& TriGrid::get_inv_gbox_world_dims() {
	return inv_gbox_world_dims;
}

const FVector& TriGrid::get_world_origin() {
	return world_origin;
}

TArray<Tri>& TriGrid::get_tribox(int i, int j, int k) {
	return grid[i][j][k];
}

// ---------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------:INIT HELPERS
// ---------------------------------------------------------------------------------------------------------------------

// This one was mostly not fun to figure out. I understand how far from a core feature getting mesh data might
// be for the average Unreal user, but making it this obtuse of a process + sparse to no documentation
// led to there being only two de-mystifying resources on ENQUEUE_RENDER_COMMAND that I could find (thank god
// for those people.) It didn't help that one of them was in Chinese, but they were the ONLY person who used
// the exposed lambda version of the macro; before reading that it was like I was on the moon figuring out what all of
// the un-debuggable pieces going into the macro were. https://www.myredstone.top/archives/1962 <<< my hero
bool TriGrid::get_vertex_data(
	const TArray<class AStaticMeshActor*>& ground_actors,
	TArray<uint16*>& ind_buf,
	TArray<FVector*>& vert_buf,
	TArray<uint32>& cts
) const {
	for (int i = 0; i < mesh_ct; i++) {
		const auto mesh = ground_actors[i]->GetStaticMeshComponent()->GetStaticMesh();
		if (!mesh->bAllowCPUAccess) {
			comm::print("ERROR TriGrid::set_buffer_data() failure at !mesh->bAllowCPUAccess");
			return false;
		}
		const FStaticMeshLODResources& LOD = mesh->GetRenderData()->LODResources[0];
		
		const uint32 tri_index_ct = LOD.IndexBuffer.IndexBufferRHI->GetSize() / sizeof(uint16);
		const uint32 vertex_ct = LOD.VertexBuffers.PositionVertexBuffer.GetNumVertices();

		if (tri_index_ct > UINT16_MAX) {
			comm::print(
				"ERROR TriGrid::set_buffer_data() failure only support meshes of up to %d verts",
				UINT16_MAX
			);
			return false;
		}
		cts.Add(tri_index_ct < vertex_ct ? tri_index_ct : vertex_ct);
		
		uint16* tri_indices = new uint16[tri_index_ct];
		ind_buf.Add(tri_indices);
		const FRawStaticIndexBuffer* findbuf = &LOD.IndexBuffer;
		
		ENQUEUE_RENDER_COMMAND(GetIndexBuffer) (
			[tri_indices, findbuf] (FRHICommandList& rhi_cmd) {
				uint16* _indices = (uint16*)RHILockIndexBuffer(
					findbuf->IndexBufferRHI,
					0,
					findbuf->IndexBufferRHI->GetSize(),
					RLM_ReadOnly
				);
				memcpy(tri_indices, _indices, findbuf->IndexBufferRHI->GetSize());
				RHIUnlockIndexBuffer(findbuf->IndexBufferRHI);
			}
		);
		// thanking a fantastic individual named user37337 on forums.unrealengine.com for this.
		// blocks the main thread until the render thread has given us what we need
		// NOTE: if blocking becomes an issue, just thread the init process
		FlushRenderingCommands(); 

		FVector* vertices = new FVector[vertex_ct];
		vert_buf.Add(vertices);
		const FPositionVertexBuffer* fposbuf = &LOD.VertexBuffers.PositionVertexBuffer;
		
		ENQUEUE_RENDER_COMMAND(GetPositionVertexBuffer) (
			[vertices, fposbuf] (FRHICommandList& rhi_cmd) {
				FVector* _verts = (FVector*)RHILockVertexBuffer(
					fposbuf->VertexBufferRHI,
					0,
					fposbuf->VertexBufferRHI->GetSize(),
					RLM_ReadOnly
				);
				memcpy(vertices, _verts, fposbuf->VertexBufferRHI->GetSize());
				RHIUnlockVertexBuffer(fposbuf->VertexBufferRHI);
			}
		);
		FlushRenderingCommands();

		for (uint32 j = 0; j < vertex_ct; j++) {
			vertices[j] = ground_actors[i]->GetTransform().TransformPosition(vertices[j]);
		}
		
		comm::print(
			"TriGrid: static mesh %d: %d vertices, %d tri indices",
			i,
			vertex_ct,
			tri_index_ct
		);
	}
	return true;
}

bool TriGrid::set_grid_dimensions(
	TArray<uint16*>& ind_buf,
	TArray<FVector*>& vert_buf,
	TArray<uint32>& cts,
	float character_space_buffer
) {
	const int init_success = init_set_grid_dimensions(ind_buf, vert_buf, cts);
	if (!init_success) {
		return false;
	}
	for (int i = 1; i < mesh_ct; i++) {
		const FVector* vertices = vert_buf[i];
		for (uint32 j = 0; j < cts[i]; j++) {
			const FVector& world_position = vertices[j];
			if (world_position.X < wmin.X) {
				wmin.X = world_position.X;
			}
			else if (world_position.X > wmax.X) {
				wmax.X = world_position.X;
			}
			if (world_position.Y < wmin.Y) {
				wmin.Y = world_position.Y;
			}
			else if (world_position.Y > wmax.Y) {
				wmax.Y = world_position.Y;
			}
			if (world_position.Z < wmin.Z) {
				wmin.Z = world_position.Z;
			}
			else if (world_position.Z > wmax.Z) {
				wmax.Z = world_position.Z;
			}
		}
	}

	wmin = wmin - character_space_buffer;
	wmax = wmax + character_space_buffer;
	comm::print("TriGrid: min = (%.2f, %.2f, %.2f)", wmin.X, wmin.Y, wmin.Z);
	comm::print("TriGrid: max = (%.2f, %.2f, %.2f)", wmax.X, wmax.Y, wmax.Z);

	const FVector world_prism_side_lengths = wmax - wmin;
	gbox_world_dims = world_prism_side_lengths / GSPACE_SIDELEN;
	inv_gbox_world_dims = FVector(
		1.0f / gbox_world_dims.X,
		1.0f / gbox_world_dims.Y,
		1.0f / gbox_world_dims.Z
	);
	world_origin = wmin;
	
	return true;
}

// Such a minor thing, but this allows else-ifs throughout + no redoing any work
bool TriGrid::init_set_grid_dimensions(TArray<uint16*>& ind_buf, TArray<FVector*>& vert_buf, TArray<uint32>& cts) {
	const FVector* first_mesh_vertices = vert_buf[0];
	const FVector& fworld_position = first_mesh_vertices[0];
	wmin = fworld_position;
	wmax = fworld_position;	
	
	for (uint32 i = 1; i < cts[0]; i++) {
		const FVector& world_position = first_mesh_vertices[i];
		if (world_position.X < wmin.X) {
			wmin.X = world_position.X;
		}
		else if (world_position.X > wmax.X) {
			wmax.X = world_position.X;
		}
		if (world_position.Y < wmin.Y) {
			wmin.Y = world_position.Y;
		}
		else if (world_position.Y > wmax.Y) {
			wmax.Y = world_position.Y;
		}
		if (world_position.Z < wmin.Z) {
			wmin.Z = world_position.Z;
		}
		else if (world_position.Z > wmax.Z) {
			wmax.Z = world_position.Z;
		}
	}
	return true;
}

bool TriGrid::populate_grid(TArray<uint16*>& ind_buf, TArray<FVector*>& vert_buf, TArray<uint32>& cts) {
	for (int i = 0; i < mesh_ct; i++) {
		const FVector* vertices = vert_buf[i];
		const uint16* indices = ind_buf[i];
		const uint32 vertex_ct = cts[i];
		const uint32 tri_end = vertex_ct - 3;
		for (uint32 j = 0; j < tri_end; j += 3) {
			const uint16& a_index = indices[j];
			const uint16& b_index = indices[j + 1];
			const uint16& c_index = indices[j + 2];
			if (
				a_index < 0 || a_index >= vertex_ct
				|| b_index < 0 || b_index >= vertex_ct
				|| c_index < 0 || c_index >= vertex_ct
			) {
				comm::print("WARNING TriGrid::populate_grid() likely read garbage values in indices");
				continue;	
			}
			const FVector& a = vertices[a_index];
			const FVector& b = vertices[b_index];
			const FVector& c = vertices[c_index];
			Tri tri (a, b, c, i);
			const FVector gspace_pos = (tri.center - world_origin) * inv_gbox_world_dims;
			const int32 gspace_x = FMath::Floor(gspace_pos.X);
			const int32 gspace_y = FMath::Floor(gspace_pos.Y);
			const int32 gspace_z = FMath::Floor(gspace_pos.Z);
			if (
				gspace_x < 0 || gspace_x >= GSPACE_SIDELEN
				|| gspace_y < 0 || gspace_y >= GSPACE_SIDELEN
				|| gspace_z < 0 || gspace_z >= GSPACE_SIDELEN
			) {
				comm::print("WARNING TriGrid::populate_grid() likely read garbage values in vertices");
				continue;
			}
			auto& tribox = grid[gspace_x][gspace_y][gspace_z];
			tribox.Add(tri);
		}	
	}

	return true;
}

// testing whether a tri is inside another mesh and culling it; aids in creating a single continuous orientation mesh
bool TriGrid::line_test_cull(const TArray<AStaticMeshActor*>& ground_actors) {
	
	// any smaller and the test breaks; must have to do with inherent imprecision and/or some implementation detail
	// of line tracing
	constexpr int DEPTH_NUDGE_DELTA = 1.0f; 
	constexpr int EXIT_CT = 100;
	TArray<Tri> inner_tris;
	TArray<FIntVector> inner_tri_boxes;
	
	TArray<TArray<float>> mesh_hit_trackers;
	const int ground_actor_ct = ground_actors.Num();
	for (int i = 0; i < ground_actor_ct; i++) {
		mesh_hit_trackers.Add(TArray<float>());
	}
	
	GSPACE_ITERATE_START

	auto& tribox = grid[i][j][k];
	for (int m = 0; m < tribox.Num(); m++) {
		auto& tri = tribox[m];
		const FVector* tri_pts[3] {&tri.a, &tri.b, &tri.c};
		bool inside[3] {false, false, false};
		// line intersection count test. Does tri point occur after an odd number of intersections with other meshes?
		for (int n = 0; n < 3; n++) {
			const FVector& pt = *tri_pts[n];
			const FVector tr_high (pt.X, pt.Y, wmax.Z);
			const FVector tr_low (pt.X, pt.Y, wmin.Z);
			FVector tr_start = tr_high;

			// going down
			float depth = 0.0f;
			int exit_ctr = 0;
			while (exit_ctr < EXIT_CT) {
				exit_ctr++;
				FHitResult hit;
				world->LineTraceSingleByChannel(
					hit,
					tr_start,
					tr_low,
					ECC_WorldStatic
				);
				if (hit.Distance == 0.0f) {
					break;	
				}
				depth += hit.Distance;
				AStaticMeshActor* sm = Cast<AStaticMeshActor>(hit.Actor);
				if (sm) {
					const int sm_index = ground_actors.Find(sm);
					if (sm_index > -1 && sm_index != tri.mesh_index) {
						mesh_hit_trackers[sm_index].Add(depth);
					}
				}
				depth += DEPTH_NUDGE_DELTA;
				tr_start.Z -= hit.Distance + DEPTH_NUDGE_DELTA;
			}

			// going up
			depth = tr_high.Z - tr_low.Z;
			tr_start = tr_low;
			while (exit_ctr < EXIT_CT) {
				exit_ctr++;
				FHitResult hit;
				world->LineTraceSingleByChannel(
					hit,
					tr_start,
					tr_high,
					ECC_WorldStatic
				);
				if (hit.Distance == 0.0f) {
					break;
				}
				depth -= hit.Distance;
				AStaticMeshActor* sm = Cast<AStaticMeshActor>(hit.Actor);
				if (sm) {
					const int sm_index = ground_actors.Find(sm);
					if (sm_index > -1 && sm_index != tri.mesh_index) {
						mesh_hit_trackers[sm_index].Add(depth);
					}
				}
				depth -= DEPTH_NUDGE_DELTA;
				tr_start.Z += hit.Distance + DEPTH_NUDGE_DELTA;
			}
			
			float pt_depth = tr_high.Z - pt.Z;
			for (int o = 0; o < ground_actor_ct && !inside[n]; o++) {
				auto& hit_tracker = mesh_hit_trackers[o];
				hit_tracker.Sort();
				int hit_ct_m1 = hit_tracker.Num() - 1;
				for (int p = 0; p < hit_ct_m1; p += 2) {
					if (pt_depth > hit_tracker[p] && pt_depth < hit_tracker[p + 1]) {
						inside[n] = true;
						break;
					}
				}
			}

			for (int o = 0; o < ground_actor_ct; o++) {
				mesh_hit_trackers[o].Empty();
			}
		}
		// testing intersections with lines floating above tri edges
		// wouldn't be necessary except the line test has to nudge the tr_start by 1 cm after each collision
		// or it doesn't work.
		for (int n = 0; n < 3; n++) {
			if (inside[n]) {
				continue;
			}
			for (int o = 0; o < 3; o++) {
				if (n == o || inside[o]) {
					continue;
				}
				FHitResult hit;
				FVector tr_start = *tri_pts[n] + tri.normal;
				FVector tr_end =  *tri_pts[o] + tri.normal;
				world->LineTraceSingleByChannel(
					hit,
					tr_start,
					tr_end,
					ECC_WorldStatic
				);
				if (hit.Distance > 0.0f) {
					inside[o] = true;
				}
			}
		}
		if (inside[0] || inside[1] || inside[2]) {
			if (inside[0] && inside[1] && inside[2]) {
				inner_tris.Add(tri);
				inner_tri_boxes.Add(FIntVector(i, j, k));
			}
			else {
				Tri overlapping_tri(tri);
				if (inside[0]) {
					overlapping_tri.flags &= ~Tri::A_OK;
				}
				if (inside[1]) {
					overlapping_tri.flags &= ~Tri::B_OK;
				}
				if (inside[2]) {
					overlapping_tri.flags &= ~Tri::C_OK;
				}
				overlapping_tri.flags |= Tri::OVERLAP;
				overlapping_tris.Add(overlapping_tri);
				overlap_tri_boxes.Add(FIntVector(i, j, k));
			}
		}
	}
	
	GSPACE_ITERATE_END

	for (int i = 0; i < inner_tri_boxes.Num(); i++) {
		auto& itb = inner_tri_boxes[i];
		auto& tribox = grid[itb.X][itb.Y][itb.Z];
		tribox.RemoveSingle(inner_tris[i]);
	}
	
	return true;
}

// assumes tris are non-coplanar and that meshes are enclosed by a single continuous surface
void TriGrid::generate_new_triangles_from_overlapping_tris() {
	const int overlapping_tri_ct = overlapping_tris.Num();
	TArray<TArray<TriEdge>> edges;
	for (int i = 0; i < overlapping_tri_ct; i++) {
		TArray<TriEdge> tri_edges;
		const Tri& tri = overlapping_tris[i];
		// bidirectional edges for intersection testing, unidirectional edges for building polygons commented:
		tri_edges.Add(TriEdge(tri.a, tri.b));	// a -> b
		tri_edges.Add(TriEdge(tri.b, tri.a));	// a -> b
		tri_edges.Add(TriEdge(tri.b, tri.c));	// b -> c
		tri_edges.Add(TriEdge(tri.c, tri.b));	// b -> c
		tri_edges.Add(TriEdge(tri.c, tri.a));	// c -> a
		tri_edges.Add(TriEdge(tri.a, tri.c));	// c -> a
		edges.Add(tri_edges);
	}
	
	TArray<TArray<PolyPoint>> poly_points;
	poly_points.Init(TArray<PolyPoint>(), overlapping_tri_ct);
	for (int i = 0; i < overlapping_tri_ct; i++) {
		const Tri& tri = overlapping_tris[i];
		TArray<PolyPoint>& tri_pts = poly_points[i];
		tri_pts.Add(PolyPoint(tri.a));
		tri_pts.Add(PolyPoint(tri.b));
		tri_pts.Add(PolyPoint(tri.c));
		PolyPoint& pt_a = tri_pts[0];
		PolyPoint& pt_b = tri_pts[1];
		PolyPoint& pt_c = tri_pts[2];
		if (tri.flags & Tri::A_OK && tri.flags & Tri::B_OK) {
			pt_a.pending_edges.Add(1);
		}
		if (tri.flags & Tri::B_OK && tri.flags & Tri::C_OK) {
			pt_b.pending_edges.Add(2);
		}
		if (tri.flags & Tri::C_OK && tri.flags & Tri::A_OK) {
			pt_c.pending_edges.Add(0);
		}
	}

	find_polypoints(edges, poly_points, overlapping_tri_ct);
	generate_polygons(poly_points, overlapping_tri_ct);
	generate_tris_from_polygons();
	add_new_tris_to_grid();	
}

// Each intersection of tri T with tri Q places two new points on Q. Continuous meshes form chains of new points along
// intersecting tris. If tris have original points A, B, C, generate_polygons() connects
// all new points in the A -> B -> C -> A direction, excluding those points that are inside other meshes.
// The direction of intersection point connections can only be inferred once all points are connected (here), when
// we know the chain of new points begins at one tri edge and terminates at another. The new set of points forms a polygon.
// The weakness of this method is that it excludes any polygons where all original tri points are inside other meshes,
// but some tri surface area is still exposed.
void TriGrid::generate_polygons(TArray<TArray<PolyPoint>>& poly_points, int overlapping_tri_ct) {
	for (int i = 0; i < overlapping_tri_ct; i++) {
		PolyPoint* start_pt;
		TArray<PolyPoint>& tri_pts = poly_points[i];
		Tri& overlapping_tri = overlapping_tris[i];
		if (overlapping_tri.flags & Tri::A_OK) {
			start_pt = &poly_points[i][0];	
		}
		else if (overlapping_tri.flags & Tri::B_OK) {
			start_pt = &poly_points[i][1];	
		}
		else { // C_OK
			start_pt = &poly_points[i][2];	
		}
		
		TArray<PolyPoint*> searched;
		searched.Add(start_pt);
		PolyPoint* search_pt = nullptr;
		
		const TArray<int>& pending_edges = start_pt->pending_edges;
		const int pending_edges_ct = pending_edges.Num();
		if (pending_edges_ct == 0) {
			continue;
		}
		float shortest_sq_dist = FLT_MAX;
		const FVector& search_pt_loc = start_pt->pt;
		if (pending_edges_ct == 1) {
			search_pt = &tri_pts[pending_edges[0]];
		}
		else for (int j = 0; j < pending_edges_ct; j++) {
			const float sq_dist = FVector::DistSquared(search_pt_loc, tri_pts[pending_edges[j]].pt);
			if (sq_dist < shortest_sq_dist) {
				shortest_sq_dist = sq_dist;
				search_pt = &tri_pts[pending_edges[j]];
			}
		}
		if (search_pt == nullptr) {
			continue;
		}
		searched.Add(search_pt);
		start_pt->edge = search_pt;
		searched.Add(search_pt);
		FVector prev_edge_dir = (search_pt->pt - start_pt->pt).GetSafeNormal();
		FVector prev_edge_right = FVector::CrossProduct(prev_edge_dir, overlapping_tri.normal).GetSafeNormal();

		// NOTE leaving edges that haven't been "search_pt" out of "searched" since pt merges might lead us back to
		// previously un "search_pt"-ed nodes
		bool closed_polygon = false;
		while (true) {
			const TArray<int>& potential_edges = search_pt->pending_edges;
			const int potential_edge_ct = potential_edges.Num();
			if (potential_edge_ct == 0) {
				break;
			}
			PolyPoint* new_search_pt = nullptr;
			float smallest_theta = FLT_MAX;
			const FVector& search_pt_pos = search_pt->pt;
			FVector new_edge_dir;
			for (int j = 0; j < potential_edge_ct; j++) {
				PolyPoint* edge = &tri_pts[potential_edges[j]];
				if (edge == start_pt && search_pt != start_pt->edge) {
					search_pt->edge = edge;
					closed_polygon = true;
					break;
				}
				if (searched.Find(edge) != -1) {
					continue;
				}
				FVector edge_dir = (edge->pt - search_pt_pos).GetSafeNormal();
				const float sign_theta = FMath::Acos(FVector::DotProduct(edge_dir, prev_edge_right));
				const float sign = sign_theta < HALF_PI ? -1.0f : 1.0f;
				const float theta = FMath::Acos(FVector::DotProduct(edge_dir, prev_edge_dir)) * sign;
				if (theta < smallest_theta) {
					smallest_theta = theta;
					new_search_pt = edge;
					new_edge_dir = edge_dir;
				}
			}
			if (new_search_pt == nullptr || closed_polygon) {
				break;
			}
			search_pt->edge = new_search_pt;
			prev_edge_dir = new_edge_dir;
			search_pt = new_search_pt;
			searched.Add(search_pt);
		}
		if (closed_polygon) {
			Polygon polygon;
			polygon.normal = overlapping_tris[i].normal;
			polygon.mesh_index = overlapping_tris[i].mesh_index;
			polygon.overlapping_tri_index = i;
			start_pt = search_pt;
			do {
				polygon.points.Add(search_pt->pt);
				search_pt = search_pt->edge;
			} while (search_pt != start_pt);
			overlap_polygons.Add(polygon);
		}
	}
}

void TriGrid::generate_tris_from_polygons() {
	for (int i = 0; i < overlap_polygons.Num(); i++) {
		const Polygon& polygon = overlap_polygons[i];
		const TArray<FVector>& points = polygon.points;
		const int pt_ct = points.Num();
		if (pt_ct == 3) {
			const FVector center = (points[0] + points[1] + points[2]) * ONE_THIRD;
			new_tris.Add(Tri(points[0], points[1], points[2], polygon.mesh_index));
			overlapping_tris[polygon.overlapping_tri_index].flags &= ~Tri::OVERLAP;
			continue;
		}
		bool* pt_unavailable = new bool[pt_ct];
		int start_i = 0;
		
		for (int tri_a = 0; tri_a < pt_ct; ) {
			memset(pt_unavailable, 0, pt_ct * sizeof(bool));
			TArray<Tri> attempt_tris;
			int failure_ct = 0;
			int available_ct = pt_ct;
			bool fill_success = false;

			while (available_ct >= 3) {
				while (pt_unavailable[tri_a]) {
					tri_a++;
					if(tri_a >= pt_ct) {
						tri_a = 0;
					}
				}
				int tri_b = tri_a + 1;
				if (tri_b >= pt_ct) {
					tri_b = 0;
				}
				while (pt_unavailable[tri_b]) {
					tri_b++;
					if (tri_b >= pt_ct) {
						tri_b = 0;
					}
				}
				int tri_c = tri_b + 1;
				if (tri_c >= pt_ct) {
					tri_c = 0;
				}
				while (pt_unavailable[tri_c]) {
					tri_c++;
					if (tri_c >= pt_ct) {
						tri_c = 0;
					}
				}
				if (tri_a == tri_b || tri_a == tri_c || tri_b == tri_c) {
					comm::print("ERROR TriGrid::generate_tris_from_polygons(): bad unavailability management");
					break;
				}

				// I think this is the "ear clipping" method of triangle-izing a polygon
				const FVector& pt_a = points[tri_a];
				const FVector& pt_b = points[tri_b];
				const FVector& pt_c = points[tri_c];
				const FVector u = pt_a - pt_b;
				const FVector v = pt_c - pt_b;
				const FVector w = FVector::CrossProduct(v, polygon.normal).GetSafeNormal();
				const float phi = FMath::Acos(FVector::DotProduct(u, w));
				if (phi >= HALF_PI) {
					tri_a = tri_a == pt_ct - 1 ? 0 : tri_a + 1;
					failure_ct++;
					if (failure_ct >= available_ct) {
						break;
					}
					continue;
				}
				failure_ct = 0;
				attempt_tris.Add(Tri(pt_a, pt_b, pt_c, polygon.mesh_index));
				if (available_ct == 3) {
					fill_success = true;
					break;
				}
				pt_unavailable[tri_b] = true;
				available_ct--;
			}
			if (fill_success) {
				overlapping_tris[polygon.overlapping_tri_index].flags &= ~Tri::OVERLAP;
				for (int j = 0; j < attempt_tris.Num(); j++) {
					new_tris.Add(attempt_tris[j]);
				}
				break;
			}
			start_i += 1;
			tri_a = start_i;
		}
		delete pt_unavailable;
	}	
}

void TriGrid::add_new_tris_to_grid() {
	// remove overlapping triangles from which new triangles could be made; flag failed ones
	for (int i = 0; i < overlapping_tris.Num(); i++) {
		Tri& tri = overlapping_tris[i];
		if (tri.flags & Tri::OVERLAP) {
			const FIntVector& ot_box = overlap_tri_boxes[i];
			auto& tribox = grid[ot_box.X][ot_box.Y][ot_box.Z];
			const int tri_index = tribox.Find(tri);
			if (tri_index == -1) {
				comm::print("ERROR TriGrid::add_new_tris_to_grid() overlapping tri missing");
			}
			else {
				tribox[tri_index].flags |= Tri::OVERLAP;
			}
		}
		else {
			const FIntVector& ot_box = overlap_tri_boxes[i];
			grid[ot_box.X][ot_box.Y][ot_box.Z].RemoveSingle(tri);
		}
	}
	// add new triangles
	for (int i = 0; i < new_tris.Num(); i++) {
		Tri& tri = new_tris[i];
		const FVector gspace_pos = (tri.center - world_origin) * inv_gbox_world_dims;
		const int32 gspace_x = FMath::Floor(gspace_pos.X);
		const int32 gspace_y = FMath::Floor(gspace_pos.Y);
		const int32 gspace_z = FMath::Floor(gspace_pos.Z);
		grid[gspace_x][gspace_y][gspace_z].Add(tri);
	}
}

void TriGrid::find_polypoints(
	TArray<TArray<TriEdge>>& edges,
	TArray<TArray<PolyPoint>>& poly_points,
	int overlapping_tri_ct
) {
	for (int i = 0; i < overlapping_tri_ct - 1; i++) {
		TArray<TriEdge>& edges_i = edges[i];
		TArray<PolyPoint>& points_i = poly_points[i];
		Tri& tri_i = overlapping_tris[i];
		for (int j = i + 1; j < overlapping_tri_ct; j++) {
			Tri& tri_j = overlapping_tris[j];
			if (tri_j.mesh_index == tri_i.mesh_index) {
				continue;
			}
			if (FMath::Acos(FVector::DotProduct(tri_i.normal, tri_j.normal)) < PI/64.0f) {
				continue;
			}
			PolyPoint intersections_i_on_j[2];
			PolyPoint intersections_j_on_i[2];
			TArray<TriEdge>& edges_j = edges[j];
			TArray<PolyPoint>& points_j = poly_points[j];
			const int intersection_ct_i_on_j = get_intersections(edges_i, tri_j, intersections_i_on_j);
			const int intersection_ct_j_on_i = get_intersections(edges_j, tri_i, intersections_j_on_i );
	
			if (intersection_ct_i_on_j == 2 && intersection_ct_j_on_i == 0) {
				PolyPoint& intersection_a = intersections_i_on_j[0];
				PolyPoint& intersection_b = intersections_i_on_j[1];
				
				add_pending_connections(points_i, intersection_a, intersection_b, tri_i.flags);
				intersection_a.tri_edge_flags = 0;
				intersection_b.tri_edge_flags = 0;
				add_pending_connections(points_j, intersection_a, intersection_b, tri_j.flags);
			}
			else if (intersection_ct_j_on_i == 2 && intersection_ct_i_on_j == 0) {
				PolyPoint& intersection_a = intersections_j_on_i[0];
				PolyPoint& intersection_b = intersections_j_on_i[1];
				
				add_pending_connections(points_j, intersection_a, intersection_b, tri_j.flags);
				intersection_a.tri_edge_flags = 0;
				intersection_b.tri_edge_flags = 0;
				add_pending_connections(points_i, intersection_a, intersection_b, tri_i.flags);
			}
			else if (intersection_ct_i_on_j == 1) {
				if (intersection_ct_j_on_i == 1) {
					PolyPoint& intersection_a = intersections_i_on_j[0];
					PolyPoint intersection_a_cpy = intersection_a;
					intersection_a_cpy.tri_edge_flags = 0;
					PolyPoint& intersection_b = intersections_j_on_i[0];
					PolyPoint intersection_b_cpy = intersection_b;
					intersection_b_cpy.tri_edge_flags = 0;
					
					add_pending_connections(points_i, intersection_a, intersection_b_cpy, tri_i.flags);	
					add_pending_connections(points_j, intersection_a_cpy, intersection_b, tri_j.flags);	
				}
				else {
					// degenerate pairing. If triangle A's edges intersect triangle B only once, triangle B's edges
					// must do the same with A
					comm::print(
						"WARNING TriGrid::generate_polygons_from_overlapping_tris() degenerate pairing found A."
					);
				}
			}
			else if (!(intersection_ct_i_on_j == 0 && intersection_ct_j_on_i == 0)) {
				// degenerate pairing. Intersections must be 2 & 0, 0 & 2, 1 & 1, or 0 & 0
				comm::print(
					"WARNING TriGrid::generate_polygons_from_overlapping_tris() degenerate pairing found B."
				);
			}
		}
	}
}

int TriGrid::get_intersections(
	TArray<TriEdge>& tri_edges,
	const Tri& overlapping_tri,
	PolyPoint* intersections
) {
	FVector poi;
	int intersection_ct = 0;
	for (int k = 0; k < 6; k++) {
		TriEdge& edge = tri_edges[k];
		const bool this_intersection = set_point_of_intersection(
			edge.origin,
			edge.norm,
			edge.len,
			overlapping_tri,
			poi
		);
		if (this_intersection) {
			PolyPoint pt(poi);
			if (k < 2) {
				pt.tri_edge_flags |= EDGE_AB;
			}
			else if (k < 4) {
				pt.tri_edge_flags |= EDGE_BC;	
			}
			else {
				pt.tri_edge_flags |= EDGE_CA;
			}
			intersections[intersection_ct++] = pt;
			if (intersection_ct == 2) {
				break;
			}
		}
	}
	return intersection_ct;
}

// "pending" here just means we don't yet necessarily know the direction of the connection, since Polygons are
// unidirectional closed graphs
inline void TriGrid::add_pending_connections(TArray<PolyPoint>& points, PolyPoint& a, PolyPoint& b, int tri_flags) {
	int a_index = points.Find(a);
	if (a_index == -1) {
		a_index = points.Add(a);
		dbg_overlapping_tri_intersection_pts.Add(a.pt);
	}
	int b_index = points.Find(b);
	if (b_index == -1) {
		b_index = points.Add(b);
		dbg_overlapping_tri_intersection_pts.Add(b.pt);
	}
	
	PolyPoint* true_a = &points[a_index];
	if (a.tri_edge_flags > 0 && a_index > 2) {
		add_tri_edge_pending_connections(points, a_index, a.tri_edge_flags, tri_flags);
	}
	PolyPoint* true_b = &points[b_index];
	if (b.tri_edge_flags > 0 && b_index > 2) {
		add_tri_edge_pending_connections(points, b_index, b.tri_edge_flags, tri_flags);
	}
	if (true_a != true_b) {
		if (!true_a->already_pending(b_index)) {
			true_a->pending_edges.Add(b_index);
		}
		if (!true_b->already_pending(a_index)) {
			true_b->pending_edges.Add(a_index);
		}
	}
}

// Get poi between an edge and another triangle. returns false when no intersection
// https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d
// Uses Cramer's Rule to solve a system of equations with as many unknowns as there are equations.
// Is uni-directional (which appears to make sense of meshes/raycasts being uni-directional in hits)
bool TriGrid::set_point_of_intersection(
	const FVector& origin,
	const FVector& edge_norm,
	float edge_len,
	const Tri& tri,
	FVector& point_of_intersection
) {
	const DoubleVector e1 (tri.b - tri.a);
	const DoubleVector e2 (tri.c - tri.a);
	const DoubleVector scaled_tri_normal = DoubleVector::CrossProduct(e1, e2);
	const double det = -DoubleVector::DotProduct(edge_norm, scaled_tri_normal);
	const double inv_det = 1.0 / det;
	const DoubleVector ao (origin - tri.a);
	const DoubleVector dao  = DoubleVector::CrossProduct(ao, edge_norm);
	const double u = DoubleVector::DotProduct(e2, dao) * inv_det;
	const double v = -DoubleVector::DotProduct(e1, dao) * inv_det;
	const double t = DoubleVector::DotProduct(ao, scaled_tri_normal) * inv_det;
	point_of_intersection = origin + t * edge_norm;
	return det >= 1e-6f && t >= 0.0f && u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f && edge_len > t;
}

// when a point lands on a tri edge, it is pre-connected here; these are important points because they relate
// direction of graph connection to later create a polygon
void TriGrid::add_tri_edge_pending_connections(TArray<PolyPoint>& points, int pt_index, int pt_flags, int tri_flags) {
	if (pt_flags & EDGE_AB) {
		if (tri_flags & Tri::A_OK && !points[0].already_pending(pt_index)) {
			points[0].pending_edges.Add(pt_index);
		}
		if (tri_flags & Tri::B_OK && !points[pt_index].already_pending(1)) {
			points[pt_index].pending_edges.Add(1);
		}
	}
	else if (pt_flags & EDGE_BC) {
		if (tri_flags & Tri::B_OK && !points[1].already_pending(pt_index)) {
			points[1].pending_edges.Add(pt_index);
		}
		if (tri_flags & Tri::C_OK && !points[pt_index].already_pending(2)) {
			points[pt_index].pending_edges.Add(2);
		}
	}
	else {
		if (tri_flags & Tri::C_OK && !points[2].already_pending(pt_index)) {
			points[2].pending_edges.Add(pt_index);
		}
		if (tri_flags & Tri::A_OK && !points[pt_index].already_pending(0)) {
			points[pt_index].pending_edges.Add(0);
		}
	}
}

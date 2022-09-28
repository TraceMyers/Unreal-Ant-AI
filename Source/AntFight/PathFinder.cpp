#include "PathFinder.h"
#include "Common.h"


FPathFinder::FPathFinder() {
	thread = nullptr;	
}

FPathFinder::~FPathFinder() {
	if (thread) {
		thread->Kill();
		thread->WaitForCompletion();
		delete thread;
	}	
}

bool FPathFinder::start_thread(
	VNode _start,
	VNode _end,
	VNode* _path_buf,
	int* _path_len,
	FThreadSafeBool* _full_path_found,
	AINav* _ai_nav
) {
	if (!thread) {
		start_node = _start;
		end_node = _end;
		path_buf = _path_buf;
		path_len = _path_len;
		full_path_found = _full_path_found;
		ai_nav = _ai_nav;
		run_thread = true;
		thread = FRunnableThread::Create(this, TEXT("FPathFinder"));
		if (!thread) {
			return false;
		}
		return true;
	}
	return false;
}

void FPathFinder::stop_thread() {
	if (thread) {
		thread->Kill();
		thread->WaitForCompletion();
		delete thread;
		thread = nullptr;
	}
}

#pragma endregion

bool FPathFinder::Init() {
	return true;
}

uint32 FPathFinder::Run() {
	end_node_loc = end_node->location;
	search_stack.Add(start_node);
	search_vector = start_node;
	start_node->fore_weight = FVector::Distance(start_node->location, end_node_loc);
	start_node->back_weight = 0.0f;
	
	bool success = false;
	int total_step_ct = 0;
	while (run_thread) {
		success = search_multistep();
		total_step_ct += STEP_CT;
		if (total_step_ct > 100000) {
			run_thread = false;
			break;
		}
	}

	// TODO: program exit? don't need to do this
	// was previously just copying in backwards, which was fine, but doing it this way
	// has the benefit of partial paths; this way is: just going back to find the start then
	// copying up to length of the path or up to PATH_MAX_LEN
	VNode backward_traveler = search_vector;
	VNode next_node = backward_traveler->link;
	while (backward_traveler != start_node) {
		const VNode save_node = backward_traveler;
		backward_traveler = next_node;
		next_node = backward_traveler->link;
		backward_traveler->link = save_node;
	}
	VNode forward_traveler = start_node;
	path_buf[0] = start_node;
	int len = 1;
	while(forward_traveler != search_vector && len < AINav::PATH_MAX_LEN) {
		forward_traveler = forward_traveler->link;
		path_buf[len++] = forward_traveler;
	}
	*path_len = len;
	if (success) {
		*full_path_found = true;	
	}
	else {
		*full_path_found = false;	
	}
	
	ai_nav->pathfinding_finished();
	return 0;
}

void FPathFinder::Stop() {
	run_thread = false;	
}

// basic A*, running step_ct steps before checking back for thread kill
bool FPathFinder::search_multistep() {
	
	for (int a = 0; a < STEP_CT; a++) {
		
		search_stack.RemoveSingle(search_vector);
		auto& sv_edges = search_vector->edges;
		auto& sv_weights = search_vector->weights;
		const int sv_edge_ct = search_vector->edges.Num();
		const float sv_back_weight = search_vector->back_weight;

		VNode sv_candidate = nullptr;
		float sv_candidate_total_weight = sv_back_weight + search_vector->fore_weight;

		for (int i = 0; i < sv_edge_ct; i++) {
			VNode edge = sv_edges[i];

			if (edge == end_node) {
				end_node->link = search_vector;
				search_vector = end_node;
				run_thread = false;
				return true;
			}
			
			const float edge_weight = sv_weights[i];
			const float edge_back_weight = edge->back_weight;
			const float total_back_weight = sv_back_weight + edge_weight;

			// if the edge has already been updated, update its back weight if the search vector has a shorter path
			if (edge->link != nullptr) {
				if (total_back_weight < edge_back_weight) {
					edge->back_weight = total_back_weight;
					edge->link = search_vector;
				}
			}
			// if the edge hasn't been searched yet, see if we're getting closer and add it to the stack
			else {
				search_stack.Add(edge);
				edge->back_weight = total_back_weight;
				edge->fore_weight = FVector::Distance(edge->location, end_node_loc);
				edge->link = search_vector;
			}

			// if this one is closer than the current search vector, we don't need to search for the next sv
			const float new_total_weight = edge->back_weight + edge->fore_weight;
			if (new_total_weight < sv_candidate_total_weight) {
				sv_candidate = edge;
				sv_candidate_total_weight = new_total_weight;	
			}
		}
		
		const int search_stack_len = search_stack.Num();
		if (search_stack_len == 0) {
			run_thread = false;
			return false;
		}

		search_vector = sv_candidate;
		if (search_vector != nullptr) {
			continue;
		}
		float lowest_total_weight = FLT_MAX;
		for (int i = 0; i < search_stack_len; i++) {
			const VNode candidate = search_stack[i];
			const float total_weight = candidate->back_weight + candidate->fore_weight;
			if (total_weight < lowest_total_weight) {
				lowest_total_weight = total_weight;
				search_vector = candidate;
			}
		}
		if (search_vector == nullptr) {
			run_thread = false;
			return false;
		}
	}
	return false;
}




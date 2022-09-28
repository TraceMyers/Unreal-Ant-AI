#pragma once

#include "Common.h"
#include "Tri.h"
#include "DataElement.h"

// NOTE: This was crashing while trying to alloc block sizes 150-250 Tris, and making reallocation contingencies
// for malloc failures in a fast real time app is very time consuming thing. So, I just went back to TArray for now.
// it's in a quarterish-rewritten state at the moment.

// This implementation useful because you can choose only to fill the gaps when you need to, rather
// than every time you remove an element. This allows for in-loop removal, and maybe more performant removal.
// Also pre-allocates for a few elements statically, so it's good for collections of small numbers of elements
// Worse than typical implementations in many cases, but *so* nice in many others.
// TODO: was originally Template, made Tri-only to debug, should try template again

// class TriArray {
// 	
// public:
//
//     TriArray();
//     // TriArray(const TriArray& b);
//     
//     // if elements have been removed and gap_fill has not been called, refers only to the index after the last element
//     // if the array has no gaps, truly refers to the number of elements
//     int len() const;
//     // TriArray& operator = (const TriArray& b);
//     // If an element is removed, gap fill should be called before using [], otherwise may return invalid data
//     Tri& operator[] (int index);
//     const Tri& operator[] (int index) const;
//     // for use anytime, but expect nullptrs in-between data if remove has been called with no gap_fill
//     Tri* get(int index);
//     int append(const Tri& e);
// 	bool remove_at(int index);
// 	bool remove(const Tri& e);
//     int find(const Tri& e);
//             
// private:
//
//     static constexpr int STATIC_DATA_SZ = 6; // any more and Unreal throws a fit when instantiating TriGrid
// 	static constexpr int HEAP_DATA_MIN_SZ = 0;
// 	static constexpr int MIN_SZ = STATIC_DATA_SZ + HEAP_DATA_MIN_SZ;
// 	static constexpr int INIT_HEAP_EXPANSION = 20;
// 	static constexpr int HEAP_EXPANSION_ACCEL = 10;
// 	static constexpr int HEAP_PTR_CT = 10;
//
//     DataElement static_data[STATIC_DATA_SZ];
//     DataElement* heap_data[HEAP_PTR_CT];
// 	int heap_block_szs[HEAP_PTR_CT];
//     int in_i;
//     int top;
//     int total_sz;
// 	int heap_ptr_top;
// 	int heap_expansion = INIT_HEAP_EXPANSION;
//     void find_top_static();
//     void find_top_heap();
//     // previous indices and pointers no longer guaranteed to be valid after calling gap_fill
//     // slower than bulk memcpy but easiest to make implementation of this
//     void gap_fill();
// 	int get_total_sz() const;
// 	int get_in_i() const;
// 	void get_heap_indices(int& heap_ptr_i, int& index) const;
// 	void next_free_heap_indices(int& heap_ptr_i, int& index) const;
// 	
// 	
//     
// };
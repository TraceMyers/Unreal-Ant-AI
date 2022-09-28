#include "TriArray.h"
#include "Tri.h"
#include "CoreMinimal.h"

// TriArray::TriArray() : in_i{0}, top{0}, total_sz{MIN_SZ}, heap_ptr_top{1} {
//     heap_data[0] = nullptr;
//     if (HEAP_DATA_MIN_SZ > 0) {
//         heap_data[0] = new DataElement[HEAP_DATA_MIN_SZ];
//         memset(heap_block_szs, 0, HEAP_DATA_MIN_SZ * sizeof(int));
//         if (!heap_data) {
//             heap_ptr_top = 0;
//             total_sz = STATIC_DATA_SZ;
//         }
//         else {
//             heap_block_szs[0] = HEAP_DATA_MIN_SZ;
//             memset(heap_data, 0, sizeof(DataElement) * HEAP_DATA_MIN_SZ);
//         }
//     }
//     else {
//         heap_ptr_top = 0;
//         total_sz = STATIC_DATA_SZ;
//     }
//     memset(static_data, 0, sizeof(DataElement) * STATIC_DATA_SZ);
// }
//
// // TriArray::TriArray(const TriArray& b) {
// //     DataElement* new_heap = (DataElement*)(malloc(b.heap_sz * sizeof(DataElement)));
// //     memcpy((*this).static_data, b.static_data, STATIC_DATA_SZ * sizeof(DataElement));
// //     memcpy(new_heap, b.heap_data, sizeof(DataElement) * b.heap_sz);
// //     (*this).heap_data = new_heap;
// //     (*this).total_sz = b.total_sz;
// //     (*this).heap_sz = b.heap_sz;
// //     (*this).in_i = b.get_in_i();
// //     (*this).top = b.len();
// // }
//
// int TriArray::get_total_sz() const {
//     return total_sz;
// }
//
// int TriArray::get_in_i() const {
//     return in_i;
// }
//
// void TriArray::get_heap_indices(int& heap_ptr_i, int& heap_i) const {
//     heap_i -= STATIC_DATA_SZ;
//     heap_ptr_i = 0;
//     for (; heap_ptr_i < heap_ptr_top; heap_ptr_i++) {
//         const int block_index = heap_i - heap_block_szs[heap_ptr_i];
//         if (block_index < 0) {
//             break;
//         }
//         heap_i = block_index;
//     }
// }
//
// void TriArray::next_free_heap_indices(int& heap_ptr_i, int& heap_i) const {
//     
// }
//
// int TriArray::len() const {
//      return top;        
//  }
//
// // TriArray& TriArray::operator = (const TriArray& b) {
// //     // if (this == &b) {
// //     //     return *this;
// //     // }
// //     
// //     memcpy((*this).static_data, b.static_data, TriArray::STATIC_DATA_SZ * sizeof(DataElement));
// //     if ((*this).heap_data != nullptr) {
// //         delete (*this).heap_data;
// //     }
// //     if (b.top > TriArray::STATIC_DATA_SZ) {
// //         DataElement* new_heap = (DataElement*)(malloc(b.heap_sz * sizeof(DataElement)));
// //         memcpy(new_heap, b.heap_data, sizeof(DataElement) * b.heap_sz);
// //         (*this).heap_data = new_heap;
// //     }
// //     (*this).total_sz = b.total_sz;
// //     (*this).heap_sz = b.heap_sz;
// //     (*this).in_i = b.get_in_i();
// //     (*this).top = b.len();
// //     
// //     return *this;
// // }
//
// Tri& TriArray::operator[] (int index) {
//     if (index < top) {
//         if (index < STATIC_DATA_SZ) {
//             return static_data[index].tri;
//         }
//         int outer_i;
//         get_heap_indices(outer_i, index);
//         return heap_data[outer_i][index].tri;
//     }
//     comm::print("ERROR TriArray::[] index %d >= top %d", index, top);
//     exit(1);
// }
//
// const Tri& TriArray::operator[] (int index) const {
//     if (index < top) {
//         if (index < STATIC_DATA_SZ) {
//             return static_data[index].tri;
//         }
//         int outer_i;
//         get_heap_indices(outer_i, index);
//         return heap_data[outer_i][index].tri;
//     }
//     comm::print("ERROR TriArray::[] index %d >= top %d", index, top);
//     exit(1);
// }
//
// Tri* TriArray::get(int index) {
//     if (index < top) {
//         if (index < STATIC_DATA_SZ) {
//             DataElement& e = static_data[index];
//             return e.occupied ? &e.tri : nullptr;
//         }
//         int outer_i;
//         get_heap_indices(outer_i, index);
//         DataElement& e = heap_data[outer_i][index];
//         return e.occupied ? &e.tri : nullptr;
//     }
//     comm::print("ERROR TriArray::get() index %d >= top %d", index, top);
//     exit(1);
// }
//
// int TriArray::append(const Tri& e) {
//     const int append_i = in_i;
//     int outer_i = -1;
//     int inner_i = in_i;
//     if (in_i < STATIC_DATA_SZ) {
//         DataElement& static_elem = static_data[in_i];
//         static_elem.tri = e;
//         static_elem.occupied = true;
//         while (in_i < STATIC_DATA_SZ && static_data[in_i].occupied) {
//             in_i++;
//         }
//         // handle below
//     }
//     else {
//         get_heap_indices(outer_i, inner_i);
//         DataElement& heap_elem = heap_data[outer_i][inner_i];
//         heap_elem.tri = e;
//         heap_elem.occupied = true;
//     }
//     if (in_i >= STATIC_DATA_SZ) {
//         if (outer_i == -1) {
//             get_heap_indices(outer_i, inner_i);
//         }
//         next_free_heap_indices(outer_i, in_i);
//         // while (heap_i < heap_sz && heap_data[heap_i].occupied) {
//         //     heap_i++;
//         // }
//         // in_i = heap_i + STATIC_DATA_SZ;
//     }
//     if (in_i == total_sz) {
//         const int new_heap_sz = heap_sz + HEAP_EXPANSION;
//         // DataElement* new_heap = (DataElement*)(malloc(new_heap_sz * sizeof(DataElement)));
//         
//         DataElement* new_heap = new DataElement[new_heap_sz];
//         
//         memset(new_heap, 0, new_heap_sz);
//         memcpy(new_heap, heap_data, sizeof(DataElement) * heap_sz);
//
//         delete heap_data;
//
//         heap_data = new_heap;
//         total_sz += HEAP_EXPANSION;
//         heap_sz = new_heap_sz;
//     }
//     if (in_i > top) {
//         top = in_i;
//     }
//     return append_i;
// }
//
// int TriArray::find(const Tri& e) {
//     if (top <= STATIC_DATA_SZ) {
//         for (int i = 0; i < top; i++) {
//             if (e == static_data[i].tri) {
//                 return i;
//             }	
//         }
//     }
//     else {
//         for (int i = 0; i < STATIC_DATA_SZ; i++) {
//             if (e == static_data[i].tri) {
//                 return i;
//             }	
//         }
//         const int heap_top = top - STATIC_DATA_SZ;
//         for (int i = 0; i < heap_top; i++) {
//             if (e == heap_data[i].tri) {
//                 return i + STATIC_DATA_SZ;
//             }	
//         }
//     }
//     return -1;
// }
//
// void TriArray::find_top_static() {
//     top--;
//     while (top > 0 && static_data[top - 1].occupied) {
//         top--;
//     }
// }
//
// void TriArray::find_top_heap() {
//     top--;
//     int heap_top = top - STATIC_DATA_SZ;
//     while (heap_top > 0 && heap_data[heap_top - 1].occupied) {
//         heap_top--;
//     }
//     if (heap_top == 0) {
//        top = STATIC_DATA_SZ;
//        while (top > 0 && static_data[top - 1].occupied) {
//            top--;
//        }
//     }
//     else {
//        top = heap_top + STATIC_DATA_SZ; 
//     }
// }
//
// bool TriArray::remove_at(int index) {
//     if (index < STATIC_DATA_SZ) {
//         static_data[index].occupied = false;
//         if (top == index + 1) {
//             find_top_static();
//         }
//     }
//     else {
//         int heap_index = index - STATIC_DATA_SZ;
//         static_data[heap_index].occupied = false;
//         if (top == heap_index + 1) {
//             find_top_heap();
//         }
//     }
//     if (index < in_i) {
//         in_i = index;
//     }
//     return true;
// }
//
// bool TriArray::remove(const Tri& e) {
//     const int index = find(e);
//     if (index > -1) {
//         return remove_at(index);
//     }
//     return false;
// }
//
// void TriArray::gap_fill() {
//     DataElement** e_ptrs = new DataElement*[total_sz];
//     int ctr = 0;
//     for (int i = 0; i < top && i < STATIC_DATA_SZ; i++) {
//         DataElement& e = static_data[i];
//         if (e.occupied) {
//             e_ptrs[ctr++] = &e;
//         }
//     }
//     const int heap_top = top - STATIC_DATA_SZ;
//     for (int i = 0; i < heap_top; i++) {
//         DataElement& e = heap_data[i];
//         if (e.occupied) {
//             e_ptrs[ctr++] = &e;
//         }
//     }
//     for (int i = 0; i < ctr && i < STATIC_DATA_SZ; i++) {
//         static_data[i] = *e_ptrs[i];
//     }
//     for (int i = ctr; i < STATIC_DATA_SZ; i++) {
//         static_data[i].occupied = false;
//     }
//     ctr -= STATIC_DATA_SZ;
//     if (ctr > 0) {
//         for (int i = 0; i < ctr; i++) {
//             heap_data[i] = *e_ptrs[i+STATIC_DATA_SZ];
//         }
//         for (int i = ctr; i < heap_sz; i++) {
//             heap_data[i] = *e_ptrs[i+STATIC_DATA_SZ];
//         }
//     }
//     delete e_ptrs;
// }
//
//     // for use anytime 
//     // void tarray_copy(TArray<T>& tarray, bool tarray_presized=false) {
//     //     int tarray_i = 0;
//     //     if (tarray_presized) {
//     //         for (int i = 0; i < top && i < STATIC_DATA_SZ; i++) {
//     //             DataElement &e = static_data[i];
//     //             if (e.occupied) {
//     //                 tarray[tarray_i++] = e.tri;
//     //             }
//     //         }
//     //         const int heap_top = top - STATIC_DATA_SZ;
//     //         for (int i = 0; i < heap_top; i++) {
//     //             DataElement &e = heap_data[i];
//     //             if (e.occupied) {
//     //                 tarray[tarray_i++] = e.tri;
//     //             }
//     //         }
//     //     }
//     // }
//     //
//     // // for use only if gap_fill called after removing.tris
//     // void tarray_copy_unsafe(TArray<T>& tarray, bool tarray_presized=false) {
//     //     
//     // }
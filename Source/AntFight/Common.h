#pragma once

// #define ORIENTATION_DEBUG 1
// #define TRIGRID_DEBUG 1
// #define AINAV_DEBUG 1
// #define ANTAIDISPATCH_DEBUG 1

namespace comm {

	static ECollisionChannel ECC_STATIC_MESHES = ECC_GameTraceChannel1;
	constexpr int MAX_TIMES = 100;

	static void print(const char* format_str, ...) {
		char print_buffer[2048];
		va_list args;
		va_start(args, format_str);
		vsnprintf(print_buffer, 2048, format_str, args);
		const FString fstr(print_buffer);
		UE_LOG(LogTemp, Warning, TEXT("%s"), *fstr);
	}
	
	void set_world(UWorld* _world);
	const FVector& get_hit_loc();
	const AActor* get_hit_actor();
	float trace_hit_ground(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor=nullptr);
	float trace_hit_static_actor(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor=nullptr);
	float trace_hit_nonground_static_actor(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor=nullptr);
	float trace_hit_ant(const FVector& tr_start, const FVector& tr_end, AActor* ignored_actor=nullptr);

	// benchmarking
	void log_start_time();
	void log_end_time();
	double get_time_avg();
	double get_time_max();
	int get_time_ct();
	bool times_full();
	int get_times_capacity();
	void clear_times();
}
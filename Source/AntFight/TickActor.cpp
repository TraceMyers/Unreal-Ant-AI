#include "TickActor.h"

ATickActor::ATickActor() {
	PrimaryActorTick.bCanEverTick = true;
}

void ATickActor::BeginPlay() {
	Super::BeginPlay();
}

void ATickActor::Tick(float DeltaTime) {
	for (int i = 0; i < tick_funcs.Num(); i++) {
		tick_funcs[i](DeltaTime);	
	}
}

void ATickActor::add_func(void (*func)(float)) {
	tick_funcs.Add(func);
}


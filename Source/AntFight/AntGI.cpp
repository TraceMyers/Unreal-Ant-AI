#include "AntGI.h"
#include "BPSpawner.h"
#include "Common.h"
#include "TickActor.h"


void UAntGI::Init() {
	Super::Init();
	tick_actor = Cast<ATickActor>(GetWorld()->SpawnActor(ATickActor::StaticClass()));
}

void UAntGI::Shutdown() {
	Super::Shutdown();
}

void UAntGI::StartGameInstance() {
	Super::StartGameInstance();
}

void UAntGI::spawn_line(const FVector& a, const FVector& b) const {
	if (spawner) {
		spawner->spawn_line(a, b);
	}
	else {
		comm::print("ERROR UAntGI::spawn_line() no spawner");
	}
}

void UAntGI::spawn_line_actor(const FVector& a, const FVector& b) const {
	if (spawner) {
		spawner->spawn_line_actor(a, b);
	}
	else {
		comm::print("ERROR UAntGI::spawn_line_actor() no spawner");
	}
}

void UAntGI::destroy_line_actors() const {
	if (spawner) {
		spawner->destroy_line_actors();
	}
	else {
		comm::print("ERROR UAntGI::destroy_line_actors() no spawner");
	}
}

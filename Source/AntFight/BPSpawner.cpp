#include "BPSpawner.h"

#include "AntGI.h"
#include "Line.h"
#include "Common.h"

ABPSpawner::ABPSpawner() {
	PrimaryActorTick.bCanEverTick = true;
}

void ABPSpawner::BeginPlay() {
	Super::BeginPlay();
	UAntGI* gi = Cast<UAntGI>(GetGameInstance());
	gi->spawner = this;
}

void ABPSpawner::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
}

void ABPSpawner::spawn_line(const FVector& a, const FVector& b) {
	const FVector diff = b - a;
	const FRotator rot = diff.Rotation();
	UNiagaraComponent* beam = UNiagaraFunctionLibrary::SpawnSystemAtLocation(GetWorld(), beam_system, a);
	beam->SetNiagaraVariableVec3(FString("Beam Start001"), a);
	beam->SetNiagaraVariableVec3(FString("Beam End"), b);
}

void ABPSpawner::spawn_line_actor(const FVector& a, const FVector& b) {
	// FVector lifted_a = a + FVector::UpVector * 5.0f;
	// FVector lifted_b = b + FVector::UpVector * 5.0f;
	const FVector diff = b - a;
	const FRotator rot = diff.Rotation();
	FActorSpawnParameters asp;
	ALine* line = Cast<ALine>(GetWorld()->SpawnActor(LineBP));
	line->SetActorScale3D(FVector(diff.Size(), 1.0f, 1.0f));
	line->SetActorLocation(a);
	line->SetActorRotation(rot);
	lines.Add(line);
}

void ABPSpawner::destroy_line_actors() {
	for (int i = 0; i < lines.Num(); i++) {
		lines[i]->Destroy();
	}
	lines.Empty();
}


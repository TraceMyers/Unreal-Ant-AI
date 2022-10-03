#include "Line.h"

#include "Common.h"

ALine::ALine() {
	PrimaryActorTick.bCanEverTick = false;
	USceneComponent* root = CreateDefaultSubobject<USceneComponent>(FName("root"));
	SetRootComponent(root);
	tform = CreateDefaultSubobject<USceneComponent>(FName("transform"));
	tform->SetupAttachment(root);
	sm = CreateDefaultSubobject<UStaticMeshComponent>(FName("mesh"));
	sm->SetupAttachment(tform);
}

void ALine::BeginPlay() {
	Super::BeginPlay();
}

void ALine::BeginDestroy() {
	Super::BeginDestroy();
}

void ALine::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
}


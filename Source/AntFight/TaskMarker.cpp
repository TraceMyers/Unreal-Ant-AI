#include "TaskMarker.h"

ATaskMarker::ATaskMarker() {
	PrimaryActorTick.bCanEverTick = true;
	USceneComponent* root = GetRootComponent();
	if (!root) {
		root = CreateDefaultSubobject<USceneComponent>(FName("root"));
		SetRootComponent(root);
	}
	mesh_bottom = CreateDefaultSubobject<USceneComponent>(FName("mesh bottom"));
	mesh_bottom->SetupAttachment(root);
	mesh = CreateDefaultSubobject<UStaticMeshComponent>(FName("mesh"));
	mesh->SetupAttachment(mesh_bottom);
}

void ATaskMarker::BeginPlay() {
	Super::BeginPlay();
	
}

void ATaskMarker::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
}


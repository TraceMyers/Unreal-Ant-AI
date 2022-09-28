#include "ShareData.h"

AShareData::AShareData() {
	PrimaryActorTick.bCanEverTick = false;
}

void AShareData::BeginPlay() {
	Super::BeginPlay();
	
}

void AShareData::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);

}


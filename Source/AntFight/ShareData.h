#pragma once

#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"
#include "GameFramework/Actor.h"
#include "ShareData.generated.h"

UCLASS()
class ANTFIGHT_API AShareData : public AActor {
	GENERATED_BODY()
	
public:	
	AShareData();

	UPROPERTY(BlueprintReadWrite, EditInstanceOnly)
	TArray<AStaticMeshActor*> ground_actors;

protected:
	virtual void BeginPlay() override;

public:	
	virtual void Tick(float DeltaTime) override;

};

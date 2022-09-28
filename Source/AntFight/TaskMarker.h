#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TaskMarker.generated.h"

UCLASS()
class ANTFIGHT_API ATaskMarker : public AActor {
	GENERATED_BODY()
	
public:

	UPROPERTY(BlueprintReadWrite, VisibleAnywhere)
	UStaticMeshComponent* mesh;
	UPROPERTY(BlueprintReadWrite, VisibleAnywhere)
	USceneComponent* mesh_bottom;
	
	ATaskMarker();
	virtual void Tick(float DeltaTime) override;

protected:
	
	virtual void BeginPlay() override;

private:

};

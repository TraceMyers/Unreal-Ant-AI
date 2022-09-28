#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Line.generated.h"

UCLASS()
class ANTFIGHT_API ALine : public AActor {
	GENERATED_BODY()
	
public:
	
	ALine();
	virtual void Tick(float DeltaTime) override;
	virtual void BeginDestroy() override;

	UPROPERTY(BlueprintReadWrite)
	USceneComponent* tform;
	UPROPERTY(BlueprintReadWrite, EditInstanceOnly)
	UStaticMeshComponent* sm;	

	
protected:
	
	virtual void BeginPlay() override;

private:


};

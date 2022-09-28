#pragma once

#include "CoreMinimal.h"
#include "Line.h"
#include "NiagaraCommon.h"
#include "GameFramework/Actor.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraComponent.h"
#include "BPSpawner.generated.h"

UCLASS()
class ANTFIGHT_API ABPSpawner : public AActor {
	GENERATED_BODY()
	
public:	
	
	ABPSpawner();
	virtual void Tick(float DeltaTime) override;
	void spawn_line(const FVector& a, const FVector& b);
	void spawn_line_actor(const FVector& a, const FVector &b);
	void destroy_line_actors();

	UPROPERTY(EditDefaultsOnly, Category="Spawning")
	UNiagaraSystem* beam_system;
	
	UPROPERTY(EditDefaultsOnly, Category="Spawning")
	TSubclassOf<ALine> LineBP;
	
protected:
	
	virtual void BeginPlay() override;

private:
	
	
	TArray<ALine*> lines;
	
};

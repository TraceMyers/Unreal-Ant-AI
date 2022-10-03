#pragma once

#include "CoreMinimal.h"
#include "TriGrid.h"
#include "AINav.h"
#include "GameFramework/GameModeBase.h"
#include "AntGMB.generated.h"

UCLASS()
class ANTFIGHT_API AAntGMB : public AGameModeBase {
	GENERATED_BODY()

	AAntGMB(const FObjectInitializer& ObjectInitializer);
	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(EEndPlayReason::Type reason) override;
	
	UFUNCTION(Exec, Category=ExecFunctions)
	void ground_mesh_vis();
	
public:

	TriGrid* get_tri_grid();

private:

	TriGrid tri_grid;
	AINav ai_nav;
	
};

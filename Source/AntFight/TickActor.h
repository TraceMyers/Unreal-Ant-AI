#pragma once

#include "CoreMinimal.h"
#include "Cling.h"
#include "GameFramework/Actor.h"
#include "TickActor.generated.h"

UCLASS()
class ANTFIGHT_API ATickActor : public AActor {
	GENERATED_BODY()
	
public:
	
	ATickActor();
	virtual void Tick(float DeltaTime) override;
	void add_func(void (*)(float));

protected:
	
	virtual void BeginPlay() override;

private:
	TArray<void (*)(float)> tick_funcs; 

};

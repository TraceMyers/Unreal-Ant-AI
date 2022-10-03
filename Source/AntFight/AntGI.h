#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "AntGI.generated.h"

UCLASS()
class ANTFIGHT_API UAntGI : public UGameInstance {
	
	GENERATED_BODY()
	
public:

	UPROPERTY()
	class ATickActor* tick_actor;
	UPROPERTY()
	class ABPSpawner* spawner;
	
	virtual void Init() override;
	virtual void Shutdown() override;
	virtual void StartGameInstance() override;
	void spawn_line(const FVector& a, const FVector& b) const;
	void spawn_line_actor(const FVector& a, const FVector& b) const;
	void destroy_line_actors() const;

private:
	
	
};

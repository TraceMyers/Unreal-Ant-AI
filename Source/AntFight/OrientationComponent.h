#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OrientationComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class ANTFIGHT_API UOrientationComponent : public UActorComponent {
	GENERATED_BODY()

public:
	
	UOrientationComponent();
	virtual void TickComponent(
		float DeltaTime,
		ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;
	void init(class TriGrid* tri_grid, float capsule_radius, float capsule_half_height);
	void get_new_orientation(const FVector& loc, const FVector& up, FQuat& orientation, const FVector& move);

protected:
	
	virtual void BeginPlay() override;

private:

	FVector move_cached;
	
	class TriGrid* tri_grid;
	float capsule_radius;
	float capsule_half_height;
	float head_on_hit_max;
	float head_on_hit_normalizer;

	struct Tri* near_tri;
	struct Tri* head_on_hit_tri;

	void find_near_tri(const FVector& loc);
	void find_near_tris(const FVector& loc, const FVector& head_on_hit_loc);
	static void set_orientation(const FVector& to_normal, const FVector& up, FQuat& ori);
	void set_weighted_orientation(const FVector& up, float hit_dist, FQuat& ori) const;
		
};


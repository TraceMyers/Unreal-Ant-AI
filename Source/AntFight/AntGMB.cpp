#include "AntGMB.h"
#include "AntGI.h"
#include "Common.h"
#include "TriGrid.h"
#include "ShareData.h"
#include "AntAIDispatch.h"
#include "Kismet/GameplayStatics.h"

class AAntAIDispatch;

AAntGMB::AAntGMB(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {
	PrimaryActorTick.bStartWithTickEnabled = true;
	PrimaryActorTick.bCanEverTick = true;
}

void AAntGMB::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) {
	Super::InitGame(MapName, Options, ErrorMessage);
}

void AAntGMB::BeginPlay() {
	AShareData* share_data = Cast<AShareData>(UGameplayStatics::GetActorOfClass(GetWorld(), AShareData::StaticClass()));
	if (share_data != nullptr) {
		tri_grid.init(GetWorld(), Cast<UAntGI>(GetGameInstance()), share_data->ground_actors, 100.0f);
		ai_nav.build_graph(GetWorld(), &tri_grid);
	}
	AAntAIDispatch* ai_dispatch = Cast<AAntAIDispatch>(UGameplayStatics::GetActorOfClass(GetWorld(), AAntAIDispatch::StaticClass()));
	if (ai_dispatch != nullptr) {
		ai_dispatch->set_ai_nav(&ai_nav);
	}
	comm::set_world(GetWorld());
	Super::BeginPlay();
}

void AAntGMB::Tick(float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	tri_grid.dbg_draw(DeltaSeconds);
	ai_nav.tick(DeltaSeconds);
}

void AAntGMB::EndPlay(EEndPlayReason::Type reason) {
	ai_nav.kill_pathfinder();
	Super::EndPlay(reason);
}

void AAntGMB::ground_mesh_vis() {
	tri_grid.dbg_toggle_ground_mesh_visibility();
}

TriGrid* AAntGMB::get_tri_grid() {
	return &tri_grid;
}

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "AntPC.generated.h"

UCLASS()
class ANTFIGHT_API AAntPC : public APlayerController {
	GENERATED_BODY()
	
	UFUNCTION(Exec, Category=ExecFunctions)
	void cling_draw(int val);
};

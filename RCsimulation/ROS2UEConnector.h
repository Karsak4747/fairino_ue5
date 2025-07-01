// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "sys/ipc.h"
#include "sys/shm.h"
#include "ROS2UEConnector.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class RCSIMULATION_API UROS2UEConnector : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UROS2UEConnector();
	UFUNCTION(BlueprintCallable)
	void WriteJointCommand(int joint, float value);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
private:
	key_t key;
	int shmid;
	uint8_t* data;	
	struct JointCommands{
		unsigned long msg_id;
		float joint_command[6];
	}JointCommandMsg;
};

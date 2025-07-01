// Fill out your copyright notice in the Description page of Project Settings.


#include "ROS2UEConnector.h"

// Sets default values for this component's properties
UROS2UEConnector::UROS2UEConnector()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UROS2UEConnector::BeginPlay()
{
	Super::BeginPlay();

	key = ftok("/dev/shm/data.conf", 1);
	shmid = shmget(key, 256, 0666|IPC_CREAT);
	if(shmid == -1){
		UE_LOG(LogTemp, Warning, TEXT("ERROR CREATING SHARED MEMORY SEGMENT"));
	}else{
		data = (uint8_t*)shmat(shmid, NULL, 0);
		if(data == (void*)-1)
		{
			UE_LOG(LogTemp, Warning, TEXT("ERROR CREATING SHARED MEMORY SEGMENT"));
		} 
	}
	// ...
	
}


void UROS2UEConnector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	shmdt(data);
	// ...
}



// Called every frame
void UROS2UEConnector::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	static int msg_id = 0;
	std::string msg = "Hello world";
	const char* str_arr = msg.c_str();

	int* q = (int*)data;
	*q = msg_id;
	q++;
	msq_id++;
	char* s = (char*)q;
	for(int i = 0; i < msg.length() + 1; i++){
		*s = str_arr[i];
		s++;
	}
}


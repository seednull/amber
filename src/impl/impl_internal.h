#pragma once

#include "amber_internal.h"

#include "common/pool.h"

typedef struct Impl_Instance_t
{
	Amber_InstanceTable *vtbl;
	Amber_Pool armatures;
	// Amber_Pool poses;
	// Amber_Pool sequences;
	// Amber_Pool tweens;
} Impl_Instance;

typedef struct Impl_Armature_t
{
	uint32_t joint_count;
	int32_t *joint_parents;
	uint32_t *joint_name_offsets;
	char *joint_name_memory;
} Impl_Armature;

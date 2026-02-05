#pragma once

#include "amber_internal.h"

#include "common/pool.h"

typedef struct Impl_Instance_t
{
	Amber_InstanceTable *vtbl;
	Amber_Pool armatures;
	Amber_Pool poses;
	Amber_Pool sequences;
} Impl_Instance;

typedef struct Impl_Armature_t
{
	uint32_t joint_count;
	int32_t *joint_parents;
	uint32_t *joint_name_offsets;
	char *joint_name_memory;
} Impl_Armature;

typedef struct Impl_Pose_t
{
	Amber_Armature armature;
	Amber_Transform *transforms;
} Impl_Pose;

typedef struct Impl_SequenceCurve_t
{
	uint32_t key_count;
	Amber_SequenceKey *keys;
} Impl_SequenceCurve;

typedef struct Impl_SequenceJointCurve_t
{
	Impl_SequenceCurve position_curves[3];
	Impl_SequenceCurve rotation_curves[4];
	Impl_SequenceCurve scale_curves[3];
} Impl_SequenceJointCurve;

typedef struct Impl_Sequence_t
{
	Amber_Armature armature;
	uint32_t joint_count;
	uint32_t *joint_indices;
	Impl_SequenceJointCurve *joint_curves;
	Impl_SequenceJointCurve *root_motion_curve;
	float min_time;
	float max_time;
} Impl_Sequence;

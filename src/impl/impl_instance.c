#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/*
 */
static AMBER_INLINE float amber_floatMin(float a, float b)
{
	return a < b ? a : b;
}

static AMBER_INLINE float amber_floatMax(float a, float b)
{
	return a > b ? a : b;
}

static AMBER_INLINE float amber_floatClamp(float value, float min_val, float max_val)
{
	return value < min_val ? min_val : (value > max_val ? max_val : value);
}

static AMBER_INLINE Amber_Vec3 amber_vec3Mad(const Amber_Vec3 *a, float s, const Amber_Vec3 *b)
{
	return (Amber_Vec3)
	{
		a->x * s + b->x,
		a->y * s + b->y,
		a->z * s + b->z
	};
}

static AMBER_INLINE Amber_Quat amber_quatMad(const Amber_Quat *a, float s, const Amber_Quat *b)
{
	return (Amber_Quat)
	{
		a->x * s + b->x,
		a->y * s + b->y,
		a->z * s + b->z,
		a->w * s + b->w
	};
}

static AMBER_INLINE Amber_Quat amber_quatNormalize(const Amber_Quat *q)
{
	float len = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
	float inv_len = (len > 0.0f) ? 1.0f / sqrtf(len) : 0.0f;

	return (Amber_Quat)
	{
		q->x * inv_len,
		q->y * inv_len,
		q->z * inv_len,
		q->w * inv_len
	};
}

static AMBER_INLINE void amber_invertTransform(const Amber_Transform *src_transform, Amber_Transform *dst_transform)
{
	AMBER_UNUSED(src_transform);
	AMBER_UNUSED(dst_transform);
}

static AMBER_INLINE void amber_mulTransform(const Amber_Transform *src_transform_a, const Amber_Transform *src_transform_b, Amber_Transform *dst_transform)
{
	AMBER_UNUSED(src_transform_a);
	AMBER_UNUSED(src_transform_b);
	AMBER_UNUSED(dst_transform);
}

static AMBER_INLINE float amber_fetchCurveValue(const Amber_SequenceCurve *curve, float time)
{
	assert(curve);
	assert(curve->key_count > 0);

	if (time <= curve->keys[0].time)
		return curve->keys[0].value;

	if (time >= curve->keys[curve->key_count - 1].time)
		return curve->keys[curve->key_count - 1].value;

	for (uint32_t i = 0; i < curve->key_count - 1; ++i)
	{
		const Amber_SequenceKey *key0 = &curve->keys[i];
		const Amber_SequenceKey *key1 = &curve->keys[i + 1];

		if (key0->time <= time && time <= key1->time)
		{
			float t = amber_floatClamp((time - key0->time) / (key1->time - key0->time), 0.0f, 1.0f);
			return key0->value * (1.0f - t) + key1->value * t;
		}
	}

	assert(0);
	return 0.0f;
}

/*
 */
static void impl_destroyArmature(Impl_Instance *instance_ptr, Impl_Armature *armature_ptr)
{
	assert(instance_ptr);
	assert(armature_ptr);

	AMBER_UNUSED(instance_ptr);

	free(armature_ptr->joint_name_memory);
	free(armature_ptr->joint_name_offsets);
	free(armature_ptr->joint_parents);
}

static void impl_destroyPose(Impl_Instance *instance_ptr, Impl_Pose *pose_ptr)
{
	assert(instance_ptr);
	assert(pose_ptr);

	AMBER_UNUSED(instance_ptr);

	free(pose_ptr->transforms);
}

static void impl_destroySequence(Impl_Instance *instance_ptr, Impl_Sequence *sequence_ptr)
{
	assert(instance_ptr);
	assert(sequence_ptr);

	AMBER_UNUSED(instance_ptr);

	for (uint32_t i = 0; i < sequence_ptr->joint_count; ++i)
	{
		Impl_SequenceJointCurve *joint_curve = &sequence_ptr->joint_curves[i];

		for (uint32_t j = 0; j < 3; ++j)
			free(joint_curve->position_curves[j].keys);

		for (uint32_t j = 0; j < 4; ++j)
			free(joint_curve->rotation_curves[j].keys);

		for (uint32_t j = 0; j < 3; ++j)
			free(joint_curve->scale_curves[j].keys);
	}

	free(sequence_ptr->joint_curves);
	free(sequence_ptr->joint_indices);
}

/*
 */
Amber_Result impl_instanceCreateArmature(Amber_Instance this, const Amber_ArmatureDesc *desc, Amber_Armature* armature)
{
	assert(this);
	assert(desc);
	assert(desc->joint_count > 0);
	assert(desc->joint_parents);
	assert(armature);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	// TODO: maybe do depth-first prefix ordering in case we have badly ordered input hierarchy?
	for (uint32_t i = 0; i < desc->joint_count; ++i)
		assert(desc->joint_parents[i] < (int32_t)i);

	int32_t *parents = (int32_t*)malloc(sizeof(int32_t) * desc->joint_count);
	memcpy(parents, desc->joint_parents, sizeof(int32_t) * desc->joint_count);

	char *name_memory = NULL;
	uint32_t *name_offsets = NULL;
	
	if (desc->joint_names != NULL)
	{
		name_offsets = (uint32_t *)malloc(sizeof(uint32_t) * desc->joint_count);

		uint32_t total_length = 0;
		for (uint32_t i = 0; i < desc->joint_count; ++i)
		{
			assert(desc->joint_names[i]);
			name_offsets[i] = total_length;
			total_length += (uint32_t)strlen(desc->joint_names[i]) + 1;
		}

		name_memory = (char *)malloc(sizeof(char) * total_length);
		memset(name_memory, 0, sizeof(char) * total_length);

		char *name_ptr = name_memory;
		for (uint32_t i = 0; i < desc->joint_count; ++i)
		{
			uint32_t current = name_offsets[i];
			uint32_t next = (i + 1 == desc->joint_count) ? total_length : name_offsets[i + 1];

			uint32_t length = next - current - 1;
			assert(current < next);

			memcpy(name_ptr, desc->joint_names[i], sizeof(char) * length);
			name_ptr = &name_memory[next];
		}
	}

	Impl_Armature result = {0};
	result.joint_count = desc->joint_count;
	result.joint_parents = parents;
	result.joint_name_memory = name_memory;
	result.joint_name_offsets = name_offsets;

	*armature = (Amber_Armature)amber_poolAddElement(&instance_ptr->armatures, &result);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceCreatePose(Amber_Instance this, const Amber_PoseDesc *desc, Amber_Pose *pose)
{
	assert(this);
	assert(desc);
	assert(pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)desc->armature);
	assert(armature_ptr);

	Amber_Transform *transforms = (Amber_Transform *)malloc(sizeof(Amber_Transform) * armature_ptr->joint_count);

	Impl_Pose result = {0};
	result.armature = desc->armature;
	result.transforms = transforms;
	
	*pose = (Amber_Pose)amber_poolAddElement(&instance_ptr->poses, &result);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceCreateSequence(Amber_Instance this, const Amber_SequenceDesc *desc, Amber_Sequence *sequence)
{
	assert(this);
	assert(desc);
	assert(desc->joint_count > 0);
	assert(desc->joint_indices);
	assert(desc->joint_curves);
	assert(sequence);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	uint32_t *joint_indices = (uint32_t *)malloc(sizeof(uint32_t) * desc->joint_count);
	memcpy(joint_indices, desc->joint_indices, sizeof(uint32_t) * desc->joint_count);

	Impl_SequenceJointCurve *joint_curves = (Impl_SequenceJointCurve *)malloc(sizeof(Impl_SequenceJointCurve) * desc->joint_count);
	memset(joint_curves, 0, sizeof(Impl_SequenceJointCurve) * desc->joint_count);
	
	float min_time = FLT_MAX;
	float max_time = -FLT_MAX;

	for (uint32_t i = 0; i < desc->joint_count; ++i)
	{
		const Amber_SequenceJointCurve *src_joint_curve = &desc->joint_curves[i];
		Impl_SequenceJointCurve *dst_joint_curve = &joint_curves[i];

		const Amber_SequenceCurve *src_curves[10] =
		{
			&src_joint_curve->position_curves[0],
			&src_joint_curve->position_curves[1],
			&src_joint_curve->position_curves[2],
			&src_joint_curve->rotation_curves[0],
			&src_joint_curve->rotation_curves[1],
			&src_joint_curve->rotation_curves[2],
			&src_joint_curve->rotation_curves[3],
			&src_joint_curve->scale_curves[0],
			&src_joint_curve->scale_curves[1],
			&src_joint_curve->scale_curves[2],
		};

		Impl_SequenceCurve *dst_curves[10] =
		{
			&dst_joint_curve->position_curves[0],
			&dst_joint_curve->position_curves[1],
			&dst_joint_curve->position_curves[2],
			&dst_joint_curve->rotation_curves[0],
			&dst_joint_curve->rotation_curves[1],
			&dst_joint_curve->rotation_curves[2],
			&dst_joint_curve->rotation_curves[3],
			&dst_joint_curve->scale_curves[0],
			&dst_joint_curve->scale_curves[1],
			&dst_joint_curve->scale_curves[2],
		};

		for (uint32_t j = 0; j < 10; ++j)
		{
			const Amber_SequenceCurve *src_curve = src_curves[j];
			Impl_SequenceCurve *dst_curve = dst_curves[j];

			if (src_curve->key_count == 0)
				continue;
			
			dst_curve->key_count = src_curve->key_count;
			dst_curve->keys = (Amber_SequenceKey *)malloc(sizeof(Amber_SequenceKey) * src_curve->key_count);

			for (uint32_t k = 0; k < src_curve->key_count; ++k)
			{
				dst_curve->keys[k] = src_curve->keys[k];

				float time = src_curve->keys[k].time;
				min_time = amber_floatMin(min_time, time);
				max_time = amber_floatMax(max_time, time);
			}
		}
	}

	Impl_Sequence result = {0};
	result.armature = desc->armature;
	result.joint_count = desc->joint_count;
	result.joint_indices = joint_indices;
	result.joint_curves = joint_curves;
	result.min_time = min_time;
	result.max_time = max_time;

	*sequence = (Amber_Sequence)amber_poolAddElement(&instance_ptr->sequences, &result);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceDestroyArmature(Amber_Instance this, Amber_Armature armature)
{
	assert(this);
	assert(armature);

	Amber_PoolHandle handle = (Amber_PoolHandle)armature;
	assert(handle != AMBER_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, handle);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	amber_poolRemoveElement(&instance_ptr->armatures, handle);

	impl_destroyArmature(instance_ptr, armature_ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceDestroyPose(Amber_Instance this, Amber_Pose pose)
{
	assert(this);
	assert(pose);

	Amber_PoolHandle handle = (Amber_PoolHandle)pose;
	assert(handle != AMBER_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, handle);
	assert(pose_ptr);

	amber_poolRemoveElement(&instance_ptr->poses, handle);

	impl_destroyPose(instance_ptr, pose_ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceDestroySequence(Amber_Instance this, Amber_Sequence sequence)
{
	assert(this);
	assert(sequence);

	Amber_PoolHandle handle = (Amber_PoolHandle)sequence;
	assert(handle != AMBER_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Sequence *sequence_ptr = (Impl_Sequence *)amber_poolGetElement(&instance_ptr->sequences, handle);
	assert(sequence_ptr);

	amber_poolRemoveElement(&instance_ptr->sequences, handle);

	impl_destroySequence(instance_ptr, sequence_ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceDestroy(Amber_Instance this)
{
	assert(this);

	Impl_Instance *ptr = (Impl_Instance *)this;

	{
		uint32_t head = amber_poolGetHeadIndex(&ptr->sequences);
		while (head != AMBER_POOL_HANDLE_NULL)
		{
			Impl_Sequence *sequence_ptr = (Impl_Sequence *)amber_poolGetElementByIndex(&ptr->sequences, head);
			impl_destroySequence(ptr, sequence_ptr);

			head = amber_poolGetNextIndex(&ptr->sequences, head);
		}

		amber_poolShutdown(&ptr->sequences);
	}

	{
		uint32_t head = amber_poolGetHeadIndex(&ptr->poses);
		while (head != AMBER_POOL_HANDLE_NULL)
		{
			Impl_Pose *pose_ptr = (Impl_Pose *)amber_poolGetElementByIndex(&ptr->poses, head);
			impl_destroyPose(ptr, pose_ptr);

			head = amber_poolGetNextIndex(&ptr->poses, head);
		}

		amber_poolShutdown(&ptr->poses);
	}

	{
		uint32_t head = amber_poolGetHeadIndex(&ptr->armatures);
		while (head != AMBER_POOL_HANDLE_NULL)
		{
			Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElementByIndex(&ptr->armatures, head);
			impl_destroyArmature(ptr, armature_ptr);

			head = amber_poolGetNextIndex(&ptr->armatures, head);
		}

		amber_poolShutdown(&ptr->armatures);
	}

	free(ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceCopyPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose);
	assert(src_pose_ptr);
	assert(src_pose_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	assert(src_pose_ptr->armature == dst_pose_ptr->armature);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)src_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	memcpy(dst_pose_ptr->transforms, src_pose_ptr->transforms, sizeof(Amber_Transform) * armature_ptr->joint_count);

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceMapPose(Amber_Instance this, Amber_Pose pose, Amber_Transform **transforms)
{
	assert(this);
	assert(pose);
	assert(transforms);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)pose);
	assert(pose_ptr);
	assert(pose_ptr->transforms);

	*transforms = pose_ptr->transforms;
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceUnmapPose(Amber_Instance this, Amber_Pose pose)
{
	assert(this);
	assert(pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)pose);
	assert(pose_ptr);
	assert(pose_ptr->transforms);

	AMBER_UNUSED(pose_ptr);

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceFetchPose(Amber_Instance this, Amber_Sequence sequence, float time, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(sequence);
	AMBER_UNUSED(time);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceConvertToAdditivePose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose src_reference_pose, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_pose);
	AMBER_UNUSED(src_reference_pose);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceConvertToLocalPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose);
	assert(src_pose_ptr);
	assert(src_pose_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	assert(src_pose_ptr->armature == dst_pose_ptr->armature);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)src_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	Amber_Transform parent_inv = {0};
	for (int32_t i = armature_ptr->joint_count - 1; i >= 0; --i)
	{
		int32_t parent = armature_ptr->joint_parents[i];
		assert(parent < i);

		if (parent == -1)
			continue;

		amber_invertTransform(&src_pose_ptr->transforms[parent], &parent_inv);
		amber_mulTransform(&parent_inv, &src_pose_ptr->transforms[i], &dst_pose_ptr->transforms[i]);
	}

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceConvertToWorldPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose);
	assert(src_pose_ptr);
	assert(src_pose_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	assert(src_pose_ptr->armature == dst_pose_ptr->armature);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)src_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	for (uint32_t i = 0; i < armature_ptr->joint_count; ++i)
	{
		int32_t parent = armature_ptr->joint_parents[i];
		assert(parent < (int32_t)i);

		if (parent == -1)
			continue;

		amber_mulTransform(&src_pose_ptr->transforms[parent], &src_pose_ptr->transforms[i], &dst_pose_ptr->transforms[i]);
	}

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceBlendPoses(Amber_Instance this, uint32_t src_pose_count, const Amber_Pose *src_poses, const float *src_weights, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose_count > 0);
	assert(src_poses);
	assert(src_weights);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)dst_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	memset(dst_pose_ptr->transforms, 0, sizeof(Amber_Transform) * armature_ptr->joint_count);
	for (uint32_t i = 0; i < src_pose_count; ++i)
	{
		Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_poses[i]);
		assert(src_pose_ptr);
		assert(src_pose_ptr->transforms);
		assert(src_pose_ptr->armature == dst_pose_ptr->armature);

		float src_weight = src_weights[i];

		for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
		{
			const Amber_Transform *src_transform = &src_pose_ptr->transforms[j];
			Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];

			// TODO: check quaternion hemisphere

			dst_transform->position = amber_vec3Mad(&src_transform->position, src_weight, &dst_transform->position);
			dst_transform->rotation = amber_quatMad(&src_transform->rotation, src_weight, &dst_transform->rotation);
			dst_transform->scale = amber_vec3Mad(&src_transform->scale, src_weight, &dst_transform->scale);
		}
	}

	for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
	{
		Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];
		dst_transform->rotation = amber_quatNormalize(&dst_transform->rotation);
	}

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceBlendAdditivePoses(Amber_Instance this, Amber_Pose src_pose, uint32_t src_additive_pose_count, const Amber_Pose *src_additive_poses, const float *src_weights, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_pose);
	AMBER_UNUSED(src_additive_pose_count);
	AMBER_UNUSED(src_additive_poses);
	AMBER_UNUSED(src_weights);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

/*
 */
static Amber_InstanceTable instance_vtbl =
{
	impl_instanceCreateArmature,
	impl_instanceCreatePose,
	impl_instanceCreateSequence,

	impl_instanceDestroyArmature,
	impl_instanceDestroyPose,
	impl_instanceDestroySequence,
	impl_instanceDestroy,

	impl_instanceCopyPose,
	impl_instanceMapPose,
	impl_instanceUnmapPose,

	impl_instanceFetchPose,

	impl_instanceConvertToAdditivePose,
	impl_instanceConvertToLocalPose,
	impl_instanceConvertToWorldPose,

	impl_instanceBlendPoses,
	impl_instanceBlendAdditivePoses,
};

/*
 */
Amber_Result impl_createInstance(const Amber_InstanceDesc *desc, Amber_Instance *instance)
{
	assert(desc);
	assert(instance);

	AMBER_UNUSED(desc);

	Impl_Instance *ptr = (Impl_Instance *)malloc(sizeof(Impl_Instance));
	assert(ptr);

	// vtable
	ptr->vtbl = &instance_vtbl;

	// data

	// pools
	amber_poolInitialize(&ptr->armatures, sizeof(Impl_Armature), 32);
	amber_poolInitialize(&ptr->poses, sizeof(Impl_Pose), 32);
	amber_poolInitialize(&ptr->sequences, sizeof(Impl_Sequence), 32);

	*instance = (Amber_Instance)ptr;
	return AMBER_SUCCESS;
}

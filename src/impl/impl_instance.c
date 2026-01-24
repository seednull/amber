#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>

/*
 */
static AMBER_INLINE void amber_invertTransform(const Amber_Transform *src_transform, Amber_Transform *dst_transform)
{
	assert(src_transform);
	assert(dst_transform);
}

static AMBER_INLINE void amber_mulTransform(const Amber_Transform *src_transform_a, const Amber_Transform *src_transform_b, Amber_Transform *dst_transform)
{
	assert(src_transform_a);
	assert(src_transform_b);
	assert(dst_transform);
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

Amber_Result impl_instanceCreateSampler(Amber_Instance this, const Amber_SamplerDesc *desc, Amber_Sampler *sampler)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(desc);
	AMBER_UNUSED(sampler);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceCreateSequence(Amber_Instance this, const Amber_SequenceDesc *desc, Amber_Sequence *sequence)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(desc);
	AMBER_UNUSED(sequence);

	return AMBER_NOT_IMPLEMENTED;
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

Amber_Result impl_instanceDestroySampler(Amber_Instance this, Amber_Sampler sampler)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(sampler);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceDestroySequence(Amber_Instance this, Amber_Sequence sequence)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(sequence);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceDestroy(Amber_Instance this)
{
	assert(this);

	Impl_Instance *ptr = (Impl_Instance *)this;

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

Amber_Result impl_instanceSamplePose(Amber_Instance this, Amber_Sequence sequence, Amber_Sampler sampler, float time, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(sequence);
	AMBER_UNUSED(sampler);
	AMBER_UNUSED(time);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
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

		for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
		{
			const Amber_Transform *src_transform = &src_pose_ptr->transforms[j];
			Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];
			float src_weight = src_weights[j];

			// dst_transform += src_transform * src_weight
		}
	}

	for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
	{
		Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];

		// quat_normalize(&dst_transform->rotation);
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
	impl_instanceCreateSampler,
	impl_instanceCreateSequence,

	impl_instanceDestroyArmature,
	impl_instanceDestroyPose,
	impl_instanceDestroySampler,
	impl_instanceDestroySequence,
	impl_instanceDestroy,

	impl_instanceCopyPose,
	impl_instanceMapPose,
	impl_instanceUnmapPose,

	impl_instanceFetchPose,
	impl_instanceSamplePose,

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

	*instance = (Amber_Instance)ptr;
	return AMBER_SUCCESS;
}

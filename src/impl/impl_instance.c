#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>

/*
 */
void impl_destroyArmature(Impl_Instance *instance_ptr, Impl_Armature *armature_ptr)
{
	assert(instance_ptr);
	assert(armature_ptr);

	AMBER_UNUSED(instance_ptr);

	free(armature_ptr->joint_name_memory);
	free(armature_ptr->joint_name_offsets);
	free(armature_ptr->joint_parents);
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

Amber_Result impl_instanceCreatePose(Amber_Instance this, Amber_Armature armature, Amber_Pose *pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(armature);
	AMBER_UNUSED(pose);

	return AMBER_NOT_IMPLEMENTED;
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

	amber_poolRemoveElement(&instance_ptr->armatures, handle);

	impl_destroyArmature(instance_ptr, armature_ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceDestroyPose(Amber_Instance this, Amber_Pose pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(pose);

	return AMBER_NOT_IMPLEMENTED;
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

	uint32_t head = amber_poolGetHeadIndex(&ptr->armatures);
	while (head != AMBER_POOL_HANDLE_NULL)
	{
		Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElementByIndex(&ptr->armatures, head);
		impl_destroyArmature(ptr, armature_ptr);

		head = amber_poolGetNextIndex(&ptr->armatures, head);
	}

	amber_poolShutdown(&ptr->armatures);

	free(ptr);
	return AMBER_SUCCESS;
}

Amber_Result impl_instanceCopyPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_pose);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceMapPose(Amber_Instance this, Amber_Pose pose, Amber_Transform **transforms)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(pose);
	AMBER_UNUSED(transforms);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceUnmapPose(Amber_Instance this, Amber_Pose pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(pose);

	return AMBER_NOT_IMPLEMENTED;
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

Amber_Result impl_instanceEvaluatePose(Amber_Instance this, Amber_Sequence sequence, float time, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(sequence);
	AMBER_UNUSED(time);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceInvertPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_pose);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceMultiplyPoses(Amber_Instance this, Amber_Pose src_pose_a, Amber_Pose src_pose_b, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_pose_a);
	AMBER_UNUSED(src_pose_b);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instanceBlendPoses(Amber_Instance this, uint32_t pose_count, const Amber_Pose *poses, const float *weights, Amber_Pose dst_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(pose_count);
	AMBER_UNUSED(poses);
	AMBER_UNUSED(weights);
	AMBER_UNUSED(dst_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instancePoseToModelSpace(Amber_Instance this, Amber_Pose src_local_pose, Amber_Pose dst_model_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_local_pose);
	AMBER_UNUSED(dst_model_pose);

	return AMBER_NOT_IMPLEMENTED;
}

Amber_Result impl_instancePoseToLocalSpace(Amber_Instance this, Amber_Pose src_model_pose, Amber_Pose dst_local_pose)
{
	AMBER_UNUSED(this);
	AMBER_UNUSED(src_model_pose);
	AMBER_UNUSED(dst_local_pose);

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

	impl_instanceSamplePose,
	impl_instanceEvaluatePose,
	impl_instanceInvertPose,
	impl_instanceMultiplyPoses,
	impl_instanceBlendPoses,

	impl_instancePoseToModelSpace,
	impl_instancePoseToLocalSpace,
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

	*instance = (Amber_Instance)ptr;
	return AMBER_SUCCESS;
}

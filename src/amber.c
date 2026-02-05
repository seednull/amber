#include "amber_internal.h"

#include <assert.h>
#include <string.h>

/*
 */
typedef struct Amber_InstanceInternal_t
{
	Amber_InstanceTable *vtbl;
} Amber_InstanceInternal;

/*
 */
Amber_Result amberCreateInstance(const Amber_InstanceDesc *desc, Amber_Instance *instance)
{
	return impl_createInstance(desc, instance);
}

Amber_Result amberGetInstanceTable(Amber_Instance instance, Amber_InstanceTable *instance_table)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	if (instance_table == NULL)
		return AMBER_INVALID_OUTPUT_ARGUMENT;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);

	memcpy(instance_table, ptr->vtbl, sizeof(Amber_InstanceTable));
	return AMBER_SUCCESS;
}

/*
 */
Amber_Result amberCreateArmature(Amber_Instance instance, const Amber_ArmatureDesc *desc, Amber_Armature* armature)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createArmature);

	return ptr->vtbl->createArmature(instance, desc, armature);
}

Amber_Result amberCreatePose(Amber_Instance instance, const Amber_PoseDesc *desc, Amber_Pose *pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createPose);

	return ptr->vtbl->createPose(instance, desc, pose);
}

Amber_Result amberCreateSequence(Amber_Instance instance, const Amber_SequenceDesc *desc, Amber_Sequence *sequence)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createSequence);

	return ptr->vtbl->createSequence(instance, desc, sequence);
}

Amber_Result amberDestroyArmature(Amber_Instance instance, Amber_Armature armature)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyArmature);

	return ptr->vtbl->destroyArmature(instance, armature);
}

Amber_Result amberDestroyPose(Amber_Instance instance, Amber_Pose pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyPose);

	return ptr->vtbl->destroyPose(instance, pose);
}

Amber_Result amberDestroySequence(Amber_Instance instance, Amber_Sequence sequence)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroySequence);

	return ptr->vtbl->destroySequence(instance, sequence);
}

Amber_Result amberDestroyInstance(Amber_Instance instance)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyInstance);

	return ptr->vtbl->destroyInstance(instance);
}

Amber_Result amberCopyPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->copyPose);

	return ptr->vtbl->copyPose(instance, src_pose, dst_pose);
}

Amber_Result amberMultiplyPose(Amber_Instance instance, Amber_Pose src_pose_a, Amber_Pose src_pose_b, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->multiplyPose);

	return ptr->vtbl->multiplyPose(instance, src_pose_a, src_pose_b, dst_pose);
}

Amber_Result amberInvertPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->invertPose);

	return ptr->vtbl->invertPose(instance, src_pose, dst_pose);
}

Amber_Result amberMapPose(Amber_Instance instance, Amber_Pose pose, Amber_Transform **transforms)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->mapPose);

	return ptr->vtbl->mapPose(instance, pose, transforms);
}

Amber_Result amberUnmapPose(Amber_Instance instance, Amber_Pose pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->unmapPose);

	return ptr->vtbl->unmapPose(instance, pose);
}

Amber_Result amberSampleRootMotion(Amber_Instance instance, Amber_Sequence sequence, float prev_time, float time, Amber_Transform *dst_transform)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->sampleRootMotion);

	return ptr->vtbl->sampleRootMotion(instance, sequence, prev_time, time, dst_transform);
}

Amber_Result amberSamplePose(Amber_Instance instance, Amber_Sequence sequence, float time, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->samplePose);

	return ptr->vtbl->samplePose(instance, sequence, time, dst_pose);
}

Amber_Result amberBlendPoses(Amber_Instance instance, uint32_t src_pose_count, const Amber_Pose *src_poses, const float *src_weights, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->blendPoses);

	return ptr->vtbl->blendPoses(instance, src_pose_count, src_poses, src_weights, dst_pose);
}

Amber_Result amberComputeAdditivePose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose src_reference_pose, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->computeAdditivePose);

	return ptr->vtbl->computeAdditivePose(instance, src_pose, src_reference_pose, dst_pose);
}

Amber_Result amberApplyAdditivePoses(Amber_Instance instance, Amber_Pose src_pose, uint32_t src_additive_pose_count, const Amber_Pose *src_additive_poses, const float *src_weights, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->applyAdditivePoses);

	return ptr->vtbl->applyAdditivePoses(instance, src_pose, src_additive_pose_count, src_additive_poses, src_weights, dst_pose);
}

Amber_Result amberConvertToWorldPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->convertToWorldPose);

	return ptr->vtbl->convertToWorldPose(instance, src_pose, dst_pose);
}

Amber_Result amberConvertToLocalPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose)
{
	if (instance == AMBER_NULL_HANDLE)
		return AMBER_INVALID_INSTANCE;

	Amber_InstanceInternal *ptr = (Amber_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->convertToLocalPose);

	return ptr->vtbl->convertToLocalPose(instance, src_pose, dst_pose);
}

#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/*
 */
static AMBER_INLINE float amber_floatMin(float a, float b);
static AMBER_INLINE float amber_floatMax(float a, float b);
static AMBER_INLINE float amber_floatClamp(float value, float min_val, float max_val);
static AMBER_INLINE Amber_Vec3 amber_vec3Mad(Amber_Vec3 a, float s, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_vec3Add(Amber_Vec3 a, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_vec3Sub(Amber_Vec3 a, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_vec3Mul(Amber_Vec3 a, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_vec3Div(Amber_Vec3 a, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_vec3Lerp(Amber_Vec3 a, Amber_Vec3 b, float t);
static AMBER_INLINE Amber_Quat amber_quatMad(Amber_Quat a, float s, Amber_Quat b);
static AMBER_INLINE Amber_Quat amber_quatMul(Amber_Quat a, Amber_Quat b);
static AMBER_INLINE Amber_Quat amber_quatMulVec3(Amber_Quat a, Amber_Vec3 b);
static AMBER_INLINE Amber_Vec3 amber_quatRotateVec3(Amber_Quat a, Amber_Vec3 v);
static AMBER_INLINE float amber_quatDot(Amber_Quat a, Amber_Quat b);
static AMBER_INLINE Amber_Quat amber_quatConjugate(Amber_Quat q);
static AMBER_INLINE Amber_Quat amber_quatNormalize(Amber_Quat q);
static AMBER_INLINE Amber_Quat amber_quatLerp(Amber_Quat a, Amber_Quat b, float t);
static AMBER_INLINE Amber_Transform amber_invertTransform(Amber_Transform t);
static AMBER_INLINE Amber_Transform amber_mulTransform(Amber_Transform a, Amber_Transform b);

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

static AMBER_INLINE Amber_Vec3 amber_vec3Mad(Amber_Vec3 a, float s, Amber_Vec3 b)
{
	return (Amber_Vec3)
	{
		a.x * s + b.x,
		a.y * s + b.y,
		a.z * s + b.z
	};
}

static AMBER_INLINE Amber_Vec3 amber_vec3Add(Amber_Vec3 a, Amber_Vec3 b)
{
	return (Amber_Vec3)
	{
		a.x + b.x,
		a.y + b.y,
		a.z + b.z
	};
}

static AMBER_INLINE Amber_Vec3 amber_vec3Sub(Amber_Vec3 a, Amber_Vec3 b)
{
	return (Amber_Vec3)
	{
		a.x - b.x,
		a.y - b.y,
		a.z - b.z
	};
}

static AMBER_INLINE Amber_Vec3 amber_vec3Mul(Amber_Vec3 a, Amber_Vec3 b)
{
	return (Amber_Vec3)
	{
		a.x * b.x,
		a.y * b.y,
		a.z * b.z
	};
}

static AMBER_INLINE Amber_Vec3 amber_vec3Div(Amber_Vec3 a, Amber_Vec3 b)
{
	return (Amber_Vec3)
	{
		a.x / b.x,
		a.y / b.y,
		a.z / b.z
	};
}

static AMBER_INLINE Amber_Vec3 amber_vec3Lerp(Amber_Vec3 a, Amber_Vec3 b, float t)
{
	return (Amber_Vec3)
	{
		a.x + (b.x - a.x) * t,
		a.y + (b.y - a.y) * t,
		a.z + (b.z - a.z) * t,
	};
}

static AMBER_INLINE Amber_Quat amber_quatMad(Amber_Quat a, float s, Amber_Quat b)
{
	return (Amber_Quat)
	{
		a.x * s + b.x,
		a.y * s + b.y,
		a.z * s + b.z,
		a.w * s + b.w
	};
}

static AMBER_INLINE Amber_Quat amber_quatMul(Amber_Quat a, Amber_Quat b)
{
	return (Amber_Quat)
	{
		// linear combination + cross product
		a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y,
		a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z,
		a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x,

		// mul                - dot product
		a.w * b.w             - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static AMBER_INLINE Amber_Quat amber_quatMulVec3(Amber_Quat a, Amber_Vec3 b)
{
	return (Amber_Quat)
	{
		// linear combination + cross product
		a.w * b.x             + a.y * b.z - a.z * b.y,
		a.w * b.y             + a.z * b.x - a.x * b.z,
		a.w * b.z             + a.x * b.y - a.y * b.x,

		//                    - dot product
		                      - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static AMBER_INLINE Amber_Vec3 amber_quatRotateVec3(Amber_Quat a, Amber_Vec3 v)
{
	Amber_Quat t = amber_quatMulVec3(a, v);
	t = amber_quatMul(t, amber_quatConjugate(a));

	return (Amber_Vec3)
	{
		t.x,
		t.y,
		t.z
	};
}

static AMBER_INLINE float amber_quatDot(Amber_Quat a, Amber_Quat b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static AMBER_INLINE Amber_Quat amber_quatConjugate(Amber_Quat q)
{
	return (Amber_Quat)
	{
		-q.x,
		-q.y,
		-q.z,
		 q.w,
	};
}

static AMBER_INLINE Amber_Quat amber_quatNormalize(Amber_Quat q)
{
	float len = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	float inv_len = (len > 0.0f) ? 1.0f / sqrtf(len) : 0.0f;

	return (Amber_Quat)
	{
		q.x * inv_len,
		q.y * inv_len,
		q.z * inv_len,
		q.w * inv_len
	};
}

static AMBER_INLINE Amber_Quat amber_quatLerp(Amber_Quat a, Amber_Quat b, float t)
{
	Amber_Quat q = (Amber_Quat)
	{
		a.x + (b.x - a.x) * t,
		a.y + (b.y - a.y) * t,
		a.z + (b.z - a.z) * t,
		a.w + (b.w - a.w) * t,
	};

	return amber_quatNormalize(q);
}

static AMBER_INLINE Amber_Transform amber_invertTransform(Amber_Transform t)
{
	Amber_Vec3 scale = {1.0f / t.scale.x, 1.0f / t.scale.y, 1.0f / t.scale.z};
	Amber_Quat rotation = amber_quatConjugate(t.rotation);
	Amber_Vec3 position = amber_quatRotateVec3(rotation, amber_vec3Mul(scale, (Amber_Vec3){-t.position.x, -t.position.y, -t.position.z}));

	return (Amber_Transform)
	{
		position,
		rotation,
		scale,
	};
}

static AMBER_INLINE Amber_Transform amber_mulTransform(Amber_Transform a, Amber_Transform b)
{
	Amber_Vec3 scale = amber_vec3Mul(a.scale, b.scale);
	Amber_Quat rotation = amber_quatMul(a.rotation, b.rotation);
	Amber_Vec3 position = amber_vec3Add(a.position, amber_quatRotateVec3(a.rotation, amber_vec3Mul(a.scale, b.position)));

	return (Amber_Transform)
	{
		position,
		rotation,
		scale,
	};
}

static AMBER_INLINE float amber_fetchCurveValue(const Impl_SequenceCurve *curve, float time)
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
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	Amber_Transform *transforms = (Amber_Transform *)malloc(sizeof(Amber_Transform) * armature_ptr->joint_count);

	if (desc->joint_transforms)
	{
		assert(armature_ptr->joint_count == desc->joint_count);
		memcpy(transforms, desc->joint_transforms, sizeof(Amber_Transform) * armature_ptr->joint_count);
	}
	else
		memset(transforms, 0, sizeof(Amber_Transform) * armature_ptr->joint_count);

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

Amber_Result impl_instanceMultiplyPose(Amber_Instance this, Amber_Pose src_pose_a, Amber_Pose src_pose_b, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose_a);
	assert(src_pose_b);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *src_pose_a_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose_a);
	assert(src_pose_a_ptr);
	assert(src_pose_a_ptr->transforms);

	Impl_Pose *src_pose_b_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose_b);
	assert(src_pose_b_ptr);
	assert(src_pose_b_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	assert(src_pose_a_ptr->armature == dst_pose_ptr->armature);
	assert(src_pose_b_ptr->armature == dst_pose_ptr->armature);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)dst_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	for (uint32_t i = 0; i < armature_ptr->joint_count; ++i)
		dst_pose_ptr->transforms[i] = amber_mulTransform(src_pose_a_ptr->transforms[i], src_pose_b_ptr->transforms[i]);

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceInvertPose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose dst_pose)
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
		dst_pose_ptr->transforms[i] = amber_invertTransform(src_pose_ptr->transforms[i]);

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

Amber_Result impl_instanceSamplePose(Amber_Instance this, Amber_Sequence sequence, float time, Amber_Pose dst_pose)
{
	assert(this);
	assert(sequence);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Sequence *sequence_ptr = (Impl_Sequence *)amber_poolGetElement(&instance_ptr->sequences, (Amber_PoolHandle)sequence);
	assert(sequence_ptr);
	assert(sequence_ptr->joint_count > 0);
	assert(sequence_ptr->joint_curves);
	assert(sequence_ptr->joint_indices);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	Impl_Armature *dst_armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)dst_pose_ptr->armature);
	assert(dst_armature_ptr);
	assert(dst_armature_ptr->joint_count > 0);
	assert(dst_armature_ptr->joint_parents);

	for (uint32_t i = 0; i < dst_armature_ptr->joint_count; ++i)
	{
		dst_pose_ptr->transforms[i] = (Amber_Transform)
		{
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f,
			1.0f, 1.0f, 1.0f,
		};
	}

	for (uint32_t i = 0; i < sequence_ptr->joint_count; ++i)
	{
		uint32_t index = sequence_ptr->joint_indices[i];
		assert(index < dst_armature_ptr->joint_count);

		const Impl_SequenceJointCurve *src_joint_curve = &sequence_ptr->joint_curves[i];
		assert(src_joint_curve);

		Amber_Transform *dst_joint_transform = &dst_pose_ptr->transforms[index];

		const Impl_SequenceCurve *src_curves[10] =
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

		float *dst_values[10] =
		{
			&dst_joint_transform->position.x,
			&dst_joint_transform->position.y,
			&dst_joint_transform->position.z,

			&dst_joint_transform->rotation.x,
			&dst_joint_transform->rotation.y,
			&dst_joint_transform->rotation.z,
			&dst_joint_transform->rotation.w,

			&dst_joint_transform->scale.x,
			&dst_joint_transform->scale.y,
			&dst_joint_transform->scale.z,
		};

		for (uint32_t j = 0; j < 10; ++j)
		{
			if (src_curves[j]->key_count == 0)
				continue;

			*dst_values[j] = amber_fetchCurveValue(src_curves[j], time);
		}
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

		if (src_weight == 0.0f)
			continue;

		for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
		{
			const Amber_Transform *src_transform = &src_pose_ptr->transforms[j];
			Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];

			Amber_Quat src_rotation = src_transform->rotation;
			Amber_Quat dst_rotation = dst_transform->rotation;

			if (amber_quatDot(src_rotation, dst_rotation) < 0.0f)
				dst_rotation = (Amber_Quat){-dst_rotation.x, -dst_rotation.y, -dst_rotation.z, -dst_rotation.w};

			dst_transform->position = amber_vec3Mad(src_transform->position, src_weight, dst_transform->position);
			dst_transform->rotation = amber_quatMad(src_rotation, src_weight, dst_rotation);
			dst_transform->scale = amber_vec3Mad(src_transform->scale, src_weight, dst_transform->scale);
		}
	}

	for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
	{
		Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];
		dst_transform->rotation = amber_quatNormalize(dst_transform->rotation);
	}

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceComputeAdditivePose(Amber_Instance this, Amber_Pose src_pose, Amber_Pose src_reference_pose, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose);
	assert(src_pose_ptr);
	assert(src_pose_ptr->transforms);

	Impl_Pose *src_reference_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_reference_pose);
	assert(src_reference_pose_ptr);
	assert(src_reference_pose_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	assert(src_pose_ptr->armature == dst_pose_ptr->armature);
	assert(src_reference_pose_ptr->armature == dst_pose_ptr->armature);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)src_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	for (uint32_t i = 0; i < armature_ptr->joint_count; ++i)
	{
		const Amber_Transform *src_transform = &src_pose_ptr->transforms[i];
		const Amber_Transform *src_reference_transform = &src_reference_pose_ptr->transforms[i];

		Amber_Transform *dst_transform = &dst_pose_ptr->transforms[i];

		dst_transform->position = amber_vec3Sub(src_transform->position, src_reference_transform->position);
		dst_transform->rotation = amber_quatMul(amber_quatConjugate(src_reference_transform->rotation), src_transform->rotation);
		dst_transform->scale = amber_vec3Div(src_transform->scale, src_reference_transform->scale);
	}

	return AMBER_SUCCESS;
}

Amber_Result impl_instanceApplyAdditivePoses(Amber_Instance this, Amber_Pose src_pose, uint32_t src_additive_pose_count, const Amber_Pose *src_additive_poses, const float *src_weights, Amber_Pose dst_pose)
{
	assert(this);
	assert(src_pose);
	assert(src_additive_pose_count > 0);
	assert(src_additive_poses);
	assert(src_weights);
	assert(dst_pose);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Pose *src_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_pose);
	assert(src_pose_ptr);
	assert(src_pose_ptr->transforms);

	Impl_Pose *dst_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)dst_pose);
	assert(dst_pose_ptr);
	assert(dst_pose_ptr->transforms);

	Impl_Armature *armature_ptr = (Impl_Armature *)amber_poolGetElement(&instance_ptr->armatures, (Amber_PoolHandle)dst_pose_ptr->armature);
	assert(armature_ptr);
	assert(armature_ptr->joint_count > 0);
	assert(armature_ptr->joint_parents);

	assert(src_pose_ptr->armature == dst_pose_ptr->armature);
	memcpy(dst_pose_ptr->transforms, src_pose_ptr->transforms, sizeof(Amber_Transform) * armature_ptr->joint_count);

	for (uint32_t i = 0; i < src_additive_pose_count; ++i)
	{
		Impl_Pose *src_additive_pose_ptr = (Impl_Pose *)amber_poolGetElement(&instance_ptr->poses, (Amber_PoolHandle)src_additive_poses[i]);
		assert(src_additive_pose_ptr);
		assert(src_additive_pose_ptr->transforms);
		assert(src_additive_pose_ptr->armature == dst_pose_ptr->armature);

		float src_weight = src_weights[i];

		if (src_weight == 0.0f)
			continue;

		for (uint32_t j = 0; j < armature_ptr->joint_count; ++j)
		{
			const Amber_Transform *src_additive_transform = &src_additive_pose_ptr->transforms[j];
			Amber_Transform *dst_transform = &dst_pose_ptr->transforms[j];

			dst_transform->position = amber_vec3Mad(src_additive_transform->position, src_weight, dst_transform->position);
			dst_transform->rotation = amber_quatLerp(dst_transform->rotation, amber_quatMul(dst_transform->rotation, src_additive_transform->rotation), src_weight);
			dst_transform->scale = amber_vec3Lerp(dst_transform->scale, amber_vec3Mul(dst_transform->scale, src_additive_transform->scale), src_weight);
		}
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
			dst_pose_ptr->transforms[i] = src_pose_ptr->transforms[i];
		else
			dst_pose_ptr->transforms[i] = amber_mulTransform(dst_pose_ptr->transforms[parent], src_pose_ptr->transforms[i]);
	}

	return AMBER_SUCCESS;
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

	for (int32_t i = armature_ptr->joint_count - 1; i >= 0; --i)
	{
		int32_t parent = armature_ptr->joint_parents[i];
		assert(parent < i);

		if (parent == -1)
			dst_pose_ptr->transforms[i] = src_pose_ptr->transforms[i];
		else
			dst_pose_ptr->transforms[i] = amber_mulTransform(amber_invertTransform(src_pose_ptr->transforms[parent]), src_pose_ptr->transforms[i]);
	}

	return AMBER_SUCCESS;
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
	impl_instanceMultiplyPose,
	impl_instanceInvertPose,
	impl_instanceMapPose,
	impl_instanceUnmapPose,
	impl_instanceSamplePose,

	impl_instanceBlendPoses,
	impl_instanceComputeAdditivePose,
	impl_instanceApplyAdditivePoses,

	impl_instanceConvertToWorldPose,
	impl_instanceConvertToLocalPose,
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

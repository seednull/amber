#pragma once

#include <stdint.h> // TODO: get rid of this dependency later

// Version
#define AMBER_VERSION_MAJOR 1
#define AMBER_VERSION_MINOR 0
#define AMBER_VERSION_PATCH 0
#define AMBER_VERSION "1.0.0-dev"

// Platform specific defines
#if defined(_WIN32)
	#define AMBER_EXPORT	__declspec(dllexport)
	#define AMBER_IMPORT	__declspec(dllimport)
	#define AMBER_INLINE	__forceinline
	#define AMBER_RESTRICT	__restrict
#else
	#define AMBER_EXPORT	__attribute__((visibility("default")))
	#define AMBER_IMPORT
	#define AMBER_INLINE	__inline__
	#define AMBER_RESTRICT	__restrict
#endif

#if defined(AMBER_SHARED_LIBRARY)
	#define AMBER_APIENTRY extern AMBER_EXPORT
#else
	#define AMBER_APIENTRY extern AMBER_IMPORT
#endif

#if !defined(AMBER_NULL_HANDLE)
	#define AMBER_NULL_HANDLE 0
#endif

#define AMBER_DEFINE_HANDLE(TYPE) typedef uint64_t TYPE

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// Opaque handles
AMBER_DEFINE_HANDLE(Amber_Instance);
AMBER_DEFINE_HANDLE(Amber_Armature);
AMBER_DEFINE_HANDLE(Amber_Sequence);
AMBER_DEFINE_HANDLE(Amber_Pose);

// Enums
typedef enum Amber_Result_t
{
	AMBER_SUCCESS = 0,
	AMBER_NOT_IMPLEMENTED,
	AMBER_INVALID_INSTANCE,
	AMBER_INVALID_OUTPUT_ARGUMENT,

	// FIXME: add more error codes for internal errors
	AMBER_INTERNAL_ERROR,

	AMBER_RESULT_ENUM_MAX,
	AMBER_RESULT_ENUM_FORCE32 = 0x7FFFFFFF,
} Amber_Result;

// Structs
typedef struct Amber_Vec2_t
{
	float x, y;
} Amber_Vec2;

typedef struct Amber_Vec3_t
{
	float x, y, z;
} Amber_Vec3;

typedef struct Amber_Quat_t
{
	float x, y, z, w;
} Amber_Quat;

typedef struct Amber_Transform_t
{
	Amber_Vec3 position;
	Amber_Quat rotation;
	Amber_Vec3 scale;
} Amber_Transform;

typedef struct Amber_InstanceDesc_t
{
	uint32_t reserved;
	// TODO: allocator context
	// TOOD: flags?
} Amber_InstanceDesc;

typedef struct Amber_ArmatureDesc_t
{
	uint32_t joint_count;
	const int32_t *joint_parents;
	const char **joint_names;
} Amber_ArmatureDesc;

typedef struct Amber_PoseDesc_t
{
	Amber_Armature armature;
} Amber_PoseDesc;

typedef struct Amber_SequenceKey_t
{
	float time;
	float value;
	Amber_Vec2 tangent_left;
	Amber_Vec2 tangent_right;
} Amber_SequenceKey;

typedef struct Amber_SequenceCurve_t
{
	uint32_t key_count;
	const Amber_SequenceKey *keys;
} Amber_SequenceCurve;

typedef struct Amber_SequenceJointCurve_t
{
	Amber_SequenceCurve position_curves[3];
	Amber_SequenceCurve rotation_curves[4];
	Amber_SequenceCurve scale_curves[3];
} Amber_SequenceJointCurve;

typedef struct Amber_SequenceDesc_t
{
	Amber_Armature armature;
	uint32_t joint_count;
	const uint32_t *joint_indices;
	const Amber_SequenceJointCurve *joint_curves;
} Amber_SequenceDesc;

// Function pointers
typedef Amber_Result (*PFN_amberCreateArmature)(Amber_Instance instance, const Amber_ArmatureDesc *desc, Amber_Armature* armature);
typedef Amber_Result (*PFN_amberCreatePose)(Amber_Instance instance, const Amber_PoseDesc *desc, Amber_Pose *pose);
typedef Amber_Result (*PFN_amberCreateSequence)(Amber_Instance instance, const Amber_SequenceDesc *desc, Amber_Sequence *sequence);

typedef Amber_Result (*PFN_amberDestroyArmature)(Amber_Instance instance, Amber_Armature armature);
typedef Amber_Result (*PFN_amberDestroyPose)(Amber_Instance instance, Amber_Pose pose);
typedef Amber_Result (*PFN_amberDestroySequence)(Amber_Instance instance, Amber_Sequence sequence);
typedef Amber_Result (*PFN_amberDestroyInstance)(Amber_Instance instance);

typedef Amber_Result (*PFN_amberCopyPose)(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);
typedef Amber_Result (*PFN_amberMapPose)(Amber_Instance instance, Amber_Pose pose, Amber_Transform **transforms);
typedef Amber_Result (*PFN_amberUnmapPose)(Amber_Instance instance, Amber_Pose pose);
typedef Amber_Result (*PFN_amberSamplePose)(Amber_Instance instance, Amber_Sequence sequence, float time, Amber_Pose dst_pose);

typedef Amber_Result (*PFN_amberBlendPoses)(Amber_Instance instance, uint32_t src_pose_count, const Amber_Pose *src_poses, const float *src_weights, Amber_Pose dst_pose);
typedef Amber_Result (*PFN_amberComputeAdditivePose)(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose src_reference_pose, Amber_Pose dst_pose);
typedef Amber_Result (*PFN_amberApplyAdditivePoses)(Amber_Instance instance, Amber_Pose src_pose, uint32_t src_additive_pose_count, const Amber_Pose *src_additive_poses, const float *src_weights, Amber_Pose dst_pose);

typedef Amber_Result (*PFN_amberConvertToWorldPose)(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);
typedef Amber_Result (*PFN_amberConvertToLocalPose)(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);

typedef struct Amber_InstanceTable_t
{
	PFN_amberCreateArmature createArmature;
	PFN_amberCreatePose createPose;
	PFN_amberCreateSequence createSequence;

	PFN_amberDestroyArmature destroyArmature;
	PFN_amberDestroyPose destroyPose;
	PFN_amberDestroySequence destroySequence;
	PFN_amberDestroyInstance destroyInstance;

	PFN_amberCopyPose copyPose;
	PFN_amberMapPose mapPose;
	PFN_amberUnmapPose unmapPose;
	PFN_amberSamplePose samplePose;

	PFN_amberBlendPoses blendPoses;
	PFN_amberComputeAdditivePose computeAdditivePose;
	PFN_amberApplyAdditivePoses applyAdditivePoses;

	PFN_amberConvertToWorldPose convertToWorldPose;
	PFN_amberConvertToLocalPose convertToLocalPose;
} Amber_InstanceTable;

// API
#if !defined(AMBER_NO_PROTOTYPES)
AMBER_APIENTRY Amber_Result amberCreateInstance(const Amber_InstanceDesc *desc, Amber_Instance* instance);
AMBER_APIENTRY Amber_Result amberGetInstanceTable(Amber_Instance instance, Amber_InstanceTable *instance_table);

AMBER_APIENTRY Amber_Result amberCreateArmature(Amber_Instance instance, const Amber_ArmatureDesc *desc, Amber_Armature* armature);
AMBER_APIENTRY Amber_Result amberCreatePose(Amber_Instance instance, const Amber_PoseDesc *desc, Amber_Pose *pose);
AMBER_APIENTRY Amber_Result amberCreateSequence(Amber_Instance instance, const Amber_SequenceDesc *desc, Amber_Sequence *sequence);

AMBER_APIENTRY Amber_Result amberDestroyArmature(Amber_Instance instance, Amber_Armature armature);
AMBER_APIENTRY Amber_Result amberDestroyPose(Amber_Instance instance, Amber_Pose pose);
AMBER_APIENTRY Amber_Result amberDestroySequence(Amber_Instance instance, Amber_Sequence sequence);
AMBER_APIENTRY Amber_Result amberDestroyInstance(Amber_Instance instance);

AMBER_APIENTRY Amber_Result amberCopyPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);
AMBER_APIENTRY Amber_Result amberMapPose(Amber_Instance instance, Amber_Pose pose, Amber_Transform **transforms);
AMBER_APIENTRY Amber_Result amberUnmapPose(Amber_Instance instance, Amber_Pose pose);
AMBER_APIENTRY Amber_Result amberSamplePose(Amber_Instance instance, Amber_Sequence sequence, float time, Amber_Pose dst_pose);

AMBER_APIENTRY Amber_Result amberBlendPoses(Amber_Instance instance, uint32_t src_pose_count, const Amber_Pose *src_poses, const float *src_weights, Amber_Pose dst_pose);
AMBER_APIENTRY Amber_Result amberComputeAdditivePose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose src_reference_pose, Amber_Pose dst_pose);
AMBER_APIENTRY Amber_Result amberApplyAdditivePoses(Amber_Instance instance, Amber_Pose src_pose, uint32_t src_additive_pose_count, const Amber_Pose *src_additive_poses, const float *src_weights, Amber_Pose dst_pose);

AMBER_APIENTRY Amber_Result amberConvertToWorldPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);
AMBER_APIENTRY Amber_Result amberConvertToLocalPose(Amber_Instance instance, Amber_Pose src_pose, Amber_Pose dst_pose);
#endif

#ifdef __cplusplus
}
#endif

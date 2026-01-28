#include <amber.h>
#include <cassert>
#include <iostream>

void testPoses(Amber_Instance instance)
{
	static const char *names[] =
	{
		"spine",
		"spine.001", "spine.002", "spine.003", "spine.004", "spine.005", "spine.006",
		"shoulder.L", "upper_arm.L", "forearm.L", "hand.L",
		"shoulder.R", "upper_arm.R", "forearm.R", "hand.R",
		"thigh.L", "shin.L", "foot.L", "toe.L",
		"thigh.R", "shin.R", "foot.R", "toe.R",
	};

	static int32_t parents[] =
	{
		-1,
		0, 1, 2, 3, 4, 5,
		3, 7, 8, 9,
		3, 11, 12, 13,
		0, 15, 16, 17,
		0, 19, 20, 21,
	};

	static Amber_Vec3 rest_positions[] =
	{
		0.000000f, 0.055200f, 1.009900f,
		0.000000f, 0.152220f, -0.000000f,
		0.000000f, 0.136637f, 0.000000f,
		0.000000f, 0.172888f, 0.000000f,
		0.000000f, 0.192578f, -0.000000f,
		0.000000f, 0.066163f, -0.000000f,
		0.000000f, 0.062701f, 0.000000f,
		0.018300f, 0.137221f, 0.078251f,
		0.007792f, 0.200791f, -0.020385f,
		0.000000f, 0.288510f, 0.000000f,
		-0.000000f, 0.262836f, -0.000000f,
		-0.018300f, 0.137221f, 0.078251f,
		-0.007792f, 0.200791f, -0.020385f,
		-0.000000f, 0.288510f, 0.000000f,
		0.000000f, 0.262836f, -0.000000f,
		0.098000f, 0.070819f, 0.025942f,
		-0.000000f, 0.536370f, -0.000000f,
		0.000000f, 0.454215f, 0.000000f,
		-0.000000f, 0.129246f, 0.000000f,
		-0.098000f, 0.070819f, 0.025942f,
		0.000000f, 0.536370f, -0.000000f,
		0.000000f, 0.454215f, 0.000000f,
		0.000000f, 0.129246f, 0.000000f,
	};

	static Amber_Quat rest_rotations[] =
	{
		0.790455f, 0.000000f, 0.000000f, 0.612520f,
		-0.064476f, 0.000000f, 0.000000f, 0.997919f,
		-0.077464f, 0.000000f, 0.000000f, 0.996995f,
		0.001627f, 0.000000f, 0.000000f, 0.999999f,
		0.201738f, 0.000000f, 0.000000f, 0.979440f,
		-0.094858f, 0.000000f, 0.000000f, 0.995491f,
		-0.093712f, 0.000000f, 0.000000f, 0.995599f,
		-0.605155f, -0.345862f, -0.356134f, 0.622363f,
		-0.055420f, 0.740604f, -0.270910f, 0.612407f,
		0.181814f, 0.001342f, -0.040503f, 0.982498f,
		-0.025047f, -0.003554f, -0.028186f, 0.999283f,
		-0.605155f, 0.345862f, 0.356134f, 0.622363f,
		-0.055420f, -0.740604f, 0.270910f, 0.612407f,
		0.181814f, -0.001342f, 0.040503f, 0.982498f,
		-0.025047f, 0.003554f, 0.028186f, 0.999283f,
		0.986515f, 0.000000f, 0.000000f, 0.163672f,
		0.087542f, 0.000000f, 0.000000f, 0.996161f,
		-0.527364f, 0.000000f, 0.000000f, 0.849639f,
		-0.000000f, 0.961249f, -0.275682f, 0.000000f,
		0.986515f, 0.000000f, 0.000000f, 0.163672f,
		0.087542f, 0.000000f, 0.000000f, 0.996161f,
		-0.527364f, 0.000000f, 0.000000f, 0.849639f,
		-0.000000f, 0.961249f, -0.275682f, 0.000000f,
	};
	
	Amber_ArmatureDesc armature_desc = {23, parents, names};
	Amber_Armature armature = AMBER_NULL_HANDLE;

	Amber_Result result = amberCreateArmature(instance, &armature_desc, &armature);
	assert(result == AMBER_SUCCESS);

	Amber_PoseDesc pose_desc = {armature};
	Amber_Pose pose = AMBER_NULL_HANDLE;
	Amber_Pose test_pose = AMBER_NULL_HANDLE;

	result = amberCreatePose(instance, &pose_desc, &pose);
	assert(result == AMBER_SUCCESS);

	Amber_Transform *pose_transforms = NULL;
	result = amberMapPose(instance, pose, &pose_transforms);
	assert(result == AMBER_SUCCESS);

	for (uint32_t i = 0; i < armature_desc.joint_count; ++i)
	{
		Amber_Transform *t = &pose_transforms[i];

		t->scale = {1.0f, 1.0f, 1.0f};
		t->rotation = rest_rotations[i];
		t->position = rest_positions[i];
	}

	result = amberCreatePose(instance, &pose_desc, &test_pose);
	assert(result == AMBER_SUCCESS);

	result = amberCopyPose(instance, pose, test_pose);
	assert(result == AMBER_SUCCESS);

	result = amberConvertToWorldPose(instance, pose, pose);
	assert(result == AMBER_SUCCESS);

	result = amberConvertToLocalPose(instance, pose, pose);
	assert(result == AMBER_SUCCESS);

	Amber_Transform *test_transforms = NULL;
	result = amberMapPose(instance, test_pose, &test_transforms);
	assert(result == AMBER_SUCCESS);

	const float eps = 0.00001f;

	for (uint32_t i = 0; i < armature_desc.joint_count; ++i)
	{
		const Amber_Transform *test = &test_transforms[i];
		const Amber_Transform *current = &pose_transforms[i];

		assert(fabs(test->position.x - current->position.x) < eps);
		assert(fabs(test->position.y - current->position.y) < eps);
		assert(fabs(test->position.z - current->position.z) < eps);

		assert(fabs(test->rotation.x - current->rotation.x) < eps);
		assert(fabs(test->rotation.y - current->rotation.y) < eps);
		assert(fabs(test->rotation.z - current->rotation.z) < eps);
		assert(fabs(test->rotation.w - current->rotation.w) < eps);

		assert(fabs(test->scale.x - current->scale.x) < eps);
		assert(fabs(test->scale.y - current->scale.y) < eps);
		assert(fabs(test->scale.z - current->scale.z) < eps);
	}

	result = amberUnmapPose(instance, pose);
	assert(result == AMBER_SUCCESS);

	result = amberUnmapPose(instance, test_pose);
	assert(result == AMBER_SUCCESS);

	result = amberDestroyPose(instance, pose);
	assert(result == AMBER_SUCCESS);

	result = amberDestroyPose(instance, test_pose);
	assert(result == AMBER_SUCCESS);

	result = amberDestroyArmature(instance, armature);
	assert(result == AMBER_SUCCESS);
}

int main()
{
	Amber_Instance instance = AMBER_NULL_HANDLE;

	Amber_InstanceDesc instance_desc =
	{
	};

	Amber_Result result = amberCreateInstance(&instance_desc, &instance);
	assert(result == AMBER_SUCCESS);

	testPoses(instance);

	result = amberDestroyInstance(instance);
	assert(result == AMBER_SUCCESS);

	return 0;
}

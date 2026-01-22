#include <amber.h>
#include <cassert>
#include <iostream>

void testArmatures(Amber_Instance instance)
{
	const char *names[] = { "root", "left_hip", "left_thigh", "left_calf", "right_hip", "right_thigh", "right_calf" };
	int32_t parents[] =   { -1,      0,          1,            2,           0,           4,             5           };
	
	Amber_ArmatureDesc desc = { 7, parents, names };
	Amber_Armature armature = AMBER_NULL_HANDLE;

	Amber_Result result = amberCreateArmature(instance, &desc, &armature);
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

	testArmatures(instance);

	result = amberDestroyInstance(instance);
	assert(result == AMBER_SUCCESS);

	return 0;
}

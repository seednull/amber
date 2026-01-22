#pragma once

#include <amber.h>

#define AMBER_UNUSED(x) do { (void)(x); } while(0)

Amber_Result impl_createInstance(const Amber_InstanceDesc *desc, Amber_Instance *instance);

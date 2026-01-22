#pragma once

#include <amber.h>

#define AMBER_POOL_MAX_ELEMENTS		0x00FFFFFF
#define AMBER_POOL_MAX_GENERATIONS	0xFF
#define AMBER_POOL_HANDLE_NULL		0xFFFFFFFF

typedef uint32_t Amber_PoolHandle;

typedef struct Amber_Pool_t
{
	uint8_t *data;
	uint8_t *generations;
	uint32_t *nexts;
	uint32_t *prevs;
	uint32_t head;
	uint32_t tail;

	uint32_t element_size;
	uint32_t size;
	uint32_t capacity;

	uint32_t *masks;
	uint32_t *indices;
	uint32_t num_free_indices;
} Amber_Pool;

Amber_Result amber_poolInitialize(Amber_Pool *pool, uint32_t element_size, uint32_t capacity);
Amber_Result amber_poolShutdown(Amber_Pool *pool);

Amber_PoolHandle amber_poolAddElement(Amber_Pool *pool, const void *data);
Amber_Result amber_poolRemoveElement(Amber_Pool *pool, Amber_PoolHandle handle);
void *amber_poolGetElement(const Amber_Pool *pool, Amber_PoolHandle handle);

void *amber_poolGetElementByIndex(const Amber_Pool *pool, uint32_t index);
uint32_t amber_poolGetHeadIndex(const Amber_Pool *pool);
uint32_t amber_poolGetTailIndex(const Amber_Pool *pool);
uint32_t amber_poolGetNextIndex(const Amber_Pool *pool, uint32_t index);
uint32_t amber_poolGetPrevIndex(const Amber_Pool *pool, uint32_t index);
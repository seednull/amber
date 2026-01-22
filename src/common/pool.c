#include "pool.h"
#include "intrinsics.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>

/*
 */
static AMBER_INLINE Amber_PoolHandle amber_poolHandlePack(uint32_t index, uint8_t generation)
{
	return (Amber_PoolHandle)((index << 8) | generation);
}

static AMBER_INLINE uint32_t amber_poolHandleGetIndex(Amber_PoolHandle handle)
{
	return (uint32_t)(handle >> 8);
}

static AMBER_INLINE uint8_t amber_poolHandleGetGeneration(Amber_PoolHandle handle)
{
	return (uint8_t)(handle & 0xFF);
}

/*
 */
static AMBER_INLINE uint32_t amber_poolGrabIndex(Amber_Pool *pool)
{
	assert(pool);
	assert(pool->num_free_indices > 0);

	uint32_t index = pool->indices[pool->num_free_indices - 1];
	pool->num_free_indices--;

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	pool->masks[mask_index] &= ~(1 << mask_bit);

	return index;
}

static AMBER_INLINE void amber_poolReleaseIndex(Amber_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->num_free_indices < pool->capacity);
	assert(index < pool->capacity);

	pool->num_free_indices++;
	pool->indices[pool->num_free_indices - 1] = index;

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	pool->masks[mask_index] |= 1 << mask_bit;
}

static AMBER_INLINE uint32_t amber_poolIsIndexFree(const Amber_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(index < pool->capacity);

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	uint32_t free_mask = pool->masks[mask_index];
	uint32_t element_mask = 1 << mask_bit;

	return free_mask & element_mask;
}

static AMBER_INLINE uint32_t amber_poolGetNumMasks(const Amber_Pool *pool)
{
	assert(pool);

	return alignUp(pool->capacity, 32) / 32;
}

/*
 */
Amber_Result amber_poolInitialize(Amber_Pool *pool, uint32_t element_size, uint32_t capacity)
{
	assert(pool);
	assert(element_size > 0);

	memset(pool, 0, sizeof(Amber_Pool));

	pool->element_size = element_size;
	pool->capacity = capacity;
	pool->num_free_indices = capacity;
	pool->head = AMBER_POOL_HANDLE_NULL;
	pool->tail = AMBER_POOL_HANDLE_NULL;

	if (capacity > 0)
	{
		uint32_t num_masks = amber_poolGetNumMasks(pool);

		pool->data = (uint8_t *)malloc(element_size * capacity);
		pool->generations = (uint8_t *)malloc(sizeof(uint8_t) * capacity);
		pool->nexts = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->prevs = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->indices = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->masks = (uint32_t *)malloc(sizeof(uint32_t) * num_masks);

		for (uint32_t i = 0; i < capacity; ++i)
			pool->indices[i] = capacity - i - 1;

		memset(pool->nexts, AMBER_POOL_HANDLE_NULL, sizeof(uint32_t) * capacity);
		memset(pool->prevs, AMBER_POOL_HANDLE_NULL, sizeof(uint32_t) * capacity);
		memset(pool->masks, 0xFFFFFFFF, sizeof(uint32_t) * num_masks);
		memset(pool->generations, 0, sizeof(uint8_t) * capacity);
	}

	return AMBER_SUCCESS;
}

Amber_Result amber_poolShutdown(Amber_Pool *pool)
{
	assert(pool);

	free(pool->data);
	free(pool->generations);
	free(pool->nexts);
	free(pool->prevs);
	free(pool->indices);
	free(pool->masks);

	memset(pool, 0, sizeof(Amber_Pool));

	return AMBER_SUCCESS;
}

/*
 */
Amber_PoolHandle amber_poolAddElement(Amber_Pool *pool, const void *data)
{
	assert(pool);
	assert(data);

	if (pool->size == pool->capacity)
	{
		uint32_t old_capacity = pool->capacity;
		uint32_t old_num_masks = amber_poolGetNumMasks(pool);

		pool->capacity = (pool->capacity == 0) ? 1 : pool->capacity * 2;
		uint32_t new_num_masks = amber_poolGetNumMasks(pool);

		pool->data = (uint8_t *)realloc(pool->data, pool->element_size * pool->capacity);
		pool->generations = (uint8_t *)realloc(pool->generations, sizeof(uint8_t) * pool->capacity);
		pool->nexts = (uint32_t *)realloc(pool->nexts, sizeof(uint32_t) * pool->capacity);
		pool->prevs = (uint32_t *)realloc(pool->prevs, sizeof(uint32_t) * pool->capacity);
		pool->indices = (uint32_t *)realloc(pool->indices, sizeof(uint32_t) * pool->capacity);

		if (old_num_masks != new_num_masks)
			pool->masks = (uint32_t *)realloc(pool->masks, sizeof(uint32_t) * new_num_masks);

		for (uint32_t i = old_capacity; i < pool->capacity; ++i)
		{
			pool->num_free_indices++;
			pool->indices[pool->num_free_indices - 1] = old_capacity + pool->capacity - i - 1;
			pool->generations[i] = 0;
			pool->nexts[i] = AMBER_POOL_HANDLE_NULL;
			pool->prevs[i] = AMBER_POOL_HANDLE_NULL;
		}

		for (uint32_t i = old_num_masks; i < new_num_masks; ++i)
			pool->masks[i] = 0xFFFFFFFF;
	}

	assert(pool->num_free_indices > 0);

	uint32_t index = amber_poolGrabIndex(pool);

	uint8_t *data_ptr = pool->data + index * pool->element_size;
	uint8_t *generation_ptr = pool->generations + index;

	if (pool->head == AMBER_POOL_HANDLE_NULL)
		pool->head = index;

	if (pool->tail == AMBER_POOL_HANDLE_NULL)
	{
		pool->tail = index;
	}
	else
	{
		pool->nexts[pool->tail] = index;
		pool->prevs[index] = pool->tail;

		pool->tail = index;
	}

	memcpy(data_ptr, data, pool->element_size);

	uint8_t generation = *generation_ptr + 1;
	*generation_ptr = max(1, generation);

	pool->size++;

	return amber_poolHandlePack(index, *generation_ptr);
}

Amber_Result amber_poolRemoveElement(Amber_Pool *pool, Amber_PoolHandle handle)
{
	assert(pool);

	if (handle == AMBER_POOL_HANDLE_NULL)
		return AMBER_INTERNAL_ERROR;

	if (pool->size == 0)
		return AMBER_INTERNAL_ERROR;

	uint32_t index = amber_poolHandleGetIndex(handle);
	uint8_t generation = amber_poolHandleGetGeneration(handle);

	if (pool->generations[index] != generation)
		return AMBER_INTERNAL_ERROR;

	if (amber_poolIsIndexFree(pool, index))
		return AMBER_INTERNAL_ERROR;

	uint32_t prev = pool->prevs[index];
	uint32_t next = pool->nexts[index];

	pool->prevs[index] = AMBER_POOL_HANDLE_NULL;
	pool->nexts[index] = AMBER_POOL_HANDLE_NULL;

	if (next != AMBER_POOL_HANDLE_NULL)
		pool->prevs[next] = prev;

	if (prev != AMBER_POOL_HANDLE_NULL)
		pool->nexts[prev] = next;

	if (pool->head == index)
		pool->head = next;

	if (pool->tail == index)
		pool->tail = prev;

	amber_poolReleaseIndex(pool, index);
	pool->size--;

	return AMBER_SUCCESS;
}

void *amber_poolGetElement(const Amber_Pool *pool, Amber_PoolHandle handle)
{
	assert(pool);
	assert(pool->size > 0);

	if (handle == AMBER_POOL_HANDLE_NULL)
		return NULL;

	uint32_t index = amber_poolHandleGetIndex(handle);
	uint8_t generation = amber_poolHandleGetGeneration(handle);

	if (pool->generations[index] != generation)
		return NULL;

	if (amber_poolIsIndexFree(pool, index))
		return NULL;

	return pool->data + index * pool->element_size;
}

void *amber_poolGetElementByIndex(const Amber_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != AMBER_POOL_HANDLE_NULL);

	return pool->data + index * pool->element_size;
}

uint32_t amber_poolGetHeadIndex(const Amber_Pool *pool)
{
	assert(pool);
	return pool->head;
}

uint32_t amber_poolGetTailIndex(const Amber_Pool *pool)
{
	assert(pool);
	return pool->tail;
}

uint32_t amber_poolGetNextIndex(const Amber_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != AMBER_POOL_HANDLE_NULL);
	
	return pool->nexts[index];
}

uint32_t amber_poolGetPrevIndex(const Amber_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != AMBER_POOL_HANDLE_NULL);
	
	return pool->prevs[index];
}

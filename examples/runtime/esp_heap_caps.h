// esp_heap_caps.h - the ESP-IDF heap capability calls the JetExamples scenes
// make, on a part with one kind of RAM.
//
// Every call the scenes make is one of three things: a hint to place large
// allocations in external PSRAM, a query for free memory to print, or an aligned
// allocation. There is no PSRAM here, so the hint does nothing, PSRAM queries
// report zero, and everything else is the ordinary heap.
#pragma once

#include <malloc.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define MALLOC_CAP_EXEC     (1u << 0)
#define MALLOC_CAP_32BIT    (1u << 1)
#define MALLOC_CAP_8BIT     (1u << 2)
#define MALLOC_CAP_DMA      (1u << 3)
#define MALLOC_CAP_SPIRAM   (1u << 10)
#define MALLOC_CAP_INTERNAL (1u << 11)
#define MALLOC_CAP_DEFAULT  (1u << 12)

/* The sdkconfig value the scenes restore after raising the PSRAM threshold. */
#ifndef CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL
#define CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL 16384
#endif

static inline void heap_caps_malloc_extmem_enable(size_t limit)
{
	(void)limit;
}

static inline size_t heap_caps_get_free_size(uint32_t caps)
{
	if (caps & MALLOC_CAP_SPIRAM)
		return 0;
	struct mallinfo mi = mallinfo();
	return (size_t)mi.fordblks;
}

static inline size_t heap_caps_get_total_size(uint32_t caps)
{
	if (caps & MALLOC_CAP_SPIRAM)
		return 0;
	struct mallinfo mi = mallinfo();
	return (size_t)mi.arena;
}

static inline size_t heap_caps_get_largest_free_block(uint32_t caps)
{
	/* newlib does not report the largest free block. The free total is an
	 * upper bound, and these values are only ever printed. */
	return heap_caps_get_free_size(caps);
}

static inline void *heap_caps_malloc(size_t size, uint32_t caps)
{
	(void)caps;
	return malloc(size);
}

static inline void *heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps)
{
	(void)caps;
	return memalign(alignment, size);
}

static inline void heap_caps_free(void *ptr)
{
	free(ptr);
}

/**
 * @file pyro_core_mem.cpp
 * @brief 把全局 operator new/delete 重定向到 FreeRTOS 堆（移植自 PYRo Core/Memory）。
 *
 * 为什么必须一起移植：UART 驱动内部使用 std::map / std::vector / std::function，
 * 它们的动态分配走全局 operator new。若不重定向，分配会落到 newlib 的 _sbrk 堆上——
 * 既不受 FreeRTOS 堆记账，也没有线程安全（newlib 的 __malloc_lock 在本工程是空桩），
 * 失败时还会走 std::__throw_bad_alloc() → abort() → fprintf(stderr) → stdio。
 * 本文件与 `pyro_core_dma_heap.c` 的区别：这里接管的是标准 C++ 分配，DMA 堆是驱动专用的。
 */

#include <cstdlib>
#include "FreeRTOS.h"

void *operator new(const std::size_t size)
{
    void *ptr = pvPortMalloc(size);
    return ptr;
}

void *operator new[](const std::size_t size)
{
    void *ptr = pvPortMalloc(size);
    return ptr;
}

void operator delete(void *ptr) noexcept
{
    vPortFree(ptr);
}

void operator delete[](void *ptr) noexcept
{
    vPortFree(ptr);
}

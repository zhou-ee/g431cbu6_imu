/**
 * @file    spsc_race_test.cpp
 * @brief   SPSC 环形缓冲的并发正确性验证（主机侧，双线程真抢占）。
 *
 * 目的：用真实的多线程竞争证明三件事——
 *   ① 消费者永远读不到"半写"样本（生产者的 timestamp 与 payload 必须自洽）；
 *   ② 时间戳在单个消费者视角下严格单调（不倒退、不重复）；
 *   ③ 生产者/消费者各自只写自己的索引，因此无需任何锁。
 *
 * 判定方式：生产者写入自洽元组 (seq, seq^0x5A5A5A5A)，
 * 消费者逐样本校验两者关系；任何一个不成立即为撕裂，立即失败。
 *
 * 编译：g++ -std=c++17 -O2 -pthread spsc_race_test.cpp -o spsc_race_test
 */

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "imu_base_drv.h"

namespace
{

constexpr uint16_t kCapacity = 64U;    /* 必须是 2 的幂 */
constexpr uint32_t kPerStream = 4000000U;  /* 每路生产量 */

struct item_t
{
    uint32_t seq;
    uint32_t chk;
};

sample_stream_t<item_t> g_stream{};
item_t                  g_buf[kCapacity]{};

std::atomic<uint32_t> g_consumed{0};
std::atomic<bool>     g_tear{false};
std::atomic<bool>     g_disorder{false};
std::atomic<bool>     g_done{false};

/* 生产者：只写 tail，且先写 payload 再 publish。 */
void producer()
{
    for (uint32_t i = 0; i < kPerStream; ++i)
    {
        item_t *slot = g_stream.reserve();
        if (slot == nullptr)
        {
            /* A1：满则丢新样。这里退让一下让消费者追上，不算错误。 */
            std::this_thread::yield();
            --i;
            continue;
        }
        const uint32_t seq = i + 1U;
        /* 先写"数据"，最后一个字段写入后才 publish —— 与驱动侧一致。 */
        slot->seq = seq;
        slot->chk = seq ^ 0x5A5A5A5AU;
        g_stream.publish();
    }
    g_done = true;
}

/* 消费者：只写 head；批首取 available() 快照，逐样本校验。 */
void consumer()
{
    uint32_t last_seq = 0U;
    while (!g_done || g_stream.available() != 0U)
    {
        const uint16_t n = g_stream.available();
        if (n == 0U)
        {
            std::this_thread::yield();
            continue;
        }
        for (uint16_t i = 0; i < n; ++i)
        {
            const item_t &s = g_stream.at(i);
            /* ① 撕裂检测：两个字段必须来自同一次写入 */
            if (s.chk != (s.seq ^ 0x5A5A5A5AU))
                g_tear = true;
            /* ② 单调性：消费者视角下 seq 必须严格递增 */
            if (s.seq <= last_seq)
                g_disorder = true;
            last_seq = s.seq;
        }
        g_stream.release(n);
        g_consumed += n;
    }
}

} // namespace

int main()
{
    g_stream.sample   = g_buf;
    g_stream.capacity = kCapacity;
    g_stream.head     = 0U;
    g_stream.tail     = 0U;

    std::thread p(producer);
    std::thread c(consumer);
    p.join();
    std::printf("produced=%u consumed=%u\n", kPerStream, g_consumed.load());
    c.join();

    int failures = 0;
    if (g_tear)
    {
        std::printf("  FAIL torn sample observed (producer/consumer saw partial write)\n");
        ++failures;
    }
    if (g_disorder)
    {
        std::printf("  FAIL non-monotonic sequence observed\n");
        ++failures;
    }
    if (g_consumed.load() != kPerStream)
    {
        std::printf("  FAIL consumed %u != produced %u\n", g_consumed.load(), kPerStream);
        ++failures;
    }
    std::printf("%s: %s\n", failures == 0 ? "PASS" : "FAILED",
                failures == 0 ? "no tearing, seq monotonic, no sample lost"
                              : "concurrency invariant violated");
    return failures == 0 ? 0 : 1;
}

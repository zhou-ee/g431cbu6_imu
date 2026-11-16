/**
 * @file    spsc_mutation_test.cpp
 * @brief   变异测试：证明 spsc_race_test 的判据不是空测。
 *
 * 如果并发测试对错误的实现也报 PASS，它就没有证据价值。
 * 这里复刻两个历史缺陷，每次都必须被检测出来：
 *
 *   变异 1（半写可见）：生产者先 publish() 再填 payload
 *           —— 等价于旧 acc_alloc() 里"先 ++count 后写槽"的顺序。
 *   变异 2（覆盖最旧）：生产者满了去移动 head
 *           —— 等价于旧 push() 的"满则 ++head"。
 *
 * 判据：每个变异都必须被对应的检测器捕获，否则本测试自身失败。
 */

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

namespace
{

constexpr uint16_t kCapacity  = 64U;
constexpr uint32_t kPerStream = 500000U;

struct item_t
{
    uint32_t seq;
    uint32_t chk;
};

struct ring_t
{
    item_t *         buf{};
    uint16_t         cap{};
    volatile uint16_t head{};
    volatile uint16_t tail{};

    uint16_t available() const
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(tail) -
                                      static_cast<uint16_t>(head)) &
                                     static_cast<uint16_t>(cap - 1U));
    }
    const item_t &at(uint16_t i) const
    {
        return buf[(static_cast<uint16_t>(head) + i) & static_cast<uint16_t>(cap - 1U)];
    }
    void release(uint16_t n) { head = static_cast<uint16_t>((head + n) & (cap - 1U)); }
};

/* ---------------- 变异 1：先 publish 后写数据 ---------------- */
void run_publish_before_write(bool &tear, bool &lost)
{
    std::vector<item_t> buf(kCapacity);
    ring_t r;
    r.buf = buf.data();
    r.cap = kCapacity;
    std::atomic<bool> done{false};
    std::atomic<bool> t{false};

    std::thread p(
        [&]()
        {
            for (uint32_t i = 0; i < kPerStream; ++i)
            {
                item_t *slot = &r.buf[r.tail];
                /* ✗ 错误顺序：索引先可见，数据后写 */
                r.tail = static_cast<uint16_t>((r.tail + 1U) & (kCapacity - 1U));
                slot->seq = i + 1U;
                slot->chk = (i + 1U) ^ 0x5A5A5A5AU;
            }
            done = true;
        });

    std::thread c(
        [&]()
        {
            while (!done || r.available() != 0U)
            {
                const uint16_t n = r.available();
                for (uint16_t i = 0; i < n; ++i)
                {
                    const item_t &s = r.at(i);
                    if (s.seq != 0U && s.chk != (s.seq ^ 0x5A5A5A5AU))
                        t = true;
                }
                if (n != 0U) r.release(n);
            }
        });
    p.join();
    c.join();
    tear = t.load();
    lost = false;
}

/* ---------------- 变异 2：满了覆盖最旧（生产者写 head） ---------------- */
void run_overwrite_oldest(bool &tear, bool &disorder)
{
    std::vector<item_t> buf(kCapacity);
    ring_t r;
    r.buf = buf.data();
    r.cap = kCapacity;
    std::atomic<bool> done{false};
    std::atomic<bool> d{false};
    std::atomic<uint32_t> consumed{0};

    std::thread p(
        [&]()
        {
            for (uint32_t i = 0; i < kPerStream; ++i)
            {
                const uint16_t t = r.tail;
                if (static_cast<uint16_t>((t + 1U) & (kCapacity - 1U)) == r.head)
                    r.head = static_cast<uint16_t>((r.head + 1U) & (kCapacity - 1U)); /* ✗ */
                r.buf[t].seq = i + 1U;
                r.buf[t].chk = (i + 1U) ^ 0x5A5A5A5AU;
                r.tail = static_cast<uint16_t>((t + 1U) & (kCapacity - 1U));
            }
            done = true;
        });

    std::thread c(
        [&]()
        {
            uint32_t last = 0U;
            while (!done || r.available() != 0U)
            {
                const uint16_t n = r.available();
                for (uint16_t i = 0; i < n; ++i)
                {
                    const item_t &s = r.at(i);
                    if (s.seq <= last) d = true; /* 倒退或重复 */
                    last = s.seq;
                }
                if (n != 0U) r.release(n);
                consumed += n;
            }
        });
    p.join();
    c.join();
    tear      = false;
    disorder  = d.load();
}

} // namespace

int main()
{
    bool tear1 = false, lost1 = false;
    bool tear2 = false, disorder2 = false;

    run_publish_before_write(tear1, lost1);
    run_overwrite_oldest(tear2, disorder2);

    int failures = 0;
    std::printf("mutation 1 (publish before write): tear=%d\n", tear1);
    if (!tear1)
    {
        std::printf("  FAIL detector missed half-written sample\n");
        ++failures;
    }
    std::printf("mutation 2 (overwrite oldest, producer writes head): disorder=%d\n", disorder2);
    if (!disorder2)
    {
        std::printf("  FAIL detector missed head-moved-under-consumer\n");
        ++failures;
    }

    std::printf("%s: %s\n", failures == 0 ? "PASS" : "FAILED",
                failures == 0 ? "both historical defects detected by the same checks"
                              : "detector is not effective");
    return failures == 0 ? 0 : 1;
}

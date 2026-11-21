#ifndef MAP_H
#define MAP_H
#include <array>
#include <cstring>
namespace pyro
{
/**
 * @brief 轻量线性映射表（定容、无动态分配、ISR 只读）。
 *
 * 相对上游的加固（每处均有"移植"注释）：
 *   1. `_size` 类内初始化为 0（上游未初始化，非静态存储期对象上是 UB）；
 *   2. `erase()` 只在找到键时才移动元素并递减计数；
 *   3. `operator[]` 增加容量检查，满了返回哨兵并可通过 `full()` 查询；
 *   4. 存储用 C 数组而非 `std::array`：后者的 `operator[]` 会实例化
 *      `__glibcxx_assert` → `fprintf` + `abort` 路径，把 newlib 的
 *      stdio/malloc 链进固件，且该断言在本表的 ISR 调用点上是可达的；
 *   5. 构造/析构改为 `= default`：用户析构会让函数内静态对象需要
 *      `__cxa_atexit` 注册，裸机上永不执行；
 *   6. 删除上游的 `operator[](...) const` 重载：它在 const 成员里调用
 *      非 const 的 `exist()/find()`，任何实例化都会编译失败。
 */
template <typename K, typename V> class map_t
{

  public: // Linear map list
    /* 容量上限。注意：can_drv_t::_registerlist 用它作为可注册 CAN ID 的上限，
     * 与 pyro_can_drv.h 里的 MAX_ID_REGIST_NUM(32) 并不一致（上游遗留）。
     * 需要更多 ID 时改大这里，代价 = _max_size × (sizeof(K) + sizeof(V)) 字节静态 RAM。 */
    constexpr static size_t _max_size = 10;

    map_t()  = default;
    ~map_t() = default;

    V &operator[](const K &key)
    {
        const int index = find(key);
        if (index != -1)
        {
            return _values[index];
        }

        /* 容量耗尽：返回哨兵，绝不越界写（调用方应先用 full() 判断） */
        if (_size >= static_cast<int>(_max_size))
        {
            _overflow = V();
            return _overflow;
        }

        _keys[_size]   = key;
        _values[_size] = V();
        _size++;
        return _values[_size - 1];
    }

    int find(const K &key) const
    {
        for (int i = 0; i < _size; i++)
        {
            if (key == _keys[i])
            {
                return i;
            }
        }
        return -1;
    }
    bool exist(const K &key) const
    {
        if (find(key) != -1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int size() const
    {
        return _size;
    }

    bool full() const
    {
        return _size >= static_cast<int>(_max_size);
    }

    void clear()
    {
        _size = 0;
    }

    void erase(const K &key)
    {
        int index = find(key);
        /* 移植修正：上游在“键不存在”时也会无条件 _size--，会把表尾有效项挤出表外。
         * 这里只在真正找到键时才移动元素并递减计数。 */
        if (index == -1)
        {
            return;
        }
        for (int i = index + 1; i < _size; i++)
        {
            _keys[i - 1]   = _keys[i];
            _values[i - 1] = _values[i];
        }
        _size--;
    }

  private:
    /* 移植修正：上游此处未初始化，非静态存储期对象上首次 operator[] 会读到随机表长。
     * 增加类内初始化，静态对象行为不变。 */
    int _size = 0;
    /* 移植修正：std::array → C 数组；值初始化避免未定义值。 */
    K _keys[_max_size]{};
    V _values[_max_size]{};
    /* 容量耗尽时的哨兵返回值 */
    V _overflow = V();
};
}; // namespace pyro


#endif

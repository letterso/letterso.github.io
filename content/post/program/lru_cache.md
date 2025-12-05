---
layout:     post
title:       "LRU 缓存算法"
subtitle:    ""
description: "LRU 缓存算法说明和实现"
date:        2025-04-30
author:      "LETTER"
image:       ""
tags:
    - C++
    - 算法
categories:  ["program" ]
---

> 转载自 https://cseek.github.io/posts/lru-cache/

> 我们在使用安卓手机的时候，上划屏幕打开的任务栏中，排最前面的后台任务永远是最近我们使用过的, 而那些很久没被使用的后台任务会被放到最后，这种功能就可以使用 LRU 算法来实现。

## 什么是 LRU 算法？
LRU（Least Recently Used）是一种缓存淘汰策略，其核心思想是移除最近最少使用的数据，保留最近访问的数据。

## 代码实现

```c++
#include <list>
#include <unordered_map>

template<typename KeyType, typename ValueType>
class LRUCache {
private:
    size_t m_capacity;  // 缓存容量
    std::list<std::pair<KeyType, ValueType>> m_cache_list;  // 双向链表存储键值对
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, ValueType>>::iterator> m_cache_map;  // 哈希表映射

public:
    // 显式构造函数，避免隐式转换
    explicit LRUCache(size_t capacity) 
        : m_capacity(capacity) {}

    // 查询操作（返回bool表示是否存在，通过引用返回value）
    bool get(const KeyType& key, ValueType& value) {
        auto it = m_cache_map.find(key);
        if (it == m_cache_map.end()) {
            return false;  // 键不存在
        }
        // 移动节点到链表头部
        m_cache_list.splice(m_cache_list.begin(), m_cache_list, it->second);
        value = it->second->second;  // 通过引用返回值
        return true;
    }

    // 插入/更新操作
    void put(const KeyType& key, const ValueType& value) {
        auto it = m_cache_map.find(key);
        if (it != m_cache_map.end()) {
            // 更新值并移动到头部
            it->second->second = value;
            m_cache_list.splice(m_cache_list.begin(), m_cache_list, it->second);
            return;
        }
        // 插入新节点到头部
        m_cache_list.emplace_front(key, value);
        m_cache_map[key] = m_cache_list.begin();
        // 容量检查，删除尾部节点
        if (m_cache_map.size() > m_capacity) {
            auto last_key = m_cache_list.back().first;
            m_cache_map.erase(last_key);
            m_cache_list.pop_back();
        }
    }

    // 检查键是否存在
    bool contains(const KeyType& key) const {
        return m_cache_map.find(key) != m_cache_map.end();
    }

    // 获取当前缓存大小
    size_t size() const {
        return m_cache_map.size();
    }
};
```
在上面的代码中，可以看到，为了避免每次都去遍历 list 容器，所以采用 unordered_map 无需容器来缓存每个 key 对应的 list 迭代器，从而提升访问速度。
下面是这个缓存算法的用法：

```c++
#include <iostream>
#include <string>

int main() {
  // 键类型为 int，值类型为 std::string
  LRUCache<int, std::string> cache(3);
  // 插入数据
  cache.put(1, "One");
  cache.put(2, "Two");
  cache.put(3, "Three");
  // 查询数据
  std::string value;
  if (cache.get(2, value)) { // 命中，value="Two"
    std::cout << "Value of key 2: " << value << std::endl;
  }
  // 触发淘汰
  cache.put(4, "Four"); // 插入第四个条目，淘汰最久未使用的键1
  // 检查键是否存在
  std::cout << "Contains key 1? " << cache.contains(1)
            << std::endl; // 输出0（false）
  return 0;
}
```
#ifndef XPBD_TYPED_STORE_HPP
#define XPBD_TYPED_STORE_HPP

#include "xpbd/entity.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace xpbd {

template <typename T>
class TypedStore {
public:
    Entity create(EntityType type, const T& value)
    {
        std::uint32_t index = 0;
        if (!freeList_.empty()) {
            index = freeList_.back();
            freeList_.pop_back();
        } else {
            assert(data_.size() <= static_cast<std::size_t>(Entity::kInvalidIndex));
            index = static_cast<std::uint32_t>(data_.size());
            data_.push_back(T{});
            generations_.push_back(1);
            alive_.push_back(0);
        }

        data_[index] = value;
        alive_[index] = 1;
        return Entity::make(type, index, generations_[index]);
    }

    bool destroy(Entity entity)
    {
        const std::uint32_t index = entity.index();
        if (!alive(entity)) {
            return false;
        }

        alive_[index] = 0;
        generations_[index] = nextGeneration(generations_[index]);
        freeList_.push_back(index);
        return true;
    }

    bool alive(Entity entity) const
    {
        const std::uint32_t index = entity.index();
        return index < generations_.size() &&
               generations_[index] == entity.generation();
    }

    T* get(Entity entity)
    {
        return alive(entity) ? &data_[entity.index()] : nullptr;
    }

    const T* get(Entity entity) const
    {
        return alive(entity) ? &data_[entity.index()] : nullptr;
    }

    void destroyAll()
    {
        freeList_.clear();
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index] != 0) {
                generations_[index] = nextGeneration(generations_[index]);
            }
            alive_[index] = 0;
            freeList_.push_back(index);
        }
    }

    void release()
    {
        data_.clear();
        generations_.clear();
        alive_.clear();
        freeList_.clear();
    }

    std::size_t slots() const
    {
        return data_.size();
    }

    std::size_t aliveCount() const
    {
        return data_.size() - freeList_.size();
    }

    T* data()
    {
        return data_.empty() ? nullptr : data_.data();
    }

    const T* data() const
    {
        return data_.empty() ? nullptr : data_.data();
    }

    const std::uint8_t* aliveData() const
    {
        return alive_.empty() ? nullptr : alive_.data();
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func)
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index] == 0) {
                continue;
            }
            func(Entity::make(type, index, generations_[index]), data_[index]);
        }
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func) const
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index] == 0) {
                continue;
            }
            func(Entity::make(type, index, generations_[index]), data_[index]);
        }
    }

private:
    static std::uint16_t nextGeneration(std::uint16_t generation)
    {
        ++generation;
        return generation == 0 ? 1 : generation;
    }

    std::vector<T> data_;
    std::vector<std::uint16_t> generations_;
    std::vector<std::uint8_t> alive_;
    std::vector<std::uint32_t> freeList_;
};

}  // namespace xpbd

#endif  // XPBD_TYPED_STORE_HPP

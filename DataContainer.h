#pragma once
#include <vector>
#include <memory>
#include "MsgBase.h"
#include "Data.h"

class SystemDataContainer {
private:
    std::vector<std::unique_ptr<DataBase>> data_list_;

public:
    SystemDataContainer() = default;

    // Add a new SystemData of any size N
    template <size_t N>
    void add(const SystemData<N>& data)
    {
        data_list_.emplace_back(std::make_unique<SystemData<N>>(data));
    }

    size_t size() const
    {
        return data_list_.size();
    }

    const DataBase& operator[](size_t i) const
    {
        return *data_list_.at(i);
    }

    void printValue() const
    {
        for (const auto& data : data_list_)
        {
            data->printValue();
        }
    }

    size_t dataSize() const
    {
        size_t total = 0;
        for (const auto& data : data_list_) {
            total += data->dataSize();
        }
        return total;
    }

    // Serialize data_list into a buffer
    void serialize(std::vector<uint8_t>& buffer) const
    {
        buffer.reserve(dataSize());

        for (const auto& data : data_list_)
        {
            auto chunk = data->serialize();
            buffer.insert(buffer.end(), chunk.begin(), chunk.end());
        }
    }

    // Deserialize buffer message into all the SystemData stored in data_list_
    bool deserialize(const std::vector<uint8_t>& buffer)
    {
        size_t offset = 0;

        for (auto& data : data_list_)
        {
            size_t single_data_size = data->dataSize();
            if (offset + single_data_size > buffer.size())
            {
                std::cerr << "Buffer too small for deserializing SystemDataContainer.\n";
                return false;
            }

            data->readFromBuffer(buffer.data() + offset);
            offset += single_data_size;
        }

        return true;
    }

    // Clear the container
    void clear()
    {
        data_list_.clear();
    }
};

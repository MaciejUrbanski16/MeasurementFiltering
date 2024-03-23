#pragma once
#include <iostream>
#include <vector>
#include "wx/vector.h"
#include "wx/gdicmn.h"

//template <typename T>
class PlotElementsBuffer
{
public:
	explicit PlotElementsBuffer() : size_(100), buffer_(100, {0,0}), head_(0), count_(0) {}
    explicit PlotElementsBuffer(const uint64_t size) : size_(size), buffer_(size, { 0,0 }), head_(0), count_(0) {}

    void AddElement(const wxRealPoint& element) {
        buffer_.push_back(element);
        if (buffer_.size() > size_) {
            buffer_.erase(buffer_.begin());
        }
    }

    void Clear()
    {
        buffer_.clear();
    }

    wxVector <wxRealPoint> getBuffer() const { return buffer_; }

private:
    void IncrementIndex(size_t& index) const {
        index = (index + 1) % size_;
    }

    size_t size_;
    wxVector <wxRealPoint> buffer_;
    size_t head_;
    size_t count_;
};


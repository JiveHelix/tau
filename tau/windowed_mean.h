#pragma once


#include <jive/circular_index.h>
#include "tau/eigen_shim.h"


namespace tau
{


template<typename Value>
class WindowedMean
{
public:
    using Window =
        Eigen::Matrix
        <
            Value,
            Eigen::Dynamic,
            Eigen::Dynamic,
            Eigen::RowMajor
        >;

    using Row =
        Eigen::Matrix
        <
            Value,
            1,
            Eigen::Dynamic,
            Eigen::RowMajor
        >;

    WindowedMean()
        :
        rowIndex_(jive::CircularIndex<0>::Create(1)),
        sampleCount_(),
        columnCount_(),
        isInitialized_(),
        window_()
    {

    }

    WindowedMean(size_t sampleCount, size_t columnCount)
        :
        rowIndex_(jive::CircularIndex<0>::Create(sampleCount)),
        sampleCount_(tau::Index(sampleCount)),
        columnCount_(tau::Index(columnCount)),
        isInitialized_(false),
        window_(Window::Zero(this->sampleCount_, this->columnCount_))
    {

    }

    void Update(const Row &row)
    {
        this->window_.row(tau::Index(this->rowIndex_++)) = row;

        if (this->rowIndex_ == 0)
        {
            // We have a complete array from which to compute the mean.
            this->isInitialized_ = true;
        }
    }

    void Reset()
    {
        this->isInitialized_ = false;
        this->window_ = Window::Zero(this->sampleCount_, this->columnCount_);
        this->rowIndex_.Reset();
    }

    bool IsInitialized()
    {
        return this->isInitialized_;
    }

    auto GetMean() const
    {
        return this->window_.colwise().mean();
    }

private:
    jive::CircularIndex<0> rowIndex_;
    Eigen::Index sampleCount_;
    Eigen::Index columnCount_;
    bool isInitialized_;
    Window window_;
};


} // end namespace mv

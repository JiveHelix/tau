#pragma once


#include <Eigen/Dense>
#include <cmath>


namespace tau
{


template<typename Float>
class SlidingWindow
{
public:
    explicit SlidingWindow(int N)
        :
        windowLength_(N),
        circularBuffer_(N),
        writeIndex_(0),
        filledCount_(0),
        mean_(Float(0.0)),
        sumMeanDifferenceSquared_(Float(0.0))
    {

    }

    void Add(double value)
    {
        if (this->filledCount_ < this->windowLength_)
        {
            this->circularBuffer_(this->writeIndex_) = value;
            ++this->filledCount_;
            double delta = value - this->mean_;
            this->mean_ += delta / this->filledCount_;
            this->sumMeanDifferenceSquared_ += delta * (value - this->mean_);
        }
        else
        {
            double old = this->circularBuffer_(this->writeIndex_);
            this->circularBuffer_(this->writeIndex_) = value;

            // remove old value
            double deltaOut = old - this->mean_;
            this->mean_ -= (deltaOut / this->windowLength_);

            this->sumMeanDifferenceSquared_ -= (deltaOut * (old - this->mean_));

            // add new value
            double deltaIn = value - this->mean_;
            this->mean_ += (deltaIn / this->windowLength_);
            this->sumMeanDifferenceSquared_ += deltaIn * (value - this->mean_);
        }

        // advance circular index after everything else
        this->writeIndex_ = (this->writeIndex_ + 1) % this->windowLength_;
    }

    double GetMean() const
    {
        return this->mean_;
    }

    double GetStdDev() const
    {
        if (this->filledCount_ > 1)
        {
            return std::sqrt(
                this->sumMeanDifferenceSquared_ / (this->filledCount_ - 1));
        }

        return 0.0;
    }

private:
    const int windowLength_;
    Eigen::VectorXd circularBuffer_;
    int writeIndex_;
    int filledCount_;
    double mean_;

    // Σ (x − mean)^2
    double sumMeanDifferenceSquared_;
};


} // end namespace tau

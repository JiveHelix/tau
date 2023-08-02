#pragma once

#include <algorithm>
#include "tau/vector2d.h"
#include "tau/region.h"
#include "tau/scale.h"


namespace tau
{


template<typename T, typename ScaleType = double>
struct View
{
    static_assert(std::is_signed_v<T>, "View must use signed indices");

    Region<T> source;
    Region<T> target;
    Scale<ScaleType> scale;


    static constexpr auto fields = std::make_tuple(
        fields::Field(&View::source, "source"),
        fields::Field(&View::target, "target"),
        fields::Field(&View::scale, "scale"));

    /*
     * @param view The window on the source image, with position relative to the
     *             origin of the source.
     * @param sourceSize The unscaled size of the source image.
     * @param scale The scale applied to the source image.
     */
    View(
        const Region<T> &view,
        const Size<T> &sourceSize,
        const Scale<ScaleType> &scale_)
        :
        source{{{0, 0}, sourceSize}},
        target{view},
        scale{scale_}
    {
        auto scaledSourceRegion = this->source * this->scale;

        // The view is a clipping window on the scaled source image.
        // The source image is at position (0, 0)
        this->source = scaledSourceRegion.Intersect(view) / this->scale;

        // When the view is positive, start painting the target at zero.
        // When the view is shifted negative, start painting the target at
        // a positive shift of the same magnitude.
        Point2d<T> targetTopLeft(
            std::min(static_cast<T>(0), view.topLeft.x),
            std::min(static_cast<T>(0), view.topLeft.y));

        targetTopLeft *= -1;

        // The size of the target is limited by the source data we have to
        // paint it.
        auto targetIntersection = view.Intersect(scaledSourceRegion);

        // When the source view is smaller than the target, the target is
        // clipped by the source.
        this->target = Region<T>{{targetTopLeft, targetIntersection.size}};

        assert(this->source.topLeft.y < sourceSize.height);
        assert(this->source.topLeft.x < sourceSize.width);

        if constexpr (std::is_integral_v<T>)
        {
            // Floating-point types have rounding errors that make this check
            // noisy.
            assert(this->source.GetBottomRight().x <= sourceSize.width);
            assert(this->source.GetBottomRight().y <= sourceSize.height);
        }
    }

    bool HasArea() const
    {
        return this->source.size.HasArea()
            && this->target.size.HasArea();
    }
};


template<typename T>
std::ostream & operator<<(std::ostream &outputStream, const View<T> &view)
{
    return outputStream << fields::DescribeCompact(view);
}


} // end namespace tau

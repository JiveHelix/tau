#pragma once

#include "tau/region.h"
#include "tau/scale.h"


namespace tau 
{


template<typename T, typename ScaleType = double>
struct View
{
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
        target{{view}},
        scale{{scale_}}
    {
        auto scaledSourceRegion = this->source * scale_;

        // The view is a clipping window on the scaled source image.
        // The source image is at position (0, 0)
        this->source = scaledSourceRegion.Intersect(view) / scale_;

        // When the source view is smaller than the target, the target is
        // clipped by the source.
        this->target = view.Intersect(scaledSourceRegion);

        assert(this->source.topLeft < sourceSize.ToPoint());
        assert(this->source.GetBottomRight() <= sourceSize.ToPoint());
    }

    bool HasArea() const
    {
        return (this->source.size.GetArea() > 0)
            && (this->target.size.GetArea() > 0);
    }
};


template<typename T>
std::ostream & operator<<(std::ostream &outputStream, const View<T> &view)
{
    return outputStream << fields::DescribeCompact(view);
}


} // end namespace tau

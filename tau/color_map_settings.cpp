#include <tau/color_map_settings.h>


template struct pex::Group
    <
        tau::ColorMapSettingsSchema<int32_t>::template Schema,
        tau::ColorMapSettingsFinisher<int32_t>
    >;

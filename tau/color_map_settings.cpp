#include <tau/color_map_settings.h>


template struct pex::Group
    <
        tau::ColorMapSettingsFields,
        tau::ColorMapSettingsTemplate<int32_t>::template Template,
        tau::ColorMapSettingsCustom<int32_t>
    >;

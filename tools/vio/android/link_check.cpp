// Link check: an Android shared library that constructs OpenVINS's VioManager.
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
extern "C" int hsv2_vio_link_check() {
    ov_msckf::VioManagerOptions params;
    params.print_and_load(nullptr);
    ov_msckf::VioManager sys(params);
    return sys.initialized() ? 1 : 0;
}

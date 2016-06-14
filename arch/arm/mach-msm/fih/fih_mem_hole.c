/**
 * This module exists for the express purpose of removing memory
 * via the msm memory-remove mechanism (see
 * Documentation/devicetree/bindings/arm/msm/memory-reserve.txt). Compiling
 * this module into a kernel is essentially the means by which any
 * nodes in the device tree with compatible =
 * "qcom,msm-mem-hole" will be "activated", thus providing a
 * convenient mechanism for enabling/disabling memory removal
 * (qcom,memory-*).
 */

#include <linux/module.h>

#define FIH_MEM_HOLE_COMPAT_STR  "qcom,fih-mem-hole"

EXPORT_COMPAT(FIH_MEM_HOLE_COMPAT_STR);

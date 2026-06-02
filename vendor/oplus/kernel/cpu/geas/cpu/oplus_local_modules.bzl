load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_geas_cpu",
        srcs = native.glob([
            "geas.h",
        ]),
        conditional_srcs = {
            "CONFIG_OPLUS_FEATURE_GEAS_CPU": {
                True:["geas_cpu.c"],
                False:["empty.c"],
            },
        },
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_geas_cpu",
        module_list = [
            "oplus_bsp_geas_cpu",
        ],
    )

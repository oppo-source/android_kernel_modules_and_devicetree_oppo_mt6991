load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module", "oplus_ddk_get_target", "oplus_ddk_get_variant", "bazel_support_platform")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():
    target = oplus_ddk_get_target()
    variant  = oplus_ddk_get_variant()
    tv = "{}_{}".format(target, variant)

    if bazel_support_platform == "qcom":
        deps = [
            "//vendor/oplus/kernel/cpu/game_opt:oplus_bsp_game_opt",
            "//vendor/qcom/opensource/graphics-kernel:{}_msm_kgsl".format(tv),
        ]
    elif bazel_support_platform == "mtk":
        deps = ["//vendor/oplus/kernel/cpu/game_opt:oplus_bsp_game_opt"]

    define_oplus_ddk_module(
        name = "oplus_bsp_geas_system",
        srcs = native.glob([
            "geas.h",
        ]),
        conditional_srcs = {
            "CONFIG_OPLUS_FEATURE_GEAS": {
                True:["geas.c"],
                False:["empty.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_FDRIVE": {
                True:["bwmon_geas.h",
                      "geas_frame_drive.c",
                      "geas_sysctrl.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_BWMON": {
                True:["geas_bwmon.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_GPU": {
                True:["geas_gpu.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_EMI": {
                True:["geas_emi.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_MEMLAT": {
                True:["geas_memlat.c"],
            },
            "CONFIG_OPLUS_FEATURE_GEAS_NPU": {
                True:["geas_gpu.c"],
            },
        },
        ko_deps = deps,
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_geas_system",
        module_list = [
            "oplus_bsp_geas_system",
        ],
    )


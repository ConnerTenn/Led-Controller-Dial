const std = @import("std");
const Build = std.Build;
const com = @import("Pico-Zig/build-common.zig");

pub fn build(build_config: *Build) void {
    const options = Build.StaticLibraryOptions{
        .name = "led-controller",
        .optimize = build_config.standardOptimizeOption(Build.StandardOptimizeOptionOptions{
            .preferred_optimize_mode = .ReleaseFast,
        }),
        .target = build_config.standardTargetOptions(Build.StandardTargetOptionsArgs{
            .default_target = com.rp2350_target,
        }),
        .root_source_file = build_config.path("src/main.zig"),
    };
    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    // const mode = b.standardReleaseOptions();

    const lib = build_config.addStaticLibrary(options);
    // lib.linkLibC();
    // lib.linkSystemLibrary("c");

    // == Add the pico module ==
    const pico_module = build_config.addModule("pico", .{
        .root_source_file = build_config.path("Pico-Zig/pico.zig"),
    });

    com.configureOptions(build_config, pico_module, com.Options{
        .target = .rp2350,
    });

    lib.root_module.addImport("pico", pico_module);

    // == Add the required includes ==
    com.addPicoIncludes(build_config, pico_module, .rp2350);
    com.addArmIncludes(build_config, pico_module);

    com.addInclude(build_config, &lib.root_module, "./build/");
    com.addPicoIncludes(build_config, &lib.root_module, .rp2350);
    com.addArmIncludes(build_config, &lib.root_module);

    // == Define the install artifact ==
    const lib_artifact = build_config.addInstallArtifact(lib, .{});

    const build_step = build_config.step("build", "Build the application static library");
    build_step.dependOn(&lib_artifact.step);
}

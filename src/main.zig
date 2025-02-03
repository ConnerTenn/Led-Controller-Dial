const std = @import("std");
const math = std.math;
const tau = math.tau;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const hardware = pico.hardware;

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    hardware.gpio.default_led.init(hardware.gpio.Gpio.Config{
        .direction = .out,
    });

    csdk.sleep_ms(2000);
    stdio.print("== Led Controller ==\n", .{});

    var toggle = true;
    while (true) {
        hardware.gpio.default_led.put(toggle);
        csdk.sleep_ms(200);
        toggle = !toggle;
    }
}

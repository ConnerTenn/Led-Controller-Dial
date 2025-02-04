const std = @import("std");
const math = std.math;
const tau = math.tau;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const hardware = pico.hardware;
const WS2812 = pico.library.WS2812;

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    hardware.gpio.default_led.init(hardware.gpio.Gpio.Config{
        .direction = .out,
    });

    csdk.sleep_ms(2000);
    stdio.print("== Led Controller ==\n", .{});

    var ws2812 = WS2812.WS2812.create(hardware.gpio.Pin.create(0)) catch |err| {
        stdio.print("Failed to initialize ws2812: {}", .{err});
        return;
    };
    ws2812.init();

    stdio.print("Put pixel\n", .{});
    ws2812.putPixel(WS2812.Pixel.create(255, 255, 0, 0));

    var toggle = true;
    while (true) {
        hardware.gpio.default_led.put(toggle);
        csdk.sleep_ms(200);
        toggle = !toggle;
    }
}

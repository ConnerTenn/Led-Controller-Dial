const std = @import("std");
const math = std.math;
const tau = math.tau;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const hardware = pico.hardware;
const library = pico.library;
const WS2812 = library.WS2812;
const colour = library.colour;

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
    // ws2812.putPixel(WS2812.Pixel.create(0, 0, 255, 255));

    // var toggle = true;
    var hue: f32 = 0.0;
    while (true) {
        // hardware.gpio.default_led.put(toggle);
        // csdk.sleep_ms(200);
        // toggle = !toggle;

        const hsv = colour.HSV.create(hue, 1.0, 1.0);

        ws2812.putPixel(WS2812.Pixel.fromRGB(colour.RGB.fromHSV(hsv), 0.0));
        csdk.sleep_ms(1);
        hue += 0.001;
    }
}

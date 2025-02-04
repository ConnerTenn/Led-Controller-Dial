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

    //== Setup Objects ==
    //LED controller
    stdio.print("Create Led Strip\n", .{});
    var led_strip = library.led_strip.LedStrip(2).create(hardware.gpio.Pin.create(0)) catch |err| {
        stdio.print("Failed to initialize led strip: {}", .{err});
        return;
    };
    led_strip.init();

    //Motor
    stdio.print("Create DutyCycle Sampler\n", .{});
    var duty_cycle_sampler = pico.library.duty_cycle.DutyCycle.create(hardware.gpio.Pin.create(19)) catch |err| {
        stdio.print("Error:{}\n", .{err});
        return;
    };
    duty_cycle_sampler.init();

    //== Loop ==
    stdio.print("== Loop ==\n", .{});
    while (true) {
        const hue = duty_cycle_sampler.readDutyCycle();

        const hsv = colour.HSV.create(hue, 1.0, 1.0);
        const pixel_colour = WS2812.Pixel.fromRGB(colour.RGB.fromHSV(hsv), 0.0);

        stdio.print("Pixel: R:{} G:{} B:{} W:{}\r", .{
            pixel_colour.rgbw.red,
            pixel_colour.rgbw.green,
            pixel_colour.rgbw.blue,
            pixel_colour.rgbw.white,
        });

        for (led_strip.getFrontBuffer()) |*pixel| {
            pixel.* = pixel_colour;
        }

        led_strip.render();

        csdk.sleep_ms(1);
    }

    // var toggle = true;
    // while (true) {
    //     hardware.gpio.default_led.put(toggle);
    //     csdk.sleep_ms(200);
    //     toggle = !toggle;
    // }
}

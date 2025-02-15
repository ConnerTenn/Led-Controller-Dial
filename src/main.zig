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

    hardware.gpio.default_led.init(hardware.gpio.Pin.Config{
        .direction = .out,
    });

    csdk.sleep_ms(2000);
    stdio.print("== Led Controller ==\n", .{});

    //== Setup Objects ==
    //Buttons
    const button_up = hardware.gpio.Pin.create(28);
    _ = button_up; // autofix
    const button_center = hardware.gpio.Pin.create(26);
    _ = button_center; // autofix
    const button_down = hardware.gpio.Pin.create(27);
    _ = button_down; // autofix

    //LED controller
    stdio.print("Create Led Strip\n", .{});
    var led_strip = library.led_strip.LedStrip(4).create(hardware.gpio.Pin.create(9)) catch |err| {
        stdio.print("Failed to initialize led strip: {}", .{err});
        return;
    };
    led_strip.init();

    //Motor
    stdio.print("Create DutyCycle Sampler\n", .{});
    var duty_cycle_sampler = pico.library.duty_cycle.DutyCycle.create(hardware.gpio.Pin.create(3)) catch |err| {
        stdio.print("Error:{}\n", .{err});
        return;
    };
    duty_cycle_sampler.init();

    //Display
    var display = pico.library.gu128x32.GU128x32.create(
        hardware.gpio.Pin.create(18),
        hardware.gpio.Pin.create(19),
        hardware.gpio.Pin.create(16),
        hardware.gpio.Pin.create(17),
        hardware.gpio.Pin.create(20),
        hardware.gpio.Pin.create(21),
        .spi0,
    );
    display.init();

    display.writeCommand(pico.library.gu128x32.GU128x32.DisplayOnOff{
        .byte1 = .{
            .layer_0 = .active,
            .layer_1 = .active,
        },
        .byte2 = .{
            .and_op = 0,
            .xor_op = 0,
            .gram_enable = .on,
            .gram_invert = .normal,
        },
    });
    display.writeCommand(pico.library.gu128x32.GU128x32.AddressModeSet{
        .byte1 = .{
            .increment_x = .increment,
            .increment_y = .fixed,
        },
    });

    // display.writeCommand(pico.library.gu128x32.GU128x32.DataWrite{
    //     .byte1 = .{
    //         .data = 0b10001011,
    //     },
    // });
    // for (0..128) |idx| {
    //     display.writeCommand(pico.library.gu128x32.GU128x32.DataWrite{
    //         .byte1 = .{
    //             .data = @intCast(idx),
    //         },
    //     });
    // }
    // display.writeCommand(pico.library.gu128x32.GU128x32.DataWriteYAddress{
    //     .byte1 = .{},
    //     .byte2 = .{
    //         .gram_y_addr = 1,
    //     },
    // });
    // for (128..256) |idx| {
    //     display.writeCommand(pico.library.gu128x32.GU128x32.DataWrite{
    //         .byte1 = .{
    //             .data = @intCast(idx),
    //         },
    //     });
    // }
    // display.writeCommand(pico.library.gu128x32.GU128x32.AddressRead{});
    // display.display_buffer.setPixel(0, 0, true);
    // display.display_buffer.setPixel(1, 0, true);
    // display.display_buffer.drawLine(1, 2, 126, 30, true);
    // display.display_buffer.display_buffer[1][5] = 0b00111110;
    // display.display_buffer.display_buffer[1][6] = 0b01010001;
    // display.display_buffer.display_buffer[1][7] = 0b01001001;
    // display.display_buffer.display_buffer[1][8] = 0b01000101;
    // display.display_buffer.display_buffer[1][9] = 0b00111110;
    // display.display_buffer.print("R: {d: >3}", .{0}, 3, 0);
    // display.render();

    //== Loop ==
    stdio.print("== Loop ==\n", .{});
    while (true) {
        const hue = duty_cycle_sampler.readDutyCycle();

        const hsv = colour.HSV.create(hue, 1.0, 1.0);
        const pixel_colour = WS2812.Pixel.fromRGB(colour.RGB.fromHSV(hsv), 0.0);

        // stdio.print("Pixel: R:{d: >3} G:{d: >3} B:{d: >3} W:{d: >3}\r", .{
        //     pixel_colour.rgbw.red,
        //     pixel_colour.rgbw.green,
        //     pixel_colour.rgbw.blue,
        //     pixel_colour.rgbw.white,
        // });

        for (led_strip.getBackBuffer()) |*pixel| {
            pixel.* = pixel_colour;
        }

        led_strip.render();

        display.display_buffer.print("R: {d: >3}", .{pixel_colour.rgbw.red}, 0, 0);
        display.display_buffer.print("G: {d: >3}", .{pixel_colour.rgbw.green}, 1, 0);
        display.display_buffer.print("B: {d: >3}", .{pixel_colour.rgbw.blue}, 2, 0);
        display.display_buffer.print("W: {d: >3}", .{pixel_colour.rgbw.white}, 3, 0);
        display.render();
    }

    // var toggle = true;
    // while (true) {
    //     hardware.gpio.default_led.put(toggle);
    //     csdk.sleep_ms(200);
    //     toggle = !toggle;
    // }
}

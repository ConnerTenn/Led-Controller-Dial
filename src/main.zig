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
    const button_center = hardware.gpio.Pin.create(26);
    const button_down = hardware.gpio.Pin.create(27);

    button_up.init(hardware.gpio.Pin.Config{
        .direction = .in,
        .pull = .{ .up = false, .down = true },
    });
    button_center.init(hardware.gpio.Pin.Config{
        .direction = .in,
        .pull = .{ .up = false, .down = true },
    });
    button_down.init(hardware.gpio.Pin.Config{
        .direction = .in,
        .pull = .{ .up = false, .down = true },
    });

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

    const sensor = duty_cycle_sampler.getSensor();

    const pid = pico.library.pid.PIDcontrol.create(
        3.0,
        0.0,
        0.05,
    );

    csdk.gpio_set_function(10, csdk.GPIO_FUNC_PWM); //UL
    csdk.gpio_set_function(11, csdk.GPIO_FUNC_PWM); //UH
    csdk.gpio_set_function(12, csdk.GPIO_FUNC_PWM); //VL
    csdk.gpio_set_function(13, csdk.GPIO_FUNC_PWM); //VH
    csdk.gpio_set_function(14, csdk.GPIO_FUNC_PWM); //WL
    csdk.gpio_set_function(15, csdk.GPIO_FUNC_PWM); //WH
    var motor = pico.library.motor.Motor.create(
        6,
        7,
        5,
        7,
        sensor,
        pid,
    );

    duty_cycle_sampler.init();
    motor.init();

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
    display.display_buffer.print("Zeroing motor...", .{}, 0, 0);
    display.render();

    for (0..200) |sample_idx| {
        //Drive to the target angle
        const target_angle = tau * @as(f32, @floatFromInt(sample_idx)) / 200.0;

        motor.setTorque(1.0, 0.0, target_angle);
        csdk.sleep_ms(10);
    }

    const zeroMotor = struct {
        const dead_zone = 0.02;
        var zeroed = false;

        fn targetAngle(angle: f32, delta_time_s: f32) f32 {
            _ = delta_time_s; // autofix
            const delta_err = pico.math.deltaError(f32, angle, 0, tau);
            stdio.print("angle: {d:.3}  delta_err: {d:.3}\n", .{ angle, delta_err });

            if (@abs(delta_err) < dead_zone) {
                zeroed = true;
                return 0;
            }
            return -math.sign(delta_err) * 0.3;
        }

        fn zeroMotor(motor_obj: *pico.library.motor.Motor) void {
            while (!zeroed) {
                motor_obj.update(targetAngle);
            }
        }
    }.zeroMotor;
    zeroMotor(&motor);
    motor.setTorque(1.0, 0.0, 0.0);

    // var angle: f32 = 0;
    // while (true) {
    //     motor.setTorque(1.0, 0.0, angle);
    //     angle += 0.0001 * tau;
    //     csdk.sleep_ms(1);
    // }

    display.display_buffer.clear();
    display.render();

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

        // display.display_buffer.print("R: {d: >3}", .{pixel_colour.rgbw.red}, 0, 0);
        // display.display_buffer.print("G: {d: >3}", .{pixel_colour.rgbw.green}, 1, 0);
        // display.display_buffer.print("B: {d: >3}", .{pixel_colour.rgbw.blue}, 2, 0);
        // display.display_buffer.print("W: {d: >3}", .{pixel_colour.rgbw.white}, 3, 0);
        display.display_buffer.print("Hue: {d: <.3}", .{hsv.hue}, 0, 0);
        display.display_buffer.print("B1:{s}  B2:{s}  B3:{s}", .{
            if (!button_up.get()) "O" else "X",
            if (!button_center.get()) "O" else "X",
            if (!button_down.get()) "O" else "X",
        }, 3, 0);
        display.render();
    }

    // var toggle = true;
    // while (true) {
    //     hardware.gpio.default_led.put(toggle);
    //     csdk.sleep_ms(200);
    //     toggle = !toggle;
    // }
}

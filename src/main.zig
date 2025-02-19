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

const LedController = @import("led_controller.zig").LedController;

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    hardware.gpio.default_led.init(hardware.gpio.Pin.Config{
        .direction = .out,
    });

    csdk.sleep_ms(2000);
    stdio.print("== Online ==\n", .{});

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
    var motor = pico.library.motor.Motor(false).create(
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

    //== Initialization ==
    display.display_buffer.print("Zeroing motor...", .{}, 0, 0);
    display.render();

    const zeroMotor = struct {
        const dead_zone = 0.02;
        var zeroed = false;

        fn targetAngle(angle: f32, delta_time_s: f32) f32 {
            _ = delta_time_s; // autofix
            const delta_err = pico.math.deltaError(f32, angle, 0, tau);
            // stdio.print("angle: {d:.3}  delta_err: {d:.3}\n", .{ angle, delta_err });

            if (@abs(delta_err) < dead_zone) {
                zeroed = true;
                return 0;
            }

            return -math.sign(delta_err) * 0.3;
        }

        fn zeroMotor(motor_obj: *pico.library.motor.Motor(false)) void {
            while (!zeroed) {
                motor_obj.update(targetAngle);
            }
        }
    }.zeroMotor;
    zeroMotor(&motor);
    motor.setTorque(1.0, 0.0, 0.0);

    display.display_buffer.clear();
    display.render();

    //== Led Controller ==

    stdio.print("== Led Controller ==\n", .{});

    var led_controller = LedController(4).create(
        display,
        motor,
        button_up,
        button_center,
        button_down,
        led_strip,
    );

    led_controller.run();

    // var toggle = true;
    // while (true) {
    //     hardware.gpio.default_led.put(toggle);
    //     csdk.sleep_ms(200);
    //     toggle = !toggle;
    // }
}

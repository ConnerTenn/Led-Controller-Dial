const std = @import("std");
const math = std.math;
const tau = math.tau;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;

const Pin = pico.hardware.gpio.Pin;
const Motor = pico.library.motor.Motor;
const LedStrip = pico.library.led_strip.LedStrip;
const colour = pico.library.colour;
const WS2812 = pico.library.WS2812;
const gu128x32 = pico.library.gu128x32;

pub fn LedController(num_leds: comptime_int) type {
    return struct {
        const Self = @This();

        display: gu128x32.GU128x32,
        motor: Motor(false),
        button_up: Pin,
        button_center: Pin,
        button_down: Pin,
        led_strip: LedStrip(num_leds),

        hsv: colour.HSV = colour.HSV.create(0, 1.0, 0.5),
        white: f32 = 0,

        control_mode: enum {
            debug,
            hue,
        } = .debug,
        render_mode: enum {
            solid,
        } = .solid,

        pub fn create(
            display: pico.library.gu128x32.GU128x32,
            motor: Motor(false),
            button_up: Pin,
            button_center: Pin,
            button_down: Pin,
            led_strip: LedStrip(num_leds),
        ) Self {
            return Self{
                .display = display,
                .motor = motor,
                .button_up = button_up,
                .button_center = button_center,
                .button_down = button_down,
                .led_strip = led_strip,
            };
        }

        pub fn run(self: *Self) noreturn {
            while (true) {
                // stdio.print("loop\n", .{});
                self.display.display_buffer.clear();

                // stdio.print("control mode\n", .{});
                switch (self.control_mode) {
                    .debug => self.controlDebug(),
                    .hue => self.controlHue(),
                }

                // stdio.print("render mode\n", .{});
                switch (self.render_mode) {
                    .solid => self.renderSolid(),
                }

                // stdio.print("render\n", .{});
                self.led_strip.render();
                self.display.render();
            }
        }

        //== Control Functions ==

        fn controlHue(self: *Self) void {
            self.hsv.hue = self.motor.getAngle() / math.tau;

            self.display.display_buffer.print("Hue: {d: <.3}", .{self.hsv.hue}, 1, 38);
            // stdio.print("Hue: {d: <.3}", .{self.hsv.hue});

            // const bar_position: u7 = @as(u7, @intFromFloat(pico.math.mod(
            //     f32,
            //     -self.hsv.hue * gu128x32.DisplayBuffer.num_columns,
            //     gu128x32.DisplayBuffer.num_columns,
            //     .euclidean,
            // )));
            const bar_position: u7 = @intFromFloat(pico.math.remap(
                f32,
                self.hsv.hue,
                1.0,
                0.0,
                5,
                gu128x32.DisplayBuffer.num_columns - 5,
            ));
            // stdio.print("bar_position: {}", .{bar_position});

            self.display.display_buffer.drawRectangle(4, 2, gu128x32.DisplayBuffer.num_columns - 1 - 4, 22, true);
            self.display.display_buffer.drawLine(bar_position, 3, bar_position, 21, true);

            self.display.display_buffer.print("R   Y   G   C   B   M   R", .{}, 3, 2);
        }

        fn controlDebug(self: *Self) void {
            const angle = self.motor.sensor.getAngle() / math.tau;

            self.display.display_buffer.print("Angle: {d: <.3}", .{angle}, 0, 0);

            self.display.display_buffer.print("B1:{s}  B2:{s}  B3:{s}", .{
                if (!self.button_up.get()) "O" else "X",
                if (!self.button_center.get()) "O" else "X",
                if (!self.button_down.get()) "O" else "X",
            }, 3, 0);

            if (self.button_up.get() and self.button_center.get() and self.button_down.get()) {
                self.control_mode = .hue;
            }
        }

        //== Render Functions ==

        fn renderSolid(self: *Self) void {
            const pixel_colour = WS2812.Pixel.fromRGB(colour.RGB.fromHSV(self.hsv), self.white);

            for (self.led_strip.getBackBuffer()) |*pixel| {
                pixel.* = pixel_colour;
            }
        }
    };
}

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

        const ControlModes = enum {
            debug,
            hue,
            brightness,
        };

        const control_mode_sequence = [_]ControlModes{
            .hue, .brightness,
        };

        display: gu128x32.GU128x32,
        motor: Motor(false),
        button_up: Button,
        button_center: Button,
        button_down: Button,
        led_strip: LedStrip(num_leds),

        hsv: colour.HSV = colour.HSV.create(0, 1.0, 0.5),
        white: f32 = 0,
        brightness: f32 = 1.0,

        // control_mode: ControlModes = .debug,
        control_mode_sequence_idx: ?u32 = null,
        prev_control_mode_sequence_idx: ?u32 = null,
        render_mode: enum {
            solid,
        } = .solid,
        motor_mode: union(enum) {
            passive,
            manual,
            target: struct {
                target_angle: f32,
            },
            ratchet: struct {
                number_gredations: u16,
            },
        } = .passive,

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
                .button_up = Button.create(button_up, false),
                .button_center = Button.create(button_center, false),
                .button_down = Button.create(button_down, false),
                .led_strip = led_strip,
            };
        }

        pub fn run(self: *Self) noreturn {
            self.motor.setTorque(0.0, 0.0, 0);

            while (true) {
                // == Setup ==
                self.button_up.update();
                self.button_center.update();
                self.button_down.update();
                const dial_pos = pico.math.remap(
                    f32,
                    self.motor.sensor.getAngle(),
                    math.tau,
                    0.0,
                    0.0,
                    1.0,
                );

                // stdio.print("loop\n", .{});
                self.display.display_buffer.clear();

                // == Input Handling ==
                self.prev_control_mode_sequence_idx = self.control_mode_sequence_idx;
                if (self.button_up.justPressed()) {
                    if (self.control_mode_sequence_idx) |*control_mode_sequence_idx| {
                        control_mode_sequence_idx.* += 1;
                        if (control_mode_sequence_idx.* >= control_mode_sequence.len) {
                            control_mode_sequence_idx.* = 0;
                        }
                    }
                }

                if (self.button_down.justPressed()) {
                    if (self.control_mode_sequence_idx) |*control_mode_sequence_idx| {
                        if (control_mode_sequence_idx.* == 0) {
                            control_mode_sequence_idx.* = control_mode_sequence.len - 1;
                        } else {
                            control_mode_sequence_idx.* -= 1;
                        }
                    }
                }

                // == Control Mode ==
                // stdio.print("control mode\n", .{});
                if (self.control_mode_sequence_idx) |control_mode_sequence_idx| {
                    const control_mode = control_mode_sequence[control_mode_sequence_idx];
                    switch (control_mode) {
                        .hue => self.controlHue(dial_pos),
                        .brightness => self.controlBrightness(dial_pos),
                        else => unreachable,
                    }
                } else {
                    self.controlDebug(dial_pos);
                }

                // == Drive Motor ==
                if (self.motor_mode != .manual) {
                    self.motor.update(torqueFn, self);
                }

                // == Render ==
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

        fn controlHue(self: *Self, dial_pos: f32) void {
            // On entrance to this mode
            if (self.prev_control_mode_sequence_idx != self.control_mode_sequence_idx) {
                self.motor_mode = .{ .target = .{
                    .target_angle = pico.math.remap(
                        f32,
                        self.hsv.hue,
                        0.0,
                        1.0,
                        math.tau,
                        0.0,
                    ),
                } };
            }

            // Don't update the hue until the motor zeros out
            if (self.motor_mode != .target) {
                self.hsv.hue = dial_pos;
            }

            self.display.display_buffer.print("Hue: {d: <.3}", .{self.hsv.hue}, 1, 38);
            // stdio.print("Hue: {d: <.3}", .{self.hsv.hue});

            const bar_position: u7 = @intFromFloat(pico.math.remap(
                f32,
                self.hsv.hue,
                0.0,
                1.0,
                5,
                gu128x32.DisplayBuffer.num_columns - 5,
            ));

            self.display.display_buffer.drawRectangle(4, 2, 123, 22, true);
            self.display.display_buffer.drawLine(bar_position, 3, bar_position, 21, true);

            self.display.display_buffer.print("R", .{}, 3, 4);
            self.display.display_buffer.print("y", .{}, 3, 21);
            self.display.display_buffer.print("G", .{}, 3, 41);
            self.display.display_buffer.print("c", .{}, 3, 61);
            self.display.display_buffer.print("B", .{}, 3, 81);
            self.display.display_buffer.print("m", .{}, 3, 101);
            self.display.display_buffer.print("R", .{}, 3, 120);
        }

        fn controlBrightness(self: *Self, dial_pos: f32) void {
            // On entrance to this mode
            if (self.prev_control_mode_sequence_idx != self.control_mode_sequence_idx) {
                self.motor_mode = .{ .target = .{
                    .target_angle = pico.math.remap(
                        f32,
                        self.brightness,
                        0.0,
                        1.0,
                        (0.5 - 0.1) * math.tau,
                        (-0.5 + 0.1) * math.tau,
                    ),
                } };
            }

            // Don't update the hue until the motor zeros out
            if (self.motor_mode != .target) {
                var brightness: f32 = pico.math.remap(
                    f32,
                    pico.math.mod(f32, dial_pos - 0.5, 1.0, .euclidean),
                    0.1,
                    1.0 - 0.1,
                    0.0,
                    1.0,
                );

                if (brightness > 1.05) {
                    // self.motor.setTorque(0.0, (brightness - 1.0) * 10.0, self.motor.getAngle());
                    self.motor.setTorque(0.0, 1.0, self.motor.getAngle());
                    self.motor_mode = .manual;
                } else if (brightness < -0.05) {
                    // self.motor.setTorque(0.0, (brightness) * 10.0, self.motor.getAngle());
                    self.motor.setTorque(0.0, -1.0, self.motor.getAngle());
                    self.motor_mode = .manual;
                } else {
                    self.motor_mode = .passive;
                }

                brightness = @min(@max(brightness, 0.0), 1.0);

                self.brightness = brightness;
            }

            const bar_position: u7 = @intFromFloat(pico.math.remap(
                f32,
                self.brightness,
                0.0,
                1.0,
                3,
                124,
            ));

            self.display.display_buffer.drawRectangle(2, 2, 125, 22, true);
            self.display.display_buffer.fillRectangle(3, 3, bar_position, 21, true);

            self.display.display_buffer.print("Brightness  {d: >6.1}%", .{self.brightness * 100.0}, 3, 2);
        }

        fn controlDebug(self: *Self, dial_pos: f32) void {
            self.display.display_buffer.print("Dial: {d: <.3}", .{dial_pos}, 0, 0);

            self.display.display_buffer.print("B1:{s}  B2:{s}  B3:{s}", .{
                if (!self.button_up.state) "O" else "X",
                if (!self.button_center.state) "O" else "X",
                if (!self.button_down.state) "O" else "X",
            }, 3, 0);

            if (self.button_up.state and self.button_center.state and self.button_down.state) {
                self.control_mode_sequence_idx = 0;
            }
        }

        // == Motor Torque Function ==
        fn torqueFn(angle: f32, delta_time_s: f32, ctx: ?*const anyopaque) f32 {
            _ = delta_time_s; // autofix
            var torque: f32 = 0.0;

            const self: *Self = @constCast(@alignCast(@ptrCast(ctx)));
            switch (self.motor_mode) {
                .passive => {},
                .manual => unreachable,
                .target => |mode| {
                    const deadzone = 0.01;

                    const delta_error = pico.math.deltaError(f32, angle, mode.target_angle, tau);
                    pico.stdio.print("target  dE:{d:.3}\n", .{delta_error});

                    if (@abs(delta_error) < deadzone) {
                        self.motor.setTorque(0.0, 0.0, 0.0);
                        self.motor_mode = .passive;
                    } else {
                        // Ensure a minimum force is applied
                        torque = -math.sign(delta_error) * @max(@abs(delta_error), 0.05);
                    }
                },
                .ratchet => |mode| {
                    _ = mode; // autofix
                },
            }

            return torque;
        }

        //== Render Functions ==

        fn renderSolid(self: *Self) void {
            self.hsv.value = self.brightness;
            const white = self.white * self.brightness;
            const pixel_colour = WS2812.Pixel.fromRGB(colour.RGB.fromHSV(self.hsv), white);

            for (self.led_strip.getBackBuffer()) |*pixel| {
                pixel.* = pixel_colour;
            }
        }
    };
}

pub const Button = struct {
    const Self = @This();

    pin: Pin,
    inverted: bool,
    state: bool = false,
    prev_state: bool = false,

    pub fn create(pin: Pin, inverted: bool) Self {
        return Self{
            .pin = pin,
            .inverted = inverted,
        };
    }

    pub fn update(self: *Self) void {
        self.prev_state = self.state;
        //XOR operation to optionally invert pin
        self.state = self.pin.get() != self.inverted;
    }

    pub fn justPressed(self: *Self) bool {
        return self.state and !self.prev_state;
    }

    pub fn justReleased(self: *Self) bool {
        return !self.state and self.prev_state;
    }
};

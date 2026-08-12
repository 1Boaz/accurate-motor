#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cmp::PartialEq;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Output, OutputConfig, Pull};
use esp_hal::gpio::Level::Low;
use esp_hal::main;
use esp_hal::ledc::{timer, LSGlobalClkSource, Ledc, LowSpeed, channel};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::time::{Duration, Instant, Rate};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]

struct State {
    sensor1: bool,
    sensor2: bool,
}

impl PartialEq for State {
    fn eq(&self, state: &Self) -> bool {
        self.sensor1 == state.sensor1 && self.sensor2 == state.sensor2
    }
}

#[main]
fn main() -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);

    let motor_pin_1 = _peripherals.GPIO0;
    let motor_pin_2 = _peripherals.GPIO1;
    let motor_config = OutputConfig::default();
    let mut motor_pin_1 = Output::new(motor_pin_1, Low ,motor_config);
    let mut motor_pin_2 = Output::new(motor_pin_2, Low ,motor_config);

    let mut ledc = Ledc::new(_peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let _ = lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(40),
    });
    let mut channel0 = ledc.channel(channel::Number::Channel0, _peripherals.GPIO10);
    let _ = channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 100,
        drive_mode: DriveMode::PushPull,
    });


    let sensor = _peripherals.GPIO5;
    let sensor2 = _peripherals.GPIO6;
    let sensor_config = InputConfig::default().with_pull(Pull::Down);
    let sensor = Input::new(sensor, sensor_config);
    let sensor2 = Input::new(sensor2, sensor_config);
    let mut sensors_last_state = State {
        sensor1: sensor.is_high(),
        sensor2: sensor2.is_high(),
    };
    let mut sensors_state;
    let mut degrees: f32 = 0.0;
    motor_pin_1.set_high();

    loop {
        sensors_state = State{
            sensor1: sensor.is_high(),
            sensor2: sensor2.is_high(),
        };

        if sensors_state != sensors_last_state {
            match sensors_last_state {
                State {sensor1: true, sensor2: false} => {
                    if sensors_state == (State {sensor1: false, sensor2: false}) {
                        degrees += 4.5
                    } else if sensors_state == (State {sensor1: true, sensor2: true}) {
                        degrees -= 4.5
                    }
                },
                State {sensor1: true, sensor2: true} => {
                    if sensors_state == (State {sensor1: true, sensor2: false}) {
                        degrees += 4.5
                    } else if sensors_state == (State {sensor1: false, sensor2: true}) {
                        degrees -= 4.5
                    }
                },
                State {sensor1: false, sensor2: true} => {
                    if sensors_state == (State {sensor1: true, sensor2: true}) {
                        degrees += 4.5
                    } else if sensors_state == (State {sensor1: false, sensor2: false}) {
                        degrees -= 4.5
                    }
                },
                State {sensor1: false, sensor2: false} => {
                    if sensors_state == (State {sensor1: false, sensor2: true}) {
                        degrees += 4.5
                    } else if sensors_state == (State {sensor1: true, sensor2: false}) {
                        degrees -= 4.5
                    }
                },
            };
        }
        if degrees >= 360.0 {
            motor_pin_1.set_low();
            motor_pin_2.set_high();
        }
        if degrees <= -360.0 {
            motor_pin_2.set_low();
            motor_pin_1.set_high();
        }
        sensors_last_state = sensors_state;
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1) {}
    }
}

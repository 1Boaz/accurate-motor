#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_println::println;

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
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);

    let sensor = _peripherals.GPIO5;
    let config = InputConfig::default().with_pull(Pull::Down);
    let sensor = Input::new(sensor, config);
    let mut last_val = sensor.is_high();
    let mut val = sensor.is_high();
    let mut degrees = 0;
    loop {
        val = sensor.is_high();
        if last_val != val {
            degrees += 18;
            println!("changed now {} degrees", degrees);
        }
        let delay_start = Instant::now();
        last_val = val;
        while delay_start.elapsed() < Duration::from_millis(1) {}
    }
}

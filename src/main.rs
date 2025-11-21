//! RAK4631 USB CDC Serial Example
//!
//! This firmware demonstrates:
//! - USB CDC (Communication Device Class) for serial output
//! - LED blinking independent of USB connection state
//! - Proper handling of USB connect/disconnect cycles
//! - 1200 baud touch reset to enter bootloader (UF2)
//!
//! Hardware: RAK4631 WisBlock Core (nRF52840)
//! - Blue LED on P1.04
//! - USB CDC appears as /dev/cu.usbmodem123456781 (macOS)

#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, gpio, peripherals, usb};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use nrf52840_pac as pac;
use panic_halt as _;

// USB VID/PID for generic test device
const USB_VID: u16 = 0x1209; // Generic
const USB_PID: u16 = 0x0001; // Test PID

// Timing constants
const LED_BLINK_MS: u64 = 500;

// USB buffer sizes
const USB_MAX_PACKET_SIZE: u16 = 64;
const USB_DESCRIPTOR_BUF_SIZE: usize = 256;
const USB_CONTROL_BUF_SIZE: usize = 64;

// Bootloader trigger: connecting at 1200 baud resets into UF2 bootloader
const BOOTLOADER_TRIGGER_BAUD: u32 = 1200;
const DFU_MAGIC_UF2_RESET: u32 = 0x57; // Adafruit bootloader magic

// Bind USB interrupts to their handlers
bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize nRF52840 peripherals
    let p = embassy_nrf::init(Default::default());

    // Enable external high-frequency oscillator required for USB operation
    // The nRF52840 USB peripheral requires the external 32MHz crystal
    let clock = unsafe { &*pac::CLOCK::ptr() };
    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    // Configure blue LED on P1.04 (GPIO 36)
    let led = gpio::Output::new(p.P1_04, gpio::Level::Low, gpio::OutputDrive::Standard);

    // Create USB driver with hardware VBUS detection
    let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    // Configure USB device descriptor
    let mut config = Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("RAK");
    config.product = Some("RAK4631 USB CDC");
    config.serial_number = Some("12345678");
    config.max_power = 100; // mA
    config.max_packet_size_0 = USB_MAX_PACKET_SIZE as u8;

    // Allocate USB descriptor buffers
    let mut config_descriptor = [0u8; USB_DESCRIPTOR_BUF_SIZE];
    let mut bos_descriptor = [0u8; USB_DESCRIPTOR_BUF_SIZE];
    let mut msos_descriptor = [0u8; USB_DESCRIPTOR_BUF_SIZE];
    let mut control_buf = [0u8; USB_CONTROL_BUF_SIZE];
    let mut state = State::new();

    // Build USB device with CDC-ACM class
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create CDC-ACM (serial) class
    let mut class = CdcAcmClass::new(&mut builder, &mut state, USB_MAX_PACKET_SIZE);
    let mut usb = builder.build();

    // Spawn LED blink task - runs independently from boot
    spawner
        .spawn(led_task(led))
        .expect("Failed to spawn LED task");

    // Run USB stack and message sending concurrently
    let usb_fut = usb.run();

    let message_fut = async {
        let mut counter = 0u32;
        let mut message_timer = 0u32;

        loop {
            // Wait for USB enumeration
            class.wait_connection().await;

            // Main loop - check baud and send messages
            loop {
                Timer::after_millis(100).await;

                // Check for 1200 baud bootloader trigger continuously
                if class.line_coding().data_rate() == BOOTLOADER_TRIGGER_BAUD {
                    // Write magic value to GPREGRET to signal bootloader
                    unsafe {
                        let power = &*pac::POWER::ptr();
                        power.gpregret.write(|w| w.bits(DFU_MAGIC_UF2_RESET));
                    }
                    cortex_m::interrupt::free(|_| {
                        SCB::sys_reset();
                    });
                }

                // Send message every ~1 second (10 iterations * 100ms)
                message_timer += 1;
                if message_timer >= 10 {
                    message_timer = 0;

                    // Only send if DTR (Data Terminal Ready) is asserted
                    if class.dtr() {
                        let mut buf = [0u8; USB_MAX_PACKET_SIZE as usize];
                        if let Some(len) = format_message(&mut buf, counter) {
                            match class.write_packet(&buf[..len]).await {
                                Ok(_) => {}
                                Err(EndpointError::BufferOverflow) => {
                                    panic!("USB buffer overflow")
                                }
                                Err(EndpointError::Disabled) => {
                                    break; // Disconnected
                                }
                            }
                        }
                    }

                    counter = counter.wrapping_add(1);
                }
            }
        }
    };

    join(usb_fut, message_fut).await;
}

/// LED blink task - runs independently of USB connection state
#[embassy_executor::task]
async fn led_task(mut led: gpio::Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(LED_BLINK_MS).await;
        led.set_low();
        Timer::after_millis(LED_BLINK_MS).await;
    }
}

/// Format a message with counter value into the buffer
///
/// Returns the number of bytes written, or None if buffer is too small.
/// Uses manual formatting to avoid std dependencies.
fn format_message(buf: &mut [u8], counter: u32) -> Option<usize> {
    const PREFIX: &[u8] = b"Hello from nRF52840! Counter: ";
    const MAX_DIGITS: usize = 10; // u32::MAX is 10 digits
    const CRLF_LEN: usize = 2;

    // Check buffer has space for prefix + max digits + CRLF
    if buf.len() < PREFIX.len() + MAX_DIGITS + CRLF_LEN {
        return None;
    }

    let mut idx = 0;

    // Copy message prefix
    buf[..PREFIX.len()].copy_from_slice(PREFIX);
    idx += PREFIX.len();

    // Convert counter to ASCII digits (right-to-left)
    let mut digits = [0u8; MAX_DIGITS];
    let mut num = counter;
    let mut digit_count = 0;

    if num == 0 {
        digits[0] = b'0';
        digit_count = 1;
    } else {
        while num > 0 {
            digits[digit_count] = (num % 10) as u8 + b'0';
            num /= 10;
            digit_count += 1;
        }
    }

    // Write digits in correct order (reverse from how we built them)
    for i in 0..digit_count {
        buf[idx] = digits[digit_count - 1 - i];
        idx += 1;
    }

    // Add CRLF line ending for proper terminal display
    buf[idx] = b'\r';
    buf[idx + 1] = b'\n';
    idx += 2;

    Some(idx)
}

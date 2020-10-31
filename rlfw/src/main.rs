#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(alloc_prelude)]
#![feature(const_mut_refs)]

use core::alloc::Layout;

extern crate alloc;
use alloc::prelude::v1::*;

use alloc_cortex_m::CortexMHeap;
use cortex_m_rt::{entry, exception};

use panic_abort as _;

use embedded_hal::blocking::delay::DelayMs;

use atsamd_hal::clock::GenericClockController;
use atsamd_hal::delay::Delay;
use atsamd_hal::prelude::*;
use atsamd_hal::target_device::{interrupt, CorePeripherals, Peripherals};
use atsamd_hal::time::KiloHertz;
use atsamd_hal::pad::PadPin;
use atsamd_hal::sercom::SPIMaster0;



use xiao_m0::usb::UsbBus;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;

use usbd_serial::SerialPort;

use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;

use cortex_m::peripheral::NVIC;

use log::{info, warn, Level};

use smart_leds::hsv::RGB8;
use smart_leds::{brightness, SmartLedsWrite};

use ws2812_spi::Ws2812;

mod logger;
use logger::Logger;

mod hid;
use hid::DeviceReport;

/// Global allocator memory size
const ALLOC_SIZE: usize = 10 * 1024;

// Setup global allocator
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// Setup USB components
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_HID: Option<HIDClass<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

// Device serial no. (base64'd chip ID)
static mut DEVICE_SERIAL: Option<String> = None;

/// Global sys-tick counter value
static mut SYSTICK_COUNT: usize = 0;

/// SysTick interrupt handler
#[exception]
fn SysTick() {
    unsafe {
        SYSTICK_COUNT = SYSTICK_COUNT.wrapping_add(1);
    }
}

/// Number of available LEDs
const NUM_LEDS: usize = 24;

/// Shared LED data
static mut LED_DATA: [RGB8; NUM_LEDS] = [RGB8::new(0xef, 0xeb, 0xd8); NUM_LEDS];

/// Shared LED brightness
static mut LED_BRIGHTNESS: u8 = 0;

static mut RESTART_BOOTLOAD: Option<usize> = None;

/// Generate and fetch device serial with the provided prefix
fn get_device_serial(prefix: &str) -> String {
    // Read from device
    let serial_raw = atsamd_hal::serial_number();

    // Encode as hex
    let encoded = base64::encode(&serial_raw[..]);

    alloc::format!("{}-{}", prefix, encoded)
}

/// Empty type to implement write over USB serial
struct UsbSerialWriter;

/// Log writer object provides core::fmt::Write over the USB_SERIAL object
static mut LOG_WRITER: UsbSerialWriter = UsbSerialWriter;

impl core::fmt::Write for UsbSerialWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if s.len() == 0 {
            return Ok(());
        }

        cortex_m::interrupt::free(|_| unsafe {
            let serial = USB_SERIAL.as_mut().unwrap();

            let _r = serial.write(s.as_bytes());

            Ok(())
        })
    }
}

// Application main
#[entry]
fn main() -> ! {
    // Setup allocator
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, ALLOC_SIZE) }

    // Fetch peripherals
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    // Setup clocks using itnernal oscillator
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Setup device pins
    let mut pins = xiao_m0::Pins::new(peripherals.PORT);

    // Setup LED pins
    let (mut led0, mut _led1) = (
        pins.led0.into_push_pull_output(&mut pins.port),
        pins.led1.into_push_pull_output(&mut pins.port),
    );

    // Setup SPI
    let sck = pins.a8;
    let mosi = pins.a10;
    let miso = pins.a9;
    let gclk0 = clocks.gclk0();
    
    let spi = SPIMaster0::new(
        &clocks.sercom0_core(&gclk0).unwrap(),
        KiloHertz(2500),
        embedded_hal::spi::Mode {
            phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
            polarity: embedded_hal::spi::Polarity::IdleLow,
        },
        peripherals.SERCOM0,
        &mut peripherals.PM,
        (miso.into_pad(&mut pins.port), mosi.into_pad(&mut pins.port), sck.into_pad(&mut pins.port)),
    );


    // Setup USB bus allocator
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(xiao_m0::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
            &mut pins.port,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    // Load device serial
    unsafe { DEVICE_SERIAL = Some(get_device_serial("EPSB1")) };

    // Setup logging
    unsafe { Logger::init(Level::Info, &mut LOG_WRITER) };

    // Setup USB HID device
    unsafe {
        USB_HID = Some(HIDClass::new(&bus_allocator, DeviceReport::desc(), 100));
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x1209, 0xFFF1))
                .manufacturer("ElectronPowered Ltd")
                .product("ringlight v1")
                .serial_number(DEVICE_SERIAL.as_deref().unwrap_or("unknown"))
                .device_class(0xE3) // misc, should be 0x03 for HID?
                .build(),
        );
    }

    // Setup neopixels
    let mut neopixel = Ws2812::new(spi);

    // Enable USB interrupts
    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    let mut i: u32 = 0;

    loop {
        
        let _ = cortex_m::interrupt::free(|_| {
            unsafe {
                // Update neopixel
                let _ = neopixel.write(brightness(LED_DATA.iter().cloned(), LED_BRIGHTNESS));

                // Check for restart command
                match RESTART_BOOTLOAD.as_mut() {
                    Some(v) if *v == 0 => uf2_reset(),
                    Some(v) => *v -= 1,
                    None => (),
                }
            }
        });

        // Flash LED
        if (i % 10) < 5 {
            led0.set_high().unwrap();
        } else {
            led0.set_low().unwrap();
        }

        if (i % 100) == 0 {
            info!("Tick: {} ({})\r\n", i, unsafe { SYSTICK_COUNT });
        }

        // Increment loop counter
        i = i.wrapping_add(1);

        // Wait 100ms before next loop
        delay.delay_ms(100u32);
    }
}

/// U2F state pointer at RAM end - 4
const UF2_STATE_ADDR: *mut u32 = 0x2000_1FFB as *mut u32;

/// U2F magic number to hold in bootloader mode
const UF2_STATE_MAGIC: u32 = 0xf01669ef;

unsafe fn uf2_reset() {
    // Disable interrupts
    cortex_m::interrupt::disable();

    // Write magic state bytes
    core::ptr::write_volatile(UF2_STATE_ADDR, UF2_STATE_MAGIC);

    // Trigger device reset
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_command(cmd: &[u8]) {
    match cmd[0] {
        // Reset
        0x00 => {
            info!("Reset to default output");
            
            cortex_m::interrupt::free(|_| {
                unsafe {
                    for v in &mut LED_DATA {
                        *v = RGB8::new(0xef, 0xeb, 0xd8);
                    }
                    LED_BRIGHTNESS = 0;
                }
            });
        }
        // Direct RGB
        0x01 if cmd.len() == 4 => {
            info!("Set RGB to: {} {} {}", cmd[1], cmd[2], cmd[3]);

            let rgb = RGB8::new(cmd[1], cmd[2], cmd[3]);

            cortex_m::interrupt::free(|_| {
                unsafe {
                    for v in &mut LED_DATA {
                        *v = rgb;
                    }
                }
            })
        },
        // Direct HSV
        0x02 if cmd.len() == 4 => {
            info!("Set HSV to: {} {} {}", cmd[1], cmd[2], cmd[3]);

            let hsv = smart_leds::hsv::Hsv{hue: cmd[1], sat: cmd[2], val: cmd[3]};
            let rgb = smart_leds::hsv::hsv2rgb(hsv);

            cortex_m::interrupt::free(|_| {
                unsafe {
                    for v in &mut LED_DATA {
                        *v = rgb;
                    }
                }
            })
        },
        // Set brightness
        0x03 if cmd.len() == 2 => {
            info!("Set brightness to: {}", cmd[1]);

            cortex_m::interrupt::free(|_| {
                unsafe {
                    LED_BRIGHTNESS = cmd[1];
                }
            })
        },
        0xf0 => {
           info!("Restart to bootloader");
           unsafe { RESTART_BOOTLOAD = Some(10) }
        }
        _ => {
            warn!("Unhandled HID command: {:02x?}", cmd);
        }
    }
}

// Poll USB handlers device (called from USB interrupt)
fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            // Fetch endpoints
            let hid = USB_HID.as_mut().unwrap();
            let serial = USB_SERIAL.as_mut().unwrap();

            // Poll on device (auto-updates HID as required)
            usb_dev.poll(&mut [hid, serial]);

            // Read incoming HID data
            let mut buff = [0u8; 64];
            if let Ok(n) = hid.pull_raw_output(&mut buff) {
                handle_command(&buff[..n]);
            }

            // Read incoming serial data
            if let Ok(n) = serial.read(&mut buff) {
                // Loopback
                serial.write(&buff[..n]).unwrap();

                // TODO: something with this
                //...
            }
        });
    };
}

// Handle USB interrupts
#[interrupt]
fn USB() {
    poll_usb();
}

// Handle allocation errors
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

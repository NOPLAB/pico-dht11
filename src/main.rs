#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use pac::interrupt;
use rp_pico::entry;

use panic_halt as _;
// use panic_probe as _;

use rp_pico::hal;
use rp_pico::hal::pac;

use rp_pico::Pins;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use hal::{clocks::Clock, sio::Sio};

use dht_sensor::*;

// Used to demonstrate writing formatted strings
use core::fmt::Write;
use heapless::String;

static mut USB_DEV: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

fn wait_until_timeout<E, F>(
    delay: &mut dyn Delay,
    func: F,
    timeout_us: u8,
) -> Result<(), DhtError<E>>
where
    F: Fn() -> Result<bool, E>,
{
    for _ in 0..timeout_us {
        if func()? {
            return Ok(());
        }
        delay.delay_us(1_u8);
    }
    Err(DhtError::Timeout)
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    let core = pac::CorePeripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe { USB_BUS = Some(usb_bus) };

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let usb_serial = SerialPort::new(bus_ref);

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    unsafe { USB_SERIAL = Some(usb_serial) }

    unsafe { USB_DEV = Some(usb_dev) }

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut dht11_pin = hal::gpio::InOutPin::new(pins.gpio28);
    let _ = dht11_pin.set_high();

    // watchdog.start(1.secs());

    delay.delay_ms(1000);

    loop {
        // todo ここでTimeoutエラーが出る
        match dht11::Reading::read(&mut delay, &mut dht11_pin) {
            Ok(measurement) => {
                let mut text: String<128> = String::new();
                writeln!(
                    &mut text,
                    "Temp: {} Hum: {}\r\n",
                    measurement.temperature, measurement.relative_humidity
                )
                .unwrap();
                if let Some(serial_ref) = unsafe { USB_SERIAL.as_mut() } {
                    let _ = serial_ref.write(text.as_bytes());
                }
            }
            Err(err) => {
                let mut text: String<128> = String::new();
                writeln!(&mut text, "Error: {:?}\r\n", err).unwrap();
                if let Some(serial_ref) = unsafe { USB_SERIAL.as_mut() } {
                    let _ = serial_ref.write(text.as_bytes());
                }
            }
        }
        delay.delay_ms(500);
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    if let Some(dev_ref) = unsafe { USB_DEV.as_mut() } {
        if let Some(serial_ref) = unsafe { USB_SERIAL.as_mut() } {
            let _ = dev_ref.poll(&mut [serial_ref]);
        }
    }
}

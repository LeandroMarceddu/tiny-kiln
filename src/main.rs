//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use bsp::entry;
use cortex_m::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_spi_FullDuplex};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
//use max31855::*;
use bit_field::BitField;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use core::ops::RangeInclusive;
use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;
use rp2040_pac::*;
use rp_pico as bsp;

const THERMOCOUPLE_BITS: RangeInclusive<usize> = 2..=15;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio16.into_push_pull_output();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        cs_pin.set_low().unwrap();
        let mut buf: [u8; 2] = [0, 0];
        delay.delay_ms(10);
        spi.transfer(&mut buf).unwrap();
        info!("tempie {}", buf);
        cs_pin.set_high().unwrap();
        let raw = (buf[0] as u16) << 8 | (buf[1] as u16) << 0;

        let thermocouple = bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2);
        info!("term {}", convert(thermocouple));
    }
}
fn bits_to_i16(bits: u16, len: usize, divisor: i16, shift: usize) -> i16 {
    let negative = bits.get_bit(len - 1);
    if negative {
        (bits << shift) as i16 / divisor
    } else {
        bits as i16
    }
}
fn convert(count: i16) -> f32 {
    let count = count as f32;
    count * 0.25
}

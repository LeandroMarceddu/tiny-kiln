#![no_std]
#![no_main]

use bit_field::BitField;
use bsp::entry;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use core::fmt::Write;
use core::ops::RangeInclusive;
use cortex_m::prelude::{_embedded_hal_blocking_spi_Transfer};
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StrokeAlignment};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use heapless::String;

use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

use rp_pico as bsp;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const THERMOCOUPLE_BITS: RangeInclusive<usize> = 2..=15;
const SETPOINT: f32 = 1000.0;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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
    info!("Set up clocks");
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    //SPI-setup
    info!("Set up SPI");

    let mut led_pin = pins.led.into_push_pull_output();

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio13.into_push_pull_output();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI1);
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    //PWM-setup
    info!("Set up PWM");
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm7;
    pwm.default_config();
    pwm.set_ph_correct();
    pwm.enable();
    // Output channel A on PWM7 to GPIO 14
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio14);
    info!("Set up I2C");
    //set up I2C for the LCD to display temps
    let sda_pin = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();
    Text::with_baseline("TinyKiln!", Point::new(5, 5), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(3)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    display
        .bounding_box()
        .into_styled(border_stroke)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    info!("Setting up check-loops");
    // Check-loops
    let switch = pins.gpio17.into_pull_up_input();
    //door switch in series with ^ switch via relay
    loop {
        cs_pin.set_low().unwrap();
        let mut buf: [u8; 2] = [0, 0];
        delay.delay_ms(10);
        spi.transfer(&mut buf).unwrap();
        cs_pin.set_high().unwrap();
        let raw = (buf[0] as u16) << 8 | (buf[1] as u16);
        let thermocouple = convert(bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2));
        info!("temp {}", thermocouple);
        let mut s: String<16> = String::new();
        core::write!(s, "Temperatuur:\n{}", thermocouple).unwrap();
        display.clear();
        display
            .bounding_box()
            .into_styled(border_stroke)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&s, Point::new(5, 5), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        if switch.is_high().unwrap() {
            info!("Switch NOK");
            delay.delay_ms(200);
        } else {
            info!("Switch OK");
            led_pin.set_high().unwrap();
            delay.delay_ms(200);
            led_pin.set_low().unwrap();
            delay.delay_ms(200);
            if thermocouple <= SETPOINT - 20.0 {
                channel.set_duty(65535);
                info!("full power");
            }
            if (SETPOINT - 20.0..SETPOINT).contains(&thermocouple) {
                channel.set_duty(32767);
                info!("half power");
            }
            if thermocouple >= SETPOINT {
                channel.set_duty(0);
                info!("no power");
            }
        }
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

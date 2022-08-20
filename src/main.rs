#![no_std]
#![no_main]

use bit_field::BitField;
use bsp::entry;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use core::ops::RangeInclusive;
use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

use rp_pico as bsp;

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    //SPI-setup
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
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm7;
    pwm.default_config();
    pwm.set_ph_correct();
    pwm.enable();
    // Output channel A on PWM7 to GPIO 14
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio14);

    // Check-loops
    let switch = pins.gpio17.into_pull_up_input();
    //door switch in series with ^ switch via relay
    loop {
        if switch.is_high().unwrap() {
            info!("Switch NOK");
            delay.delay_ms(100);
        } else {
            info!("Switch OK");
            led_pin.set_high().unwrap();
            delay.delay_ms(500);
            led_pin.set_low().unwrap();
            delay.delay_ms(500);

            cs_pin.set_low().unwrap();
            let mut buf: [u8; 2] = [0, 0];
            delay.delay_ms(10);
            spi.transfer(&mut buf).unwrap();
            cs_pin.set_high().unwrap();
            let raw = (buf[0] as u16) << 8 | (buf[1] as u16);
            let thermocouple = convert(bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2));
            info!("temp {}", thermocouple);

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

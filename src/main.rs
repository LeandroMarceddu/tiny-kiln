#![no_std]
#![no_main]

use bit_field::BitField;
use bsp::entry;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use core::fmt::Write;
use core::ops::RangeInclusive;
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_spi_Transfer};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use heapless::String;

use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

use rp_pico as bsp;

const THERMOCOUPLE_BITS: RangeInclusive<usize> = 2..=15;
const SETPOINT: f32 = 1000.0;
const LCD_ADDRESS: u8 = 0x3F;
const EN: u8 = 0x04;
const BACKLIGHT: u8 = 0x08;

type Pins = hal::I2C<
    rp2040_pac::I2C1,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::Function<hal::gpio::I2C>>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio7, hal::gpio::Function<hal::gpio::I2C>>,
    ),
>;
fn write4bits(i2c: &mut Pins, data: u8) {
    i2c.write(LCD_ADDRESS, &[data | EN | BACKLIGHT]).unwrap();
    delay_ms2(1);
    i2c.write(LCD_ADDRESS, &[BACKLIGHT]).unwrap();
    delay_ms2(5);
}
fn delay_ms2(ms: u32) {
    let mut i = ms * 7_200;
    while i > 0 {
        i -= 1;
    }
}
fn send(i2c: &mut Pins, data: u8, mode: u8) {
    let high_bits: u8 = data & 0xf0;
    let low_bits: u8 = (data << 4) & 0xf0;
    write4bits(i2c, high_bits | mode);
    write4bits(i2c, low_bits | mode);
}

fn write(i2c: &mut Pins, data: u8) {
    send(i2c, data, 0x01);
}

fn command(i2c: &mut Pins, data: u8) {
    send(i2c, data, 0x00);
}
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

    //set up I2C for the LCD to display temps
    let sda_pin = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    write4bits(&mut i2c, 0x03 << 4);
    delay.delay_ms(5);
    write4bits(&mut i2c, 0x03 << 4);
    delay.delay_ms(5);
    write4bits(&mut i2c, 0x03 << 4);
    delay.delay_ms(5);
    write4bits(&mut i2c, 0x02 << 4);
    command(
        &mut i2c,
        0x20_u8 | // 5x8 display
        0x08_u8, // Two line display
    );

    command(
        &mut i2c,
        0x08_u8 | // Display control command
        0x04_u8 | // Display on
        0x02_u8 | // Cursor on
        0x01_u8, // Blink on
    );

    command(
        &mut i2c, 0x01_u8, // Clear display
    );

    command(
        &mut i2c,
        0x04_u8 | // Entry mode command
        0x02_u8, // Entry right
    );

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

        command(
            &mut i2c, 0x01_u8, // Clear display
        );
        let mut s: String<16> = String::new();
        core::write!(s, "Temp: {}", thermocouple).unwrap();
        for c in s.chars() {
            write(&mut i2c, c as u8);
        }

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

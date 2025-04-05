
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::{
    ptr::write_bytes,
    sync::atomic::{AtomicBool, Ordering},
};


use atmega_hal::port::mode::{Floating, Input, Output};
use atmega_hal::port::{self, Pin, Dynamic};
use atmega_hal::usart::{Baudrate, Usart};
use atmega_hal::prelude::_embedded_hal_serial_Read;

use heapless::Vec;


use embedded_hal::{delay::DelayNs, digital::InputPin};

use bin_packets::{data::PinState, ApplicationPacket, CommandPacket};
use bincode::{
    config::standard, decode_from_slice, encode_into_slice, error::{DecodeError, EncodeError}
};


use i2c_slave::*;
use panic_halt as _;
use ufmt::{uwrite, uwriteln};

type CoreClock = atmega_hal::clock::MHz16;
type Delay = atmega_hal::delay::Delay<crate::CoreClock>;

mod i2c_slave;

static TWI_INT_FLAG: AtomicBool = AtomicBool::new(false);

fn delay_ms(ms: u16) {
    Delay::new().delay_ms(u32::from(ms))
}

#[allow(dead_code)]
fn delay_us(us: u32) {
    Delay::new().delay_us(us)
}

// I2C interrupt handler
#[avr_device::interrupt(atmega2560)]
fn TWI() {
    avr_device::interrupt::free(|_| {
        TWI_INT_FLAG.store(true, Ordering::SeqCst);
    });
}

trait Read {
    fn new(pin_set: &[Pin<Input<Floating>, Dynamic>; 7]) -> Self ;
}

impl Read for PinState {
    fn new(pin_set: &[Pin<Input<Floating>, Dynamic>; 7]) -> Self {
        //.Vec<bool, 7>
        let state: Vec<bool, 7> = pin_set.iter().map(|pin| pin.is_high()).collect();
        //let mut vec: Vec<bool, 7> = Vec::new();
        //let state = pin_set.iter().map(|pin| pin.is_high()).collect_into(vec);
        PinState {
            gse_1: state[0],
            gse_2: state[1],
            te_ra: state[2],
            te_rb: state[3],
            te_1: state[4],
            te_2: state[5],
            te_3: state[6],
        }
    }
}

#[avr_device::entry]
fn main() -> ! {
    let dp = atmega_hal::Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);

    let pin_set = &[
        pins.pa0.into_floating_input().downgrade(),
        pins.pa1.into_floating_input().downgrade(),
        pins.pa2.into_floating_input().downgrade(),
        pins.pa3.into_floating_input().downgrade(),
        pins.pa4.into_floating_input().downgrade(),
        pins.pa5.into_floating_input().downgrade(),
        pins.pa6.into_floating_input().downgrade(),
    ];

    let mut serial = Usart::new(
        dp.USART0,
        pins.pe0,
        pins.pe1.into_output(),
        Baudrate::<crate::CoreClock>::new(57600),
    );
    //let mut serial = atmega_hal::default_serial!(dp, pins, 9600);

    let mut led = pins.pb7.into_output();

    // Using external pullup resistors, so pins configured as floating inputs
    let sda = pins.pd1.into_floating_input();//.into_floating_input();
    let scl = pins.pd0.into_floating_input();//into_floating_input();

    

    //-----------------------Test 1--------------------------------- 
    // loop {
    //     if sda.is_low() {
    //         uwrite!(&mut serial, "sda Low\n ").unwrap();
    //     }
    //     if sda.is_high() {
    //         uwrite!(&mut serial, "sda High\n ").unwrap();
    //     }
    //     if scl.is_low() {
    //         uwrite!(&mut serial, "scl Low\n ").unwrap();
    //     }
    //     if scl.is_high() {
    //         uwrite!(&mut serial, "scl High\n ").unwrap();
    //     }
    //     uwrite!(&mut serial, "-----------------------------\n ").unwrap();
    //     delay_ms(1000);
    // }
    //-----------------------Test 1--------------------------------- 

    
    let slave_address: u8 = 0x26;

    let mut i2c_slave: I2cSlave = I2cSlave::new(dp.TWI, slave_address, sda, scl, &TWI_INT_FLAG);

    // Enable global interrupt
    unsafe { avr_device::interrupt::enable() };


    // Value recieved from I2C Master
    // let mut buf: [u8; 4];

    ufmt::uwriteln!(&mut serial, "Slave Begin:").unwrap();

    ufmt::uwriteln!(&mut serial, "Initialized with addr: 0x{:X}", slave_address).unwrap();

    led.set_low();

    //let mut read_buf: Vec<u8, 500> = Vec::new();


    loop {
        let mut read_buf: [u8; 20] = [0u8; 20];

        i2c_slave.init(false);

        // RECEIVE
        match i2c_slave.receive(&mut read_buf) {
            Ok(_) => {
                uwrite!(&mut serial, "Received: ").unwrap();

                read_buf.iter().for_each(|b| {
                    uwrite!(&mut serial, "{} ", *b).unwrap();
                });
                uwrite!(&mut serial, "\n").unwrap();
            }
            Err(err) => {
                uwriteln!(&mut serial, "Error: {:?}", err).unwrap();
            }
        };

        let request: Result<(
            ApplicationPacket, usize), bincode::error::DecodeError>  = decode_from_slice(&read_buf, standard());

        let mut request_success: bool = false;

        match request {
            Ok((app_packet, len)) => {
                match app_packet {
                    ApplicationPacket::Command(CommandPacket::Ping) => {

                            ufmt::uwriteln!(
                                serial,
                                "App Packet Ping found",
                            ).unwrap();
                            request_success = true;
                        }

                        _ => {ufmt::uwriteln!(
                            serial,
                            "App Packet found, but no Ping recieved",
                        ).unwrap();
                    }
                }

            }

            Err(e) => {
                    ufmt::uwriteln!(
                    serial,
                    "Decoding Error"
                ).unwrap();
            }
        }

        if request_success {                
            let pin_state = PinState::new(pin_set);
            let mut write_buf: [u8; 20] = [0u8; 20]; // This probably doesn't have to be this long
            let len_encoded = encode_into_slice(pin_state, &mut write_buf, standard()).unwrap();
            // Send current pinstate to pi

            // for byte in write_buf {
            //     ufmt::uwriteln!(
            //         serial,
            //         "{}"
            //         byte as char
            //     ).unwrap();
            // }

            match i2c_slave.respond(&write_buf) {
                Ok(bytes_sent) => ufmt::uwriteln!(
                            serial,
                            "{} bytes sent"
                            bytes_sent
                        ).unwrap(),
                        
                Err(err) => uwriteln!(&mut serial, "Error: {:?}", err).unwrap(),
            }

        }

        //read_buf.clear();
        delay_ms(1000);
    }
}

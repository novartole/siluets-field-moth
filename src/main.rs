//! Field moth
//!
//! Input: no
//! Output: d5 -> MOSFET (PWM via Timer0)
//! Interrupt: once per _4_ seconds (Timer1)

/*
* Timer0: 5,6
* Timer1: 9,10
* Timer2: 3,11
*
* PWM pins: 3, 5, 6, 9, 10, 11
*
* Port A: a0..a5
* Port B: d8..d13
* Port C: d0..d7
*/

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::sync::atomic::{AtomicBool, Ordering};

use arduino_hal::{
    clock::Clock, 
    DefaultClock, 
    delay_ms, 
    Peripherals,
    simple_pwm::*
};
use avr_device::{atmega328p::TC1, interrupt};
use panic_halt as _;

// Each _one_ ties to _four_ seconds, e.g. 15 gives 1 min (4 * 15 = 60).
// Let's wait 5 mins between shows.
const COUNTER_MAX: u8 = 15 * 5; 

static mut COUNTER: u8 = 0;
static SHOW_MUST_GO_ON: AtomicBool = AtomicBool::new(true);

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // TC0 controls PWM of d5 pin. 
    // Arduino will be connected to MOSFET chip via d5 pin.
    //
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale1024);
    let mut mosfet = pins.d5.into_output().into_pwm(&timer0);
    mosfet.enable();

    rig_tc1(&dp.TC1);

    // Enable interrupts globally.
    // SAFETY: we are not inside a critical section
    unsafe { interrupt::enable() };

    loop {
        if SHOW_MUST_GO_ON.load(Ordering::Acquire) {
            SHOW_MUST_GO_ON.store(false, Ordering::Release);

            let pattern = (0..=254).rev().step_by(2).chain((0..=255).step_by(32));

            for _ in 0..2 {
                for duty in pattern.clone() {
                    mosfet.set_duty(duty);
                    delay_ms(10);
                }
                // wait 100 mls in total between two waves
                delay_ms(90);
            }

            for duty in pattern {
                mosfet.set_duty(duty);
                delay_ms(10);
            }
        }
    }
}

#[interrupt(atmega328p)]
fn TIMER1_COMPA() {
    // SAFETY: access to COUNTER happens only inside the current function.
    unsafe {
        COUNTER += 1;
        if COUNTER != COUNTER_MAX {
            return;
        }
        COUNTER = 0;
    }

    SHOW_MUST_GO_ON.store(true, Ordering::SeqCst);
}

/// Setup Timer1 to triggers an interrupt every _4_ second. 
/// Once it has counted up enough, start from beginning.
fn rig_tc1(tc1: &TC1) {
    tc1.tccr1a.write(|w| w
        // mode 4 (CTC): set WGM11 and WGM10 bits
        .wgm1().bits(0b00)
    );
    tc1.tccr1b.write(|w| w
        .cs1().prescale_1024()
        // mode 4 (CTC): set WGM13 and WGM12 bits
        .wgm1().bits(0b01)
    );
    tc1.ocr1a.write(|w| {
        // a way to calculate how many tick is needed to rich target Hz:
        //
        //     clock_hz / _target_hz_ / prescale - 1
        //
        let ticks = (4 * DefaultClock::FREQ / 1 / 1024 - 1) as u16;
        // set ticks value according to mode 4 (CTC)
        w.bits(ticks)
    });
    tc1.timsk1.write(|w| 
        // enable Compare Match Interrupt
        w.ocie1a().set_bit()
    );
}

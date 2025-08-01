#![no_std]
#![no_main]

mod executor;
mod socs;
mod time;
mod util;

use core::panic::PanicInfo;

use cortex_m::asm::{udf, wfe};
use nrf52840_pac::{CLOCK, EGU0, GPIOTE, PPI, Peripherals};

#[cfg(feature = "rtos-trace")]
use crate::trace::TASK_MAIN;
use crate::{
    executor::executor,
    socs::nrf::NrfRadioTimer,
    time::{RadioTimerApi, SyntonizedDuration},
};

const PIN_APP: (usize, bool, u8) = (0, false, 27);
const PIN_TIMER: (usize, bool, u8) = (1, false, 26);

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    {
        trace::instrument();
        rtos_trace::trace::task_exec_begin(TASK_MAIN);
    }

    let peripherals = Peripherals::take().unwrap();
    config_clock(&peripherals.CLOCK);
    config_gpiote(&peripherals.GPIOTE);
    config_ppi(&peripherals.PPI, &peripherals.GPIOTE, &peripherals.EGU0);

    let executor = executor(peripherals.EGU0);
    NrfRadioTimer::init(peripherals.RTC0);

    let mut count = 0;
    let gpiote = &peripherals.GPIOTE;

    executor.block_on(async {
        let mut timeout = NrfRadioTimer::now();
        toggle_app_pin(gpiote);
        loop {
            const DELAY: SyntonizedDuration = SyntonizedDuration::micros(31 * 3);
            timeout += DELAY;
            NrfRadioTimer::wait_until(timeout).await;
            toggle_app_pin(gpiote);
            count += 1;
            if count % 10 == 0 {
                break;
            }
        }
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    loop {
        wfe();
    }
}

fn config_clock(clock: &CLOCK) {
    clock.events_hfclkstarted.reset();
    clock
        .tasks_hfclkstart
        .write(|w| w.tasks_hfclkstart().set_bit());
    clock.events_lfclkstarted.reset();
    clock.lfclksrc.write(move |w| w.src().xtal());
    clock
        .tasks_lfclkstart
        .write(|w| w.tasks_lfclkstart().set_bit());
    // TODO: make async
    while clock
        .events_hfclkstarted
        .read()
        .events_hfclkstarted()
        .bit_is_clear()
        || clock
            .events_lfclkstarted
            .read()
            .events_lfclkstarted()
            .bit_is_clear()
    {}
    clock.events_hfclkstarted.reset();
    clock.events_lfclkstarted.reset();
}

fn config_gpiote(gpiote: &GPIOTE) {
    gpiote.config[PIN_APP.0].write(|w| {
        w.mode().task();
        w.port().bit(PIN_APP.1);
        w.psel().variant(PIN_APP.2);
        w.polarity().toggle()
    });
    gpiote.config[PIN_TIMER.0].write(|w| {
        w.mode().task();
        w.port().bit(PIN_TIMER.1);
        w.psel().variant(PIN_TIMER.2);
        w.polarity().toggle()
    });
}

fn config_ppi(ppi: &PPI, gpiote: &GPIOTE, egu: &EGU0) {
    ppi.ch[0]
        .eep
        .write(|w| w.eep().variant(egu.events_triggered[0].as_ptr() as u32));
    ppi.ch[0].tep.write(|w| {
        w.tep()
            .variant(gpiote.tasks_out[PIN_TIMER.0].as_ptr() as u32)
    });
    ppi.chenset.write(|w| w.ch0().set_bit());
}

fn toggle_app_pin(gpiote: &GPIOTE) {
    gpiote.tasks_out[PIN_APP.0].write(|w| w.tasks_out().set_bit());
}

#[panic_handler]
fn panic_handler(_: &PanicInfo) -> ! {
    udf();
}

#[cfg(feature = "rtos-trace")]
mod trace {
    #![allow(non_camel_case_types, non_snake_case)]
    use core::ptr::null_mut;

    use systemview_target::SystemView;

    rtos_trace::global_trace! {SystemView}

    struct Application;
    rtos_trace::global_application_callbacks! {Application}
    impl rtos_trace::RtosTraceApplicationCallbacks for Application {
        fn system_description() {
            systemview_target::send_system_desc_app_name!("interrupt-executor");
            systemview_target::send_system_desc_interrupt!(27, "RTC0");
            unsafe {
                SEGGER_SYSVIEW_RegisterModule(&raw mut TIMER_MODULE);
            }
        }

        fn sysclock() -> u32 {
            64_000_000
        }
    }

    pub type SEGGER_SYSVIEW_MODULE = SEGGER_SYSVIEW_MODULE_STRUCT;
    #[repr(C)]
    #[derive(Debug, Copy, Clone)]
    pub struct SEGGER_SYSVIEW_MODULE_STRUCT {
        pub sModule: *const cty::c_char,
        pub NumEvents: cty::c_ulong,
        pub EventOffset: cty::c_ulong,
        pub pfSendModuleDesc: Option<unsafe extern "C" fn()>,
        pub pNext: *mut SEGGER_SYSVIEW_MODULE,
    }

    static TIMER_MODULE_DESC: &str = "M=timer, 0 Schedule ns=%u ticks=%u\0";
    static mut TIMER_MODULE: SEGGER_SYSVIEW_MODULE = SEGGER_SYSVIEW_MODULE {
        sModule: TIMER_MODULE_DESC.as_ptr(),
        NumEvents: 1,
        EventOffset: 0,
        pfSendModuleDesc: None,
        pNext: null_mut(),
    };

    unsafe extern "C" {
        pub fn SEGGER_SYSVIEW_RegisterModule(pModule: *mut SEGGER_SYSVIEW_MODULE);
    }

    // Tasks
    pub const TASK_MAIN: u32 = 1000;
    pub const TASK_SWI: u32 = 2000;

    // Events
    pub const EVENT_SCHEDULE: u32 = 0;

    pub fn instrument() {
        static SYSTEMVIEW: SystemView = SystemView::new();
        SYSTEMVIEW.init();
        rtos_trace::trace::start();
        rtos_trace::trace::task_new_stackless(TASK_MAIN, "main\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_SWI, "SWI\0", 0);
    }

    unsafe extern "C" {
        pub fn SEGGER_SYSVIEW_RecordU32x2(
            EventId: cty::c_uint,
            Para0: cty::c_ulong,
            Para1: cty::c_ulong,
        );
    }

    pub fn record_schedule(ns: u32, ticks: u32) {
        unsafe {
            SEGGER_SYSVIEW_RecordU32x2(EVENT_SCHEDULE + TIMER_MODULE.EventOffset, ns, ticks);
        }
    }
}

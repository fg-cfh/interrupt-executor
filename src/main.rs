#![no_std]
#![no_main]

mod executor;
mod socs;
mod time;
mod util;

use core::panic::PanicInfo;

use cortex_m::asm::{udf, wfe};
use nrf52840_pac::{CLOCK, CorePeripherals, EGU0, GPIOTE, NVMC, PPI, Peripherals, RTC0, SCB, UICR};

#[cfg(feature = "rtos-trace")]
use crate::trace::TASK_MAIN;
use crate::{
    executor::executor,
    socs::nrf::NrfRadioTimer,
    time::{HardwareSignal, Pin, RadioTimerApi, RadioTimerResult, SyntonizedDuration},
};

#[allow(dead_code)]
enum GpioPort {
    P0,
    P1,
}

enum GpioteChannel {
    Alarm,
    Egu,
    Tick,
}

struct GpioteConfig {
    gpiote_channel: GpioteChannel,
    port: GpioPort,
    pin: u8,
}

impl GpioteConfig {
    const fn new(gpiote_channel: GpioteChannel, port: GpioPort, pin: u8) -> Self {
        Self {
            gpiote_channel,
            port,
            pin,
        }
    }
}

#[allow(clippy::enum_variant_names)]
enum PpiChannel {
    TimerGpiote,
    EguGpiote,
    RtcTickGpiote,
}

const PIN_ALARM: GpioteConfig = GpioteConfig::new(GpioteChannel::Alarm, GpioPort::P0, 27);
const PIN_EGU: GpioteConfig = GpioteConfig::new(GpioteChannel::Egu, GpioPort::P0, 26);
const PIN_TICK: GpioteConfig = GpioteConfig::new(GpioteChannel::Tick, GpioPort::P0, 2);

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    {
        trace::instrument();
        rtos_trace::trace::task_exec_begin(TASK_MAIN);
    }

    let core_peripherals = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();

    config_reset(&peripherals.UICR, &peripherals.NVMC, &core_peripherals.SCB);

    config_clock(&peripherals.CLOCK);
    config_gpiote(&peripherals.GPIOTE, PIN_ALARM);
    config_gpiote(&peripherals.GPIOTE, PIN_EGU);
    config_gpiote(&peripherals.GPIOTE, PIN_TICK);
    config_egu_ppi(
        &peripherals.PPI,
        &peripherals.GPIOTE,
        PIN_EGU.gpiote_channel as usize,
        &peripherals.EGU0,
        PpiChannel::EguGpiote as usize,
    );
    config_tick_ppi(
        &peripherals.PPI,
        &peripherals.GPIOTE,
        PIN_TICK.gpiote_channel as usize,
        &peripherals.RTC0,
        PpiChannel::RtcTickGpiote as usize,
    );

    NrfRadioTimer::init(
        peripherals.RTC0,
        peripherals.TIMER0,
        &peripherals.GPIOTE,
        PIN_ALARM.gpiote_channel as usize,
        &peripherals.PPI,
        PpiChannel::TimerGpiote as usize,
    );

    // let mut count = 0;
    // let gpiote = &peripherals.GPIOTE;

    let executor = executor(peripherals.EGU0);
    executor.block_on(async {
        let mut timeout = NrfRadioTimer::now();
        // toggle_gpiote_pin(gpiote, PIN_ALARM.gpiote_channel as usize);
        loop {
            const DELAY: SyntonizedDuration = SyntonizedDuration::nanos(4 * 30518);
            timeout += DELAY;
            // Safety: We run at lower priority than the timer interrupt and we
            //         run from a single task.
            let result = unsafe {
                NrfRadioTimer::schedule_event(timeout, HardwareSignal::TogglePin(Pin::Pin0)).await
            };
            // toggle_gpiote_pin(gpiote, PIN_ALARM.gpiote_channel as usize);
            assert!(matches!(result, RadioTimerResult::Ok));
            /* count += 1;
            if count % 10 == 0 {
                break;
            } */
        }
    });

    // # MAC service (running on SWI5 executor)
    //
    // Race for:
    // - a request to schedule a frame
    // - timeout of queue head timer (or "never", if the queue is empty)
    // On frame:
    // - Find an adequate slot for the frame.
    // - Calculate/check the timing of the frame.
    // - Push the frame into the queue.
    // - Calculate the timer for head with sufficient guard time (max(schedule frame, schedule event).
    // On timeout:
    // - Pop head from the queue.
    // - Send head to the driver service.

    // # Driver service (running on SWI1 executor)
    //
    // - Wait for request to schedule an event.
    // - Schedule event to timer.

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    loop {
        wfe();
    }
}

fn config_reset(uicr: &UICR, nvmc: &NVMC, scb: &SCB) {
    if uicr.pselreset[0].read().connect().is_connected() {
        // UICR is already configured.
        return;
    }

    // The UICR registers in flash are pristine or were erased. We need to
    // re-configure them. No need to erase the register to satisfy n_write
    // requirements: It just seems to have been erased by someone else.

    nvmc.config.write(|w| w.wen().wen());
    // Both pselreset configs must be the same for the configuration to take
    // effect.
    for reg in 0..=1 {
        uicr.pselreset[reg].write(|w| {
            // Use the DK's default reset pin P0.18.
            w.port().clear_bit();
            w.pin().variant(18);
            w.connect().connected()
        });
        while nvmc.ready.read().ready().bit_is_clear() {}
    }
    nvmc.config.reset();

    // UICR changes only take effect after a reset.
    soft_reset(scb);
}

fn soft_reset(scb: &SCB) {
    const AIRCR_VECTKEY_MASK: u32 = 0x05FA << 16;
    const SYSRESETREQ: u32 = 1 << 2;
    unsafe { scb.aircr.write(AIRCR_VECTKEY_MASK | SYSRESETREQ) };
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

fn config_gpiote(gpiote: &GPIOTE, config: GpioteConfig) {
    gpiote.config[config.gpiote_channel as usize].write(|w| {
        w.mode().task();
        w.port().bit(matches!(config.port, GpioPort::P1));
        w.psel().variant(config.pin);
        w.polarity().toggle()
    });
}

fn config_egu_ppi(
    ppi: &PPI,
    gpiote: &GPIOTE,
    gpiote_channel: usize,
    egu: &EGU0,
    ppi_egu_gpiote: usize,
) {
    debug_assert!(ppi_egu_gpiote <= 19);
    ppi.ch[ppi_egu_gpiote]
        .eep
        .write(|w| w.eep().variant(egu.events_triggered[0].as_ptr() as u32));
    ppi.ch[ppi_egu_gpiote].tep.write(|w| {
        w.tep()
            .variant(gpiote.tasks_out[gpiote_channel].as_ptr() as u32)
    });
    // Safety: We checked the PPI channel range.
    ppi.chenset
        .write(|w| unsafe { w.bits(1 << ppi_egu_gpiote) });
}

fn config_tick_ppi(
    ppi: &PPI,
    gpiote: &GPIOTE,
    gpiote_channel: usize,
    rtc: &RTC0,
    ppi_rtc_tick_gpiote: usize,
) {
    debug_assert!(ppi_rtc_tick_gpiote <= 19);
    ppi.ch[ppi_rtc_tick_gpiote]
        .eep
        .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
    ppi.ch[ppi_rtc_tick_gpiote].tep.write(|w| {
        w.tep()
            .variant(gpiote.tasks_out[gpiote_channel].as_ptr() as u32)
    });
    // Safety: We checked the PPI channel range.
    ppi.chenset
        .write(|w| unsafe { w.bits(1 << ppi_rtc_tick_gpiote) });
}

/* fn toggle_gpiote_pin(gpiote: &GPIOTE, gpiote_channel: usize) {
    gpiote.tasks_out[gpiote_channel].write(|w| w.tasks_out().set_bit());
} */

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
            systemview_target::send_system_desc_interrupt!(24, "TIMER0");
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

    static TIMER_MODULE_DESC: &str = "M=timer, \
        0 SchedAlarm ns=%u rt=%u, \
        1 SchedSignal ns=%u rt=%u tt=%u\0";
    static mut TIMER_MODULE: SEGGER_SYSVIEW_MODULE = SEGGER_SYSVIEW_MODULE {
        sModule: TIMER_MODULE_DESC.as_ptr(),
        NumEvents: 2,
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
    pub const EVENT_SCHEDULE_ALARM: u32 = 0;
    pub const EVENT_SCHEDULE_SIGNAL: u32 = 1;

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

        pub fn SEGGER_SYSVIEW_RecordU32x3(
            EventId: cty::c_uint,
            Para0: cty::c_ulong,
            Para1: cty::c_ulong,
            Para2: cty::c_ulong,
        );
    }

    pub fn record_schedule_alarm(ns: u32, rtc_ticks: u32) {
        unsafe {
            SEGGER_SYSVIEW_RecordU32x2(
                EVENT_SCHEDULE_ALARM + TIMER_MODULE.EventOffset,
                ns,
                rtc_ticks,
            );
        }
    }

    pub fn record_schedule_signal(ns: u32, rtc_ticks: u32, remaining_timer_ticks: u32) {
        unsafe {
            SEGGER_SYSVIEW_RecordU32x3(
                EVENT_SCHEDULE_SIGNAL + TIMER_MODULE.EventOffset,
                ns,
                rtc_ticks,
                remaining_timer_ticks,
            );
        }
    }
}

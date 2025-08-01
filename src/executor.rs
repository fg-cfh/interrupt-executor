#![allow(static_mut_refs)]

use core::{
    pin::{Pin, pin},
    ptr::{self, null_mut},
    sync::atomic::compiler_fence,
    task::{Context, RawWaker, RawWakerVTable, Waker},
};

use cortex_m::asm::wfe;
use nrf52840_pac::{EGU0, NVIC, Peripherals, interrupt};
use portable_atomic::{AtomicPtr, Ordering};
use static_cell::StaticCell;

#[cfg(feature = "rtos-trace")]
use crate::trace::TASK_SWI;

static WAKER: Waker = unsafe { Waker::from_raw(raw_waker()) };
static TASK: AtomicPtr<Pin<&mut dyn Future<Output = ()>>> = AtomicPtr::new(null_mut());
static EXECUTOR: StaticCell<InterruptExecutor> = StaticCell::new();

/// Safety: Transferring ownership of the EGU peripheral proves that only a
///         single instance of the executor can be requested.
pub fn executor(egu: EGU0) -> &'static mut InterruptExecutor {
    EXECUTOR.init(InterruptExecutor).init(egu)
}

pub struct InterruptExecutor;

impl InterruptExecutor {
    /// This method unmasks the interrupt and therefore may break concurrent
    /// critical sections. It must be called in early initialization code before
    /// concurrent critical sections might be active.
    ///
    /// Using the executor w/o calling init() will cause undefined behavior.
    fn init(&'static mut self, egu: EGU0) -> &'static mut Self {
        NVIC::unpend(interrupt::SWI0_EGU0);
        // Safety: See method doc. There should be no concurrent critical sections.
        unsafe { NVIC::unmask(interrupt::SWI0_EGU0) };
        egu.intenset.write(|w| w.triggered0().set_bit());
        self
    }

    fn egu() -> EGU0 {
        // Safety: We own the peripheral exclusively.
        unsafe { Peripherals::steal() }.EGU0
    }

    /// Associates a task with the executor and drives it to completion.
    ///
    /// Requiring a mutable reference ensures that only a single task can be
    /// scheduled at any time.
    pub fn block_on<Task: Future<Output = ()>>(&mut self, task: Task) {
        debug_assert!(NVIC::is_enabled(interrupt::SWI0_EGU0), "not initialized");

        let mut pinned_task: Pin<&mut dyn Future<Output = ()>> = pin!(task);
        compiler_fence(Ordering::Release);
        TASK.store(ptr::from_mut(&mut pinned_task).cast(), Ordering::Relaxed);

        // Initially poll once.
        Self::egu().tasks_trigger[0].write(|w| w.tasks_trigger().set_bit());

        loop {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::system_idle();
            wfe();
            if TASK.load(Ordering::Relaxed).is_null() {
                compiler_fence(Ordering::Acquire);

                // Safety: We need to extend lifetime until we're sure the task
                //         is no longer being accessed.
                #[allow(clippy::drop_non_drop)]
                drop(pinned_task);
                break;
            }
        }
    }
}

#[interrupt]
fn SWI0_EGU0() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::task_exec_begin(TASK_SWI);
    InterruptExecutor::egu().events_triggered[0].reset();
    let task = TASK.load(Ordering::Relaxed);
    compiler_fence(Ordering::Acquire);

    // Safety: We're converting from a pointer that has been generated verbatim
    //         from a valid &mut reference. Therefore the pointer will be
    //         non-null, properly aligned, dereferenceable and point to a valid
    //         pinned future. Pinning and synchronizing via the pointer ensures
    //         that pointer cannot dangle. Checking for null pointers is
    //         required to protect against spurious wake-ups.
    if let Some(task) = unsafe { task.as_mut() } {
        let mut cx = Context::from_waker(&WAKER);
        // Note: Comment in for benchmark measurements.
        unsafe { Peripherals::steal() }.GPIOTE.tasks_out[1].write(|w| w.tasks_out().set_bit());
        match task.as_mut().poll(&mut cx) {
            core::task::Poll::Ready(_) => {
                compiler_fence(Ordering::Release);
                TASK.store(null_mut(), Ordering::Relaxed);
            }
            core::task::Poll::Pending => {}
        }
    }
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::task_exec_end();
}

static VTABLE: RawWakerVTable = RawWakerVTable::new(clone_waker, wake, wake_by_ref, drop_waker);

const fn raw_waker() -> RawWaker {
    RawWaker::new(ptr::null(), &VTABLE)
}

unsafe fn clone_waker(data: *const ()) -> RawWaker {
    // Safety: We always return the same (static) vtable reference to ensure
    //         that `Waker::will_wake()` recognizes the clone.
    RawWaker::new(data, &VTABLE)
}

unsafe fn wake(_: *const ()) {
    // Safety: Triggering a task is atomic and idempotent.
    InterruptExecutor::egu().tasks_trigger[0].write(|w| w.tasks_trigger().set_bit());
}

unsafe fn wake_by_ref(_: *const ()) {
    // Safety: Triggering a task is atomic and idempotent.
    InterruptExecutor::egu().tasks_trigger[0].write(|w| w.tasks_trigger().set_bit());
}

unsafe fn drop_waker(_: *const ()) {
    // no-op
}

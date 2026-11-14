//! LanternOS hardware abstraction layer — Phase 1 prototype (RFC-0004).
//!
//! Scope of this crate today is intentionally narrow: it defines the trap-entry
//! contract that RFC-0005/ADR-0008 requires before `lantern-kernel` can implement
//! its syscall/IPC fast path. This is **not yet** the full HAL surface described
//! in `lantern-hal/ARCHITECTURE.md` — paging, timers, the interrupt controller,
//! the IOMMU, platform discovery, and the early console remain undefined here and
//! are tracked as open items in `lantern-hal/STATUS.md`.
//!
//! Per ADR-0001, `unsafe` in this crate is limited to what installing a hardware
//! trap vector inherently requires, and is isolated behind the [`Hal`] trait.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_op_in_unsafe_fn)]

mod riscv64_paging;
mod trap;

pub use riscv64_paging::{
    map as riscv64_map_page, map_megapage as riscv64_map_megapage, translate as riscv64_translate,
    unmap as riscv64_unmap, PageTable as Riscv64PageTable, PteFlags as Riscv64PteFlags,
    MEGAPAGE_SIZE as RISCV64_MEGAPAGE_SIZE, PAGE_SIZE as RISCV64_PAGE_SIZE,
    share_root_entry as riscv64_share_root_entry, for_each_table as riscv64_for_each_table,
    DIRECT_MAP_BASE as RISCV64_DIRECT_MAP_BASE, DIRECT_MAP_RAM_BASE as RISCV64_DIRECT_MAP_RAM_BASE,
    DIRECT_MAP_SPAN as RISCV64_DIRECT_MAP_SPAN, DIRECT_MAP_ROOT_INDEX as RISCV64_DIRECT_MAP_ROOT_INDEX,
};
pub use trap::{MessageTag, TrapCause, TrapFrame, TrapHandler, FLAG_ERROR, MR_COUNT, NOT_A_SYSCALL};

/// The trap/IPC contract every HAL target must implement, per ADR-0008.
///
/// The portable kernel core (`lantern-kernel`) depends only on this trait and on
/// [`TrapFrame`]'s accessor methods — never on a raw ISA register name or number,
/// per the HAL-seam discipline recorded in `lantern-kernel/ARCHITECTURE.md`.
pub trait Hal {
    /// Install the trap/exception entry point. Called once during early boot,
    /// before any user-space thread runs.
    ///
    /// On every subsequent trap, an implementation must: save the interrupted
    /// thread's full register file into a [`TrapFrame`], populate the frame's
    /// `mr0..mr3`, tag, and syscall-number fields from the architecture's fixed
    /// calling convention, invoke `handler`, then restore the (possibly
    /// modified) frame — including any tag/flags change — before returning to
    /// user space.
    ///
    /// # Safety
    /// Must be called at most once, before traps are enabled, and `handler` must
    /// remain valid for the lifetime of the system: it is invoked directly from
    /// trap context with interrupts masked. This is isolated per ADR-0001's
    /// `unsafe` review policy — installing a hardware trap vector cannot be made
    /// safe by construction.
    unsafe fn install_trap_handler(handler: TrapHandler);

    /// Builds the initial register state for a thread that has never run: `pc` is
    /// where it starts executing, `sp` its initial stack pointer, `arg0` its first
    /// argument, passed in whatever register this architecture's calling
    /// convention uses for it (riscv64: `a0`; x86-64: `rdi`).
    ///
    /// Exists because a not-yet-run thread's saved state can't come from a real
    /// trap (nothing has trapped yet) — both [`Hal::enter_thread`]'s one-time cold
    /// start and the portable kernel core's own bookkeeping for not-yet-run
    /// threads (`lantern-kernel`'s `SavedContext`) go through this, so neither ever
    /// needs to know a raw register index.
    fn initial_trap_frame(pc: usize, sp: usize, arg0: usize) -> TrapFrame;

    /// Cold-starts execution using `frame` (typically built by
    /// [`Hal::initial_trap_frame`]) — the *first* entry into a thread, as opposed
    /// to resuming one via a real trap's return path. Used exactly once per
    /// hart, to start the very first thread the kernel ever runs; every
    /// subsequent switch to any thread (including ones that have never run
    /// before) happens the normal way, by writing its state into the live
    /// [`TrapFrame`] a real trap handed the kernel and returning.
    ///
    /// # Safety
    /// `frame` must be fully and validly populated (a real, mapped entry point in
    /// its program-counter slot; a real, mapped stack in its stack-pointer slot)
    /// — this trusts it completely and transfers control into it with no further
    /// checks. Must be called at most once per hart, and only before that hart has
    /// started running any other thread.
    unsafe fn enter_thread(frame: &TrapFrame) -> !;

    /// Activates `root` (an architecture-specific physical root page-table
    /// address — `riscv64`: an `Riscv64PageTable` built via `riscv64_map_page`) as
    /// the current address space, flushing whatever needs flushing (`riscv64`:
    /// `sfence.vma`) so the switch is immediately visible. `lantern-kernel`'s
    /// context-switch code calls this whenever a thread with an address space of
    /// its own becomes current — see `state::KernelState`'s `switch_to`/
    /// `block_current`.
    ///
    /// **`x86-64` is a documented no-op**, not `unimplemented!()`: unlike
    /// `enter_thread`, this runs on every context switch, including ones
    /// `lantern-kernel`'s host (`x86_64`) unit tests exercise directly — panicking
    /// would break `cargo test` for a target that has no real caller anyway (no
    /// `x86-64` boot loader exists yet).
    ///
    /// # Safety
    /// `root` must be a valid, fully-built page table for this architecture that,
    /// at minimum, maps the code currently executing (including this function's
    /// own return address) — or execution faults immediately after activation.
    unsafe fn activate_address_space(root: usize);

    /// Nanoseconds since an arbitrary, monotonic epoch — **never** wall-clock
    /// time; no consumer needs one yet, and wall-clock time is a separate,
    /// harder problem (persistence, drift) this method doesn't take on
    /// ([RFC-0012](../../lantern-rfcs/rfcs/0012-monotonic-clock-primitive.md)/
    /// [ADR-0016](../../lantern-rfcs/adr/0016-monotonic-clock-primitive.md)).
    /// Never decreases across calls; two calls a known duration apart differ by
    /// approximately that duration. Wraps only after ~584 years at this
    /// resolution (`u64::MAX` ns) — not a practical Phase 2 concern.
    ///
    /// Read-only. The timer that interrupts on this clock is [`Hal::set_timer`]
    /// ([RFC-0027](../../lantern-rfcs/rfcs/0027-interrupt-delivery-timer-and-idle.md)); no user-space-visible
    /// clock syscall exists yet (RFC-0012's "What this does not do").
    fn monotonic_time_ns() -> u64;

    // --- Interrupts and the timer ([RFC-0027](../../lantern-rfcs/rfcs/0027-interrupt-delivery-timer-and-idle.md)).
    // Interrupts are only ever taken from U-mode: the kernel runs with them masked (ADR-0010), and its idle loop
    // waits for them without taking a trap ([`Hal::wait_for_interrupt`]).

    /// Masks interrupts in S-mode, then lets the platform controller's lines and the timer interrupt U-mode. Called
    /// once, before the first thread runs.
    ///
    /// # Safety
    /// [`Hal::install_trap_handler`] must have run, and nothing may be relying on interrupts staying off in U-mode.
    unsafe fn init_interrupts();

    /// Enables or disables one line at the platform interrupt controller. Out-of-range lines are ignored.
    fn interrupt_line_enable(line: u32, enabled: bool);

    /// Claims the highest-priority pending line, or 0 if none. A claimed line can't interrupt again until
    /// [`Hal::interrupt_complete`].
    fn interrupt_claim() -> u32;

    /// Completes a claimed line, so it can interrupt again.
    fn interrupt_complete(line: u32);

    /// Arms the timer to interrupt at `deadline_ns` on [`Hal::monotonic_time_ns`]'s clock, replacing any earlier
    /// deadline and clearing a pending timer interrupt. `u64::MAX` disarms it.
    fn set_timer(deadline_ns: u64);

    /// Whether the timer interrupt is pending.
    fn timer_pending() -> bool;

    /// Waits until an enabled interrupt is pending, **without taking it**: S-mode interrupts stay masked, and the
    /// caller then claims what's pending itself. May return early, so callers loop.
    fn wait_for_interrupt();

    /// [RFC-0030](../../lantern-rfcs/rfcs/0030-process-lifecycle-faults-and-reclamation.md): an address the kernel can
    /// write `len` bytes of RAM at `paddr` through right now: the physical address before paging is on, the direct map
    /// after. `None` if any of it isn't RAM the direct map covers, such as a device's registers.
    fn ram_window(paddr: usize, len: usize) -> Option<usize>;
}

#[cfg(target_arch = "riscv64")]
mod riscv64;
#[cfg(target_arch = "riscv64")]
pub use riscv64::Hardware;

#[cfg(target_arch = "x86_64")]
mod x86_64;
#[cfg(target_arch = "x86_64")]
pub use x86_64::Hardware;

#[cfg(not(any(target_arch = "riscv64", target_arch = "x86_64")))]
compile_error!(
    "lantern-hal: no HAL implementation for this target architecture yet \
     (riscv64 and x86-64 only, per ADR-0002)"
);

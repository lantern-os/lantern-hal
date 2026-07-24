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

mod trap;

pub use trap::{MessageTag, TrapFrame, TrapHandler, FLAG_ERROR, MR_COUNT};

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

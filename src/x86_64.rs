//! `x86-64` trap entry — a development-convenience target, not a strategic one
//! (ADR-0002).
//!
//! Per ADR-0008 this must: install an IDT/`syscall` entry that saves the full
//! register file into a [`TrapFrame`], expose `mr0..mr3` via the SysV
//! argument-register convention, the message tag via a dedicated register, and
//! the syscall number via `rax`, invoke the installed [`TrapHandler`], and
//! restore state — including any tag/flags change — on `sysret`/`iretq`.
//!
//! **Not yet implemented.** This is a Phase 1 prototype stub (RFC-0004): the
//! actual trap vector is hand-written assembly and hardware bring-up work,
//! tracked as the next item in `lantern-hal/STATUS.md`.

use crate::{Hal, TrapHandler};

/// The `x86-64` HAL implementation.
pub struct Hardware;

impl Hal for Hardware {
    unsafe fn install_trap_handler(_handler: TrapHandler) {
        unimplemented!(
            "x86-64 trap vector install — Phase 1 prototype, trap assembly not yet written"
        )
    }
}

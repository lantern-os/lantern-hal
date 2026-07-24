//! `riscv64` (RV64GC) trap entry — the strategic target (ADR-0002).
//!
//! Per ADR-0008 this must: save the full register file into a [`TrapFrame`] on
//! trap entry (`ecall`/exception), expose `mr0..mr3` via the RISC-V calling
//! convention's argument registers, the message tag via a dedicated register,
//! and the syscall number via `a7` (Linux-convention-style; final assignment to
//! be confirmed during real trap-assembly bring-up), invoke the installed
//! [`TrapHandler`], and restore state — including any tag/flags change — on
//! `sret`.
//!
//! **Not yet implemented.** This is a Phase 1 prototype stub (RFC-0004): the
//! actual trap vector is hand-written assembly and hardware bring-up work,
//! tracked as the next item in `lantern-hal/STATUS.md`.

use crate::{Hal, TrapHandler};

/// The `riscv64` HAL implementation.
pub struct Hardware;

impl Hal for Hardware {
    unsafe fn install_trap_handler(_handler: TrapHandler) {
        unimplemented!(
            "riscv64 trap vector install — Phase 1 prototype, trap assembly not yet written"
        )
    }
}

# lantern-hal — Status

**Phase:** 1 (Microkernel prototype) — open per [RFC-0004](../lantern-rfcs/rfcs/0004-phase-0-to-phase-1-transition.md); `riscv64` trap entry implemented, `x86-64` still stubbed.

## Done
- HAL abstraction surface enumerated and reviewed ([ARCHITECTURE.md](./ARCHITECTURE.md)).
- Threat model drafted and reviewed.
- Minimal HAL trait/contract defined in code (`src/`): the `Hal` trait, `TrapFrame`
  (`mr0..mr3`, message tag, syscall number, opaque raw save area), and `MessageTag`
  packing/unpacking, per [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md).
  Unit-tested (`cargo test`); no clippy warnings.
- `riscv64` trap entry/exit implemented (`src/riscv64.rs`): hand-written assembly vector
  (`stvec` Direct mode) saves the full GPR file + `sepc` through a static save area
  pointed to by `sscratch`, a Rust trampoline dispatches on `scause` (only "ecall from
  U-mode" is handled in Phase 1; anything else parks the hart), builds a `TrapFrame`,
  invokes the installed `TrapHandler`, writes back `mr0..mr3`/the tag, and advances
  `sepc` past the `ecall` before `sret`. Fixes the concrete register mapping ADR-0008
  left open: `mr0..mr3` = `a0..a3`, tag = `a4`, syscall number = `a7`. Cross-compiles
  clean (`cargo build`/`clippy --target riscv64gc-unknown-none-elf`, no warnings);
  verified by inspecting the disassembled object code against the hand-derived
  save/restore offsets. **Not yet exercised by a real `ecall` under QEMU** — that needs
  enough of `lantern-boot`/`lantern-kernel` to drive it.
  `x86-64` is unchanged: still stubbed with `unimplemented!()`.

## Next
- Implement real trap entry/exit for `x86-64` (development-convenience target): IDT/
  `syscall` entry, register save/restore.
- Exercise the `riscv64` trap entry under QEMU with an actual `ecall` once
  `lantern-boot`/`lantern-kernel` can drive it — first real hardware validation of this
  code.
- Remaining HAL surface not yet started: paging, timer, interrupt controller, IOMMU,
  platform discovery, early console (see `ARCHITECTURE.md`'s abstraction table).

## Blocked on
- Nothing currently — the kernel object/IPC ABI is fixed by
  [RFC-0005](../lantern-rfcs/rfcs/0005-syscall-ipc-abi-and-phase1-scheduling.md)/
  [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md), and the trait built on
  it now lives in code.

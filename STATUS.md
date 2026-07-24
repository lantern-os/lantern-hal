# lantern-hal — Status

**Phase:** 1 (Microkernel prototype) — open per [RFC-0004](../lantern-rfcs/rfcs/0004-phase-0-to-phase-1-transition.md); first prototype code merged (trap/IPC trait only).

## Done
- HAL abstraction surface enumerated and reviewed ([ARCHITECTURE.md](./ARCHITECTURE.md)).
- Threat model drafted and reviewed.
- Minimal HAL trait/contract defined in code (`src/`): the `Hal` trait, `TrapFrame`
  (`mr0..mr3`, message tag, syscall number, opaque raw save area), and `MessageTag`
  packing/unpacking, per [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md).
  `riscv64` and `x86-64` each have a `Hardware` type implementing `Hal`, currently stubbed
  with `unimplemented!()` — the trait shape is real, the trap assembly is not.
  Unit-tested (`cargo test`); no clippy warnings.

## Next
- Implement real trap entry/exit for `riscv64` (strategic target): hand-written trap
  vector, register save/restore, `ecall` handling under QEMU.
- Same for `x86-64` (development-convenience target).
- Remaining HAL surface not yet started: paging, timer, interrupt controller, IOMMU,
  platform discovery, early console (see `ARCHITECTURE.md`'s abstraction table).

## Blocked on
- Nothing currently — the kernel object/IPC ABI is fixed by
  [RFC-0005](../lantern-rfcs/rfcs/0005-syscall-ipc-abi-and-phase1-scheduling.md)/
  [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md), and the trait built on
  it now lives in code.

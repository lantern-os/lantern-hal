# lantern-hal — Status

**Phase:** 1 (Microkernel prototype) — open per [RFC-0004](../lantern-rfcs/rfcs/0004-phase-0-to-phase-1-transition.md); trap entry implemented for both targets.

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
- `x86-64` trap entry/exit implemented (`src/x86_64.rs`): a minimal IDT with one
  populated interrupt gate (`int 0x80`, `DPL=0`, ring0-only for now — see the module doc
  for why ring3 needs `lantern-kernel`'s not-yet-written thread/TSS support first),
  installed via `lidt`; the assembly stub pushes the GPR file onto the current stack and
  calls a Rust trampoline with the same shape/role as `riscv64`'s. Uses `int 0x80` rather
  than `syscall`/`sysret` deliberately — the latter needs per-CPU GS-base/MSR plumbing
  that doesn't exist yet; switching later is a HAL-internal change, not an ABI break.
  Fixes the concrete register mapping: `mr0..mr3` = `rdi, rsi, rdx, rcx`, tag = `r8`,
  syscall number = `rax`. Unlike `riscv64`, this crate's host target *is* `x86_64`, so
  `cargo test`/`clippy` compile this assembly directly on every normal run (no separate
  cross-target needed); verified the same way as `riscv64` by disassembling the built
  object. **Not yet exercised by a real `int 0x80` under QEMU/hardware** — same
  `lantern-boot`/`lantern-kernel` dependency as `riscv64`, and additionally: nothing
  calls `install_trap_handler` in a test, since `lidt` needs ring 0 and a hosted test
  process runs in ring 3.

## Next
- Exercise both trap entries under QEMU with a real `ecall`/`int 0x80` once
  `lantern-boot`/`lantern-kernel` can drive them — first real hardware validation of
  this code.
- `x86-64`: raise the `int 0x80` gate to `DPL=3` and handle the wider (`ss`/`rsp`-inclusive)
  hardware frame once `lantern-kernel` has real user/kernel threads and a `TSS.RSP0` to
  point at a kernel stack.
- Remaining HAL surface not yet started: paging, timer, interrupt controller, IOMMU,
  platform discovery, early console (see `ARCHITECTURE.md`'s abstraction table).

## Blocked on
- Nothing currently — the kernel object/IPC ABI is fixed by
  [RFC-0005](../lantern-rfcs/rfcs/0005-syscall-ipc-abi-and-phase1-scheduling.md)/
  [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md), and the trait built on
  it now lives in code.

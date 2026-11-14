# lantern-hal — Status

**Phase:** 1 (Microkernel prototype) — opened per [RFC-0004](../lantern-rfcs/rfcs/0004-phase-0-to-phase-1-transition.md), **closed** per [RFC-0009](../lantern-rfcs/rfcs/0009-phase-1-to-phase-2-transition.md)/[ADR-0014](../lantern-rfcs/adr/0014-phase-1-complete-phase-2-opened.md); `riscv64` trap entry implemented and validated under real QEMU; `x86-64` implemented but not yet exercised (no `x86-64` boot loader yet). This crate's own remaining "Next" items below continue as ordinary engineering work — the Roadmap's phase gate has moved on to Phase 3 (RFC-0017/ADR-0021), this crate's Phase 1 backlog hasn't.

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
  U-mode" is handled in Phase 1; anything else parks the hart; since RFC-0030 a U-mode fault is reported instead), populates a `TrapFrame`'s
  *entire* raw register file (not just `mr0..mr3`/tag), invokes the installed
  `TrapHandler`, writes the whole frame back, advancing `sepc` past the `ecall` *before*
  the handler runs (not after — see below). Fixes the concrete register mapping ADR-0008
  left open: `mr0..mr3` = `a0..a3`, tag = `a4`, syscall number = `a7`. Cross-compiles
  clean (`cargo build`/`clippy --target riscv64gc-unknown-none-elf`, no warnings).
- Also added `Hal::initial_trap_frame`/`Hal::enter_thread`: primitives for cold-starting a
  thread that has never trapped before (install_trap_handler only ever covered
  save/restore *around* a trap). Implemented for `riscv64`, reusing the trap-exit
  restore sequence; stubbed (`unimplemented!()`) for `x86-64` pending its own boot work.
- **Validated under real QEMU** (`qemu-system-riscv64 -machine virt -bios default`), via
  `lantern-boot`'s two-thread `Call`/`Recv`/`Reply` demo — see `lantern-boot/STATUS.md`.
  This is the first time any of this assembly has been exercised by an actual `ecall`,
  and it caught a real bug: the trampoline originally wrote back only `mr0..mr3`/the tag
  and advanced `sepc` *after* the handler ran, both of which silently discarded every
  context switch `lantern-kernel` performs (a switch replaces the *entire* saved state,
  `sepc` included). Fixed by writing back the full raw register file and moving the
  `sepc` advance to *before* the handler runs, so a switch's replacement value naturally
  wins instead of being clobbered afterward. No unit test could have caught this — none
  of them drive this arch-gated assembly at all, only the portable Rust around it.
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
- `riscv64` Sv39 paging (`src/riscv64_paging.rs`): portable (host-testable) page-table
  primitives — `map` (4 KiB pages, full 3-level walk), `map_megapage` (2 MiB pages, one
  branch hop), `unmap` (either size, stops at whichever level holds the leaf — added for
  [RFC-0008](../lantern-rfcs/rfcs/0008-vspace-frame-capabilities-and-elf-loader.md)'s
  `FrameInvoke::Unmap`), `translate`, `PteFlags` — plus `activate` (`csrw satp`/
  `sfence.vma`, `riscv64`-only). Added `Hal::activate_address_space` to the trait
  (documented no-op on `x86-64`, since `lantern-kernel`'s host unit tests call it on every
  context switch and there's no `x86-64` boot loader to make a real call meaningful yet).
  Not part of `Hal` itself beyond that one method. **`lantern-kernel` now does manage real
  VSpace/Frame capabilities** (RFC-0008/[ADR-0012](../lantern-rfcs/adr/0012-vspace-frame-capabilities-and-elf-loader.md)),
  calling `map`/`map_megapage`/`unmap`/`translate` from `lantern-kernel/src/frame.rs`'s
  `FrameInvoke` dispatch; `lantern-boot`'s loader still calls `map_megapage` directly, once,
  for the one thing that's deliberately *not* capability-mediated (mapping the shared
  kernel megapage into every loaded program's VSpace — `lantern-boot/loader.rs`'s
  `map_kernel_shared` doc has the full reasoning for why that one stays a direct HAL call).
  **`map_megapage` exists because of an empirically-confirmed limitation in this
  project's QEMU environment** (Debian's `qemu-system-riscv64` 10.2.1): a full 3-level
  Sv39 walk reliably page-faults on every instruction fetch immediately after
  `sfence.vma`, even though the resulting page table is independently verified
  byte-correct at every level (this crate's own `translate`, and raw physical-memory
  dumps via the QEMU monitor, both confirm it) — while a walk with only one branch hop
  (a megapage) was confirmed to work reliably in the same environment. Extensively
  debugged under real QEMU (`-d int`/`-d mmu` traces, HMP monitor register/physical-memory
  inspection, differential testing across CPU models and `-cpu` flags including
  `sv39`/`svadu`) before isolating it to specifically the second branch hop (L1 -> L0),
  not this crate's PTE construction — see `lantern-boot/STATUS.md` for the full record.
  `map`/4 KiB pages remain correct and host-tested; `lantern-boot` uses `map_megapage`
  exclusively (`FrameSize::Mega`) until this is root-caused upstream or the environment's
  QEMU is updated.
- **A monotonic clock read primitive** ([RFC-0012](../lantern-rfcs/rfcs/0012-monotonic-clock-primitive.md)/
  [ADR-0016](../lantern-rfcs/adr/0016-monotonic-clock-primitive.md)): `Hal::monotonic_time_ns()`,
  the first real implementation of this crate's own long-listed "Timekeeping" abstraction-table
  entry. `riscv64` reads the `time` CSR directly (`rdtime`, unprivileged in S-mode under
  standard OpenSBI `mcounteren` config — no trap, no SBI call) and scales by a hardcoded
  100 ns/tick (QEMU `virt`'s fixed 10 MHz timer; device-tree-sourced frequency discovery
  remains open, see "Next"). `x86-64` is `unimplemented!()`, matching `initial_trap_frame`/
  `enter_thread`'s existing precedent for methods with no real boot work to exercise them
  against yet. Deliberately narrow: no timer interrupts, no scheduler ticks, no user-space
  syscall — motivated directly by `lantern-crypto`'s `Keystore::unseal`, whose `ExpiresAt`
  caveat evaluation has had no real clock source since RFC-0011/ADR-0015 landed. Cross-compiles
  clean (`cargo build`/`clippy --target riscv64gc-unknown-none-elf`, no warnings) and the whole
  downstream dependency chain (`lantern-kernel`/`lantern-capabilities`/`lantern-crypto`/
  `lantern-filesystem`/`lantern-boot`) still builds against the new trait method. **Not yet
  QEMU-validated** — unlike the trap-entry assembly above (which a real bug hid from, only
  caught by an actual `ecall` under QEMU), nothing exercises this primitive under real hardware
  or QEMU yet; it's a single, self-contained CSR read with no kernel-side wiring, so the risk
  profile is much smaller, but this is an honest gap, not implied validation.

- **Fixed: `riscv64` `initial_trap_frame` dropped `arg0` for every thread not started
  first** (2026-10-12). It wrote `arg0` into the raw `a0` register slot only, leaving the
  frame's portable `mr(0)` at 0; `SavedContext` saves both, and the trap trampoline
  restores `raw` then re-applies `mr0..mr3` on top, so any thread first entered by a
  normal context switch started with `a0 == 0`. Only `enter_thread` (the very first
  thread, raw words only) ever saw its real `arg0`. Fixed by setting both views. Latent
  since the first two-program demo: `hello-service`'s second-started server *wanted*
  `arg0 == 0`. Found by `lantern-boot-runtime-demo`, the first demo to launch a
  second-started program with a nonzero `arg0`; QEMU-verified there and across all seven
  `lantern-boot` demos. No host test is possible (the `riscv64` module is arch-gated, like
  the trampoline itself).

- **Interrupts and the timer (2026-11-11, [RFC-0027](../lantern-rfcs/rfcs/0027-interrupt-delivery-timer-and-idle.md)/[ADR-0031](../lantern-rfcs/adr/0031-interrupt-delivery-timer-and-idle.md)).** `TrapFrame` gains a `TrapCause`, and an interrupt's frame carries
  `NOT_A_SYSCALL`, so nothing that dispatches on the syscall number can mistake an interrupted program's registers for a
  request. The `riscv64` trampoline takes the supervisor timer and external interrupts, from U-mode only, without advancing
  `sepc`. `Hal` gains `init_interrupts`, `interrupt_line_enable`, `interrupt_claim` and `interrupt_complete` (QEMU `virt`'s PLIC,
  hart 0's S-mode context), `set_timer` (the SBI TIME extension), `timer_pending`, and `wait_for_interrupt`, a `wfi` with
  interrupts still masked, which wakes on a pending interrupt without taking it. `x86-64` stubs them: no APIC yet.
- **Faults and the direct map (2026-11-13, [RFC-0030](../lantern-rfcs/rfcs/0030-process-lifecycle-faults-and-reclamation.md)/[ADR-0034](../lantern-rfcs/adr/0034-process-lifecycle-faults-and-reclamation.md)).** A synchronous exception from U-mode is `TrapCause::Fault { cause,
  value }` (`scause`, `stval`) instead of parking; a trap from S-mode still parks. `riscv64_paging` gains the direct map
  of RAM (`DIRECT_MAP_BASE`, one root entry), `share_root_entry` and `for_each_table`, and `Hal` gains `ram_window`.
  **4 KiB pages now work under QEMU:** the lifecycle demo builds every child from them, and the IRQ demo maps the UART's
  page. The 3-level-walk failure recorded below no longer reproduces; the missing TLB flush on `Map`/`Unmap` found in this
  work is one plausible cause, not a proven one.
## Next
- `x86-64`: implement `initial_trap_frame`/`enter_thread` for real, once `x86-64` boot
  work actually starts (deferred, see `lantern-boot/STATUS.md`) — needs real QEMU
  validation the same way `riscv64`'s got, not speculative asm nobody can exercise yet.
- `x86-64`: raise the `int 0x80` gate to `DPL=3` and handle the wider (`ss`/`rsp`-inclusive)
  hardware frame once `lantern-kernel` has real user/kernel threads and a `TSS.RSP0` to
  point at a kernel stack.
- Investigate the QEMU 3-level-Sv39-walk limitation above further if it starts blocking
  real work (Phase 1's demo doesn't need 4 KiB pages) — worth re-testing against a newer
  QEMU release to see if it's already fixed upstream.
- `Sstc` (`stimecmp`) as a faster timer than the SBI call, where present; and the PLIC's base address and the UART's line
  from the device tree instead of QEMU `virt` constants (RFC-0027).
- Device-tree-sourced timer frequency, replacing `monotonic_time_ns`'s hardcoded QEMU
  `virt`-specific 100 ns/tick constant — part of "platform discovery" below.
- Remaining HAL surface not yet started: IOMMU, platform discovery, the `x86-64` APIC,
  early console (see `ARCHITECTURE.md`'s abstraction table).

## Blocked on
- Nothing currently — the kernel object/IPC ABI is fixed by
  [RFC-0005](../lantern-rfcs/rfcs/0005-syscall-ipc-abi-and-phase1-scheduling.md)/
  [ADR-0008](../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md), and the trait built on
  it now lives in code.

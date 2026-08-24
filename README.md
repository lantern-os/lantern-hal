# lantern-hal

The **Hardware Abstraction Layer**: the thin seam that confines all ISA- and
platform-specific code so the rest of LanternOS stays portable. The HAL is what makes the
x86-64 (development) → RISC-V (target) journey credible ([ADR-0002](https://github.com/lantern-os/lantern-rfcs/blob/main/adr/0002-riscv-target-isa.md)).

- **Layer:** TCB-adjacent machine layer (a minimal subset is in the TCB).
- **Language:** Rust, `no_std`, with the bulk of the system's justified `unsafe` (MMIO,
  page tables, trap entry) isolated here.
- **System context:** [wiki/Hardware](https://github.com/lantern-os/lantern-docs/blob/main/wiki/Hardware.md), [wiki/Kernel](https://github.com/lantern-os/lantern-docs/blob/main/wiki/Kernel.md).

> ⚠️ **Phase 2.** Trap entry, paging, and the monotonic clock primitive are implemented and
> validated (`riscv64`); this crate's own Phase 1 backlog continues alongside the Roadmap's
> Phase 2. See [`STATUS.md`](./STATUS.md).

## In this repo
- [`ARCHITECTURE.md`](./ARCHITECTURE.md) — the abstraction contract.
- [`THREAT_MODEL.md`](./THREAT_MODEL.md).
- [`STATUS.md`](./STATUS.md).

## The rule
The portable kernel core contains **no** `target_arch` logic beyond calling the HAL. If
per-ISA code is leaking upward, that is a HAL design failure.

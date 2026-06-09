# lantern-hal — Architecture

The HAL defines a small, stable contract between the portable kernel core and the machine.
Companion to [wiki/Hardware](https://github.com/lantern-os/lantern-docs/blob/main/wiki/Hardware.md).

## What the HAL abstracts
| Concern | Examples |
| --- | --- |
| CPU context | Register save/restore, context switch, privilege transitions. |
| Address translation | Page-table format, TLB management, MMU configuration. |
| Traps & interrupts | Trap/exception entry, interrupt controller (PLIC/CLIC/APIC), IRQ routing. |
| Timekeeping | Timer setup, monotonic time, scheduler ticks. |
| DMA isolation | IOMMU configuration so user-space drivers can't read arbitrary memory. |
| Platform discovery | Device tree (RISC-V) / ACPI (x86-64), memory map, CPU topology. |
| Early console | Minimal debug output during bring-up. |

## Design constraints
- **Minimal surface:** the smaller the contract, the easier to port and to audit.
- **`unsafe` containment:** the HAL is where MMIO and raw hardware live; each `unsafe` is
  isolated behind a safe abstraction, justified in-comment, and reviewed ([ADR-0001](https://github.com/lantern-os/lantern-rfcs/blob/main/adr/0001-rust-as-primary-language.md)).
- **No policy:** the HAL exposes mechanism; the kernel and user space decide policy.
- **TCB minimalism:** only the parts the kernel must trust are in the TCB; the rest can be
  pushed toward confined drivers over time.

## Targets
- `riscv64` (RV64GC) — strategic; track H/V/crypto and CHERI/pointer-masking extensions.
- `x86-64` — development convenience only.
- Initial bring-up under QEMU for both.

## Open questions
- Exact split between "HAL in the TCB" and "drivers in confined user space" (esp. IOMMU,
  interrupt controllers).
- How much of platform discovery belongs in boot vs. HAL.
- Abstracting accelerators (NPU/crypto/FPGA) without leaking vendor specifics upward.

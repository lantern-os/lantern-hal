# lantern-hal — Threat Model

Inherits the [system threat model](https://github.com/lantern-os/lantern-docs/blob/main/wiki/Threat-Model.md). The HAL holds the
system's densest `unsafe` and touches hardware directly, so a subset of it is effectively in
the TCB.

## Assets
- Correctness of context switch and address-translation setup (isolation depends on it).
- IOMMU configuration (DMA confinement).
- Interrupt routing integrity.

## Threats and mitigations
| # | Threat | Mitigation |
| --- | --- | --- |
| H1 | `unsafe`/MMIO bug corrupts memory or breaks isolation | Isolate `unsafe` behind safe APIs; review; keep the surface tiny. |
| H2 | Misconfigured page tables leak across address spaces | Centralised, tested page-table abstraction; eventual verification. |
| H3 | DMA-capable device/driver reads arbitrary memory | IOMMU confinement configured by the HAL. |
| H4 | Interrupt misrouting enables escalation or DoS | IRQ routing tied to kernel IRQ capabilities only. |
| H5 | Per-ISA logic leaking upward causes inconsistent enforcement | Strict HAL seam; portable core forbidden from `target_arch` logic. |

## Non-goals
- Microarchitectural side channels and physical attacks (system non-goals at Phase 0).
- Trusting closed firmware below the HAL beyond what boot measures.

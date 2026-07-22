# lantern-hal — Status

**Phase:** 1 (Microkernel prototype) — open per [RFC-0004](https://github.com/lantern-os/lantern-rfcs/blob/main/rfcs/0004-phase-0-to-phase-1-transition.md); design complete, no code merged yet.

## Done
- HAL abstraction surface enumerated and reviewed ([ARCHITECTURE.md](./ARCHITECTURE.md)).
- Threat model drafted and reviewed.

## Next
- Define the minimal HAL trait/contract the kernel depends on.
- Phase 1: implement for `riscv64` + x86-64 under QEMU (context switch, paging, traps,
  timer, early console).

## Blocked on
- Kernel object/IPC ABI ([`lantern-kernel`](https://github.com/lantern-os/lantern-kernel)), which sets the HAL contract.

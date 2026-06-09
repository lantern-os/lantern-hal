# lantern-hal — Status

**Phase:** 0 (Foundations) — design only.

## Done
- HAL abstraction surface enumerated ([ARCHITECTURE.md](./ARCHITECTURE.md)).
- Threat model drafted.

## Next
- Define the minimal HAL trait/contract the kernel depends on.
- Phase 1: implement for `riscv64` + x86-64 under QEMU (context switch, paging, traps,
  timer, early console).

## Blocked on
- Kernel object/IPC ABI ([`lantern-kernel`](https://github.com/lantern-os/lantern-kernel)), which sets the HAL contract.

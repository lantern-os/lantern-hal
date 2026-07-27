//! `x86-64` trap entry — a development-convenience target, not a strategic one
//! (ADR-0002).
//!
//! Mirrors `riscv64`'s trap-entry design through x86's IDT rather than a CSR:
//! [`Hardware::install_trap_handler`] builds a minimal IDT with one populated interrupt
//! gate (vector [`SYSCALL_VECTOR`], `int 0x80`) and loads it via `lidt`. Per
//! [ADR-0008](../../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md), this module fixes
//! the concrete register mapping the ADR left to the HAL:
//!
//! | ABI field         | Register |
//! | ------------------ | -------- |
//! | `mr0..mr3`          | `rdi, rsi, rdx, rcx` (SysV's first four integer-arg registers) |
//! | message tag (raw)   | `r8` (SysV's fifth argument register) |
//! | syscall number      | `rax` |
//!
//! ## Why `int 0x80`, not `syscall`/`sysret`
//! The faster `SYSCALL`/`SYSRET` path needs `STAR`/`LSTAR`/`SFMASK` MSRs and a
//! `swapgs`-based per-CPU GS-base convention that nothing in Phase 1 establishes yet. A
//! plain interrupt gate needs neither: only a code-segment selector (read from the
//! currently loaded `cs`, since nothing here should assume it owns the GDT) and, for a
//! same-privilege trap, no stack switch. Swapping to `syscall`/`sysret` later is a
//! HAL-internal change — ADR-0008 fixes `mr0..mr3`/tag/syscall-number by name, not the
//! trap mechanism.
//!
//! ## Trap flow
//! The assembly stub pushes the 15 general-purpose registers it doesn't otherwise have
//! a home for (`rax, rbx, rcx, rdx, rsi, rdi, rbp, r8..r15`) onto the current stack,
//! then calls [`lantern_hal_x86_64_trap_trampoline`] (plain Rust) with `rsp` — now
//! pointing at that saved frame, directly followed by the hardware-pushed
//! `rip`/`cs`/`rflags` — as its argument. The trampoline builds a [`TrapFrame`], invokes
//! the installed [`TrapHandler`], writes `mr0..mr3`/the tag back into the saved frame,
//! and returns; the assembly then pops everything back and `iretq`s. Unlike `riscv64`'s
//! `ecall`, `int 0x80`'s hardware-saved `rip` already points *past* the trapping
//! instruction, so nothing here needs to adjust it (contrast `riscv64`'s `sepc + 4`).
//!
//! **Ring0-only for now.** The IDT gate is installed with `DPL=0`: a real ring3 entry
//! would push an extra `ss`/`rsp` pair (any privilege-changing interrupt does) that this
//! stub does not account for, and would need `TSS.RSP0` pointed at a real kernel stack
//! — neither exists yet, because `lantern-kernel` has no user/kernel thread distinction
//! to drive them. Raising the gate to `DPL=3` and handling the wider frame is deferred
//! until that lands, tracked in `lantern-hal/STATUS.md`. The gate is an *interrupt* gate
//! (not a trap gate), so `rflags.IF` is cleared on entry and restored by `iretq` — no
//! nested trap can land mid-handler, mirroring `riscv64`'s `sstatus.SIE`-based
//! non-reentrancy.
//!
//! Every other IDT vector (the `0..0x20` CPU exceptions, or any vector besides
//! [`SYSCALL_VECTOR`]) is left absent (`present = 0`): Phase 1 only implements the
//! syscall path, so an unhandled exception here faults rather than being caught — the
//! same "not yet in scope" gap `riscv64` documents for non-`ecall` traps, just with a
//! harsher failure mode, since x86 has no equivalent of "park the hart on an
//! unrecognized `scause`" without an IDT entry to land in.
//!
//! **Not validated on real/emulated hardware yet** — same caveat as `riscv64`: this
//! builds and passes `cargo test`/`clippy` (this crate's host target *is* `x86_64`, so
//! unlike `riscv64` this assembly is compiled on every normal host run of this crate),
//! but nothing calls `install_trap_handler` from a test, since doing so needs ring 0 —
//! `lidt` faults under a hosted OS's ring-3 process.

use core::arch::{asm, global_asm};
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::{Hal, MessageTag, TrapFrame, TrapHandler};

/// The interrupt vector this HAL dedicates to syscalls — chosen to match the familiar
/// `int 0x80` convention, not for any deeper significance.
const SYSCALL_VECTOR: usize = 0x80;

/// Saved-frame word indices, matching the assembly's push order (index 0 = current
/// `rsp` after all pushes). Words `8..15` (`r9..r15`) and `15..18` (the
/// hardware-pushed `rip`/`cs`/`rflags`) are saved/restored by the assembly but never
/// read here.
const REG_RAX: usize = 0;
const REG_RCX: usize = 2;
const REG_RDX: usize = 3;
const REG_RSI: usize = 4;
const REG_RDI: usize = 5;
const REG_R8: usize = 7;
const FRAME_WORDS: usize = 18;

/// The installed [`TrapHandler`], stored as a `usize` (0 = uninstalled).
static TRAP_HANDLER: AtomicUsize = AtomicUsize::new(0);

/// A 64-bit IDT gate descriptor (Intel SDM Vol. 3A, 6.14.1).
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct IdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    reserved: u32,
}

impl IdtEntry {
    const fn missing() -> Self {
        Self {
            offset_low: 0,
            selector: 0,
            ist: 0,
            type_attr: 0,
            offset_mid: 0,
            offset_high: 0,
            reserved: 0,
        }
    }

    /// A present, `DPL=0` 64-bit interrupt gate (type `0xE`: `rflags.IF` cleared on
    /// entry, restored by `iretq`) targeting `handler` in code segment `selector`.
    fn interrupt_gate(handler: usize, selector: u16) -> Self {
        const PRESENT: u8 = 1 << 7;
        const TYPE_INTERRUPT_GATE: u8 = 0xE;
        Self {
            offset_low: handler as u16,
            selector,
            ist: 0,
            type_attr: PRESENT | TYPE_INTERRUPT_GATE,
            offset_mid: (handler >> 16) as u16,
            offset_high: (handler >> 32) as u32,
            reserved: 0,
        }
    }
}

/// Sized to cover vector [`SYSCALL_VECTOR`]; every lower vector is left `present = 0`
/// (see the module-level doc's "not yet in scope" note).
const IDT_ENTRIES: usize = SYSCALL_VECTOR + 1;

/// Wrapped so the static can be `Sync`: written once from `install_trap_handler` before
/// `lidt` makes it live, and never mutated afterward — the CPU only ever reads it.
struct IdtCell(UnsafeCell<[IdtEntry; IDT_ENTRIES]>);
// SAFETY: see the field doc above; there is no concurrent writer.
unsafe impl Sync for IdtCell {}

static IDT: IdtCell = IdtCell(UnsafeCell::new([IdtEntry::missing(); IDT_ENTRIES]));

/// The operand `lidt` reads from; only needs to be valid for the instruction itself; the
/// IDT it points at is what must persist.
#[repr(C, packed)]
struct DescriptorTablePointer {
    limit: u16,
    base: u64,
}

global_asm!(
    r#"
.section .text
.align 16
.global lantern_hal_x86_64_trap_entry
lantern_hal_x86_64_trap_entry:
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push rbp
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15

    mov rdi, rsp
    call lantern_hal_x86_64_trap_trampoline

    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rbp
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax

    iretq
"#
);

unsafe extern "C" {
    fn lantern_hal_x86_64_trap_entry();
}

/// Called only from [`lantern_hal_x86_64_trap_entry`] with `raw` pointing at the pushed
/// frame: `raw[0..15]` the saved GPRs (see the `REG_*` constants), `raw[15..18]` the
/// hardware-pushed `rip`/`cs`/`rflags`.
///
/// # Safety
/// Must only be reached via the assembly trap entry above, which upholds the pointer
/// contract described on this function.
#[unsafe(no_mangle)]
unsafe extern "C" fn lantern_hal_x86_64_trap_trampoline(raw: *mut usize) {
    // SAFETY: `raw` points at `FRAME_WORDS` words the assembly stub just pushed onto
    // the current stack; it is exclusively ours until we return (nothing else runs on
    // this stack while we hold it, since the interrupt gate cleared `rflags.IF`).
    let regs = unsafe { core::slice::from_raw_parts_mut(raw, FRAME_WORDS) };

    let handler_addr = TRAP_HANDLER.load(Ordering::Acquire);
    debug_assert!(handler_addr != 0, "trap fired before install_trap_handler was called");
    // SAFETY: `handler_addr` was written by `install_trap_handler` from a real
    // `TrapHandler` value and never changes afterwards.
    let handler: TrapHandler = unsafe { core::mem::transmute::<usize, TrapHandler>(handler_addr) };

    let mut frame = TrapFrame::zeroed();
    frame.set_mr(0, regs[REG_RDI]);
    frame.set_mr(1, regs[REG_RSI]);
    frame.set_mr(2, regs[REG_RDX]);
    frame.set_mr(3, regs[REG_RCX]);
    frame.set_tag(MessageTag::from_raw(regs[REG_R8]));
    frame.set_syscall_number(regs[REG_RAX]);

    handler(&mut frame);

    regs[REG_RDI] = frame.mr(0);
    regs[REG_RSI] = frame.mr(1);
    regs[REG_RDX] = frame.mr(2);
    regs[REG_RCX] = frame.mr(3);
    regs[REG_R8] = frame.tag().into_raw();
    // `int 0x80`'s hardware-saved `rip` already points past the trapping instruction
    // (unlike `riscv64`'s `sepc` on `ecall`) — nothing to advance here.
}

/// The `x86-64` HAL implementation.
pub struct Hardware;

impl Hal for Hardware {
    unsafe fn install_trap_handler(handler: TrapHandler) {
        debug_assert_eq!(
            TRAP_HANDLER.load(Ordering::Relaxed),
            0,
            "install_trap_handler must be called at most once"
        );
        TRAP_HANDLER.store(handler as usize, Ordering::Release);

        let cs: u16;
        // SAFETY: reading `cs` has no side effects and is unprivileged.
        unsafe {
            asm!("mov {0:x}, cs", out(reg) cs, options(nomem, nostack, preserves_flags));
        }

        let entry_addr = lantern_hal_x86_64_trap_entry as *const () as usize;
        let entry = IdtEntry::interrupt_gate(entry_addr, cs);

        // SAFETY: caller upholds `install_trap_handler`'s contract (called once,
        // before traps are enabled, from ring 0). `IDT` is written here, and only
        // here, before `lidt` makes it live; nothing else can observe it mid-write.
        unsafe {
            (*IDT.0.get())[SYSCALL_VECTOR] = entry;

            let idtr = DescriptorTablePointer {
                limit: (core::mem::size_of::<[IdtEntry; IDT_ENTRIES]>() - 1) as u16,
                base: IDT.0.get() as u64,
            };
            asm!("lidt [{0}]", in(reg) &idtr);
        }
    }

    fn initial_trap_frame(_pc: usize, _sp: usize, _arg0: usize) -> TrapFrame {
        // `x86-64` boot bring-up (real → protected → long mode, GDT/TSS setup) is
        // out of scope for now — `riscv64` is the strategic target and the first
        // one actually driven through a real boot loader (`lantern-boot`). This
        // stays an honest stub rather than untested asm nobody can currently
        // exercise; see `lantern-hal/STATUS.md`.
        unimplemented!(
            "x86-64 initial_trap_frame — no lantern-boot x86-64 target yet to exercise it"
        )
    }

    unsafe fn enter_thread(_frame: &TrapFrame) -> ! {
        unimplemented!("x86-64 enter_thread — no lantern-boot x86-64 target yet to exercise it")
    }

    unsafe fn activate_address_space(_root: usize) {
        // A documented no-op, not `unimplemented!()` — unlike `enter_thread`, this
        // runs on every context switch, including ones `lantern-kernel`'s host
        // (`x86_64`) unit tests exercise directly. `x86-64` has no paging
        // implementation at all yet (`lantern-hal/STATUS.md`); panicking here
        // would break `cargo test` for a target with no real caller anyway (no
        // `x86-64` boot loader exists to invoke this for real).
    }
}

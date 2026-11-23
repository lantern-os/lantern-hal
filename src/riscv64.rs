//! `riscv64` (RV64GC) trap entry — the strategic target (ADR-0002).
//!
//! Per [ADR-0008](../../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md) the portable
//! kernel core never names a physical register; this module fixes the concrete mapping
//! ADR-0008 left to the HAL ("the message tag via a dedicated register... final
//! assignment to be confirmed during real trap-assembly bring-up"):
//!
//! | ABI field         | Register |
//! | ------------------ | -------- |
//! | `mr0..mr3`          | `a0..a3` |
//! | message tag (raw)   | `a4`     |
//! | syscall number      | `a7`     |
//!
//! This choice keeps `a0..a3`/`a7` on their natural SysV-style argument slots and leaves
//! `a5`/`a6` free for a future extension (e.g. a fifth/sixth MR) without renumbering.
//!
//! ## Trap flow
//! [`Hardware::install_trap_handler`] points `stvec` (Direct mode) at
//! [`lantern_hal_riscv64_trap_entry`], a hand-written assembly vector, and points
//! `sscratch` at a static 32-word raw register save area ([`RAW_FRAME`]). On every trap
//! the assembly:
//! 1. Swaps `t6` with `sscratch` to get a pointer to the save area without losing `t6`'s
//!    real value.
//! 2. Stores `x1..x30` through that pointer, then recovers real `t6` (stashed in
//!    `sscratch` by the swap) and stores it too, then restores `sscratch` to point at the
//!    save area again (needed so the *next* trap can find it).
//! 3. Stores `sepc`, switches to a dedicated trap stack, and calls
//!    [`lantern_hal_riscv64_trap_trampoline`] (plain Rust) with the save-area pointer.
//! 4. On return, restores `sepc` and all GPRs from the (possibly updated) save area and
//!    `sret`s back to the interrupted context.
//!
//! The trampoline advances `sepc` past the trapping `ecall` *before* running the
//! handler (not after — see the trampoline's own doc comment for why that ordering
//! matters once a handler can switch to a completely different thread), reads
//! `scause`, then populates a
//! [`TrapFrame`]'s *entire* raw register file (not just `mr0..mr3`/the tag) before
//! invoking the installed [`TrapHandler`], and writes the whole thing back
//! afterward. This full-frame round trip is what lets `lantern-kernel` implement a
//! context switch as "overwrite the frame with a different thread's saved state"
//! (`lantern-kernel/ARCHITECTURE.md`'s concurrency notes) — populating only
//! `mr0..mr3`/tag was an earlier version of this trampoline's design, and it
//! silently broke every context switch (see the trampoline's doc comment).
//!
//! **Causes** ([RFC-0027](../../lantern-rfcs/rfcs/0027-interrupt-delivery-timer-and-idle.md)): an `ecall` from
//! U-mode, and the supervisor timer and external interrupts, which are only ever taken from U-mode. An interrupt
//! doesn't advance `sepc`, and its frame carries [`NOT_A_SYSCALL`]. Any other exception from U-mode is a
//! [`TrapCause::Fault`] ([RFC-0030](../../lantern-rfcs/rfcs/0030-process-lifecycle-faults-and-reclamation.md)): the
//! kernel ends that program, not the system. A trap from S-mode still parks the hart: that's a kernel bug.
//!
//! **Single-hart only.** `RAW_FRAME` and the installed handler are process-wide statics,
//! not per-hart; Phase 1 does not yet have a hart-local storage story. Re-entrancy within
//! *one trap's handling* is not a concern: hardware clears `sstatus.SIE` on trap entry and
//! this code never re-enables it before `sret`. That guarantee ends at `sret`, though —
//! `sret` restores `sstatus.SIE` from `sstatus.SPIE`, so if interrupts were enabled before
//! the first trap (as they are by default under OpenSBI), they come back enabled
//! afterward. So `init_interrupts` clears `sstatus.SIE` before setting `sie`, and `enter_thread` clears `SPIE`:
//! S-mode never runs with interrupts on, and the trap entry's "from U-mode" assumption holds. The kernel's idle loop
//! waits with them masked ([`Hal::wait_for_interrupt`]). The old "`wfi` crashes" symptom in `lantern-boot` came from
//! interrupts left enabled with nothing to take them.
//!
//! **Validated under real QEMU** (`qemu-system-riscv64 -machine virt -bios default`), via
//! `lantern-boot`'s two-thread `Call`/`Recv`/`Reply` demo — the first real exercise of this
//! assembly by an actual `ecall`, which is what caught the full-frame trampoline bug
//! described above (no unit test — none of which drive this trampoline at all, since it's
//! arch-gated assembly — could have caught it).

use core::arch::{asm, global_asm};
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::{Hal, MessageTag, TrapCause, TrapFrame, TrapHandler, NOT_A_SYSCALL};

/// Words in the raw save area: `x1..x31` (31 registers, `x0` is hardwired zero and never saved), one word for `sepc`,
/// then `f0..f31` and `fcsr`. Every program runs with the hardware's floating-point unit on and shares it, so a thread's
/// FP registers are part of its state like any other: a switch that didn't save them would hand one program another's
/// values mid-calculation. They weren't saved before RFC-0035's work, while the inference engine and Wasmtime's
/// interpreter both compute in them.
const RAW_FRAME_WORDS: usize = crate::RAW_WORDS;

/// `x{N}` is stored at index `N - 1`; index `RAW_SEPC` holds `sepc`.
const RAW_SEPC: usize = 31;
const REG_A0: usize = 10 - 1;
const REG_A1: usize = 11 - 1;
const REG_A2: usize = 12 - 1;
const REG_A3: usize = 13 - 1;
const REG_A4: usize = 14 - 1;
const REG_A7: usize = 17 - 1;

/// "Environment call from U-mode": the one synchronous `scause` this trampoline handles.
const SCAUSE_ECALL_FROM_U: usize = 8;
/// `scause`'s top bit: an interrupt, not an exception.
const SCAUSE_INTERRUPT: usize = 1 << 63;
/// `sstatus.SPP`: set when the trap came from S-mode.
const SSTATUS_SPP: usize = 1 << 8;
/// Supervisor timer and external interrupts, the two RFC-0027 enables (`sie.STIE`, `sie.SEIE`).
const IRQ_S_TIMER: usize = 5;
const IRQ_S_EXTERNAL: usize = 9;

/// The machine's interrupt controller ([RFC-0039](../../lantern-rfcs/rfcs/0039-bring-up-on-the-k3-pico-itx.md)), as boot
/// read it from the device tree. The kernel maps its registers S-mode-only at their physical addresses in every VSpace
/// (`lantern-boot`'s `launch::register_kernel_template`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Riscv64Intc {
    /// A PLIC at `base`, the boot hart's S-mode `context`, and lines 1 to `lines - 1` (0 means "none").
    Plic { base: usize, context: usize, lines: u32 },
    /// AIA: the supervisor-level APLIC at `aplic_base`, in MSI mode, with sources 1 to `sources`, delivering to the boot
    /// hart's supervisor IMSIC interrupt file, which has identities 1 to `ids` and which the APLIC reaches as hart
    /// `hart_index`. Each line `n` is sent as identity `n`, so lines go up to `min(sources, ids)`.
    Aia { aplic_base: usize, sources: u32, hart_index: u32, ids: u32 },
}

/// What [`Hardware`] needs to know about the machine, set once by boot with [`riscv64_configure`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Riscv64Platform {
    pub intc: Riscv64Intc,
    /// The `time` CSR's rate, from the device tree's `timebase-frequency`.
    pub timebase_hz: u64,
    /// DMA frames are mapped non-cacheable (the CPU has `Svpbmt`), and after the kernel zeroes one through its
    /// cacheable direct map it cleans those lines out, `cbom_block` bytes at a time (`Zicbom`), so no dirty line is ever
    /// written back over what a device wrote (RFC-0039 §5). Otherwise both 0, which only a coherent machine allows.
    pub dma_uncached: bool,
    pub cbom_block: usize,
}

/// The PTE flags a DMA frame is mapped with: non-cacheable where the platform maps DMA uncached.
pub fn riscv64_dma_pte_flags() -> crate::Riscv64PteFlags {
    // SAFETY: only written once, before any reader runs (`riscv64_configure`'s contract).
    match unsafe { *PLATFORM.0.get() } {
        Some(p) if p.dma_uncached => crate::Riscv64PteFlags::NONCACHEABLE,
        _ => crate::Riscv64PteFlags::NONE,
    }
}

/// Cleans and invalidates `[vaddr, vaddr + len)` from the data caches (`cbo.flush`), where the platform maps DMA
/// uncached: a DMA frame the kernel just zeroed through its cacheable direct map.
pub fn riscv64_flush_for_dma(vaddr: usize, len: usize) {
    // SAFETY: as `riscv64_dma_pte_flags`.
    let Some(p) = (unsafe { *PLATFORM.0.get() }) else { return };
    if !p.dma_uncached || p.cbom_block == 0 {
        return;
    }
    let mut at = vaddr & !(p.cbom_block - 1);
    while at < vaddr + len {
        // SAFETY: `cbo.flush` writes back and invalidates the block holding `at`, which the caller mapped; it changes no
        // memory's contents. Encoded as its instruction word (CBO.FLUSH, rs1 = a0), for assemblers without `Zicbom`.
        unsafe { asm!(".insn i 0x0f, 2, x0, {0}, 2", in(reg) at, options(nostack)) };
        at += p.cbom_block;
    }
    // SAFETY: a fence: orders the flushes before anything a device is told after.
    unsafe { asm!("fence iorw, iorw", options(nostack)) };
}

struct PlatformCell(UnsafeCell<Option<Riscv64Platform>>);
// SAFETY: written once, by `riscv64_configure`, before any thread runs or any trap is taken; only read after that, on
// one hart (ADR-0010).
unsafe impl Sync for PlatformCell {}
static PLATFORM: PlatformCell = PlatformCell(UnsafeCell::new(None));

/// Records the machine's interrupt controller and timer rate. Nothing is assumed about either until this runs: the
/// interrupt and timer paths stop with a message if it hasn't.
///
/// # Safety
/// Call once, before [`Hal::init_interrupts`], any timer use, or any thread runs.
pub unsafe fn riscv64_configure(platform: Riscv64Platform) {
    assert!(platform.timebase_hz > 0, "a zero timebase");
    assert!(platform.cbom_block == 0 || platform.cbom_block.is_power_of_two(), "a cache block that isn't a power of two");
    // SAFETY: the contract: nothing reads it concurrently, and nothing has read it yet.
    unsafe { *PLATFORM.0.get() = Some(platform) };
}

fn platform() -> Riscv64Platform {
    // SAFETY: only written once, before any reader runs (`riscv64_configure`'s contract).
    unsafe { *PLATFORM.0.get() }.expect("the platform wasn't configured: boot must call riscv64_configure first")
}

const PLIC_ENABLE_BASE: usize = 0x2000;
const PLIC_ENABLE_STRIDE: usize = 0x80;
const PLIC_CONTEXT_BASE: usize = 0x20_0000;
const PLIC_CONTEXT_STRIDE: usize = 0x1000;

/// A PLIC's registers for one context.
struct Plic {
    base: usize,
    context: usize,
    lines: u32,
}

impl Plic {
    fn new(base: usize, context: usize, lines: u32) -> Self {
        Self { base, context, lines }
    }
    fn enable(&self) -> usize {
        self.base + PLIC_ENABLE_BASE + PLIC_ENABLE_STRIDE * self.context
    }
    fn threshold(&self) -> usize {
        self.base + PLIC_CONTEXT_BASE + PLIC_CONTEXT_STRIDE * self.context
    }
    fn claim(&self) -> usize {
        self.threshold() + 4
    }

    fn init(&self) {
        // SAFETY: the PLIC is mapped S-mode-only at its physical address in every VSpace, and only the kernel, with
        // interrupts masked, reaches it.
        unsafe { core::ptr::write_volatile(self.threshold() as *mut u32, 0) };
    }

    fn enable_line(&self, line: u32, enabled: bool) {
        if line == 0 || line >= self.lines {
            return;
        }
        let word = (self.enable() + (line as usize / 32) * 4) as *mut u32;
        let bit = 1u32 << (line % 32);
        // SAFETY: as `init`.
        unsafe {
            // Priority 0 never interrupts; 1 is the lowest that does, and the threshold is 0.
            core::ptr::write_volatile((self.base + 4 * line as usize) as *mut u32, enabled as u32);
            let current = core::ptr::read_volatile(word);
            core::ptr::write_volatile(word, if enabled { current | bit } else { current & !bit });
        }
    }

    fn claim_line(&self) -> u32 {
        // SAFETY: as `init`. Reading claim is what claims.
        unsafe { core::ptr::read_volatile(self.claim() as *const u32) }
    }

    fn complete_line(&self, line: u32) {
        if line == 0 || line >= self.lines {
            return;
        }
        // SAFETY: as `init`.
        unsafe { core::ptr::write_volatile(self.claim() as *mut u32, line) }
    }
}

/// AIA ([RFC-0039](../../lantern-rfcs/rfcs/0039-bring-up-on-the-k3-pico-itx.md) §3): an APLIC in MSI delivery mode, and the
/// boot hart's supervisor IMSIC, reached through the `siselect`/`sireg`/`stopei` CSRs. The RISC-V Advanced Interrupt
/// Architecture specification, version 1.0, gives every offset and field here.
///
/// The machine-level firmware (OpenSBI) owns the root interrupt domain: it delegates the supervisor domain and sets
/// the MSI address configuration this APLIC uses. LanternOS only configures the supervisor domain's sources.
struct Aia {
    aplic: usize,
    sources: u32,
    hart_index: u32,
    ids: u32,
}

const APLIC_DOMAINCFG: usize = 0x0000;
const APLIC_SOURCECFG: usize = 0x0004;
const APLIC_SETIPNUM: usize = 0x1cdc;
const APLIC_IN_CLRIP: usize = 0x1d00;
const APLIC_SETIENUM: usize = 0x1edc;
const APLIC_CLRIENUM: usize = 0x1fdc;
const APLIC_TARGET: usize = 0x3004;
/// `domaincfg`: interrupts enabled, MSI delivery mode.
const DOMAINCFG_IE: u32 = 1 << 8;
const DOMAINCFG_DM_MSI: u32 = 1 << 2;
/// `sourcecfg`'s source mode: inactive, or level-sensitive, active high (every device LanternOS drives is).
const SOURCE_INACTIVE: u32 = 0;
const SOURCE_LEVEL_HIGH: u32 = 6;

/// The IMSIC's indirectly-accessed registers (`siselect` is CSR 0x150, `sireg` 0x151; `stopei`, the claim CSR, 0x15c).
const IMSIC_EIDELIVERY: usize = 0x70;
const IMSIC_EITHRESHOLD: usize = 0x72;
const IMSIC_EIE0: usize = 0xc0;

impl Aia {
    fn new(aplic: usize, sources: u32, hart_index: u32, ids: u32) -> Self {
        Self { aplic, sources, hart_index, ids }
    }

    /// The highest line both sides can carry: line `n` is sent as identity `n`.
    fn lines(&self) -> u32 {
        self.sources.min(self.ids)
    }

    fn reg(&self, offset: usize) -> *mut u32 {
        (self.aplic + offset) as *mut u32
    }

    /// Selects IMSIC register `index`, then writes, sets or clears bits of it.
    fn imsic_write(index: usize, value: usize) {
        // SAFETY: `siselect`/`sireg` reach the boot hart's own supervisor interrupt file; with `sstatus.SIE` clear and
        // only the kernel touching them, nothing interleaves.
        unsafe { asm!("csrw 0x150, {0}", "csrw 0x151, {1}", in(reg) index, in(reg) value, options(nomem, nostack)) };
    }

    fn imsic_set_bits(index: usize, bits: usize) {
        // SAFETY: as `imsic_write`.
        unsafe { asm!("csrw 0x150, {0}", "csrs 0x151, {1}", in(reg) index, in(reg) bits, options(nomem, nostack)) };
    }

    fn imsic_clear_bits(index: usize, bits: usize) {
        // SAFETY: as `imsic_write`.
        unsafe { asm!("csrw 0x150, {0}", "csrc 0x151, {1}", in(reg) index, in(reg) bits, options(nomem, nostack)) };
    }

    fn init(&self) {
        // Deliver every identity, from any priority.
        Self::imsic_write(IMSIC_EITHRESHOLD, 0);
        Self::imsic_write(IMSIC_EIDELIVERY, 1);
        // SAFETY: the APLIC is mapped S-mode-only at its physical address in every VSpace; only the kernel, with
        // interrupts masked, reaches it.
        unsafe {
            for line in 1..=self.sources {
                core::ptr::write_volatile(self.reg(APLIC_SOURCECFG + 4 * (line as usize - 1)), SOURCE_INACTIVE);
            }
            core::ptr::write_volatile(self.reg(APLIC_DOMAINCFG), DOMAINCFG_IE | DOMAINCFG_DM_MSI);
        }
    }

    fn enable_line(&self, line: u32, enabled: bool) {
        if line == 0 || line > self.lines() {
            return;
        }
        let (word, bit) = (IMSIC_EIE0 + 2 * (line as usize / 64), 1usize << (line % 64));
        let n = line as usize - 1;
        // SAFETY: as `init`.
        unsafe {
            if enabled {
                core::ptr::write_volatile(self.reg(APLIC_SOURCECFG + 4 * n), SOURCE_LEVEL_HIGH);
                // MSI target: this hart, guest 0 (the supervisor file), identity `line`.
                core::ptr::write_volatile(self.reg(APLIC_TARGET + 4 * n), self.hart_index << 18 | line);
                Self::imsic_set_bits(word, bit);
                core::ptr::write_volatile(self.reg(APLIC_SETIENUM), line);
            } else {
                core::ptr::write_volatile(self.reg(APLIC_CLRIENUM), line);
                Self::imsic_clear_bits(word, bit);
                core::ptr::write_volatile(self.reg(APLIC_SOURCECFG + 4 * n), SOURCE_INACTIVE);
            }
        }
    }

    fn claim_line(&self) -> u32 {
        let top: usize;
        // SAFETY: swapping `stopei` claims the highest-priority pending identity, clearing its pending bit; it returns
        // the identity in bits 26:16, or 0 for none.
        unsafe { asm!("csrrw {0}, 0x15c, zero", out(reg) top, options(nomem, nostack)) };
        ((top >> 16) & 0x7ff) as u32
    }

    fn complete_line(&self, line: u32) {
        if line == 0 || line > self.lines() {
            return;
        }
        // In MSI mode a level-sensitive source sends one MSI per rising edge. If the device still asserts the line once
        // its handler has run, send it again, as the specification's level-sensitive handling requires, so nothing is
        // lost.
        // SAFETY: as `init`.
        unsafe {
            let word = core::ptr::read_volatile(self.reg(APLIC_IN_CLRIP + 4 * (line as usize / 32)));
            if word & (1 << (line % 32)) != 0 {
                core::ptr::write_volatile(self.reg(APLIC_SETIPNUM), line);
            }
        }
    }
}

/// The configured controller.
enum Intc {
    Plic(Plic),
    Aia(Aia),
}

fn intc() -> Intc {
    match platform().intc {
        Riscv64Intc::Plic { base, context, lines } => Intc::Plic(Plic::new(base, context, lines)),
        Riscv64Intc::Aia { aplic_base, sources, hart_index, ids } => Intc::Aia(Aia::new(aplic_base, sources, hart_index, ids)),
    }
}

/// The SBI TIME extension's id, "TIME", and `sbi_set_timer`'s function id.
const SBI_EXT_TIME: usize = 0x5449_4D45;
const SBI_SET_TIMER: usize = 0;

/// Timer ticks to nanoseconds and back, at the device tree's `timebase-frequency`
/// ([RFC-0012](../../lantern-rfcs/rfcs/0012-monotonic-clock-primitive.md), [RFC-0039](../../lantern-rfcs/rfcs/0039-bring-up-on-the-k3-pico-itx.md)):
/// 10 MHz on QEMU `virt`, 24 MHz on the K3. Through 128 bits, so neither overflows for centuries at either rate.
fn ticks_to_ns(ticks: u64, hz: u64) -> u64 {
    (ticks as u128 * 1_000_000_000 / hz as u128).min(u64::MAX as u128) as u64
}

fn ns_to_ticks(ns: u64, hz: u64) -> u64 {
    (ns as u128 * hz as u128 / 1_000_000_000).min(u64::MAX as u128) as u64
}

/// `sscratch` points here. Wrapped so the static can be `Sync`: real synchronization
/// comes from the hardware guarantee documented on the module, not from this type.
struct RawFrameCell(UnsafeCell<[usize; RAW_FRAME_WORDS]>);
// SAFETY: only ever accessed from trap context on a single hart, which the RISC-V trap
// entry above guarantees cannot re-enter itself (see the module-level "Single-hart only"
// note).
unsafe impl Sync for RawFrameCell {}

static RAW_FRAME: RawFrameCell = RawFrameCell(UnsafeCell::new([0; RAW_FRAME_WORDS]));

/// The installed [`TrapHandler`], stored as a `usize` (0 = uninstalled).
static TRAP_HANDLER: AtomicUsize = AtomicUsize::new(0);

global_asm!(
    r#"
// The trap paths save and restore the FP registers, which this target has; the assembler wants it said.
.option arch, +d
.section .text
.align 4
.global lantern_hal_riscv64_trap_entry
lantern_hal_riscv64_trap_entry:
    // t6 <- raw-frame pointer (was in sscratch); sscratch <- caller's real t6.
    csrrw t6, sscratch, t6

    sd x1,  0(t6)
    sd x2,  8(t6)
    sd x3,  16(t6)
    sd x4,  24(t6)
    sd x5,  32(t6)
    sd x6,  40(t6)
    sd x7,  48(t6)
    sd x8,  56(t6)
    sd x9,  64(t6)
    sd x10, 72(t6)
    sd x11, 80(t6)
    sd x12, 88(t6)
    sd x13, 96(t6)
    sd x14, 104(t6)
    sd x15, 112(t6)
    sd x16, 120(t6)
    sd x17, 128(t6)
    sd x18, 136(t6)
    sd x19, 144(t6)
    sd x20, 152(t6)
    sd x21, 160(t6)
    sd x22, 168(t6)
    sd x23, 176(t6)
    sd x24, 184(t6)
    sd x25, 192(t6)
    sd x26, 200(t6)
    sd x27, 208(t6)
    sd x28, 216(t6)
    sd x29, 224(t6)
    sd x30, 232(t6)

    // Recover real t6 (x31) from sscratch and store it, then point sscratch back at
    // the frame so the *next* trap can find it.
    csrr t0, sscratch
    sd t0, 240(t6)
    csrw sscratch, t6

    csrr t0, sepc
    sd t0, 248(t6)

    // The floating-point registers and their status: the thread's too.
    fsd f0, 256(t6)
    fsd f1, 264(t6)
    fsd f2, 272(t6)
    fsd f3, 280(t6)
    fsd f4, 288(t6)
    fsd f5, 296(t6)
    fsd f6, 304(t6)
    fsd f7, 312(t6)
    fsd f8, 320(t6)
    fsd f9, 328(t6)
    fsd f10, 336(t6)
    fsd f11, 344(t6)
    fsd f12, 352(t6)
    fsd f13, 360(t6)
    fsd f14, 368(t6)
    fsd f15, 376(t6)
    fsd f16, 384(t6)
    fsd f17, 392(t6)
    fsd f18, 400(t6)
    fsd f19, 408(t6)
    fsd f20, 416(t6)
    fsd f21, 424(t6)
    fsd f22, 432(t6)
    fsd f23, 440(t6)
    fsd f24, 448(t6)
    fsd f25, 456(t6)
    fsd f26, 464(t6)
    fsd f27, 472(t6)
    fsd f28, 480(t6)
    fsd f29, 488(t6)
    fsd f30, 496(t6)
    fsd f31, 504(t6)
    frcsr t0
    sd t0, 512(t6)

    mv a0, t6
    la sp, lantern_hal_riscv64_trap_stack_top
    call lantern_hal_riscv64_trap_trampoline

    // t6 is caller-saved, so the trampoline call clobbered it; sscratch still holds
    // the frame pointer (untouched by Rust), so reload from there.
    csrr t6, sscratch

    ld t0, 512(t6)
    fscsr t0
    fld f0, 256(t6)
    fld f1, 264(t6)
    fld f2, 272(t6)
    fld f3, 280(t6)
    fld f4, 288(t6)
    fld f5, 296(t6)
    fld f6, 304(t6)
    fld f7, 312(t6)
    fld f8, 320(t6)
    fld f9, 328(t6)
    fld f10, 336(t6)
    fld f11, 344(t6)
    fld f12, 352(t6)
    fld f13, 360(t6)
    fld f14, 368(t6)
    fld f15, 376(t6)
    fld f16, 384(t6)
    fld f17, 392(t6)
    fld f18, 400(t6)
    fld f19, 408(t6)
    fld f20, 416(t6)
    fld f21, 424(t6)
    fld f22, 432(t6)
    fld f23, 440(t6)
    fld f24, 448(t6)
    fld f25, 456(t6)
    fld f26, 464(t6)
    fld f27, 472(t6)
    fld f28, 480(t6)
    fld f29, 488(t6)
    fld f30, 496(t6)
    fld f31, 504(t6)

    ld t0, 248(t6)
    csrw sepc, t0

    ld x1,  0(t6)
    ld x2,  8(t6)
    ld x3,  16(t6)
    ld x4,  24(t6)
    ld x5,  32(t6)
    ld x6,  40(t6)
    ld x7,  48(t6)
    ld x8,  56(t6)
    ld x9,  64(t6)
    ld x10, 72(t6)
    ld x11, 80(t6)
    ld x12, 88(t6)
    ld x13, 96(t6)
    ld x14, 104(t6)
    ld x15, 112(t6)
    ld x16, 120(t6)
    ld x17, 128(t6)
    ld x18, 136(t6)
    ld x19, 144(t6)
    ld x20, 152(t6)
    ld x21, 160(t6)
    ld x22, 168(t6)
    ld x23, 176(t6)
    ld x24, 184(t6)
    ld x25, 192(t6)
    ld x26, 200(t6)
    ld x27, 208(t6)
    ld x28, 216(t6)
    ld x29, 224(t6)
    ld x30, 232(t6)
    // Real t6 last, from the frame — sscratch is left pointing at the frame.
    ld t6, 240(t6)

    sret

.section .bss
.align 12
.global lantern_hal_riscv64_trap_stack_bottom
lantern_hal_riscv64_trap_stack_bottom:
    .skip 65536
.global lantern_hal_riscv64_trap_stack_top
lantern_hal_riscv64_trap_stack_top:

// Cold-starts a thread that has never trapped, so there is nothing saved to
// restore from `sscratch`/`RAW_FRAME` — the caller (Rust) passes the raw-frame
// pointer directly in a0 instead. Otherwise this is exactly the restore half of
// the trap-exit sequence above (kept as a literal copy rather than a shared
// subroutine, so touching the hot trap-exit path can't accidentally affect this
// one-time cold path or vice versa).
.section .text
.align 4
.global lantern_hal_riscv64_enter_thread
lantern_hal_riscv64_enter_thread:
    mv t6, a0

    ld t0, 512(t6)
    fscsr t0
    fld f0, 256(t6)
    fld f1, 264(t6)
    fld f2, 272(t6)
    fld f3, 280(t6)
    fld f4, 288(t6)
    fld f5, 296(t6)
    fld f6, 304(t6)
    fld f7, 312(t6)
    fld f8, 320(t6)
    fld f9, 328(t6)
    fld f10, 336(t6)
    fld f11, 344(t6)
    fld f12, 352(t6)
    fld f13, 360(t6)
    fld f14, 368(t6)
    fld f15, 376(t6)
    fld f16, 384(t6)
    fld f17, 392(t6)
    fld f18, 400(t6)
    fld f19, 408(t6)
    fld f20, 416(t6)
    fld f21, 424(t6)
    fld f22, 432(t6)
    fld f23, 440(t6)
    fld f24, 448(t6)
    fld f25, 456(t6)
    fld f26, 464(t6)
    fld f27, 472(t6)
    fld f28, 480(t6)
    fld f29, 488(t6)
    fld f30, 496(t6)
    fld f31, 504(t6)

    ld t0, 248(t6)
    csrw sepc, t0

    ld x1,  0(t6)
    ld x2,  8(t6)
    ld x3,  16(t6)
    ld x4,  24(t6)
    ld x5,  32(t6)
    ld x6,  40(t6)
    ld x7,  48(t6)
    ld x8,  56(t6)
    ld x9,  64(t6)
    ld x10, 72(t6)
    ld x11, 80(t6)
    ld x12, 88(t6)
    ld x13, 96(t6)
    ld x14, 104(t6)
    ld x15, 112(t6)
    ld x16, 120(t6)
    ld x17, 128(t6)
    ld x18, 136(t6)
    ld x19, 144(t6)
    ld x20, 152(t6)
    ld x21, 160(t6)
    ld x22, 168(t6)
    ld x23, 176(t6)
    ld x24, 184(t6)
    ld x25, 192(t6)
    ld x26, 200(t6)
    ld x27, 208(t6)
    ld x28, 216(t6)
    ld x29, 224(t6)
    ld x30, 232(t6)
    ld x31, 240(t6)

    sret
"#
);

unsafe extern "C" {
    fn lantern_hal_riscv64_trap_entry();
    fn lantern_hal_riscv64_enter_thread(raw: *const usize) -> !;
}

/// Called only from [`lantern_hal_riscv64_trap_entry`] with `raw` pointing at
/// [`RAW_FRAME`]'s 32 words, `raw[0..31]` holding `x1..x31` and `raw[31]` holding `sepc`.
///
/// # Safety
/// Must only be reached via the assembly trap entry above, which upholds the pointer
/// contract described on this function and guarantees no concurrent call on this hart.
#[unsafe(no_mangle)]
unsafe extern "C" fn lantern_hal_riscv64_trap_trampoline(raw: *mut usize) {
    // SAFETY: `raw` is `RAW_FRAME`'s address, established once in
    // `install_trap_handler` before traps were enabled; the assembly guarantees
    // exclusive access for the duration of this call (see the module-level
    // "Single-hart only" note).
    let regs = unsafe { core::slice::from_raw_parts_mut(raw, RAW_FRAME_WORDS) };

    let scause: usize;
    // SAFETY: `scause` is always readable in S-mode; the read has no side effects.
    unsafe {
        asm!("csrr {0}, scause", out(reg) scause, options(nomem, nostack));
    }

    let (sstatus, stval): (usize, usize);
    // SAFETY: both are always readable in S-mode; reading has no side effects.
    unsafe {
        asm!("csrr {0}, sstatus", out(reg) sstatus, options(nomem, nostack));
        asm!("csrr {0}, stval", out(reg) stval, options(nomem, nostack));
    }
    let from_user = sstatus & SSTATUS_SPP == 0;
    let cause = match scause {
        // A trap taken in S-mode is a kernel bug, and the entry's register-swap assumes U-mode: park the hart
        // rather than continue in a state nothing understands.
        _ if !from_user => loop {
            core::hint::spin_loop();
        },
        SCAUSE_ECALL_FROM_U => TrapCause::Syscall,
        _ if scause == SCAUSE_INTERRUPT | IRQ_S_TIMER => TrapCause::TimerInterrupt,
        _ if scause == SCAUSE_INTERRUPT | IRQ_S_EXTERNAL => TrapCause::ExternalInterrupt,
        // Any other interrupt isn't enabled, so can't be taken.
        _ if scause & SCAUSE_INTERRUPT != 0 => loop {
            core::hint::spin_loop();
        },
        // RFC-0030: any other exception from U-mode ends that program, not the system.
        _ => TrapCause::Fault { cause: scause, value: stval },
    };

    if cause != TrapCause::Syscall {
        // An interrupt: the interrupted instruction hasn't run yet, so `sepc` stays where it is. Every register is
        // the program's own, and goes back unchanged unless the kernel switches threads.
        run_handler(regs, cause);
        return;
    }

    // `ecall` doesn't auto-advance `pc`; without this, resuming the trapping
    // thread in place would trap right back into the same `ecall` forever. Done
    // *before* the handler runs (not after) so that a handler which switches to a
    // completely different thread — replacing every word of `regs`, `sepc`
    // included, via `set_raw_word` below — naturally overrides this rather than
    // needing to know whether that happened. Getting this backwards was a real
    // bug: it silently discarded every context switch, since the old "advance
    // after" step stomped on whatever `sepc` a switched-to thread had brought
    // with it. Found by actually running a context switch under QEMU — no unit
    // test (which never exercises this trampoline) could have caught it.
    regs[RAW_SEPC] = regs[RAW_SEPC].wrapping_add(4);

    run_handler(regs, TrapCause::Syscall);
}

/// Fills a [`TrapFrame`] from the saved registers, runs the installed handler, and writes the result back.
fn run_handler(regs: &mut [usize], cause: TrapCause) {
    let handler_addr = TRAP_HANDLER.load(Ordering::Acquire);
    debug_assert!(handler_addr != 0, "trap fired before install_trap_handler was called");
    // SAFETY: `handler_addr` was written by `install_trap_handler` from a real
    // `TrapHandler` value and never changes afterwards.
    let handler: TrapHandler = unsafe { core::mem::transmute::<usize, TrapHandler>(handler_addr) };

    let mut frame = TrapFrame::zeroed();
    // Populate the *entire* raw register file, not just mr0..mr3/tag/syscall
    // number — arch-aware kernel code (via `Hal::initial_trap_frame`'s raw
    // layout and `TrapFrame::raw_word`) saves/restores complete thread state
    // through this frame across a context switch, not just the portable
    // mr0..mr3/tag surface.
    for (i, word) in regs.iter().enumerate() {
        frame.set_raw_word(i, *word);
    }
    frame.set_mr(0, regs[REG_A0]);
    frame.set_mr(1, regs[REG_A1]);
    frame.set_mr(2, regs[REG_A2]);
    frame.set_mr(3, regs[REG_A3]);
    frame.set_tag(MessageTag::from_raw(regs[REG_A4]));
    frame.set_cause(cause);
    frame.set_syscall_number(if cause == TrapCause::Syscall { regs[REG_A7] } else { NOT_A_SYSCALL });

    handler(&mut frame);
    // The kernel ran on the trap stack: if it reached the canary, it also wrote whatever lies below the stack, and the
    // kernel's state can't be trusted. Stop, loudly, rather than carry on corrupted.
    if trap_stack_canary() != TRAP_STACK_CANARY {
        panic!("lantern-hal: the trap stack overflowed");
    }

    // Write the full raw register file back first — this is what actually
    // carries a context switch's replacement state (sepc included) into the
    // real registers. Then re-apply mr0..mr3/tag on top: a handler that didn't
    // switch context only ever touches those via `set_mr`/`set_tag`, which
    // write to `frame`'s separate `mrs`/`tag` fields, not `raw` — without this
    // second step its reply would be silently lost under the full-array
    // restore above. (A handler that *did* switch context already wrote both
    // consistently via `SavedContext`, so this is a harmless no-op then.)
    for (i, word) in regs.iter_mut().enumerate() {
        *word = frame.raw_word(i);
    }
    regs[REG_A0] = frame.mr(0);
    regs[REG_A1] = frame.mr(1);
    regs[REG_A2] = frame.mr(2);
    regs[REG_A3] = frame.mr(3);
    regs[REG_A4] = frame.tag().into_raw();
}

/// The kernel's stack while it handles a trap, in `.bss` with its size fixed in the assembly above. It was 4 KiB until a
/// CNode of 160 slots (2.5 KiB), built on it and then moved, overflowed it silently into kernel state
/// ([RFC-0031](../../lantern-rfcs/rfcs/0031-storage-a-block-driver-and-the-store-on-disk.md)).
pub const TRAP_STACK_BYTES: usize = 65536;
/// Written at the stack's lowest word, and checked after every trap.
const TRAP_STACK_CANARY: u64 = 0x5afe_57ac_c0de_f00d;
/// Every other byte of the stack, at install, so [`riscv64_trap_stack_high_water`] can tell what was ever used.
const TRAP_STACK_PAINT: u8 = 0xa5;

unsafe extern "C" {
    static lantern_hal_riscv64_trap_stack_bottom: u8;
}

fn trap_stack_bottom() -> *mut u8 {
    // Only the symbol's address is taken: the trap stack's lowest byte.
    core::ptr::addr_of!(lantern_hal_riscv64_trap_stack_bottom) as *mut u8
}

fn trap_stack_canary() -> u64 {
    // SAFETY: the stack's lowest 8 bytes, 4 KiB-aligned, inside the stack, which nothing else references.
    unsafe { core::ptr::read_volatile(trap_stack_bottom() as *const u64) }
}

/// The most of the trap stack ever used, in bytes, since [`Hal::install_trap_handler`]: a measurement, for sizing it.
pub fn riscv64_trap_stack_high_water() -> usize {
    let bottom = trap_stack_bottom();
    // SAFETY: the trap stack, read as bytes; nothing is using it while this runs in a trap handler below its frames, or
    // outside one.
    let untouched = (8..TRAP_STACK_BYTES).take_while(|&i| unsafe { core::ptr::read_volatile(bottom.add(i)) } == TRAP_STACK_PAINT).count();
    TRAP_STACK_BYTES - 8 - untouched
}

/// The `riscv64` HAL implementation.
pub struct Hardware;

impl Hal for Hardware {
    unsafe fn install_trap_handler(handler: TrapHandler) {
        debug_assert_eq!(
            TRAP_HANDLER.load(Ordering::Relaxed),
            0,
            "install_trap_handler must be called at most once"
        );
        TRAP_HANDLER.store(handler as usize, Ordering::Release);
        // SAFETY: the trap stack isn't in use before the first trap, which can't come before this function returns.
        unsafe {
            core::ptr::write_bytes(trap_stack_bottom(), TRAP_STACK_PAINT, TRAP_STACK_BYTES);
            core::ptr::write_volatile(trap_stack_bottom() as *mut u64, TRAP_STACK_CANARY);
        }

        let frame_ptr = RAW_FRAME.0.get() as usize;
        // SAFETY: caller upholds `install_trap_handler`'s contract (called once,
        // before traps are enabled). `sscratch`/`stvec` are supervisor CSRs private to
        // this hart; writing them has no effect until the next trap.
        unsafe {
            asm!("csrw sscratch, {0}", in(reg) frame_ptr, options(nomem, nostack));

            // stvec mode bits [1:0] = 00 (Direct); the entry point is 4-byte aligned
            // (`.align 4` above), so its address already has those bits clear.
            let entry_addr = lantern_hal_riscv64_trap_entry as *const () as usize;
            asm!("csrw stvec, {0}", in(reg) entry_addr, options(nomem, nostack));
            // `sstatus.FS` off would make the trap entry's first `fsd` an illegal instruction: turn the floating-point
            // unit on (Initial) for good. Every thread's FP state is saved and restored with its other registers.
            asm!("csrs sstatus, {0}", in(reg) 1usize << 13, options(nomem, nostack));
        }
    }

    fn initial_trap_frame(pc: usize, sp: usize, arg0: usize) -> TrapFrame {
        let mut frame = TrapFrame::zeroed();
        frame.set_raw_word(RAW_SEPC, pc);
        frame.set_raw_word(1, sp); // x2 = sp
        frame.set_raw_word(REG_A0, arg0);
        // `a0` has two views in a `TrapFrame` — the raw register file and the
        // portable `mr(0)` — and they must agree: the trap trampoline above
        // writes `raw` back, then re-applies `mr0..mr3` *on top*. Setting only
        // the raw word meant `arg0` survived `enter_thread` (raw only) but was
        // silently zeroed for every thread first started by a normal context
        // switch. Invisible until a second-started program actually needed a
        // nonzero `arg0` (`lantern-boot`'s runtime demo, launching
        // `keystore-service` in its signing profile).
        frame.set_mr(0, arg0);
        frame
    }

    unsafe fn enter_thread(frame: &TrapFrame) -> ! {
        let mut raw = [0usize; RAW_FRAME_WORDS];
        for (i, word) in raw.iter_mut().enumerate() {
            *word = frame.raw_word(i);
        }
        // SAFETY: `sstatus` is always writable in S-mode; clearing bits has no
        // side effect until the `sret` below actually consults them.
        unsafe {
            // Force U-mode entry explicitly, rather than relying on whatever
            // `sstatus.SPP`/`SPIE` happen to already hold (residual state from
            // whatever ran before this — OpenSBI's own setup, in practice, which
            // is not a documented guarantee this crate should depend on).
            // SPP=0 (bit 8): `sret` drops to U-mode, not back to S-mode — the
            // actual confinement boundary Phase 1's threads rely on.
            // SPIE=0 (bit 5): `sret` leaves interrupts disabled (`sstatus.SIE`
            // takes SPIE's value) — Phase 1 has no interrupt/timer handling yet
            // (`lantern-hal/STATUS.md`), so entering with them already enabled
            // is not safe. This is also the fix for the `wfi` crash `lantern-boot`
            // documented (plausibly caused by exactly this: interrupts silently
            // enabling on the first `sret`).
            asm!("csrc sstatus, {0}", in(reg) 0x120usize, options(nomem, nostack));
        }
        // SAFETY: `raw` is a fully-populated 32-word array in the same layout the
        // assembly above expects (x1..x31 then sepc); caller upholds
        // `enter_thread`'s contract that `frame` itself is validly populated.
        unsafe { lantern_hal_riscv64_enter_thread(raw.as_ptr()) }
    }

    unsafe fn activate_address_space(root: usize) {
        // SAFETY: forwarded from this function's own contract.
        unsafe { crate::riscv64_paging::activate(root) }
    }

    fn monotonic_time_ns() -> u64 {
        let ticks: u64;
        // SAFETY: `time` is a read-only CSR, unprivileged-readable in S-mode
        // under standard OpenSBI `mcounteren` configuration (this crate already
        // assumes OpenSBI's default setup elsewhere — see
        // `install_trap_handler`'s module doc). The read has no side effects.
        unsafe {
            asm!("rdtime {0}", out(reg) ticks, options(nomem, nostack));
        }
        ticks_to_ns(ticks, platform().timebase_hz)
    }

    unsafe fn init_interrupts() {
        // SAFETY: S-mode CSR and PLIC writes, before any thread runs. SIE is cleared first, so nothing is taken in
        // S-mode (ADR-0010); then `sie` lets the PLIC's lines and the timer interrupt U-mode, which RISC-V always
        // takes regardless of `sstatus.SIE`.
        unsafe {
            asm!("csrc sstatus, {0}", in(reg) 1usize << 1, options(nomem, nostack));
        }
        match intc() {
            Intc::Plic(p) => p.init(),
            Intc::Aia(a) => a.init(),
        }
        Self::set_timer(u64::MAX);
        // SAFETY: as above.
        unsafe {
            asm!("csrs sie, {0}", in(reg) (1usize << IRQ_S_EXTERNAL) | (1usize << IRQ_S_TIMER), options(nomem, nostack));
        }
    }

    fn interrupt_line_enable(line: u32, enabled: bool) {
        match intc() {
            Intc::Plic(p) => p.enable_line(line, enabled),
            Intc::Aia(a) => a.enable_line(line, enabled),
        }
    }

    fn interrupt_claim() -> u32 {
        match intc() {
            Intc::Plic(p) => p.claim_line(),
            Intc::Aia(a) => a.claim_line(),
        }
    }

    fn interrupt_complete(line: u32) {
        match intc() {
            Intc::Plic(p) => p.complete_line(line),
            Intc::Aia(a) => a.complete_line(line),
        }
    }

    fn set_timer(deadline_ns: u64) {
        let ticks = if deadline_ns == u64::MAX { u64::MAX } else { ns_to_ticks(deadline_ns, platform().timebase_hz) };
        // SAFETY: an SBI call to M-mode firmware. `sbi_set_timer` clobbers only `a0` and `a1`, which are declared.
        unsafe {
            asm!(
                "ecall",
                inout("a0") ticks as usize => _,
                lateout("a1") _,
                in("a6") SBI_SET_TIMER,
                in("a7") SBI_EXT_TIME,
                options(nostack),
            );
        }
    }

    fn timer_pending() -> bool {
        let sip: usize;
        // SAFETY: `sip` is always readable in S-mode.
        unsafe { asm!("csrr {0}, sip", out(reg) sip, options(nomem, nostack)) };
        sip & (1 << IRQ_S_TIMER) != 0
    }

    fn ram_window(paddr: usize, len: usize) -> Option<usize> {
        use crate::riscv64_paging::DIRECT_MAP_BASE;
        let (ram, span) = crate::riscv64_paging::direct_map();
        let end = paddr.checked_add(len)?;
        if paddr < ram || end > ram + span {
            return None;
        }
        let satp: usize;
        // SAFETY: `satp` is always readable in S-mode.
        unsafe { asm!("csrr {0}, satp", out(reg) satp, options(nomem, nostack)) };
        // Bare mode: physical addresses are the addresses.
        Some(if satp >> 60 == 0 { paddr } else { DIRECT_MAP_BASE + (paddr - ram) })
    }

    fn wait_for_interrupt() {
        // SAFETY: `wfi` only waits. With `sstatus.SIE` clear, a pending interrupt wakes it without being taken: the
        // RISC-V spec makes `wfi` independent of the global enables. That's why the kernel's idle loop needs no
        // S-mode trap entry.
        unsafe { asm!("wfi", options(nomem, nostack)) };
    }}

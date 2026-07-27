//! The trap-frame contract fixed by
//! [ADR-0008](../../lantern-rfcs/adr/0008-kernel-syscall-ipc-abi.md): the portable
//! kernel core touches only these fixed names — `mr0..mr3`, the message tag, the
//! syscall number — and never a raw physical register or offset.

/// Number of fast-path message registers in the ADR-0008 syscall ABI.
pub const MR_COUNT: usize = 4;

/// Bit 0 of [`MessageTag::flags`] is the error flag on return, per ADR-0008.
pub const FLAG_ERROR: u16 = 1 << 0;

/// The register state the HAL hands to the portable kernel core on syscall/trap
/// entry, and takes back — possibly modified — on return.
///
/// Layout beyond the accessor methods below is architecture-specific and owned by
/// each `arch` module; the portable core never interprets `raw` directly, per
/// ADR-0008 ("the portable kernel core never references an ISA register name
/// directly").
pub struct TrapFrame {
    syscall_num: usize,
    tag: usize,
    mrs: [usize; MR_COUNT],
    /// Opaque save area for the rest of the interrupted thread's register file
    /// (program counter, stack pointer, callee-saved registers, ...). Sized
    /// generously for a Phase 1 prototype; exact contents are set by each
    /// architecture's trap entry, not by the portable core.
    raw: [usize; 32],
}

impl TrapFrame {
    /// A zeroed frame, for an architecture's trap entry to build a fresh thread's
    /// initial state or a scratch save area.
    pub const fn zeroed() -> Self {
        Self { syscall_num: 0, tag: 0, mrs: [0; MR_COUNT], raw: [0; 32] }
    }

    pub fn syscall_number(&self) -> usize {
        self.syscall_num
    }

    pub fn set_syscall_number(&mut self, value: usize) {
        self.syscall_num = value;
    }

    pub fn tag(&self) -> MessageTag {
        MessageTag::from_raw(self.tag)
    }

    pub fn set_tag(&mut self, tag: MessageTag) {
        self.tag = tag.into_raw();
    }

    /// Read fast-path message register `index` (`0..MR_COUNT`).
    ///
    /// # Panics
    /// Panics if `index >= MR_COUNT`. An out-of-range index here is a kernel bug,
    /// not caller-controlled input — caller-supplied message length is validated
    /// against `MAX_MSG_WORDS` before the portable core ever indexes an `mr`, per
    /// ADR-0008.
    pub fn mr(&self, index: usize) -> usize {
        self.mrs[index]
    }

    pub fn set_mr(&mut self, index: usize, value: usize) {
        self.mrs[index] = value;
    }

    /// Raw architecture-owned save-area word `index`. Only `arch` modules should
    /// call this; the portable kernel core has no fixed name for anything stored
    /// here and must not depend on its layout.
    pub fn raw_word(&self, index: usize) -> usize {
        self.raw[index]
    }

    pub fn set_raw_word(&mut self, index: usize, value: usize) {
        self.raw[index] = value;
    }
}

/// The ADR-0008 message tag: `{ label: u32, length: u12, extra_caps: u4, flags: u16 }`,
/// packed into one 64-bit machine word. Field widths sum to 64 bits, matching the
/// `usize` word size on both Phase 1 targets (`riscv64`, `x86-64`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MessageTag {
    pub label: u32,
    pub length: u16,
    pub extra_caps: u8,
    pub flags: u16,
}

const LENGTH_BITS: u32 = 12;
const EXTRA_CAPS_BITS: u32 = 4;
const FLAGS_BITS: u32 = 16;

const LENGTH_MASK: usize = (1 << LENGTH_BITS) - 1;
const EXTRA_CAPS_MASK: usize = (1 << EXTRA_CAPS_BITS) - 1;
const FLAGS_MASK: usize = (1 << FLAGS_BITS) - 1;

impl MessageTag {
    // `inline(always)`: called from both S-mode code (the riscv64 trap
    // trampoline, on every trap) and, via `lantern-boot`'s `.user_text`-section
    // thread bodies, real U-mode code. Sv39 can't mark a single physical page
    // fetchable from both privilege levels at once (see `paging.rs`'s module
    // doc) — inlining means there's no separately-callable symbol that would
    // have to live on one side or the other.
    #[inline(always)]
    pub const fn from_raw(raw: usize) -> Self {
        let label = (raw >> (LENGTH_BITS + EXTRA_CAPS_BITS + FLAGS_BITS)) as u32;
        let length = ((raw >> (EXTRA_CAPS_BITS + FLAGS_BITS)) & LENGTH_MASK) as u16;
        let extra_caps = ((raw >> FLAGS_BITS) & EXTRA_CAPS_MASK) as u8;
        let flags = (raw & FLAGS_MASK) as u16;
        Self { label, length, extra_caps, flags }
    }

    #[inline(always)]
    pub const fn into_raw(self) -> usize {
        ((self.label as usize) << (LENGTH_BITS + EXTRA_CAPS_BITS + FLAGS_BITS))
            | ((self.length as usize & LENGTH_MASK) << (EXTRA_CAPS_BITS + FLAGS_BITS))
            | ((self.extra_caps as usize & EXTRA_CAPS_MASK) << FLAGS_BITS)
            | (self.flags as usize & FLAGS_MASK)
    }

    pub const fn is_error(&self) -> bool {
        self.flags & FLAG_ERROR != 0
    }
}

/// The portable kernel's trap/syscall dispatch entry point, installed via
/// [`crate::Hal::install_trap_handler`]. Called by the architecture's trap entry
/// with interrupts masked and the full register file already saved into `frame`.
pub type TrapHandler = fn(frame: &mut TrapFrame);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_tag_roundtrips_through_raw() {
        let tag = MessageTag { label: 0xDEAD_BEEF, length: 0xABC, extra_caps: 0xF, flags: 0x1234 };
        assert_eq!(MessageTag::from_raw(tag.into_raw()), tag);
    }

    #[test]
    fn error_flag_bit_zero() {
        let ok = MessageTag { label: 0, length: 0, extra_caps: 0, flags: 0 };
        let err = MessageTag { label: 0, length: 0, extra_caps: 0, flags: FLAG_ERROR };
        assert!(!ok.is_error());
        assert!(err.is_error());
    }

    #[test]
    fn trap_frame_mr_accessors() {
        let mut frame = TrapFrame::zeroed();
        frame.set_mr(2, 42);
        assert_eq!(frame.mr(2), 42);
        assert_eq!(frame.mr(0), 0);
    }
}

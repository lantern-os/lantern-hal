//! Sv39 page tables (RISC-V Privileged Architecture spec) — `riscv64`'s paging format.
//!
//! **Not part of the [`crate::Hal`] trait.** `lantern-kernel` doesn't manage per-object
//! VSpace/Frame capabilities yet (`lantern-kernel/STATUS.md`) — for now, `lantern-boot`
//! builds page tables directly, so these are plain `riscv64`-specific functions it calls,
//! the same way it already owns `entry.rs`/`uart.rs`. Only address-space *activation*
//! (needed by `lantern-kernel`'s portable context-switch code) goes through the `Hal`
//! trait (see `riscv64.rs`'s `activate_address_space`) — building a table doesn't.
//!
//! Sv39: 3-level page table, 4 KiB pages, 39-bit virtual addresses (`VPN[2]:VPN[1]:VPN[0]:
//! offset`, 9+9+9+12 bits), 56-bit physical addresses in this implementation's PTEs
//! (`PPN[2]:PPN[1]:PPN[0]`, 26+9+9 bits) — the common minimal choice for a first riscv64
//! MMU (matches what most bare-metal riscv64-under-QEMU projects start with; Sv48/Sv57
//! add address-space range Phase 1 has no use for yet).
//!
//! **Identity-mapped only, for now.** Every physical address this module is given is also
//! used as the corresponding virtual address (`lantern-boot`'s own convention, not
//! inherent to Sv39) — there is no higher-half kernel or separate virtual layout yet.

pub const PAGE_SIZE: usize = 4096;
pub const PAGE_SHIFT: usize = 12;
const ENTRIES_PER_TABLE: usize = 512;
const LEVELS: usize = 3;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PteFlags(u64);

impl PteFlags {
    pub const VALID: PteFlags = PteFlags(1 << 0);
    pub const READ: PteFlags = PteFlags(1 << 1);
    pub const WRITE: PteFlags = PteFlags(1 << 2);
    pub const EXECUTE: PteFlags = PteFlags(1 << 3);
    /// Accessible from U-mode. Without this bit a page is invisible to user-mode
    /// code even though it's present in the active table (S-mode ignores this bit
    /// for its own accesses) — this is Sv39's actual confinement mechanism.
    pub const USER: PteFlags = PteFlags(1 << 4);
    const ACCESSED: PteFlags = PteFlags(1 << 6);
    const DIRTY: PteFlags = PteFlags(1 << 7);

    pub const fn union(self, other: PteFlags) -> PteFlags {
        PteFlags(self.0 | other.0)
    }
}

#[derive(Clone, Copy)]
#[repr(transparent)]
struct Pte(u64);

impl Pte {
    const fn empty() -> Self {
        Self(0)
    }

    fn is_valid(self) -> bool {
        self.0 & PteFlags::VALID.0 != 0
    }

    /// A leaf has at least one of R/W/X set; a branch (pointer to the next table
    /// level) has none — Sv39's own encoding of "is this the last level."
    fn is_leaf(self) -> bool {
        self.0 & (PteFlags::READ.0 | PteFlags::WRITE.0 | PteFlags::EXECUTE.0) != 0
    }

    /// The physical page number this entry names — either the next-level table
    /// (branch) or the mapped page itself (leaf).
    fn ppn(self) -> usize {
        ((self.0 >> 10) & 0x0FFF_FFFF_FFFF) as usize
    }

    fn branch(table_paddr: usize) -> Self {
        Self((((table_paddr >> PAGE_SHIFT) as u64) << 10) | PteFlags::VALID.0)
    }

    /// Sets `A` (accessed) and `D` (dirty) unconditionally, not just `V` — per the
    /// RISC-V address-translation algorithm, hardware faults on `pte.a = 0` (or a
    /// store to `pte.d = 0`) unless hardware A/D management (Svadu) is both
    /// implemented *and* enabled (`menvcfg.ADUE`), which nothing in this Phase 1
    /// boot chain does. Phase 1 has no lazy A/D tracking (no swapping, no
    /// copy-on-write yet) — marking every leaf pre-accessed/dirty up front is a
    /// correct, standard simplification for that, not a shortcut with a real
    /// downside yet. Missing this was a real, hard-to-diagnose bug: every access
    /// through any table built here would page-fault regardless of how correct
    /// V/R/W/X/PPN were, since S-mode ignores the U bit but not A/D.
    fn leaf(paddr: usize, flags: PteFlags) -> Self {
        let flags = flags.union(PteFlags::VALID).union(PteFlags::ACCESSED).union(PteFlags::DIRTY);
        Self((((paddr >> PAGE_SHIFT) as u64) << 10) | flags.0)
    }
}

/// One page-table level: 512 eight-byte entries, exactly one 4 KiB page.
#[repr(C, align(4096))]
pub struct PageTable {
    entries: [Pte; ENTRIES_PER_TABLE],
}

impl PageTable {
    pub const fn empty() -> Self {
        Self { entries: [Pte::empty(); ENTRIES_PER_TABLE] }
    }
}

impl Default for PageTable {
    fn default() -> Self {
        Self::empty()
    }
}

fn vpn(vaddr: usize, level: usize) -> usize {
    (vaddr >> (PAGE_SHIFT + 9 * level)) & 0x1FF
}

/// Maps one 4 KiB page: `vaddr` (identity: also used as the physical address other
/// mappings and `lantern-boot` itself assume) to `paddr` with `flags`, walking/
/// allocating intermediate levels as needed via `alloc_frame` (must return a
/// zeroed, page-aligned physical page each call — a page-table level's "all
/// entries invalid" state is the all-zero byte pattern, so a non-zeroed page would
/// look like a table full of garbage valid entries).
///
/// # Safety
/// `root` must point at a valid, currently-unmapped-or-consistently-mapped page
/// table; `alloc_frame` must always return a distinct, valid, zeroed physical
/// page (never one already in use) for as long as this table exists.
pub unsafe fn map(
    root: *mut PageTable,
    vaddr: usize,
    paddr: usize,
    flags: PteFlags,
    alloc_frame: &mut dyn FnMut() -> usize,
) {
    let mut table = root;
    for level in (1..LEVELS).rev() {
        let index = vpn(vaddr, level);
        // SAFETY: `table` is valid per this function's contract (initially
        // `root`, and thereafter a branch PTE's own PPN, which `alloc_frame`
        // guaranteed was a valid zeroed page when it was created below).
        let pte = unsafe { &mut (*table).entries[index] };
        if !pte.is_valid() {
            let new_table = alloc_frame();
            *pte = Pte::branch(new_table);
        }
        table = (pte.ppn() << PAGE_SHIFT) as *mut PageTable;
    }
    let index = vpn(vaddr, 0);
    // SAFETY: forwarded from this function's own contract.
    unsafe {
        (*table).entries[index] = Pte::leaf(paddr, flags);
    }
}

/// 2 MiB — the size of an Sv39 "megapage": a leaf placed one level up from a
/// regular 4 KiB page (at L1 instead of L0), so a walk only ever needs one
/// branch hop (root -> L1 leaf) instead of two (root -> L1 branch -> L0 leaf).
/// See [`map_megapage`]'s doc for why `lantern-boot` needs this.
pub const MEGAPAGE_SIZE: usize = 1 << (PAGE_SHIFT + 9);

/// Maps one 2 MiB megapage: `vaddr`/`paddr` (identity; both must be
/// [`MEGAPAGE_SIZE`]-aligned) with `flags`, as a leaf at Sv39's L1 level —
/// i.e. only *one* branch hop (root -> L1), never touching L0 at all.
///
/// Exists because of an empirically-confirmed limitation in this project's QEMU
/// environment (Debian's `qemu-system-riscv64` 10.2.1): a full 3-level Sv39 walk
/// (root -branch-> L1 -branch-> L0 -leaf->) reliably page-faults immediately
/// after `sfence.vma`, on *every* subsequent instruction fetch — even though the
/// resulting page table is independently verified byte-correct (this project's
/// own [`translate`], and raw physical-memory dumps via the QEMU monitor, both
/// confirm every PTE at every level exactly matches what the spec requires).
/// A walk with only *one* branch hop (root -branch-> L1 -leaf->, i.e. exactly
/// what this function builds) was confirmed to work reliably in the same
/// environment, isolating the break to specifically the second hop (L1 branch
/// -> L0) — not this crate's PTE construction. See `lantern-boot/STATUS.md` for
/// the full debugging record. [`map`]/4 KiB pages remain correct (host-tested)
/// and are what a real target should use; this function is `lantern-boot`'s
/// documented workaround until the QEMU issue is root-caused upstream or this
/// environment's QEMU is updated.
///
/// # Safety
/// As [`map`]'s.
pub unsafe fn map_megapage(
    root: *mut PageTable,
    vaddr: usize,
    paddr: usize,
    flags: PteFlags,
    alloc_frame: &mut dyn FnMut() -> usize,
) {
    debug_assert_eq!(vaddr % MEGAPAGE_SIZE, 0);
    debug_assert_eq!(paddr % MEGAPAGE_SIZE, 0);
    let l2_index = vpn(vaddr, 2);
    // SAFETY: forwarded from this function's own contract.
    let l2_pte = unsafe { &mut (*root).entries[l2_index] };
    if !l2_pte.is_valid() {
        let new_table = alloc_frame();
        *l2_pte = Pte::branch(new_table);
    }
    let l1_table = (l2_pte.ppn() << PAGE_SHIFT) as *mut PageTable;
    let l1_index = vpn(vaddr, 1);
    // SAFETY: `l1_table` is valid per the same reasoning as `map`'s `table`.
    unsafe {
        (*l1_table).entries[l1_index] = Pte::leaf(paddr, flags);
    }
}

/// Walks `root` for `vaddr`, returning its mapped physical address if present —
/// used only to check whether a mapping exists (e.g. asserting isolation:
/// confirming a table does *not* map another thread's private page), not on any
/// hot path. Stops at whichever level holds a leaf, so both [`map`]'s 4 KiB
/// pages and [`map_megapage`]'s 2 MiB ones translate correctly.
///
/// # Safety
/// `root` must point at a valid page table (as built by [`map`]/[`map_megapage`]).
pub unsafe fn translate(root: *const PageTable, vaddr: usize) -> Option<usize> {
    let mut table = root;
    for level in (1..LEVELS).rev() {
        let index = vpn(vaddr, level);
        // SAFETY: forwarded from this function's own contract; see `map`'s
        // identical reasoning for why `table` stays valid across levels.
        let pte = unsafe { (*table).entries[index] };
        if !pte.is_valid() {
            return None;
        }
        if pte.is_leaf() {
            let page_size = 1usize << (PAGE_SHIFT + 9 * level);
            return Some((pte.ppn() << PAGE_SHIFT) | (vaddr & (page_size - 1)));
        }
        table = (pte.ppn() << PAGE_SHIFT) as *const PageTable;
    }
    let index = vpn(vaddr, 0);
    // SAFETY: forwarded from this function's own contract.
    let pte = unsafe { (*table).entries[index] };
    if !pte.is_valid() || !pte.is_leaf() {
        return None;
    }
    Some((pte.ppn() << PAGE_SHIFT) | (vaddr & (PAGE_SIZE - 1)))
}

/// Activates `root` as the current address space (`satp`, Sv39 mode, ASID 0 — Phase
/// 1 doesn't use ASIDs, so every switch takes the simple, always-correct
/// full-TLB-flush path rather than the ASID-tagged fast path) and flushes the TLB.
///
/// # Safety
/// `root` must be a valid Sv39 root page table, built by [`map`], that (at least)
/// maps the code currently executing — including this function's own return
/// address — or execution faults immediately after `sfence.vma`.
///
/// Only this function needs `target_arch = "riscv64"` (real `csrw`/`sfence.vma`
/// can't assemble for any other target) — everything else in this module is plain
/// Rust bit manipulation, kept portable specifically so `cargo test` can exercise
/// the tricky part (table walking, PTE packing) on the host, the same way
/// `trap.rs`'s `MessageTag` packing is tested without needing a `riscv64` target.
#[cfg(target_arch = "riscv64")]
pub unsafe fn activate(root_paddr: usize) {
    let satp = (8usize << 60) | (root_paddr >> PAGE_SHIFT);
    // SAFETY: forwarded from this function's own contract.
    unsafe {
        core::arch::asm!(
            "csrw satp, {0}",
            "sfence.vma",
            in(reg) satp,
            options(nomem, nostack)
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_then_translate_roundtrips() {
        let mut root = PageTable::empty();
        let mut frames = [PageTable::empty(), PageTable::empty()];
        let mut next_frame = 0;
        let mut alloc = || {
            let table = &mut frames[next_frame];
            next_frame += 1;
            table as *mut PageTable as usize
        };

        let vaddr = 0x8020_3000usize;
        let paddr = 0x8020_3000usize;
        // SAFETY: `root`/`frames` are local, valid, and `alloc` never reuses a frame.
        unsafe {
            map(&mut root, vaddr, paddr, PteFlags::READ.union(PteFlags::WRITE), &mut alloc);
        }

        // SAFETY: `root` was just built by `map` above.
        let translated = unsafe { translate(&root, vaddr) };
        assert_eq!(translated, Some(paddr));
    }

    #[test]
    fn unmapped_address_translates_to_none() {
        let root = PageTable::empty();
        // SAFETY: `root` is a validly-constructed (empty) page table.
        assert_eq!(unsafe { translate(&root, 0x1000) }, None);
    }

    #[test]
    fn translate_preserves_the_page_offset() {
        let mut root = PageTable::empty();
        let mut frames = [PageTable::empty(), PageTable::empty()];
        let mut next_frame = 0;
        let mut alloc = || {
            let table = &mut frames[next_frame];
            next_frame += 1;
            table as *mut PageTable as usize
        };
        // SAFETY: as above.
        unsafe {
            map(&mut root, 0x8020_3000, 0x8030_3000, PteFlags::READ, &mut alloc);
            assert_eq!(translate(&root, 0x8020_3123), Some(0x8030_3123));
        }
    }

    #[test]
    fn megapage_map_then_translate_roundtrips() {
        let mut root = PageTable::empty();
        let mut frames = [PageTable::empty()];
        let mut next_frame = 0;
        let mut alloc = || {
            let table = &mut frames[next_frame];
            next_frame += 1;
            table as *mut PageTable as usize
        };

        let vaddr = 0x8040_0000usize; // 2 MiB-aligned
        let paddr = 0x8040_0000usize;
        // SAFETY: `root`/`frames` are local, valid, and `alloc` never reuses a frame.
        unsafe {
            map_megapage(&mut root, vaddr, paddr, PteFlags::READ.union(PteFlags::WRITE), &mut alloc);
            assert_eq!(translate(&root, vaddr), Some(paddr));
            // A megapage's own offset can be up to 2 MiB, not just 4 KiB.
            assert_eq!(translate(&root, vaddr + 0x1_2345), Some(paddr + 0x1_2345));
        }
    }
}

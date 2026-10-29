//! USB full-speed device (CDC ACM virtual COM port) for the AT32F403A's
//! USBFS cell, mirroring the vendor `at32f403a_407_usb.c` /
//! `usbd_int.c` register sequences exactly.
//!
//! Core facts taken from the vendor source (this cell is F103-*like*, not
//! F103-identical):
//! - **EPTn TC flags (TXTC/RXTC/SETUPTC) are write-zero-to-clear**:
//!   `USB_CLEAR_TXTC(eptn) = (ept &= 0xFF7F & 0x8F8F)`. The clear also
//!   zeroes the endpoint's own status bits, which is the invariant that
//!   makes the status-set idiom work (below).
//! - **Endpoint status (TXSTS bits 5:4, RXSTS bits 13:12) toggles on
//!   written 1s**: each written 1 flips that bit. The vendor
//!   `USB_SET_TXSTS` computes `current ^ new` implicitly (its masked
//!   read carries the current status), landing on the desired state;
//!   this driver computes the delta explicitly so the result is correct
//!   from any starting state.
//! - **INTSTS is write-zero-to-clear**: `usb_flag_clear` writes `~flag`.
//!   `INTSTS.TC` (bit 15) clears when the endpoint's own TC clears, so the
//!   IRQ drains pending endpoint events in a loop.
//! - **Buffer table**: slot *n* at packet-SRAM byte offset `n * 16`, with
//!   TXADDR at +0, TXCOUNT at +4, RXADDR at +8, RXCOUNT at +12. The
//!   address fields MUST be programmed (the vendor's reset handler calls
//!   `usbd_ept_buf_custom_define` for both EP0 directions) — leaving them
//!   at 0 points every buffer at the table itself.
//! - RX counts use the F103 block encoding (`USB_SET_RXLEN`: BLK2/BLK32);
//!   after an OUT transaction the RXCOUNT slot's low 10 bits hold the
//!   received bytes (`USB_GET_RX_LEN` masks `& 0x3ff`).
//! - SETUP arrives as EPT0 SETUPTC *and* RXTC; the vendor reads the setup
//!   packet, clears RXTC, and dispatches.
//! - Packet SRAM at `0x4000_6000` (512 bytes, CRM MISC1.USBBUFS = 0).
//! - The D+ pull-up enables with CFG.PUO = 0 (active-low, vendor-written).
//!
//! One CDC function is served: EP0 control, EP1 bulk IN/OUT. Shared state
//! sits behind a critical-section mutex; the task side polls.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// USBFS register base.
const USBFS_BASE: usize = 0x4000_5c00;
/// Packet SRAM base (CRM MISC1.USBBUFS = 0).
const PACKET_RAM: *mut u16 = 0x4000_6000 as *mut u16;

// Register offsets.
const CTRL: usize = 0x40;
const INTSTS: usize = 0x44;
const DEVADDR: usize = 0x4c;
const BUFTBL: usize = 0x50;
const CFG: usize = 0x60;

// CTRL bits.
const CTRL_CSRST: u32 = 1 << 0;
const CTRL_DISUSB: u32 = 1 << 1;
const CTRL_RSTIEN: u32 = 1 << 10;
const CTRL_TCIEN: u32 = 1 << 15;
// CFG bits.
const CFG_PUO: u32 = 1 << 1;
// DEVADDR bits.
const DEVADDR_CEN: u32 = 1 << 7;
// INTSTS bits (write-zero-to-clear).
const INTS_EPT_NUM: u32 = 0xf;
#[allow(dead_code)]
const INTS_INOUT: u32 = 1 << 4;
const INTS_RST: u32 = 1 << 10;
const INTS_TC: u32 = 1 << 15;

// EPTn bits (vendor constants, verbatim).
const EPT_EPTADDR: u32 = 0xf;
const EPT_TXTC: u32 = 0x80; // bit 7
const EPT_EXF: u32 = 0x100; // bit 8
const EPT_TRANS_TYPE: u32 = 0x600; // bits 10:9
const EPT_SETUPTC: u32 = 0x800; // bit 11
const EPT_RXTC: u32 = 0x8000; // bit 15

/// Fields preserved across status writes (vendor `USB_EPT_BIT_MASK`,
/// minus the status fields, which the delta form writes explicitly).
const EPT_BIT_MASK: u32 =
    EPT_TXTC | EPT_SETUPTC | EPT_EPTADDR | EPT_EXF | EPT_RXTC | EPT_TRANS_TYPE;
/// Vendor `USB_TX_MASK` / `USB_RX_MASK`.
/// Vendor `USB_CLEAR_TXTC` / `USB_CLEAR_RXTC` / `USB_CLEAR_SETUPTC` masks
/// (the value ANDed into the register: zeros clear the flag and reset the
/// endpoint's own status field to DISABLE).
const CLEAR_TXTC_MASK: u32 = 0xff7f & EPT_BIT_MASK;
const CLEAR_RXTC_MASK: u32 = 0x7fff & EPT_BIT_MASK;
const CLEAR_SETUPTC_MASK: u32 = 0xf7ff & EPT_BIT_MASK;

// Status encodings.
#[allow(dead_code)]
const STS_DISABLE: u32 = 0b00;
const STS_STALL: u32 = 0b01;
const STS_NAK: u32 = 0b10;
const STS_VALID: u32 = 0b11;

// Transfer types.
const TYPE_BULK: u32 = 0b00 << 9;
const TYPE_CONTROL: u32 = 0b01 << 9;

/// Max packet size everywhere (full-speed bulk).
const EP_SIZE: u16 = 64;

/// Buffer-table slots (byte offset = endpoint * 16: TXADDR +0, TXCOUNT
/// +4, RXADDR +8, RXCOUNT +12) and packet buffers. 2 slots + 4 × 64-byte
/// buffers = 288 bytes of the 512 available.
const SLOT: [usize; 2] = [0x00, 0x10];
const BUF_EP0_TX: usize = 0x20;
const BUF_EP0_RX: usize = 0x60;
const BUF_EP1_TX: usize = 0xa0;
const BUF_EP1_RX: usize = 0xe0;
/// TX/RX packet-buffer addresses per endpoint.
const BUF_SLOT_TX: [usize; 2] = [BUF_EP0_TX, BUF_EP1_TX];
const BUF_SLOT_RX: [usize; 2] = [BUF_EP0_RX, BUF_EP1_RX];

// --- Raw register access ---

#[inline]
fn reg_ptr(offset: usize) -> *mut u32 {
    (USBFS_BASE + offset) as *mut u32
}

#[inline]
fn reg_read(offset: usize) -> u32 {
    // SAFETY: offsets are inside the USBFS register block by construction.
    unsafe { reg_ptr(offset).read_volatile() }
}

#[inline]
fn reg_write(offset: usize, value: u32) {
    // SAFETY: offsets are inside the USBFS register block by construction.
    unsafe {
        reg_ptr(offset).write_volatile(value);
    }
}

#[inline]
fn ept_read(index: usize) -> u32 {
    reg_read(index * 4)
}

#[inline]
fn ept_write(index: usize, value: u32) {
    reg_write(index * 4, value);
}

/// All Write-0-to-Clear flags in EPTn; written as 1 to preserve whatever
/// fired between the read and the write.
const EPT_W0C_MASK: u32 = EPT_TXTC | EPT_SETUPTC | EPT_RXTC;

/// Set one endpoint's TX status to `desired`, from any current state.
///
/// The status field toggles on written 1s: each written 1 flips that
/// bit, each written 0 leaves it. Landing on `desired` therefore needs
/// the written bits to be the DELTA (`current ^ desired`) — not
/// `desired` itself. (The vendor `USB_SET_TXSTS` macro gets this right
/// by accident of composition: `epsts = reg & MASK` already carries the
/// current status, so `epsts ^= new_sts` leaves the delta in the field.
/// Writing the delta against the raw register — or equivalently writing
/// `desired` against the masked register — double-cancels and toggles
/// by `desired`: NAK (10) toggled by VALID (11) = 01 = STALL, which
/// stalled EP0's receiver at init through five rounds of "fixes".)
fn set_tx_status(index: usize, desired: u32) {
    let raw = ept_read(index);
    let current = (raw >> 4) & 0b11;
    // Invariant: everything except the toggle fields (status bits and
    // data toggles) — a written 0 leaves a toggle field unchanged.
    let invariant = raw & EPT_BIT_MASK;
    let toggle = (current ^ desired) << 4; // TXSTS lives in bits 5:4
    ept_write(index, invariant | toggle | EPT_W0C_MASK);
}

/// Set one endpoint's RX status to `desired`, from any current state
/// (semantics as [`set_tx_status`]).
fn set_rx_status(index: usize, desired: u32) {
    let raw = ept_read(index);
    let current = (raw >> 12) & 0b11;
    let invariant = raw & EPT_BIT_MASK;
    let toggle = (current ^ desired) << 12; // RXSTS lives in bits 13:12
    ept_write(index, invariant | toggle | EPT_W0C_MASK);
}

/// Vendor `USB_CLEAR_TXTC(eptn)`.
fn clear_txtc(index: usize) {
    ept_write(index, ept_read(index) & CLEAR_TXTC_MASK);
}

/// Vendor `USB_CLEAR_RXTC(eptn)`.
fn clear_rxtc(index: usize) {
    ept_write(index, ept_read(index) & CLEAR_RXTC_MASK);
}

/// Vendor `USB_CLEAR_SETUPTC(eptn)`.
fn clear_setuptc(index: usize) {
    ept_write(index, ept_read(index) & CLEAR_SETUPTC_MASK);
}

/// Vendor `USB_SET_TRANS_TYPE(eptn, type)`.
fn set_trans_type(index: usize, trans_type: u32) {
    let value = (ept_read(index) & EPT_BIT_MASK & !EPT_TRANS_TYPE) | trans_type;
    ept_write(index, value);
}

/// Vendor `USB_SET_EPT_ADDRESS(eptn, address)`: the slot number.
fn set_ept_slot(index: usize) {
    let value = (ept_read(index) & EPT_BIT_MASK & !EPT_EPTADDR) | index as u32;
    ept_write(index, value);
}

// --- Packet SRAM access ---

#[inline]
fn packet_half(offset: usize) -> u16 {
    // SAFETY: offsets are within the 512-byte packet SRAM by construction.
    unsafe { PACKET_RAM.add(offset / 2).read_volatile() }
}

/// Write a 32-bit value as two little-endian halfwords (the vendor's
/// table macros write u32).
#[inline]
fn set_packet_word(offset: usize, value: u32) {
    set_packet_half(offset, value as u16);
    set_packet_half(offset + 2, (value >> 16) as u16);
}

#[inline]
fn set_packet_half(offset: usize, value: u16) {
    // SAFETY: offsets are within the 512-byte packet SRAM by construction.
    unsafe {
        PACKET_RAM.add(offset / 2).write_volatile(value);
    }
}

/// Read `len` bytes of packet SRAM (buffers are at most 64 bytes).
fn packet_bytes(offset: usize, len: usize) -> [u8; 64] {
    let mut out = [0u8; 64];
    for i in 0..len.div_ceil(2) {
        let half = packet_half(offset + 2 * i);
        if 2 * i < len {
            out[2 * i] = half as u8;
        }
        if 2 * i + 1 < len {
            out[2 * i + 1] = (half >> 8) as u8;
        }
    }
    out
}

/// Vendor `usb_write_packet`: bytes into packet SRAM.
fn write_packet(offset: usize, bytes: &[u8]) {
    let mut i = 0;
    while i + 1 < bytes.len() {
        set_packet_half(offset + i, u16::from(bytes[i]) | (u16::from(bytes[i + 1]) << 8));
        i += 2;
    }
    if i < bytes.len() {
        set_packet_half(offset + i, u16::from(bytes[i]));
    }
}

/// Vendor `USB_SET_TXLEN`: a 32-bit write whose upper halfword is zero.
/// The upper halves of the table fields must be driven to 0 like the
/// vendor does — packet SRAM is not guaranteed to zero-initialize, and a
/// stale upper half in a count field feeds the DMA engine garbage.
fn set_tx_count(slot: usize, len: u16) {
    set_packet_word(slot + 4, u32::from(len));
}

/// Vendor `USB_GET_RX_LEN`: received byte count after an OUT.
fn rx_count(slot: usize) -> u16 {
    packet_half(slot + 12) & 0x3ff
}

/// Vendor `USB_SET_RXLEN`: F103 block encoding (BLK2/BLK32).
fn rx_count_encoding(len: u16) -> u16 {
    if len > 62 {
        let mut blocks = len >> 5;
        if len & 0x1f == 0 {
            blocks -= 1;
        }
        (blocks << 10) | 0x8000
    } else {
        let mut blocks = len >> 1;
        if len.is_multiple_of(2) {
            blocks += 1;
        }
        blocks << 10
    }
}

/// Vendor `USB_SET_RXLEN(eptn, len)` for a single-buffer endpoint.
fn set_rx_count(slot: usize, len: u16) {
    set_packet_word(slot + 12, u32::from(rx_count_encoding(len)));
}

// --- Descriptors (CDC 1.1, single function, bus-powered) ---

const DEVICE_DESC: [u8; 18] = [
    0x12, 0x01, 0x00, 0x02, // DEVICE, bcdUSB 2.0
    0x02, 0x00, 0x00, // class CDC
    EP_SIZE as u8, 0x3c, 0x2e, // EP0 size, VID 0x2e3c
    0x59, 0x58, // PID 0x5859 default (open firmware CDC)
    0x00, 0x01, 1, 2, 0, 0x01,
];
/// Byte offset of the PID halfword inside DEVICE_DESC.
const DEVICE_DESC_PID_OFFSET: usize = 8;

const CONFIG_DESC: [u8; 60] = [
    // Configuration (9): 100 mA, bus-powered.
    0x09, 0x02, 0x3c, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
    // Interface 0: CDC control (9).
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    // Header functional (5).
    0x05, 0x24, 0x00, 0x10, 0x01,
    // ACM functional (4).
    0x04, 0x24, 0x02, 0x06,
    // Union functional (5).
    0x05, 0x24, 0x06, 0x00, 0x01,
    // Call management (5).
    0x05, 0x24, 0x01, 0x00, 0x01,
    // Interface 1: CDC data (9).
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,
    // EP1 OUT bulk (7).
    0x07, 0x05, 0x01, 0x02, EP_SIZE as u8, 0x00, 0x00,
    // EP1 IN bulk (7).
    0x07, 0x05, 0x81, 0x02, EP_SIZE as u8, 0x00, 0x00,
];

const LANGID_DESC: [u8; 4] = [0x04, 0x03, 0x09, 0x04];

/// UTF-16-LE string descriptor (up to 18 characters, bLength matches).
const fn string_desc(s: &str) -> [u8; 40] {
    let mut out = [0u8; 40];
    let chars = if s.len() > 18 { 18 } else { s.len() };
    out[0] = 2 + 2 * chars as u8;
    out[1] = 0x03;
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < chars {
        out[2 + 2 * i] = bytes[i];
        i += 1;
    }
    out
}

const MANUFACTURER_DESC: [u8; 40] = string_desc("FNB58 open");
const PRODUCT_DESC: [u8; 40] = string_desc("FNB58 serial");

// --- Control-transfer state ---

#[derive(Clone, Copy, PartialEq, Eq)]
enum CtrlState {
    /// Armed for the next SETUP.
    Idle,
    /// Streaming a descriptor to the host (multi-chunk).
    DataIn,
    /// Collecting SET_LINE_CODING data.
    DataOut { left: usize },
    /// No-data request: ZLP IN status in flight.
    StatusIn,
    /// Control read finished: waiting for the status ZLP OUT.
    StatusOut,
}

/// Which static descriptor a multi-chunk control read streams.
#[derive(Clone, Copy)]
enum Descriptor {
    Device,
    Config,
    Manufacturer,
    Product,
    #[allow(dead_code)]
    LangId,
}

impl Descriptor {
    fn bytes(self) -> &'static [u8] {
        match self {
            Self::Device => &DEVICE_DESC,
            Self::Config => &CONFIG_DESC,
            Self::Manufacturer => &MANUFACTURER_DESC,
            Self::Product => &PRODUCT_DESC,
            Self::LangId => &LANGID_DESC,
        }
    }

    fn for_request(kind: u8, index: u8) -> Option<Self> {
        match kind {
            0x01 => Some(Self::Device),
            0x02 => Some(Self::Config),
            0x03 => Some(match index {
                0 => Self::LangId,
                1 => Self::Manufacturer,
                _ => Self::Product,
            }),
            _ => None,
        }
    }
}

struct State {
    /// PID reported in the device descriptor (encodes the clock path).
    pid: u16,
    ctrl: CtrlState,
    /// Multi-chunk control read bookkeeping.
    ctrl_desc: Descriptor,
    ctrl_total: usize,
    ctrl_sent: usize,
    configured: bool,
    /// Address accepted in SET_ADDRESS, applied after its status IN.
    pending_address: u8,
    line_coding: [u8; 7],
    /// EP1 TX holds data the host has not collected yet.
    tx_armed: bool,
    rx_ring: Ring,
}

impl State {
    const fn new() -> Self {
        Self {
            pid: 0x5859,
            ctrl: CtrlState::Idle,
            ctrl_desc: Descriptor::Device,
            ctrl_total: 0,
            ctrl_sent: 0,
            configured: false,
            pending_address: 0,
            line_coding: [0; 7],
            tx_armed: false,
            rx_ring: Ring::new(),
        }
    }
}

/// Fixed-capacity receive ring (four max-packet bursts of headroom; full
/// drops, console semantics).
struct Ring {
    buf: [u8; 256],
    read: usize,
    write: usize,
}

impl Ring {
    const fn new() -> Self {
        Self {
            buf: [0; 256],
            read: 0,
            write: 0,
        }
    }

    fn push(&mut self, bytes: &[u8]) {
        for &b in bytes {
            let next = (self.write + 1) % self.buf.len();
            if next == self.read {
                return;
            }
            self.buf[self.write] = b;
            self.write = next;
        }
    }

    fn take(&mut self, out: &mut [u8]) -> usize {
        let mut n = 0;
        while n < out.len() && self.read != self.write {
            out[n] = self.buf[self.read];
            self.read = (self.read + 1) % self.buf.len();
            n += 1;
        }
        n
    }
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<State>> =
    Mutex::new(RefCell::new(State::new()));

/// Bring-up progress marker for the fault handler to blink out: 1 = the
/// device connected (pull-up on), 2 = a bus reset was serviced, 3 = a
/// SETUP was dispatched, 4 = a descriptor response was queued. A crash
/// blinks the last reached value; no blink at all means the crash came
/// before the connect.
pub static CRASH_CODE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

// --- Public API ---

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("the PLL clock cannot produce the required exact 48 MHz USB clock")]
    Clock,
    #[error("the host has not configured the device yet")]
    NotConfigured,
    #[error("the operation did not complete in time")]
    Timeout,
}

/// Program the 48 MHz USB clock from the PLL (vendor
/// `crm_usb_clock_div_set`) and reset the USB cell.
///
/// The AT32F403A USB divider has no /5: its encodings divide the PLL by
/// 1.5, 1, 2.5, 2, 3.5, 3 and 4 (vendor `crm_usb_div_type`), so 48 MHz
/// requires the PLL itself to be 72, 96, 144, 168 or 192 MHz — the reason
/// the vendor's own USB examples run a 192 MHz PLL (`HICK/2 x 48`) with
/// `CRM_USB_DIV_4`. A 240 MHz PLL cannot feed USB on this device.
///
/// # Safety
///
/// Exclusive-startup contract: call once during startup, before attaching,
/// with the PLL running at one of the supported frequencies above.
pub unsafe fn enable_usb_clock(pll_hz: u32) -> Result<(), Error> {
    // Vendor crm_usb_div_type encodings for 48 MHz-capable PLLs.
    let encoding: u8 = match pll_hz {
        72_000_000 => 0x00,  // DIV_1_5
        96_000_000 => 0x02,  // DIV_2_5
        144_000_000 => 0x05, // DIV_3
        168_000_000 => 0x04, // DIV_3_5
        192_000_000 => 0x06, // DIV_4
        _ => return Err(Error::Clock),
    };
    // SAFETY: forwarded from this function's exclusive-startup contract.
    let crm = unsafe { crate::pac::at32f403a::CRM::steal() };
    critical_section::with(|_| {
        // Plain 512-byte packet SRAM (vendor default, MISC1.USBBUFS = 0).
        crm.misc1().modify(|_, w| w.usbbufs().bit(false));
        // Register-interface clock plus reset pulse (APB1).
        crm.apb1en().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(false));
        // Vendor field split: USBDIV1_0 = encoding bits 1:0, USBDIV2 = bit 2.
        crm.cfg().modify(|_, w| {
            // SAFETY: the encoding is the vendor enum value, within the field.
            unsafe { w.usbdiv1_0().bits(encoding & 0b11) };
            w.usbdiv2().bit(encoding & 0b100 != 0);
            w
        });
    });
    Ok(())
}

/// Open one endpoint with its packet buffers (vendor `usb_ept_open` for a
/// single-buffered endpoint, both buffer addresses pre-programmed as the
/// vendor's `usbd_ept_buf_custom_define` does in the reset handler).
fn open_ept(index: usize, trans_type: u32) {
    // Vendor default-init: the register starts from zero, which makes the
    // XOR status idiom deterministic (DISABLE -> NAK).
    ept_write(index, 0);
    // Buffer addresses MUST be in the table before the endpoint is armed;
    // reset leaves them at 0, which aliases every buffer onto the table.
    // The address fields are in HALFWORD units (the vendor's allocator
    // starts buffers at offset 0x40 = exactly the 8-endpoint table size
    // in halfwords; its usb_write_packet multiplies offsets by 2) — byte
    // offsets here made the hardware DMA every packet to a different
    // address than the driver read, so SETUPs landed in space and the
    // "descriptor" the host received was garbage from byte 0x40.
    set_packet_word(SLOT[index], BUF_SLOT_TX[index] as u32 / 2);
    set_packet_word(SLOT[index] + 8, BUF_SLOT_RX[index] as u32 / 2);
    set_ept_slot(index);
    set_trans_type(index, trans_type);
    set_rx_count(SLOT[index], EP_SIZE);
    set_tx_count(SLOT[index], 0);
    // Statuses NAK (XOR from DISABLE).
    set_tx_status(index, STS_NAK);
    set_rx_status(index, STS_NAK);
}

/// Bring up the CDC device and attach (D+ pull-up on).
///
/// # Safety
///
/// Exclusive contract: claims the USBFS cell and its interrupt (the board
/// unmasks `USBFS_H_CAN1_TX` or `USBFS_L_CAN1_RX0` with a handler calling
/// [`on_interrupt`]). [`enable_usb_clock`] must have run.
pub unsafe fn init(pid: u16) {
    critical_section::with(|_| {
        STATE.lock(|cell| cell.borrow_mut().pid = pid);
        // Own clock setup (vendor BSP order: CRM enables the USB register
        // clock before usb_dev_init). Doing it here — not in the board's
        // clock module — makes it impossible to attach without the cell
        // being clocked: unclocked register writes are silent no-ops, and
        // the D+ pull-up would never assert.
        // SAFETY: forwarded from this constructor's exclusive-startup contract.
        let crm = unsafe { crate::pac::at32f403a::CRM::steal() };
        // Plain 512-byte packet SRAM (MISC1.USBBUFS = 0).
        crm.misc1().modify(|_, w| w.usbbufs().bit(false));
        crm.apb1en().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(false));

        // Disconnect during setup (vendor usb_disconnect: DISUSB=1, PUO=1).
        reg_write(CTRL, reg_read(CTRL) | CTRL_DISUSB);
        reg_write(CFG, reg_read(CFG) | CFG_PUO);
        // Vendor usb_dev_init: core out of reset, flags cleared, buffer
        // table at 0, device enabled at address 0 (devaddr = 0x80),
        // RST/TC interrupt enables.
        reg_write(CTRL, reg_read(CTRL) & !CTRL_CSRST);
        reg_write(INTSTS, 0);
        reg_write(BUFTBL, 0);
        reg_write(DEVADDR, DEVADDR_CEN);
        reg_write(CTRL, reg_read(CTRL) | CTRL_TCIEN | CTRL_RSTIEN);
        open_ept(0, TYPE_CONTROL);
        open_ept(1, TYPE_BULK);
        // Arm EP0 reception for the first SETUP (vendor reset handler
        // opens EP0 OUT with RX valid).
        set_rx_status(0, STS_VALID);
    });
    // Connect: the pull-up is active-low (vendor usb_connect).
    critical_section::with(|_| {
        reg_write(CTRL, reg_read(CTRL) & !CTRL_DISUSB);
        reg_write(CFG, reg_read(CFG) & !CFG_PUO);
    });
    CRASH_CODE.store(1, core::sync::atomic::Ordering::Relaxed);
}

fn configured() -> bool {
    STATE.lock(|cell| cell.borrow().configured)
}

/// Write bytes to the host, splitting at the 64-byte packet limit. Waits
/// while the host has not collected a previous packet.
pub async fn write(bytes: &[u8]) -> Result<(), Error> {
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(2);
    let mut rest = bytes;
    while !rest.is_empty() {
        if !configured() {
            return Err(Error::NotConfigured);
        }
        let armed = STATE.lock(|cell| {
            let mut state = cell.borrow_mut();
            if state.tx_armed {
                return false;
            }
            let chunk_len = rest.len().min(EP_SIZE as usize);
            write_packet(BUF_EP1_TX, &rest[..chunk_len]);
            set_tx_count(SLOT[1], chunk_len as u16);
            set_tx_status(1, STS_VALID);
            state.tx_armed = true;
            true
        });
        if armed {
            rest = &rest[rest.len().min(EP_SIZE as usize)..];
            continue;
        }
        if embassy_time::Instant::now() >= deadline {
            return Err(Error::Timeout);
        }
        poll();
        embassy_time::Timer::after_millis(2).await;
    }
    Ok(())
}

/// Non-blocking read: returns `Ok(0)` when no bytes are pending.
pub fn try_read(buffer: &mut [u8]) -> Result<usize, Error> {
    Ok(STATE.lock(|cell| cell.borrow_mut().rx_ring.take(buffer)))
}

/// Read bytes received from the host; returns how many were copied. Waits
/// up to two seconds for the first byte.
pub async fn read(buffer: &mut [u8]) -> Result<usize, Error> {
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(2);
    loop {
        let n = STATE.lock(|cell| cell.borrow_mut().rx_ring.take(buffer));
        if n > 0 {
            return Ok(n);
        }
        if !configured() {
            return Err(Error::NotConfigured);
        }
        if embassy_time::Instant::now() >= deadline {
            return Err(Error::Timeout);
        }
        poll();
        embassy_time::Timer::after_millis(2).await;
    }
}

// --- Interrupt handling ---

/// The USBFS interrupt handler body: the board's `USBFS_H_CAN1_TX` (or
/// `USBFS_L_CAN1_RX0`) handler calls this; pure-polling applications call
/// [`poll`] from their main loop instead.
pub fn on_interrupt() {
    poll()
}


/// Service the USB cell: drain transfer completions, handle SETUP, handle
/// bus reset. Mirrors the vendor `usbd_irq_handler` flow.
///
/// Call from the board's `USBFS_H_CAN1_TX` / `USBFS_L_CAN1_RX0` interrupt
/// handler (`on_interrupt` wraps this), or poll it from the main loop —
/// the flags are plain register reads, so polling works with no NVIC
/// involvement at all. The host tolerates multi-millisecond latency per
/// control-transfer stage, so a 1-10 ms poll cadence enumerates fine.
pub fn poll() {
    // Endpoint transfer completions (vendor usbd_ept_loop_handler): each
    // per-endpoint handler clears its TC, which drops INTSTS.TC.
    let mut rounds = 0;
    while reg_read(INTSTS) & INTS_TC != 0 && rounds < 8 {
        rounds += 1;
        let sts = reg_read(INTSTS);
        let ept_num = (sts & INTS_EPT_NUM) as usize;
        if ept_num > 1 {
            // Should not happen; clear the endpoint's flags and move on.
            clear_txtc(ept_num);
            clear_rxtc(ept_num);
            continue;
        }
        eptn_handler(ept_num);
    }

    // If the core's TC deassertion lags (or a stray TC remains), force
    // the INTSTS flag clear as well; w0c semantics make this harmless
    // when the level already dropped.
    if reg_read(INTSTS) & INTS_TC != 0 {
        reg_write(INTSTS, reg_read(INTSTS) & !INTS_TC);
    }

    if reg_read(INTSTS) & INTS_RST != 0 {
        // Write-zero-to-clear (usb_flag_clear writes ~flag).
        reg_write(INTSTS, reg_read(INTSTS) & !INTS_RST);
        bus_reset();
    }
}

/// Vendor `usbd_eptn_handler`: IN completion, else SETUP, else OUT.
fn eptn_handler(index: usize) {
    let ept_val = ept_read(index);

    if ept_val & EPT_TXTC != 0 {
        // IN transaction completed.
        clear_txtc(index);
        if index == 0 {
            ep0_in();
        } else {
            // Bulk IN done: NAK until the task arms the next packet.
            set_tx_status(1, STS_NAK);
            STATE.lock(|cell| cell.borrow_mut().tx_armed = false);
        }
        return;
    }

    if ept_val & EPT_SETUPTC != 0 {
        // SETUP transaction: read the packet, clear the OUT-side flags,
        // dispatch. (The read must happen before the clears: the buffer
        // content stays only until the endpoint is re-armed.)
        let len = rx_count(SLOT[0]) as usize;
        let setup = packet_bytes(BUF_EP0_RX, len.min(8));
        clear_rxtc(0);
        clear_setuptc(0);
        handle_setup(&setup);
        return;
    }

    if ept_val & EPT_RXTC != 0 {
        // OUT transaction completed.
        clear_rxtc(index);
        let len = rx_count(SLOT[index]) as usize;
        let payload = packet_bytes(BUF_SLOT_RX[index], len);
        if index == 0 {
            ep0_out(&payload, len);
        } else {
            STATE.lock(|cell| cell.borrow_mut().rx_ring.push(&payload[..len]));
            // Re-arm bulk reception (the clear reset the status to
            // DISABLE; set VALID from DISABLE).
            set_rx_count(SLOT[1], EP_SIZE);
            set_rx_status(1, STS_VALID);
        }
    }
}

/// Bus reset: address 0, re-open both endpoints, arm EP0 (vendor
/// `usbd_reset_handler`).
fn bus_reset() {
    CRASH_CODE.store(2, core::sync::atomic::Ordering::Relaxed);
    critical_section::with(|_| {
        reg_write(DEVADDR, DEVADDR_CEN);
        open_ept(0, TYPE_CONTROL);
        open_ept(1, TYPE_BULK);
        set_rx_status(0, STS_VALID);
        STATE.lock(|cell| *cell.borrow_mut() = State::new());
    });
}

/// Vendor `usbd_core_in_handler` for EP0: multi-chunk descriptor
/// streaming, then the status ZLP OUT; address applied after STATUS_IN.
fn ep0_in() {
    let mut follow: Option<(&'static [u8], usize, usize)> = None;
    let mut apply_address = 0u8;
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        match state.ctrl {
            CtrlState::DataIn => {
                if state.ctrl_sent >= state.ctrl_total {
                    // Data stage done: status is a ZLP OUT from the host.
                    state.ctrl = CtrlState::StatusOut;
                    set_rx_count(SLOT[0], EP_SIZE);
                    set_rx_status(0, STS_VALID);
                } else {
                    let start = state.ctrl_sent;
                    let end = state.ctrl_total.min(start + EP_SIZE as usize);
                    follow = Some((state.ctrl_desc.bytes(), start, end));
                    state.ctrl_sent = end;
                }
            }
            CtrlState::StatusIn => {
                apply_address = state.pending_address;
                state.pending_address = 0;
                state.ctrl = CtrlState::Idle;
                // Rearm for the next SETUP.
                set_rx_count(SLOT[0], EP_SIZE);
                set_rx_status(0, STS_VALID);
            }
            _ => {
                state.ctrl = CtrlState::Idle;
                set_rx_count(SLOT[0], EP_SIZE);
                set_rx_status(0, STS_VALID);
            }
        }
    });
    if let Some((bytes, start, end)) = follow {
        write_packet(BUF_EP0_TX, &bytes[start..end]);
        set_tx_count(SLOT[0], (end - start) as u16);
        set_tx_status(0, STS_VALID);
    }
    if apply_address != 0 {
        reg_write(DEVADDR, DEVADDR_CEN | apply_address as u32);
    }
}

/// Vendor `usbd_core_out_handler` for EP0: SET_LINE_CODING data, or the
/// status ZLP OUT of a control read.
fn ep0_out(payload: &[u8; 64], len: usize) {
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        match state.ctrl {
            CtrlState::DataOut { left } => {
                if len >= left {
                    state.line_coding = [
                        payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
                        payload[6],
                    ];
                    state.ctrl = CtrlState::StatusIn;
                    // Status: ZLP IN.
                    set_tx_count(SLOT[0], 0);
                    set_tx_status(0, STS_VALID);
                } else {
                    state.ctrl = CtrlState::DataOut { left: left - len };
                    set_rx_count(SLOT[0], EP_SIZE);
                    set_rx_status(0, STS_VALID);
                }
            }
            _ => {
                // Status-OUT for a control read: transaction done.
                state.ctrl = CtrlState::Idle;
                set_rx_count(SLOT[0], EP_SIZE);
                set_rx_status(0, STS_VALID);
            }
        }
    });
}

/// Vendor `usbd_core_setup_handler`: standard + CDC class requests.
fn handle_setup(setup: &[u8]) {
    let request_type = setup[0];
    let request = setup[1];
    let value = u16::from(setup[2]) | (u16::from(setup[3]) << 8);
    let length = u16::from(setup[6]) | (u16::from(setup[7]) << 8);

    match (request_type, request) {
        // SET_ADDRESS: apply after the status IN completes.
        (0x00, 0x05) => {
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.pending_address = value as u8;
                state.ctrl = CtrlState::StatusIn;
            });
            set_tx_count(SLOT[0], 0);
            set_tx_status(0, STS_VALID);
        }
        // SET_CONFIGURATION: non-zero enables the bulk endpoints.
        (0x00, 0x09) => {
            if value != 0 {
                set_rx_count(SLOT[1], EP_SIZE);
                set_rx_status(1, STS_VALID);
            }
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.configured = value != 0;
                state.ctrl = CtrlState::StatusIn;
            });
            set_tx_count(SLOT[0], 0);
            set_tx_status(0, STS_VALID);
        }
        // GET_STATUS: bus-powered, no remote wake.
        (0x80, 0x00) => serve_slice(length, &[0x00, 0x00], None),
        // GET_CONFIGURATION.
        (0x80, 0x08) => {
            let configured = u8::from(configured());
            serve_slice(length, &[configured, 0x00], None);
        }
        // GET_DESCRIPTOR.
        (0x80, 0x06) => {
            let kind = (value >> 8) as u8;
            match Descriptor::for_request(kind, value as u8) {
                Some(descriptor) => serve_slice(length, descriptor.bytes(), Some(descriptor)),
                None => stall_ep0(),
            }
        }
        // GET_LINE_CODING.
        (0xA1, 0x21) => {
            let coding = STATE.lock(|cell| cell.borrow().line_coding);
            serve_slice(length, &coding, None);
        }
        // SET_LINE_CODING: 7-byte OUT data stage.
        (0x21, 0x20) => {
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.ctrl = CtrlState::DataOut { left: 7 };
            });
            set_rx_count(SLOT[0], EP_SIZE);
            set_rx_status(0, STS_VALID);
        }
        // SET_CONTROL_LINE_STATE and other no-data class requests.
        (0x21, 0x22) | (0x02, 0x01) | (0x02, 0x02) => {
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.ctrl = CtrlState::StatusIn;
            });
            set_tx_count(SLOT[0], 0);
            set_tx_status(0, STS_VALID);
        }
        _ => stall_ep0(),
    }
}

/// Queue a (possibly static) reply for a control read; `descriptor` keeps
/// streaming when the reply is longer than one packet.
fn serve_slice(length: u16, bytes: &[u8], descriptor: Option<Descriptor>) {
    CRASH_CODE.store(4, core::sync::atomic::Ordering::Relaxed);
    let total = bytes.len().min(length as usize);
    let chunk_len = total.min(EP_SIZE as usize);
    write_packet(BUF_EP0_TX, &bytes[..chunk_len]);
    // Device descriptor: patch the runtime PID (clock-path marker) over
    // the static default.
    if bytes.len() == 18 {
        let pid = STATE.lock(|cell| cell.borrow().pid);
        set_packet_half(BUF_EP0_TX + DEVICE_DESC_PID_OFFSET, pid);
    }
    set_tx_count(SLOT[0], chunk_len as u16);
    set_tx_status(0, STS_VALID);

    let more = descriptor.filter(|_| total > chunk_len);
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        match more {
            Some(descriptor) => {
                state.ctrl = CtrlState::DataIn;
                state.ctrl_desc = descriptor;
                state.ctrl_total = total;
                state.ctrl_sent = chunk_len;
            }
            None => {
                state.ctrl = CtrlState::StatusOut;
                set_rx_count(SLOT[0], EP_SIZE);
                set_rx_status(0, STS_VALID);
            }
        }
    });
}

fn stall_ep0() {
    // Both directions STALL; the next SETUP is a special transaction the
    // core delivers regardless of the armed state (it also resets the
    // statuses to NAK), so the endpoint recovers on the next request.
    set_tx_status(0, STS_STALL);
    set_rx_status(0, STS_STALL);
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        state.ctrl = CtrlState::Idle;
    });
}

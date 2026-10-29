//! USB full-speed device (CDC ACM virtual COM port) for the AT32F403A's
//! USBFS cell — an F103-style device core: per-endpoint registers, a buffer
//! table in packet SRAM, and F103 toggle semantics on the endpoint status
//! and TC bits.
//!
//! Everything here follows the vendor `at32f403a_407_usb.c/.h`:
//! - `usb_dev_init`: CTRL.CSRST off, INTSTS cleared, BUFTBL=0, DEVADDR=0x80
//!   (CEN + address 0), RST/TC interrupt enables.
//! - `usb_connect`/`usb_disconnect`: the D+ 1.5 kΩ pull-up is enabled with
//!   CFG.PUO = **0** (active-low, vendor-written).
//! - `USB_SET_TXSTS`/`USB_SET_RXSTS`: the new status is XOR-toggled into
//!   the masked register; EPTn TC flags (TXTC/SETUPTC/RXTC) are F103-style
//!   toggle bits — writing back the read value clears the set ones.
//! - INTSTS flags are write-zero-to-clear (`usb_flag_clear` writes `~flag`).
//! - Buffer table: slot *n* sits at packet-SRAM byte offset `n * 16`
//!   (`(BUFTBL + n*8 + x) * 2` in the vendor macros): TXADDR +0, TXCOUNT
//!   +4, RXADDR +8, RXCOUNT +12. EPTn.EPTADDR holds the endpoint's slot
//!   number. RX counts use the F103 block encoding (vendor BLK2/BLK32);
//!   after an OUT transaction the RXCOUNT slot's low 10 bits hold the
//!   received bytes (`USB_GET_RX_LEN` masks `& 0x3ff`).
//! - `USB_PACKET_BUFFER_ADDRESS` = 0x4000_6000 (512 bytes while CRM
//!   MISC1.USBBUFS is 0).
//!
//! One CDC function is served: EP0 control, EP1 bulk IN/OUT. Shared state
//! sits behind a critical-section mutex taken by both the interrupt
//! handler and the task API; the task side polls, so bring-up needs no
//! interrupt-priority choreography beyond unmasking the vector.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// USBFS register base.
const USBFS_BASE: usize = 0x4000_5c00;
/// Packet SRAM base while CRM MISC1.USBBUFS is 0 (512 bytes).
const PACKET_RAM: *mut u16 = 0x4000_6000 as *mut u16;

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
const INTS_INOUT: u32 = 1 << 4;
const INTS_RST: u32 = 1 << 10;
const INTS_TC: u32 = 1 << 15;
// EPTn bits.
const EPTADDR_MASK: u32 = 0xf;
const TXSTS_OFFSET: u32 = 4; // bits 5:4
#[allow(dead_code)]
const TXTC: u32 = 1 << 7;
const TRANS_TYPE_CONTROL: u32 = 0b01 << 9;
const TRANS_TYPE_BULK: u32 = 0b00 << 9;
const SETUPTC: u32 = 1 << 11;
const RXSTS_OFFSET: u32 = 12; // bits 13:12
#[allow(dead_code)]
const RXTC: u32 = 1 << 15;
/// Vendor `USB_TX_MASK`: status plus all preserved bits.
const EPT_TX_MASK: u32 = 0x30 | 0x8f8f;
/// Vendor `USB_RX_MASK`.
const EPT_RX_MASK: u32 = 0x3000 | 0x8f8f;

/// Endpoint status encodings (bits 1:0 of the field).
#[allow(dead_code)]
const STS_DISABLE: u32 = 0b00;
const STS_STALL: u32 = 0b01;
const STS_NAK: u32 = 0b10;
const STS_VALID: u32 = 0b11;

/// Max packet size everywhere (full-speed bulk).
const EP_SIZE: u16 = 64;

/// Buffer-table slots (byte offset = endpoint * 16; TXCOUNT at +4,
/// RXCOUNT at +12) and packet buffers (byte offsets inside packet SRAM),
/// 2 entries + 4 buffers = 288 bytes.
const SLOT_EP0: usize = 0x00;
const SLOT_EP1: usize = 0x10;
const BUF_EP0_TX: usize = 0x20;
const BUF_EP0_RX: usize = 0x60;
const BUF_EP1_TX: usize = 0xa0;
const BUF_EP1_RX: usize = 0xe0;

// --- Raw register access ---

#[inline]
fn reg(offset: usize) -> *mut u32 {
    (USBFS_BASE + offset) as *mut u32
}

#[inline]
fn reg_read(offset: usize) -> u32 {
    // SAFETY: offsets are inside the USBFS register block by construction.
    unsafe { reg(offset).read_volatile() }
}

#[inline]
fn reg_write(offset: usize, value: u32) {
    // SAFETY: offsets are inside the USBFS register block by construction.
    unsafe {
        reg(offset).write_volatile(value);
    }
}

fn ept_raw(index: usize) -> u32 {
    reg_read(index * 4)
}

fn set_ept_raw(index: usize, value: u32) {
    reg_write(index * 4, value);
}

/// Set one endpoint's TX status (vendor `USB_SET_TXSTS`: XOR-toggle the
/// status bits of the masked register, TC flags carried through unchanged).
fn set_tx_status(index: usize, status: u32) {
    let value = (ept_raw(index) & EPT_TX_MASK) ^ (status << TXSTS_OFFSET);
    set_ept_raw(index, value);
}

/// Set one endpoint's RX status (vendor `USB_SET_RXSTS`).
fn set_rx_status(index: usize, status: u32) {
    let value = (ept_raw(index) & EPT_RX_MASK) ^ (status << RXSTS_OFFSET);
    set_ept_raw(index, value);
}

/// Clear the endpoint's TC flags (TXTC/SETUPTC/RXTC are toggle bits:
/// writing the read value flips each set flag to zero).
fn clear_ept_tc(index: usize) {
    set_ept_raw(index, ept_raw(index));
}

// --- Packet SRAM access ---

#[inline]
fn packet_half(offset: usize) -> u16 {
    // SAFETY: offsets are within the 512-byte packet SRAM by construction.
    unsafe { PACKET_RAM.add(offset / 2).read_volatile() }
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

/// F103-style RX buffer count encoding (vendor BLK2/BLK32).
fn rx_count_encoding(len: u16) -> u16 {
    if len > 62 {
        let mut blocks = len >> 5;
        if len.is_multiple_of(0x20) {
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

fn set_tx_len(slot: usize, len: u16) {
    set_packet_half(slot + 4, len);
}

fn rx_received(slot: usize) -> u16 {
    packet_half(slot + 12) & 0x3ff
}

// --- Descriptors (CDC 1.1, single function, bus-powered) ---

const DEVICE_DESC: [u8; 18] = [
    0x12, 0x01, 0x00, 0x02, // DEVICE, bcdUSB 2.0
    0x02, 0x00, 0x00, // class CDC
    EP_SIZE as u8, 0x3c, 0x2e, // EP0 size, VID 0x2e3c
    0x59, 0x58, // PID 0x5859 (open firmware CDC)
    0x00, 0x01, 1, 2, 0, 0x01,
];

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

/// UTF-16-LE string descriptor (up to 16 characters).
const fn string_desc(s: &str) -> [u8; 40] {
    let mut out = [0u8; 40];
    out[0] = 2 + 2 * s.len() as u8;
    out[1] = 0x03;
    let bytes = s.as_bytes();
    let mut i = 0;
    while 2 + 2 * i + 1 < out.len() && i < bytes.len() {
        out[2 + 2 * i] = bytes[i];
        i += 1;
    }
    out
}

const MANUFACTURER_DESC: [u8; 40] = string_desc("FNB58 open firmware");
const PRODUCT_DESC: [u8; 40] = string_desc("FNB58 serial console");

// --- Control-transfer state ---

#[derive(Clone, Copy, PartialEq, Eq)]
enum CtrlState {
    /// Armed for the next SETUP.
    Idle,
    /// Streaming a descriptor to the host.
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
                1 => Self::Manufacturer,
                _ => Self::Product,
            }),
            _ => {
                let _ = index;
                None
            }
        }
    }
}

struct State {
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

/// Program the 48 MHz USB clock from the PLL and reset the USB cell.
///
/// # Safety
///
/// Exclusive-startup contract: call once during startup, before attaching,
/// with the PLL running at an exact multiple of 48 MHz.
pub unsafe fn enable_usb_clock(pll_hz: u32) -> Result<(), Error> {
    if !pll_hz.is_multiple_of(48_000_000) {
        return Err(Error::Clock);
    }
    let divider = pll_hz / 48_000_000;
    if divider == 0 || divider > 8 {
        return Err(Error::Clock);
    }
    // SAFETY: forwarded from this function's exclusive-startup contract.
    let crm = unsafe { crate::pac::at32f403a::CRM::steal() };
    critical_section::with(|_| {
        // Plain 512-byte packet SRAM (vendor default, MISC1.USBBUFS = 0).
        crm.misc1().modify(|_, w| w.usbbufs().bit(false));
        // Register-interface clock plus reset pulse (APB1).
        crm.apb1en().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(true));
        crm.apb1rst().modify(|_, w| w.usb().bit(false));
        // PLL / divider = 48 MHz: USBDIV2 carries bit 2 of (divider - 1).
        let raw = divider - 1;
        crm.cfg().modify(|_, w| {
            // SAFETY: divider is 1..=8, so raw is 0..=7.
            unsafe { w.usbdiv1_0().bits((raw & 0b11) as u8) };
            w.usbdiv2().bit(raw & 0b100 != 0);
            w
        });
    });
    Ok(())
}

/// Bring up the CDC device and attach (D+ pull-up on).
///
/// # Safety
///
/// Exclusive contract: claims the USBFS cell and its interrupt (the board
/// unmasks `USBFS_H_CAN1_TX` or `USBFS_L_CAN1_RX0` with a handler calling
/// [`on_interrupt`]). [`enable_usb_clock`] must have run.
pub unsafe fn init() {
    // Disconnect during setup (vendor usb_disconnect: DISUSB=1, PUO=1).
    critical_section::with(|_| {
        let ctrl = reg_read(0x40);
        reg_write(0x40, ctrl | CTRL_DISUSB);
        let cfg = reg_read(0x60);
        reg_write(0x60, cfg | CFG_PUO);
        // Core out of reset; flags cleared; buffer table at 0; device
        // enabled at address 0 (vendor devaddr = 0x80).
        let ctrl = reg_read(0x40);
        reg_write(0x40, ctrl & !CTRL_CSRST);
        reg_write(0x44, 0);
        reg_write(0x50, 0); // BUFTBL
        reg_write(0x4c, DEVADDR_CEN); // DEVADDR
        // Interrupt enables: transfer complete and bus reset only.
        let ctrl = reg_read(0x40);
        reg_write(0x40, ctrl | CTRL_TCIEN | CTRL_RSTIEN);
        // EP0 control and EP1 bulk, both directions NAK.
        configure_ept(0, TRANS_TYPE_CONTROL);
        configure_ept(1, TRANS_TYPE_BULK);
        set_packet_half(SLOT_EP0 + 12, rx_count_encoding(EP_SIZE));
        set_packet_half(SLOT_EP1 + 12, rx_count_encoding(EP_SIZE));
        set_rx_status(0, STS_VALID);
    });
    // Connect: the pull-up is active-low (vendor usb_connect).
    critical_section::with(|_| {
        let ctrl = reg_read(0x40);
        reg_write(0x40, ctrl & !CTRL_DISUSB);
        let cfg = reg_read(0x60);
        reg_write(0x60, cfg & !CFG_PUO);
    });
}

fn configure_ept(index: usize, trans_type: u32) {
    // EPTADDR = the endpoint's slot number; statuses NAK; toggle-flag TCs
    // left as-is (the write-back idiom below preserves them).
    let mut value = ept_raw(index);
    value = (value & !(EPTADDR_MASK | 0x600)) | (index as u32) | trans_type;
    set_ept_raw(index, value);
    set_tx_status(index, STS_NAK);
    set_rx_status(index, STS_NAK);
}

fn arm_rx_buffer(slot: usize, index: usize) {
    set_packet_half(slot + 12, rx_count_encoding(EP_SIZE));
    set_rx_status(index, STS_VALID);
}

fn idle_ep0() {
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        state.ctrl = CtrlState::Idle;
    });
    arm_rx_buffer(SLOT_EP0, 0);
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
            set_tx_len(SLOT_EP1, chunk_len as u16);
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
        embassy_time::Timer::after_millis(2).await;
    }
    Ok(())
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
        embassy_time::Timer::after_millis(2).await;
    }
}

// --- Interrupt handling ---

/// The USBFS interrupt: bus reset, SETUP on EP0, and transfer completions
/// routed through `INTSTS.TC` + `EPT_NUM`/`INOUT`. The board's
/// `USBFS_H_CAN1_TX` (or `USBFS_L_CAN1_RX0`) handler calls this.
pub fn on_interrupt() {
    let raw = reg_read(0x44);

    if raw & INTS_RST != 0 {
        critical_section::with(|_| {
            reg_write(0x4c, DEVADDR_CEN);
            configure_ept(0, TRANS_TYPE_CONTROL);
            configure_ept(1, TRANS_TYPE_BULK);
            set_packet_half(SLOT_EP0 + 12, rx_count_encoding(EP_SIZE));
            set_packet_half(SLOT_EP1 + 12, rx_count_encoding(EP_SIZE));
            set_rx_status(0, STS_VALID);
            STATE.lock(|cell| *cell.borrow_mut() = State::new());
        });
        // Write-zero-to-clear: 1s keep, 0 clears RST.
        reg_write(0x44, raw | INTS_RST);
    }

    // SETUP arrives on EPT0 (SETUPTC) possibly with the TC flag; service
    // it before any completion handling.
    if ept_raw(0) & SETUPTC != 0 {
        clear_ept_tc(0);
        handle_setup();
        reg_write(0x44, reg_read(0x44) | INTS_TC);
        return;
    }

    if raw & INTS_TC != 0 {
        let ept_num = (raw & INTS_EPT_NUM) as usize;
        let is_in = raw & INTS_INOUT != 0;
        match (ept_num, is_in) {
            (0, true) => ep0_in(),
            (0, false) => ep0_out(),
            (1, true) => {
                clear_ept_tc(1);
                STATE.lock(|cell| {
                    let mut state = cell.borrow_mut();
                    state.tx_armed = false;
                });
                set_tx_status(1, STS_NAK);
            }
            (1, false) => {
                clear_ept_tc(1);
                let len = rx_received(SLOT_EP1) as usize;
                let payload = packet_bytes(BUF_EP1_RX, len);
                STATE.lock(|cell| cell.borrow_mut().rx_ring.push(&payload[..len]));
                arm_rx_buffer(SLOT_EP1, 1);
            }
            _ => {}
        }
        reg_write(0x44, reg_read(0x44) | INTS_TC);
    }
}

fn ep0_in() {
    clear_ept_tc(0);
    let mut apply_address = 0u8;
    let mut chunk: Option<(&'static [u8], usize)> = None;
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        match state.ctrl {
            CtrlState::DataIn => {
                if state.ctrl_sent >= state.ctrl_total {
                    // Data stage complete: status is a ZLP OUT from host.
                    state.ctrl = CtrlState::StatusOut;
                    arm_rx_buffer(SLOT_EP0, 0);
                } else {
                    let end = state
                        .ctrl_sent
                        .saturating_add(EP_SIZE as usize)
                        .min(state.ctrl_total);
                    chunk = Some((state.ctrl_desc.bytes(), end));
                    state.ctrl_sent = end;
                }
            }
            CtrlState::StatusIn => {
                apply_address = state.pending_address;
                state.pending_address = 0;
                state.ctrl = CtrlState::Idle;
                arm_rx_buffer(SLOT_EP0, 0);
            }
            _ => {
                state.ctrl = CtrlState::Idle;
                arm_rx_buffer(SLOT_EP0, 0);
            }
        }
    });
    if let Some((bytes, end)) = chunk {
        // SAFETY: end <= bytes.len() by the ctrl_total clamp above.
        let start = end.saturating_sub(EP_SIZE as usize);
        write_packet(BUF_EP0_TX, &bytes[start..end]);
        set_tx_len(SLOT_EP0, (end - start) as u16);
        set_tx_status(0, STS_VALID);
    }
    if apply_address != 0 {
        reg_write(0x4c, DEVADDR_CEN | apply_address as u32);
    }
}

fn ep0_out() {
    clear_ept_tc(0);
    let got = rx_received(SLOT_EP0) as usize;
    let payload = packet_bytes(BUF_EP0_RX, got);
    STATE.lock(|cell| {
        let mut state = cell.borrow_mut();
        match state.ctrl {
            CtrlState::DataOut { left } => {
                if got >= left {
                    state.line_coding = [
                        payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
                        payload[6],
                    ];
                    state.ctrl = CtrlState::StatusIn;
                    set_tx_len(SLOT_EP0, 0);
                    set_tx_status(0, STS_VALID);
                } else {
                    state.ctrl = CtrlState::DataOut { left: left - got };
                    arm_rx_buffer(SLOT_EP0, 0);
                }
            }
            _ => {
                // Status-OUT for a control read, or an unexpected OUT.
                state.ctrl = CtrlState::Idle;
                arm_rx_buffer(SLOT_EP0, 0);
            }
        }
    });
}

fn handle_setup() {
    let setup = packet_bytes(BUF_EP0_RX, 8);
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
            set_tx_len(SLOT_EP0, 0);
            set_tx_status(0, STS_VALID);
        }
        // SET_CONFIGURATION: non-zero enables the bulk endpoints.
        (0x00, 0x09) => {
            if value != 0 {
                arm_rx_buffer(SLOT_EP1, 1);
            }
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.configured = value != 0;
                state.ctrl = CtrlState::StatusIn;
            });
            set_tx_len(SLOT_EP0, 0);
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
                Some(descriptor) => {
                    serve_slice(length, descriptor.bytes(), Some(descriptor))
                }
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
            arm_rx_buffer(SLOT_EP0, 0);
        }
        // SET_CONTROL_LINE_STATE and other no-data class requests.
        (0x21, 0x22) | (0x02, 0x01) | (0x02, 0x02) => {
            STATE.lock(|cell| {
                let mut state = cell.borrow_mut();
                state.ctrl = CtrlState::StatusIn;
            });
            set_tx_len(SLOT_EP0, 0);
            set_tx_status(0, STS_VALID);
        }
        _ => stall_ep0(),
    }
}

/// Queue a (possibly static) reply for a control read; `descriptor` keeps
/// streaming when the reply is longer than one packet.
fn serve_slice(length: u16, bytes: &[u8], descriptor: Option<Descriptor>) {
    let total = bytes.len().min(length as usize);
    let chunk_len = total.min(EP_SIZE as usize);
    write_packet(BUF_EP0_TX, &bytes[..chunk_len]);
    set_tx_len(SLOT_EP0, chunk_len as u16);
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
                arm_rx_buffer(SLOT_EP0, 0);
            }
        }
    });
}

fn stall_ep0() {
    set_tx_status(0, STS_STALL);
    set_rx_status(0, STS_STALL);
    idle_ep0();
}

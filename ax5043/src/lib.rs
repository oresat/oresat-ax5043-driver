extern crate spidev;
use std::{fmt, io};
use std::path::Path;
use std::convert::TryFrom;
use bitflags::bitflags;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
//use ax5043_derive::{Serialize, Deserialize};

pub mod config;
pub mod fifo;

bitflags! {
    #[derive(Debug, Copy, Clone, Default, PartialEq)]
    pub struct Status: u16 {
        const READY            = 0x8000;
        const PLL_LOCK         = 0x4000;
        const FIFO_OVER        = 0x2000;
        const FIFO_UNDER       = 0x1000;
        const THRESHOLD_FREE   = 0x0800;
        const THRESHOLD_COUNT  = 0x0400;
        const FIFO_FULL        = 0x0200;
        const FIFO_EMPTY       = 0x0100;
        const PWR_GOOD         = 0x0080;
        const PWR_INTERRUPT    = 0x0040;
        const RADIO_EVENT      = 0x0020;
        const XTAL_OSC_RUNNING = 0x0010;
        const WAKEUP_INTERRUPT = 0x0008;
        const LPOSC_INTERRUPT  = 0x0004;
        const GPADC_INTERRUPT  = 0x0002;
        const UNDEFINED        = 0x0001;
    }
}

pub trait Serialize<const S: usize> {
    fn serialize(&self) -> [u8; S];
}

pub trait Deserialize<const S: usize> {
    fn deserialize(&mut self, data: [u8; S]);
}

impl Serialize<1> for u8 {
    fn serialize(&self) -> [u8; 1] {
        [*self]
    }
}

impl Deserialize<1> for u8 {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = data[0];
    }
}

impl Serialize<2> for u16 {
    fn serialize(&self) -> [u8; 2] {
        self.to_be_bytes()
    }
}

impl Deserialize<2> for u16 {
    fn deserialize(&mut self, data: [u8; 2]) {
        *self = u16::from_be_bytes(data);
    }
}

impl Serialize<2> for i16 {
    fn serialize(&self) -> [u8; 2] {
        self.to_be_bytes()
    }
}

impl Deserialize<2> for i16 {
    fn deserialize(&mut self, data: [u8; 2]) {
        *self = i16::from_be_bytes(data);
    }
}


impl Serialize<3> for [u8; 3] {
    fn serialize(&self) -> [u8; 3] {
        *self
    }
}

impl Deserialize<3> for [u8; 3] {
    fn deserialize(&mut self, data: [u8; 3]) {
        *self = data;
    }
}

impl Serialize<3> for u32 {
    fn serialize(&self) -> [u8; 3] {
        self.to_be_bytes()[1..].try_into().unwrap()
    }
}

impl Deserialize<3> for u32 {
    fn deserialize(&mut self, data: [u8; 3]) {
        *self = (data[0] as u32) << 16 | (data[1] as u32) << 8 | (data[2] as u32)
    }
}


impl Serialize<4> for u32 {
    fn serialize(&self) -> [u8; 4] {
        self.to_be_bytes()
    }
}

impl Deserialize<4> for u32 {
    fn deserialize(&mut self, data: [u8; 4]) {
        *self = u32::from_be_bytes(data);
    }
}

// FIXME typealias for serialize + deserialize?
pub struct ReadOnly<'a, const ADDR: u16, const S: usize, R: Deserialize<S>> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: R,
}

impl<const ADDR:u16, const S: usize, R: Deserialize<S> + Copy + fmt::Debug> fmt::Debug for ReadOnly<'_, ADDR, S, R> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReadOnly")
         .field("spi", &self.spi)
         .field("value", &self.value)
         .finish()
    }
}

impl<const ADDR: u16, const S: usize, R: Deserialize<S> + Copy> ReadOnly<'_, ADDR, S, R> {
    pub fn read(&mut self) -> io::Result<R> {
       let addr = (ADDR | 0x7000).to_be_bytes() ;
       let mut stat = [0; 2];

       let tx: [u8; S] = [0; S];
       let mut rx: [u8; S] = [0; S];
       self.spi.transfer_multiple(&mut [
           SpidevTransfer::read_write(&addr, &mut stat),
           SpidevTransfer::read_write(&tx, &mut rx),
       ])?;

       self.value.deserialize(rx);
       (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
       Ok(self.value)
    }
}

pub struct ReadWrite<'a, const ADDR: u16, const S: usize, RW: Deserialize<S> + Serialize<S> + Copy> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: RW,
}

impl<const ADDR:u16, const S: usize, RW: Deserialize<S> + Serialize<S> + Copy + fmt::Debug> fmt::Debug for ReadWrite<'_, ADDR, S, RW> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReadWrite")
         .field("spi", &self.spi)
         .field("value", &self.value)
         .finish()
    }
}

impl<const ADDR: u16, const S: usize, RW: Deserialize<S> + Serialize<S> + Copy> ReadWrite<'_, ADDR, S, RW> {
    pub fn read(&mut self) -> io::Result<RW> {
       let addr = (ADDR | 0x7000).to_be_bytes();
       let mut stat = [0; 2];

       let tx: [u8; S] = [0; S];
       let mut rx: [u8; S] = [0; S];
       self.spi.transfer_multiple(&mut [
           SpidevTransfer::read_write(&addr, &mut stat),
           SpidevTransfer::read_write(&tx, &mut rx),
       ])?;

       self.value.deserialize(rx);
       (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
       Ok(self.value)
    }

    pub fn writeval(&self) -> io::Result<()> {
        let addr = (ADDR | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = self.value.serialize();
        let mut rx: [u8; S] = [0; S];

        //println!("{:?} Write 0x{:03X}: {:X?}", self.spi.inner(), ADDR, tx);

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;
        //assert_eq!(rx, [0; S]); fails TODO: what does this return? Old value? check that it
        //matches our previous known state?
        (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
        Ok(())
    }

    pub fn write(&mut self, value: RW) -> io::Result<()> {
        self.value = value;
        self.writeval()
    }
}

pub struct WriteOnly<'a, const ADDR: u16, const S: usize, W: Serialize<S> + Copy> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: W,
}

impl<const ADDR:u16, const S: usize, W: Serialize<S> + Copy + fmt::Debug> fmt::Debug for WriteOnly<'_, ADDR, S, W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WriteOnly")
         .field("spi", &self.spi)
         .field("value", &self.value)
         .finish()
    }
}

impl<const ADDR: u16, const S: usize, W: Serialize<S> + Copy> WriteOnly<'_, ADDR, S, W> {
    pub fn writeval(&self) -> io::Result<()> {
        let addr = (ADDR | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = self.value.serialize();
        let mut rx: [u8; S] = [0; S];

        //println!("{:?} Write 0x{:03X}: {:X?}", self.spi.inner(), ADDR, tx);

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;
        //assert_eq!(rx, [0; S]); fails TODO: what does this return? Old value? check that it
        //matches our previous known state?
        (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
        Ok(())
    }

    pub fn write(&mut self, value: W) -> io::Result<()> {
        self.value = value;
        self.writeval()
    }
}


#[derive(Copy, Clone, Debug)]
pub enum PwrModes{
    POWEROFF = 0,
    DEEPSLEEP = 1,
    XOEN = 0b101,
    FIFOEN = 0b111,
    SYNTHRX = 0b1000,
    RX = 0b1001,
    WORRX = 0b1011,
    SYNTHTX = 0b1100,
    TX = 0b1101,
}

impl TryFrom<u8> for PwrModes {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PwrModes::POWEROFF as u8 => Ok(PwrModes::POWEROFF),
            x if x == PwrModes::DEEPSLEEP as u8 => Ok(PwrModes::DEEPSLEEP),
            x if x == PwrModes::XOEN as u8 => Ok(PwrModes::XOEN),
            x if x == PwrModes::FIFOEN as u8 => Ok(PwrModes::FIFOEN),
            x if x == PwrModes::SYNTHRX as u8 => Ok(PwrModes::SYNTHRX),
            x if x == PwrModes::RX as u8 => Ok(PwrModes::RX),
            x if x == PwrModes::WORRX as u8 => Ok(PwrModes::WORRX),
            x if x == PwrModes::SYNTHTX as u8 => Ok(PwrModes::SYNTHTX),
            x if x == PwrModes::TX as u8 => Ok(PwrModes::TX),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct PwrFlags: u8 {
        const WDS = 0x10; // RO
        const REFEN = 0x20;
        const XOEN = 0x40;
        const RST = 0x80;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PwrMode {
    pub mode: PwrModes,
    pub flags: PwrFlags,
}

impl Default for PwrMode {
    fn default() -> Self {
        Self {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::REFEN | PwrFlags::XOEN,
        }
    }
}

impl Serialize<1> for PwrMode {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

impl Deserialize<1> for PwrMode {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PwrModes::try_from(data[0] & 0x0f).unwrap();
        self.flags = PwrFlags::from_bits(data[0] & 0xf0).unwrap();
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq)]
    pub struct PowStat: u8 {
        const SUM      = 1 << 7;
        const REF      = 1 << 6;
        const VREF     = 1 << 5;
        const VANA     = 1 << 4;
        const VMODEM   = 1 << 3; // Powers FIFO
        const BEVANA   = 1 << 2;
        const BEVMODEM = 1 << 1;
        const VIO      = 1 << 0;
    }
}

impl Deserialize<1> for PowStat {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy)]
    pub struct PowIRQMask: u8 {
        const PWRGOOD  = 1 << 7;
        const REF      = 1 << 6;
        const VREF     = 1 << 5;
        const VANA     = 1 << 4;
        const VMODEM   = 1 << 3;
        const BEVANA   = 1 << 2;
        const BEVMODEM = 1 << 1;
        const VIO      = 1 << 0;
    }
}

impl Serialize<1> for PowIRQMask {
    fn serialize(&self) -> [u8; 1] {
        [self.bits()]
    }
}

impl Deserialize<1> for PowIRQMask {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq)]
    pub struct IRQ: u16 {
        const FIFONOTEMPTY = 1 << 0;
        const FIFONOTFULL  = 1 << 1;
        const FIFOTHRCNT   = 1 << 2;
        const FIFOTHRFREE  = 1 << 3;
        const FIFOERROR    = 1 << 4;
        const PLLUNLOCK    = 1 << 5;
        const RADIOCTRL    = 1 << 6;
        const POWER        = 1 << 7;
        const XTALREADY    = 1 << 8;
        const WAKEUPTIMER  = 1 << 9;
        const LPOSC        = 1 << 10;
        const GPADC        = 1 << 11;
        const PLLRNGDONE   = 1 << 12;
    }
}

impl Deserialize<2> for IRQ {
    fn deserialize(&mut self, data: [u8; 2]) {
        *self = Self::from_bits(u16::from_be_bytes(data)).unwrap();
    }
}

impl Serialize<2> for IRQ {
    fn serialize(&self) -> [u8; 2] {
        self.bits().to_be_bytes()
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq)]
    pub struct RadioEvent: u16 {
        const DONE          = 1 << 0;
        const SETTLED       = 1 << 1;
        const RADIOSTATECHG = 1 << 2;
        const RXPARAMSETCHG = 1 << 3;
        const FRAMECLK      = 1 << 4;
    }
}

impl Deserialize<2> for RadioEvent {
    fn deserialize(&mut self, data: [u8; 2]) {
        *self = Self::from_bits(u16::from_be_bytes(data)).unwrap();
    }
}

impl Serialize<2> for RadioEvent {
    fn serialize(&self) -> [u8; 2] {
        self.bits().to_be_bytes()
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
pub enum RadioState {
    IDLE                 = 0b0000,
    POWERDOWN            = 0b0001,
    TX_PLL_SETTINGS      = 0b0100,
    TX                   = 0b0110,
    TX_TAIL              = 0b0111,
    RX_PLL_SETTINGS      = 0b1000,
    RX_ANTENNA_SELECTION = 0b1001,
    RX_PREAMBLE_1        = 0b1100,
    RX_PREAMBLE_2        = 0b1101,
    RX_PREAMBLE_3        = 0b1110,
    RX                   = 0b1111,
}

impl TryFrom<u8> for RadioState {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == RadioState::IDLE                 as u8 => Ok(RadioState::IDLE),
            x if x == RadioState::POWERDOWN            as u8 => Ok(RadioState::POWERDOWN),
            x if x == RadioState::TX_PLL_SETTINGS      as u8 => Ok(RadioState::TX_PLL_SETTINGS),
            x if x == RadioState::TX                   as u8 => Ok(RadioState::TX),
            x if x == RadioState::TX_TAIL              as u8 => Ok(RadioState::TX_TAIL),
            x if x == RadioState::RX_PLL_SETTINGS      as u8 => Ok(RadioState::RX_PLL_SETTINGS),
            x if x == RadioState::RX_ANTENNA_SELECTION as u8 => Ok(RadioState::RX_ANTENNA_SELECTION),
            x if x == RadioState::RX_PREAMBLE_1        as u8 => Ok(RadioState::RX_PREAMBLE_1),
            x if x == RadioState::RX_PREAMBLE_2        as u8 => Ok(RadioState::RX_PREAMBLE_2),
            x if x == RadioState::RX_PREAMBLE_3        as u8 => Ok(RadioState::RX_PREAMBLE_3),
            x if x == RadioState::RX                   as u8 => Ok(RadioState::RX),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for RadioState {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Default for RadioState {
    fn default() -> Self {
        Self::IDLE
    }
}


bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq)]
    pub struct XtalStatus: u8 {
        const XTAL_RUN = 1 << 0;
    }
}

impl Deserialize<1> for XtalStatus {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy)]
    pub struct PinState: u8 {
        const SYSCLK = 1 << 0;
        const DCLK   = 1 << 1;
        const DATA   = 1 << 2;
        const IRQ    = 1 << 3;
        const ANTSEL = 1 << 4;
        const PWRAMP = 1 << 5;
    }
}

impl Deserialize<1> for PinState {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum PFSysClkMode {
    ZERO            = 0b00000,
    ONE             = 0b00001,
    Z               = 0b00010,
    F_XTAL_INVERT   = 0b00011,
    F_XTAL          = 0b00100,
    F_XTAL_DIV_2    = 0b00101,
    F_XTAL_DIV_4    = 0b00110,
    F_XTAL_DIV_8    = 0b00111,
    F_XTAL_DIV_16   = 0b01000,
    F_XTAL_DIV_32   = 0b01001,
    F_XTAL_DIV_64   = 0b01010,
    F_XTAL_DIV_128  = 0b01011,
    F_XTAL_DIV_256  = 0b01100,
    F_XTAL_DIV_512  = 0b01101,
    F_XTAL_DIV_1024 = 0b01110,
    LPO             = 0b01111,
    TEST            = 0b11111,
}

impl TryFrom<u8> for PFSysClkMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFSysClkMode::ZERO            as u8 => Ok(PFSysClkMode::ZERO),
            x if x == PFSysClkMode::ONE             as u8 => Ok(PFSysClkMode::ONE),
            x if x == PFSysClkMode::Z               as u8 => Ok(PFSysClkMode::Z),
            x if x == PFSysClkMode::F_XTAL_INVERT   as u8 => Ok(PFSysClkMode::F_XTAL_INVERT),
            x if x == PFSysClkMode::F_XTAL          as u8 => Ok(PFSysClkMode::F_XTAL),
            x if x == PFSysClkMode::F_XTAL_DIV_2    as u8 => Ok(PFSysClkMode::F_XTAL_DIV_2),
            x if x == PFSysClkMode::F_XTAL_DIV_4    as u8 => Ok(PFSysClkMode::F_XTAL_DIV_4),
            x if x == PFSysClkMode::F_XTAL_DIV_8    as u8 => Ok(PFSysClkMode::F_XTAL_DIV_8),
            x if x == PFSysClkMode::F_XTAL_DIV_16   as u8 => Ok(PFSysClkMode::F_XTAL_DIV_16),
            x if x == PFSysClkMode::F_XTAL_DIV_32   as u8 => Ok(PFSysClkMode::F_XTAL_DIV_32),
            x if x == PFSysClkMode::F_XTAL_DIV_64   as u8 => Ok(PFSysClkMode::F_XTAL_DIV_64),
            x if x == PFSysClkMode::F_XTAL_DIV_128  as u8 => Ok(PFSysClkMode::F_XTAL_DIV_128),
            x if x == PFSysClkMode::F_XTAL_DIV_256  as u8 => Ok(PFSysClkMode::F_XTAL_DIV_256),
            x if x == PFSysClkMode::F_XTAL_DIV_512  as u8 => Ok(PFSysClkMode::F_XTAL_DIV_512),
            x if x == PFSysClkMode::F_XTAL_DIV_1024 as u8 => Ok(PFSysClkMode::F_XTAL_DIV_1024),
            x if x == PFSysClkMode::LPO             as u8 => Ok(PFSysClkMode::LPO),
            x if x == PFSysClkMode::TEST            as u8 => Ok(PFSysClkMode::TEST),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy)]
    pub struct PFSysClkFlags: u8 {
        const PULLUP = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFSysClk {
    pub mode: PFSysClkMode,
    pub flags: PFSysClkFlags,
}

impl Default for PFSysClk {
    fn default() -> Self {
        Self {
            mode: PFSysClkMode::F_XTAL_DIV_16,
            flags: PFSysClkFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFSysClk {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFSysClkMode::try_from(data[0] & 0x1f).unwrap();
        self.flags = PFSysClkFlags::from_bits(data[0] & 0xe0).unwrap();
    }
}

impl Serialize<1> for PFSysClk {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy)]
    pub struct PFFlags: u8 {
        const INVERT = 1 << 6;
        const PULLUP = 1 << 7;
    }
}

#[derive(Copy, Clone, Debug)]
pub enum PFDClkMode {
    ZERO = 0b000,
    ONE  = 0b001,
    Z    = 0b010,
    IN   = 0b011,
    OUT  = 0b100,
    NONE = 0b101,
    TEST = 0b111,
}

impl TryFrom<u8> for PFDClkMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFDClkMode::ZERO as u8 => Ok(PFDClkMode::ZERO),
            x if x == PFDClkMode::ONE  as u8 => Ok(PFDClkMode::ONE),
            x if x == PFDClkMode::Z    as u8 => Ok(PFDClkMode::Z),
            x if x == PFDClkMode::IN   as u8 => Ok(PFDClkMode::IN),
            x if x == PFDClkMode::OUT  as u8 => Ok(PFDClkMode::OUT),
            x if x == PFDClkMode::NONE as u8 => Ok(PFDClkMode::NONE),
            x if x == PFDClkMode::TEST as u8 => Ok(PFDClkMode::TEST),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFDClk {
    pub mode: PFDClkMode,
    pub flags: PFFlags,
}

impl Default for PFDClk {
    fn default() -> Self {
        Self {
            mode: PFDClkMode::OUT,
            flags: PFFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFDClk {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFDClkMode::try_from(data[0] & 0x7).unwrap();
        self.flags = PFFlags::from_bits(data[0] & 0xC0).unwrap();
    }
}

impl Serialize<1> for PFDClk {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}


#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum PFDataMode {
    ZERO      = 0b0000,
    ONE       = 0b0001,
    Z         = 0b0010,
    FRAME_IO  = 0b0011,
    MODEM_IO  = 0b0100,
    ASYNC_IO  = 0b0101,
    MODEM_OUT = 0b0111,
    TEST      = 0b1111,
}

impl TryFrom<u8> for PFDataMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFDataMode::ZERO      as u8 => Ok(PFDataMode::ZERO),
            x if x == PFDataMode::ONE       as u8 => Ok(PFDataMode::ONE),
            x if x == PFDataMode::Z         as u8 => Ok(PFDataMode::Z),
            x if x == PFDataMode::FRAME_IO  as u8 => Ok(PFDataMode::FRAME_IO),
            x if x == PFDataMode::MODEM_IO  as u8 => Ok(PFDataMode::MODEM_IO),
            x if x == PFDataMode::ASYNC_IO  as u8 => Ok(PFDataMode::ASYNC_IO),
            x if x == PFDataMode::MODEM_OUT as u8 => Ok(PFDataMode::MODEM_OUT),
            x if x == PFDataMode::TEST      as u8 => Ok(PFDataMode::TEST),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFData {
    pub mode: PFDataMode,
    pub flags: PFFlags,
}

impl Default for PFData {
    fn default() -> Self {
        Self {
            mode: PFDataMode::MODEM_OUT,
            flags: PFFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFData {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFDataMode::try_from(data[0] & 0xF).unwrap();
        self.flags = PFFlags::from_bits(data[0] & 0xC0).unwrap();
    }
}

impl Serialize<1> for PFData {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}


#[derive(Copy, Clone, Debug)]
pub enum PFIRQMode {
    ZERO = 0b000,
    ONE  = 0b001,
    Z    = 0b010,
    IRQ  = 0b011,
    TEST = 0b111,
}

impl TryFrom<u8> for PFIRQMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFIRQMode::ZERO as u8 => Ok(PFIRQMode::ZERO),
            x if x == PFIRQMode::ONE  as u8 => Ok(PFIRQMode::ONE),
            x if x == PFIRQMode::Z    as u8 => Ok(PFIRQMode::Z),
            x if x == PFIRQMode::IRQ  as u8 => Ok(PFIRQMode::IRQ),
            x if x == PFIRQMode::TEST as u8 => Ok(PFIRQMode::TEST),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFIRQ {
    pub mode: PFIRQMode,
    pub flags: PFFlags,
}

impl Default for PFIRQ {
    fn default() -> Self {
        Self {
            mode: PFIRQMode::IRQ,
            flags: PFFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFIRQ {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFIRQMode::try_from(data[0] & 0x7).unwrap();
        self.flags = PFFlags::from_bits(data[0] & 0xC0).unwrap();
    }
}

impl Serialize<1> for PFIRQ {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

#[derive(Copy, Clone, Debug)]
pub enum PFAntSelMode {
    ZERO      = 0b000,
    ONE       = 0b001,
    Z         = 0b010,
    BBTUNECLK = 0b011,
    EXTTCXO   = 0b100,
    DAC       = 0b101,
    ANTSEL    = 0b110,
    TEST      = 0b111,
}

impl TryFrom<u8> for PFAntSelMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFAntSelMode::ZERO      as u8 => Ok(PFAntSelMode::ZERO),
            x if x == PFAntSelMode::ONE       as u8 => Ok(PFAntSelMode::ONE),
            x if x == PFAntSelMode::Z         as u8 => Ok(PFAntSelMode::Z),
            x if x == PFAntSelMode::BBTUNECLK as u8 => Ok(PFAntSelMode::BBTUNECLK),
            x if x == PFAntSelMode::EXTTCXO   as u8 => Ok(PFAntSelMode::EXTTCXO),
            x if x == PFAntSelMode::DAC       as u8 => Ok(PFAntSelMode::DAC),
            x if x == PFAntSelMode::ANTSEL    as u8 => Ok(PFAntSelMode::ANTSEL),
            x if x == PFAntSelMode::TEST      as u8 => Ok(PFAntSelMode::TEST),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFAntSel {
    pub mode: PFAntSelMode,
    pub flags: PFFlags,
}

impl Default for PFAntSel {
    fn default() -> Self {
        Self {
            mode: PFAntSelMode::ANTSEL,
            flags: PFFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFAntSel {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFAntSelMode::try_from(data[0] & 0x0f).unwrap();
        self.flags = PFFlags::from_bits(data[0] & 0xf0).unwrap();
    }
}

impl Serialize<1> for PFAntSel {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}


#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum PFPwrAmpMode {
    ZERO           = 0b0000,
    ONE            = 0b0001,
    Z              = 0b0010,
    DIBIT_SYNC_IO  = 0b0011,
    DIBIT_SYNC_OBS = 0b0100,
    DAC            = 0b0101,
    PWRAMP         = 0b0110,
    EXTTCXO        = 0b0111,
    TEST           = 0b1111,
}

impl TryFrom<u8> for PFPwrAmpMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PFPwrAmpMode::ZERO           as u8 => Ok(PFPwrAmpMode::ZERO),
            x if x == PFPwrAmpMode::ONE            as u8 => Ok(PFPwrAmpMode::ONE),
            x if x == PFPwrAmpMode::Z              as u8 => Ok(PFPwrAmpMode::Z),
            x if x == PFPwrAmpMode::DIBIT_SYNC_IO  as u8 => Ok(PFPwrAmpMode::DIBIT_SYNC_IO),
            x if x == PFPwrAmpMode::DIBIT_SYNC_OBS as u8 => Ok(PFPwrAmpMode::DIBIT_SYNC_OBS),
            x if x == PFPwrAmpMode::DAC            as u8 => Ok(PFPwrAmpMode::DAC),
            x if x == PFPwrAmpMode::PWRAMP         as u8 => Ok(PFPwrAmpMode::PWRAMP),
            x if x == PFPwrAmpMode::EXTTCXO        as u8 => Ok(PFPwrAmpMode::EXTTCXO),
            x if x == PFPwrAmpMode::TEST           as u8 => Ok(PFPwrAmpMode::TEST),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PFPwrAmp {
    pub mode: PFPwrAmpMode,
    pub flags: PFFlags,
}

impl Default for PFPwrAmp {
    fn default() -> Self {
        Self {
            mode: PFPwrAmpMode::PWRAMP,
            flags: PFFlags::empty(),
        }
    }
}

impl Deserialize<1> for PFPwrAmp {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PFPwrAmpMode::try_from(data[0] & 0x0f).unwrap();
        self.flags = PFFlags::from_bits(data[0] & 0xf0).unwrap();
    }
}

impl Serialize<1> for PFPwrAmp {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum PLLVCORefDiv {
    F_XTAL = 0b00,
    F_XTAL_DIV_2 = 0b01,
    F_XTAL_DIV_4 = 0b10,
    F_XTAL_DIV_8 = 0b11,
}

impl TryFrom<u8> for PLLVCORefDiv {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PLLVCORefDiv::F_XTAL        as u8 => Ok(PLLVCORefDiv::F_XTAL),
            x if x == PLLVCORefDiv::F_XTAL_DIV_2  as u8 => Ok(PLLVCORefDiv::F_XTAL_DIV_2),
            x if x == PLLVCORefDiv::F_XTAL_DIV_4  as u8 => Ok(PLLVCORefDiv::F_XTAL_DIV_4),
            x if x == PLLVCORefDiv::F_XTAL_DIV_8  as u8 => Ok(PLLVCORefDiv::F_XTAL_DIV_8),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct PLLVCODivFlags: u8 {
        const RFDIV = 1 << 2;
        const VCOSEL = 1 << 4;
        const VCO2INT = 1 << 5;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PLLVCODiv {
    pub mode: PLLVCORefDiv,
    pub flags: PLLVCODivFlags,
}

impl Default for PLLVCODiv {
    fn default() -> Self {
        Self {
            mode: PLLVCORefDiv::F_XTAL,
            flags: PLLVCODivFlags::empty(),
        }
    }
}

impl Deserialize<1> for PLLVCODiv {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode  = PLLVCORefDiv::try_from(data[0] & 0x0f).unwrap();
        self.flags = PLLVCODivFlags::from_bits(data[0] & 0xf0).unwrap();
    }
}

impl Serialize<1> for PLLVCODiv {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct PLLRangingFlags: u8 {
        const RNG_START = 1 << 4;
        const RNGERR = 1 << 5;
        const PLL_LOCK = 1 << 6;
        const STICKY_LOCK = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PLLRanging {
    pub vcor: u8,
    pub flags: PLLRangingFlags,
}

impl Default for PLLRanging {
    fn default() -> Self {
        Self {
            vcor: 0b1000,
            flags: PLLRangingFlags::empty(),
        }
    }
}

impl Deserialize<1> for PLLRanging {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.vcor  = data[0] & 0x0f;
        self.flags = PLLRangingFlags::from_bits(data[0] & 0xf0).unwrap();
    }
}

impl Serialize<1> for PLLRanging {
    fn serialize(&self) -> [u8; 1] {
        [self.vcor & 0x0f | self.flags.bits()]
    }
}

#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum LockDetDly {
    d6ns = 0b00,
    d9ns = 0b01,
    d12ns = 0b10,
    d14ns = 0b11,
}

impl TryFrom<u8> for LockDetDly {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == LockDetDly::d6ns  as u8 => Ok(LockDetDly::d6ns),
            x if x == LockDetDly::d9ns  as u8 => Ok(LockDetDly::d9ns),
            x if x == LockDetDly::d12ns as u8 => Ok(LockDetDly::d12ns),
            x if x == LockDetDly::d14ns as u8 => Ok(LockDetDly::d14ns),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct LockDetFlags: u8 {
        const DLY_AUTOMATIC = 0;
        const DLY_MANUAL = 1 << 2;
    }
}


#[derive(Debug, Clone, Copy)]
pub struct  PLLLockDet {
    pub delay: LockDetDly,
    pub flags: LockDetFlags,
    pub readback: LockDetDly,
}

impl Default for PLLLockDet {
    fn default() -> Self {
        Self {
            delay: LockDetDly::d14ns,
            flags: LockDetFlags::empty(),
            readback: LockDetDly::d14ns,
        }
    }
}

impl Deserialize<1> for PLLLockDet {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.delay = LockDetDly::try_from(data[0] & 0x03).unwrap();
        self.flags = LockDetFlags::from_bits(data[0] & 0x04).unwrap();
        self.readback = LockDetDly::try_from((data[0] & 0xc0) >> 6).unwrap();
    }
}

impl Serialize<1> for PLLLockDet {
    fn serialize(&self) -> [u8; 1] {
        [self.delay as u8 | self.flags.bits() | (self.readback as u8) << 6 ]
    }
}

#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum ModulationMode {
    ASK          = 0b0000,
    ASK_COHERENT = 0b0001,
    PSK          = 0b0100,
    OQPSK        = 0b0110,
    MSK          = 0b0111,
    FSK          = 0b1000,
    FSK_4        = 0b1001,
    AFSK         = 0b1010,
    FM           = 0b1011,
}

impl TryFrom<u8> for ModulationMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == ModulationMode::ASK          as u8 => Ok(ModulationMode::ASK),
            x if x == ModulationMode::ASK_COHERENT as u8 => Ok(ModulationMode::ASK_COHERENT),
            x if x == ModulationMode::PSK          as u8 => Ok(ModulationMode::PSK),
            x if x == ModulationMode::OQPSK        as u8 => Ok(ModulationMode::OQPSK),
            x if x == ModulationMode::MSK          as u8 => Ok(ModulationMode::MSK),
            x if x == ModulationMode::FSK          as u8 => Ok(ModulationMode::FSK),
            x if x == ModulationMode::FSK_4        as u8 => Ok(ModulationMode::FSK_4),
            x if x == ModulationMode::AFSK         as u8 => Ok(ModulationMode::AFSK),
            x if x == ModulationMode::FM           as u8 => Ok(ModulationMode::FM),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct ModulationFlags: u8 {
        const RX_HALFSPEED = 1 << 4;
    }
}


#[derive(Debug, Clone, Copy)]
pub struct Modulation {
    pub mode: ModulationMode,
    pub flags: ModulationFlags,
}

impl Default for Modulation {
    fn default() -> Self {
        Self {
            mode: ModulationMode::FSK,
            flags: ModulationFlags::empty(),
        }
    }
}

impl Deserialize<1> for Modulation {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.mode = ModulationMode::try_from(data[0] & 0x0F).unwrap();
        self.flags = ModulationFlags::from_bits(data[0] & 0x10).unwrap();
    }
}

impl Serialize<1> for Modulation {
    fn serialize(&self) -> [u8; 1] {
        [self.mode as u8 | self.flags.bits()]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct Encoding: u8 {
        const INV    = 1 << 0;
        const DIFF   = 1 << 1;
        const SCRAM  = 1 << 2;
        const MANCH  = 1 << 3;
        const NOSYNC = 1 << 4;
        const NRZ    = 0;
        const NRZI   = Self::INV.bits() | Self::DIFF.bits();
        const FM1    = Self::INV.bits() | Self::DIFF.bits() | Self::MANCH.bits();
        const FM0    = Self::DIFF.bits() | Self::MANCH.bits();
    }
}

impl Default for Encoding {
    fn default() -> Self {
        Self::DIFF
    }
}

impl Deserialize<1> for Encoding {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

impl Serialize<1> for Encoding {
    fn serialize(&self) -> [u8; 1] {
        [ self.bits() ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum FrameMode {
    #[default]
    RAW                = 0b000,
    RAW_SOFT_BITS      = 0b001,
    HDLC               = 0b010,
    RAW_PATTERN_MATCH  = 0b011,
    WIRELESS_MBUS      = 0b100,
    WIRELESS_MBUS_4TO6 = 0b101,
}

impl TryFrom<u8> for FrameMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == FrameMode::RAW                as u8 => Ok(FrameMode::RAW),
            x if x == FrameMode::RAW_SOFT_BITS      as u8 => Ok(FrameMode::RAW_SOFT_BITS),
            x if x == FrameMode::HDLC               as u8 => Ok(FrameMode::HDLC),
            x if x == FrameMode::RAW_PATTERN_MATCH  as u8 => Ok(FrameMode::RAW_PATTERN_MATCH),
            x if x == FrameMode::WIRELESS_MBUS      as u8 => Ok(FrameMode::WIRELESS_MBUS),
            x if x == FrameMode::WIRELESS_MBUS_4TO6 as u8 => Ok(FrameMode::WIRELESS_MBUS_4TO6),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum CRCMode {
    #[default]
    OFF   = 0b000,
    CCITT = 0b001,
    CRC16 = 0b010,
    DNP   = 0b011,
    CRC32 = 0b110,
}

impl TryFrom<u8> for CRCMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == CRCMode::OFF   as u8 => Ok(CRCMode::OFF),
            x if x == CRCMode::CCITT as u8 => Ok(CRCMode::CCITT),
            x if x == CRCMode::CRC16 as u8 => Ok(CRCMode::CRC16),
            x if x == CRCMode::DNP   as u8 => Ok(CRCMode::DNP),
            x if x == CRCMode::CRC32 as u8 => Ok(CRCMode::CRC32),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Default, Clone, Copy)]
    pub struct FramingFlags: u8 {
        const ABORT = 1 << 0;
        const FRMRX = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Framing {
    pub frmmode: FrameMode,
    pub crcmode: CRCMode,
    pub flags: FramingFlags,
}

impl Deserialize<1> for Framing {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.frmmode = FrameMode::try_from((data[0] & 0x0e) >> 1).unwrap();
        self.crcmode = CRCMode::try_from((data[0] & 0x70) >> 4).unwrap();
        self.flags = FramingFlags::from_bits(data[0] & 0x81).unwrap();
    }
}

impl Serialize<1> for Framing {
    fn serialize(&self) -> [u8; 1] {
        [(self.frmmode as u8) << 1 | (self.crcmode as u8) << 4 | self.flags.bits()]
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum ModCfgF {
    #[default]
    UNSHAPED = 0b00,
    GAUSSIAN_BT_0p3 = 0b10,
    GAUSSIAN_BT_0p5 = 0b11,
}

impl TryFrom<u8> for ModCfgF {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == ModCfgF::UNSHAPED        as u8 => Ok(ModCfgF::UNSHAPED),
            x if x == ModCfgF::GAUSSIAN_BT_0p3 as u8 => Ok(ModCfgF::GAUSSIAN_BT_0p3),
            x if x == ModCfgF::GAUSSIAN_BT_0p5 as u8 => Ok(ModCfgF::GAUSSIAN_BT_0p5),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for ModCfgF {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = ModCfgF::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for ModCfgF {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum SlowRamp {
    #[default]
    STARTUP_1b = 0b00,
    STARTUP_2b = 0b01,
    STARTUP_4b = 0b10,
    STARTUP_8b = 0b11,
}

impl TryFrom<u8> for SlowRamp {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == SlowRamp::STARTUP_1b as u8 => Ok(SlowRamp::STARTUP_1b),
            x if x == SlowRamp::STARTUP_2b as u8 => Ok(SlowRamp::STARTUP_2b),
            x if x == SlowRamp::STARTUP_4b as u8 => Ok(SlowRamp::STARTUP_4b),
            x if x == SlowRamp::STARTUP_8b as u8 => Ok(SlowRamp::STARTUP_8b),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct ModCfgAFlags: u8 {
        const TXDIFF      = 1 << 0;
        const TXSE        = 1 << 1;
        const AMPLSHAPE   = 1 << 2;
        const PLLLCK_GATE = 1 << 6;
        const BROWN_GATE  = 1 << 7;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ModCfgA {
    pub slowramp: SlowRamp,
    pub flags: ModCfgAFlags,
}

impl Default for ModCfgA {
    fn default() -> Self {
        Self {
            slowramp: SlowRamp::default(),
            flags: ModCfgAFlags::TXDIFF | ModCfgAFlags::AMPLSHAPE,
        }
    }
}

impl Deserialize<1> for ModCfgA {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.slowramp = SlowRamp::try_from((data[0] & 0x30) >> 4).unwrap();
        self.flags = ModCfgAFlags::from_bits(data[0] & 0xCF).unwrap();
    }
}

impl Serialize<1> for ModCfgA {
    fn serialize(&self) -> [u8; 1] {
        [(self.slowramp as u8) << 4 |  self.flags.bits()]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FIFOStat: u8 {
        const EMPTY       = 1 << 0;
        const FULL        = 1 << 1;
        const UNDER       = 1 << 2;
        const OVER        = 1 << 3;
        const CNT_THR     = 1 << 4;
        const FREE_THR    = 1 << 5;
        const AUTO_COMMIT = 1 << 7;
    }
}

impl Default for FIFOStat {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Deserialize<1> for FIFOStat {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::from_bits(data[0]).unwrap();
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum FIFOCmds {
    #[default]
    NOOP         = 0b000000,
    ASK_COHERENT = 0b000001,
    CLEAR_ERROR  = 0b000010,
    CLEAR_DATA   = 0b000011,
    COMMIT       = 0b000100,
    ROLLBACK     = 0b000101,
}

impl TryFrom<u8> for FIFOCmds {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == FIFOCmds::NOOP         as u8 => Ok(FIFOCmds::NOOP),
            x if x == FIFOCmds::ASK_COHERENT as u8 => Ok(FIFOCmds::ASK_COHERENT),
            x if x == FIFOCmds::CLEAR_ERROR  as u8 => Ok(FIFOCmds::CLEAR_ERROR),
            x if x == FIFOCmds::CLEAR_DATA   as u8 => Ok(FIFOCmds::CLEAR_DATA),
            x if x == FIFOCmds::COMMIT       as u8 => Ok(FIFOCmds::COMMIT),
            x if x == FIFOCmds::ROLLBACK     as u8 => Ok(FIFOCmds::ROLLBACK),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FIFOCmdFlags: u8 {
        const AUTO_COMMIT = 1 << 7;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FIFOCmd {
    pub mode: FIFOCmds,
    pub flags: FIFOCmdFlags,
}

impl Default for FIFOCmd {
    fn default() -> Self {
        Self {
            mode: FIFOCmds::NOOP,
            flags: FIFOCmdFlags::empty(),
        }
    }
}

impl Serialize<1> for FIFOCmd{
    fn serialize(&self) -> [u8; 1] {
        [ self.mode as u8 | self.flags.bits() ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum PerfF10 {
    #[default]
    XO = 0x03,
    TCXO = 0x04,
    FreqGT43MHz = 0x0D,
}

impl TryFrom<u8> for PerfF10 {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == PerfF10::XO            as u8 => Ok(PerfF10::XO),
            x if x == PerfF10::TCXO          as u8 => Ok(PerfF10::TCXO),
            x if x == PerfF10::FreqGT43MHz as u8 => Ok(PerfF10::FreqGT43MHz),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for PerfF10 {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for PerfF10 {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum PerfF11 {
    XO = 0x07,
    TCXO = 0x00,
    #[default]
    Reset = 0x80, // Undocumented, value on reset
}

impl TryFrom<u8> for PerfF11 {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::XO   as u8  => Ok(Self::XO),
            x if x == Self::TCXO as u8  => Ok(Self::TCXO),
            x if x == Self::Reset as u8 => Ok(Self::Reset),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for PerfF11 {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for PerfF11 {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum PerfF34 {
    RFDIV_unset = 0x08,
    RFDIV_set = 0x28,
    #[default]
    Reset = 0x0F, // Undocumented, value on reset
}

impl TryFrom<u8> for PerfF34 {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::RFDIV_unset as u8 => Ok(Self::RFDIV_unset),
            x if x == Self::RFDIV_set   as u8 => Ok(Self::RFDIV_set),
            x if x == Self::Reset       as u8 => Ok(Self::Reset),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for PerfF34 {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for PerfF34 {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum PerfF35 {
    #[default]
    FreqLT24p8MHz = 0x10,
    FreqGE24p8MHz = 0x11,
}

impl TryFrom<u8> for PerfF35 {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::FreqLT24p8MHz as u8 => Ok(Self::FreqLT24p8MHz),
            x if x == Self::FreqGE24p8MHz as u8 => Ok(Self::FreqGE24p8MHz),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for PerfF35 {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for PerfF35 {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}

#[derive(Debug, Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum FLT {
    EXTERNAL    = 0b00,
    INTERNAL_x1 = 0b01,
    INTERNAL_x2 = 0b10,
    INTERNAL_x5 = 0b11,
}

impl TryFrom<u8> for FLT {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::EXTERNAL    as u8 => Ok(Self::EXTERNAL),
            x if x == Self::INTERNAL_x1 as u8 => Ok(Self::INTERNAL_x1),
            x if x == Self::INTERNAL_x2 as u8 => Ok(Self::INTERNAL_x2),
            x if x == Self::INTERNAL_x5 as u8 => Ok(Self::INTERNAL_x5),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum FreqSel {
    A = 0,
    B = 1 << 7,
}

impl TryFrom<u8> for FreqSel {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::A as u8 => Ok(Self::A),
            x if x == Self::B as u8 => Ok(Self::B),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct PLLLoopFlags: u8 {
        const FILTEN = 1 << 2;
        const DIRECT = 1 << 3;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PLLLoop {
    pub filter: FLT,
    pub flags: PLLLoopFlags,
    pub freqsel: FreqSel
}

impl Default for PLLLoop {
    fn default() -> Self {
        Self {
            filter: FLT::INTERNAL_x1,
            flags: PLLLoopFlags::DIRECT,
            freqsel: FreqSel::A
        }
    }
}

impl Deserialize<1> for PLLLoop {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.filter = FLT::try_from(data[0] & 0x3).unwrap();
        self.flags  = PLLLoopFlags::from_bits(data[0] & 0xC).unwrap();
        self.freqsel = FreqSel::try_from(data[0] & 0x80).unwrap();
    }
}

impl Serialize<1> for PLLLoop {
   fn serialize(&self) -> [u8; 1] {
        [ self.filter as u8 | self.flags.bits() | self.freqsel as u8 ]
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[allow(non_camel_case_types)]
pub enum PLLRngClk {
    XTAL_DIV_2pow8  = 0b000,
    XTAL_DIV_2pow9  = 0b001,
    XTAL_DIV_2pow10 = 0b010,
    #[default]
    XTAL_DIV_2pow11 = 0b011,
    XTAL_DIV_2pow12 = 0b100,
    XTAL_DIV_2pow13 = 0b101,
    XTAL_DIV_2pow14 = 0b110,
    XTAL_DIV_2pow15 = 0b111,
}

impl TryFrom<u8> for PLLRngClk {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::XTAL_DIV_2pow8  as u8 => Ok(Self::XTAL_DIV_2pow8),
            x if x == Self::XTAL_DIV_2pow9  as u8 => Ok(Self::XTAL_DIV_2pow9),
            x if x == Self::XTAL_DIV_2pow10 as u8 => Ok(Self::XTAL_DIV_2pow10),
            x if x == Self::XTAL_DIV_2pow11 as u8 => Ok(Self::XTAL_DIV_2pow11),
            x if x == Self::XTAL_DIV_2pow12 as u8 => Ok(Self::XTAL_DIV_2pow12),
            x if x == Self::XTAL_DIV_2pow13 as u8 => Ok(Self::XTAL_DIV_2pow13),
            x if x == Self::XTAL_DIV_2pow14 as u8 => Ok(Self::XTAL_DIV_2pow14),
            x if x == Self::XTAL_DIV_2pow15 as u8 => Ok(Self::XTAL_DIV_2pow15),
            _ => Err(()),
        }
    }
}

impl Deserialize<1> for PLLRngClk {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Self::try_from(data[0]).unwrap();
    }
}

impl Serialize<1> for PLLRngClk {
    fn serialize(&self) -> [u8; 1] {
        [ *self as u8 ]
    }
}


bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct PLLVCOIFlags: u8 {
        const AUTOMATIC = 0;
        const MANUAL = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PLLVCOI {
    pub bias: u8,
    pub flags: PLLVCOIFlags,
}

impl Default for PLLVCOI {
    fn default() -> Self {
        Self {
            bias: 0b010010,
            flags: PLLVCOIFlags::AUTOMATIC,
        }
    }
}

impl Deserialize<1> for PLLVCOI {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.bias = data[0] & 0x3F;
        self.flags  = PLLVCOIFlags::from_bits(data[0] & 0x80).unwrap();
    }
}

impl Serialize<1> for PLLVCOI {
   fn serialize(&self) -> [u8; 1] {
        [ self.bias | self.flags.bits() ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AGCGain {
    pub attack: u8,
    pub decay: u8,
}

impl Default for AGCGain {
    fn default() -> Self {
        Self {
            attack: 0b0100,
            decay: 0b1011,
        }
    }
}

impl Deserialize<1> for AGCGain {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.attack = data[0] & 0x0F;
        self.decay  = data[0] & 0xF0;
    }
}

impl Serialize<1> for AGCGain {
   fn serialize(&self) -> [u8; 1] {
        [ self.attack | self.decay << 4]
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AGCHyst {
    pub hyst: u8,
}

impl Deserialize<1> for AGCHyst {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.hyst = data[0] & 0x07;
    }
}

impl Serialize<1> for AGCHyst {
   fn serialize(&self) -> [u8; 1] {
        [ self.hyst ]
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AGCMinMax {
    pub min: u8,
    pub max: u8,
}

impl Deserialize<1> for AGCMinMax {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.min = data[0] & 0x07;
        self.max = (data[0] >> 3) & 0x07;
    }
}

impl Serialize<1> for AGCMinMax {
   fn serialize(&self) -> [u8; 1] {
        [ self.min | self.max << 3 ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TimeGain {
    pub exponent: u8,
    pub mantissa: u8,
}

impl Default for TimeGain {
    fn default() -> Self {
        Self {
            exponent: 0b1000,
            mantissa: 0b1111,
        }
    }
}

impl Deserialize<1> for TimeGain {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.exponent = data[0] & 0x0F;
        self.mantissa = data[0] & 0xF0;
    }
}

impl Serialize<1> for TimeGain {
   fn serialize(&self) -> [u8; 1] {
        [ self.exponent | self.mantissa << 4 ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DRGain {
    pub exponent: u8,
    pub mantissa: u8,
}

impl Default for DRGain {
    fn default() -> Self {
        Self {
            exponent: 0b0010,
            mantissa: 0b1111,
        }
    }
}

impl Deserialize<1> for DRGain {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.exponent = data[0] & 0x0F;
        self.mantissa = data[0] & 0xF0;
    }
}

impl Serialize<1> for DRGain {
   fn serialize(&self) -> [u8; 1] {
        [ self.exponent | self.mantissa << 4 ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PhaseGain {
    pub gain: u8,
    pub filter: u8,
}

impl Default for PhaseGain {
    fn default() -> Self {
        Self {
            gain: 0b0011,
            filter: 0b11,
        }
    }
}

impl Deserialize<1> for PhaseGain {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x0F;
        self.filter = (data[0] >> 6) & 0x03
    }
}

impl Serialize<1> for PhaseGain {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain | self.filter << 6 ]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FreqGainAFlags: u8 {
        const AMPLGATE = 1 << 4;
        const HALFMOD  = 1 << 5;
        const MODULO   = 1 << 6;
        const LIM      = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FreqGainA {
    pub gain: u8,
    pub flags: FreqGainAFlags,
}

impl Default for FreqGainA {
    fn default() -> Self {
        Self {
            gain: 0b1111,
            flags: FreqGainAFlags::empty(),
        }
    }
}

impl Deserialize<1> for FreqGainA {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x0F;
        self.flags = FreqGainAFlags::from_bits(data[0] & 0xF0).unwrap();
    }
}

impl Serialize<1> for FreqGainA {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain | self.flags.bits() ]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FreqGainBFlags: u8 {
        const AVG    = 1 << 6;
        const FREEZE = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FreqGainB {
    pub gain: u8,
    pub flags: FreqGainBFlags,
}

impl Default for FreqGainB {
    fn default() -> Self {
        Self {
            gain: 0b1111,
            flags: FreqGainBFlags::empty(),
        }
    }
}

impl Deserialize<1> for FreqGainB {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x1F;
        self.flags = FreqGainBFlags::from_bits(data[0] & 0xC0).unwrap();
    }
}

impl Serialize<1> for FreqGainB {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain | self.flags.bits() ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FreqGainC {
    pub gain: u8,
}

impl Default for FreqGainC {
    fn default() -> Self {
        Self {
            gain: 0b01010,
        }
    }
}

impl Deserialize<1> for FreqGainC {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x1F;
    }
}

impl Serialize<1> for FreqGainC {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain ]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FreqGainDFlags: u8 {
        const FREEZE = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FreqGainD {
    pub gain: u8,
    pub flags: FreqGainDFlags,
}

impl Default for FreqGainD {
    fn default() -> Self {
        Self {
            gain: 0b01010,
            flags: FreqGainDFlags::empty(),
        }
    }
}

impl Deserialize<1> for FreqGainD {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x1F;
        self.flags = FreqGainDFlags::from_bits(data[0] & 0x80).unwrap();
    }
}

impl Serialize<1> for FreqGainD {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain | self.flags.bits() ]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct AmplGainFlags: u8 {
        const AGC = 1 << 6;
        const AVG = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AmplGain {
    pub gain: u8,
    pub flags: AmplGainFlags,
}

impl Default for AmplGain {
    fn default() -> Self {
        Self {
            gain: 0b0110,
            flags: AmplGainFlags::AGC,
        }
    }
}

impl Deserialize<1> for AmplGain {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.gain = data[0] & 0x0F;
        self.flags = AmplGainFlags::from_bits(data[0] & 0xC0).unwrap();
    }
}

impl Serialize<1> for AmplGain {
   fn serialize(&self) -> [u8; 1] {
        [ self.gain | self.flags.bits() ]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct FourFSKFlags: u8 {
        const UPDATE = 1 << 4;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FourFSK {
    pub decay: u8,
    pub flags: FourFSKFlags,
}

impl Default for FourFSK {
    fn default() -> Self {
        Self {
            decay: 0b0110,
            flags: FourFSKFlags::UPDATE,
        }
    }
}

impl Deserialize<1> for FourFSK {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.decay = data[0] & 0x0F;
        self.flags = FourFSKFlags::from_bits(data[0] & 0x10).unwrap();
    }
}

impl Serialize<1> for FourFSK {
   fn serialize(&self) -> [u8; 1] {
        [ self.decay | self.flags.bits() ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BBOffsRes {
    pub res_int_a: u8,
    pub res_int_b: u8,
}

impl Default for BBOffsRes {
    fn default() -> Self {
        Self {
            res_int_a: 0b1000,
            res_int_b: 0b1000,
        }
    }
}

impl Deserialize<1> for BBOffsRes {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.res_int_a = data[0] & 0x0F;
        self.res_int_b = data[0] & 0xF0;
    }
}

impl Serialize<1> for BBOffsRes {
   fn serialize(&self) -> [u8; 1] {
        [ self.res_int_a | self.res_int_b << 3 ]
    }
}


bitflags! {
    #[derive(Default, Debug, Clone, Copy)]
    pub struct Diversity: u8 {
        const DIVENA = 1 << 0;
        const ANTSEL = 1 << 1;
    }
}

impl Deserialize<1> for Diversity {
    fn deserialize(&mut self, data: [u8; 1]) {
        *self = Diversity::from_bits(data[0]).unwrap();
    }
}

impl Serialize<1> for Diversity {
   fn serialize(&self) -> [u8; 1] {
        [ self.bits() ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MaxRFOffset {
    pub offset: u32,
    pub freq_offset_correction: bool
}

impl Default for MaxRFOffset {
    fn default() -> Self {
        Self {
            offset: 0x01687,
            freq_offset_correction: false,
        }
    }
}

impl Deserialize<3> for MaxRFOffset {
    fn deserialize(&mut self, data: [u8; 3]) {
        let value  = (data[0] as u32) << 16 | (data[1] as u32) << 8 | (data[2] as u32);
        self.offset = value & 0xF_FFFF;
        self.freq_offset_correction = (value & (1 << 23)) != 0;
    }
}

impl Serialize<3> for MaxRFOffset {
   fn serialize(&self) -> [u8; 3] {
        let value = self.offset | ( if self.freq_offset_correction { 1 << 23 } else { 0 });
        value.to_be_bytes()[1..].try_into().unwrap()
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RxParamSet {
    Set0 = 0b00,
    Set1 = 0b01,
    Set2 = 0b10,
    Set3 = 0b11,
}

impl TryFrom<u8> for RxParamSet {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == Self::Set0 as u8 => Ok(Self::Set0),
            x if x == Self::Set1 as u8 => Ok(Self::Set1),
            x if x == Self::Set2 as u8 => Ok(Self::Set2),
            x if x == Self::Set3 as u8 => Ok(Self::Set3),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RxParamSets {
    pub ps0: RxParamSet,
    pub ps1: RxParamSet,
    pub ps2: RxParamSet,
    pub ps3: RxParamSet,
}

impl Default for RxParamSets {
    fn default() -> Self {
        Self {
            ps0: RxParamSet::Set0,
            ps1: RxParamSet::Set0,
            ps2: RxParamSet::Set0,
            ps3: RxParamSet::Set0,
        }
    }
}

impl Deserialize<1> for RxParamSets {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.ps0 = RxParamSet::try_from(data[0] & ((0x3 << 0) >> 0)).unwrap();
        self.ps1 = RxParamSet::try_from(data[0] & ((0x3 << 2) >> 2)).unwrap();
        self.ps2 = RxParamSet::try_from(data[0] & ((0x3 << 4) >> 4)).unwrap();
        self.ps3 = RxParamSet::try_from(data[0] & ((0x3 << 6) >> 6)).unwrap();
    }
}

impl Serialize<1> for RxParamSets {
   fn serialize(&self) -> [u8; 1] {
        [ (self.ps0 as u8) << 0 | (self.ps1 as u8) << 2 | (self.ps2 as u8) << 4 | (self.ps3 as u8) << 6 ]
    }
}

//#[derive(Copy, Clone, Debug)]
//pub enum RxParamSpecial {
//    Normal,
//    CoarseAGC,
//    BBOffsetAcq,
//}

#[derive(Debug, Clone, Copy)]
pub struct RxParamCurSet {
    pub index: u8,
    pub number: RxParamSet,
    pub special: u8,
}

impl Default for RxParamCurSet {
    fn default() -> Self {
        Self {
            index: 0,
            number: RxParamSet::Set0,
            special: 0,
        }
    }
}

impl Deserialize<1> for RxParamCurSet {
    fn deserialize(&mut self, data: [u8; 1]) {
        self.index = data[0] & ((0x3 << 0) >> 0);
        self.number = RxParamSet::try_from(data[0] & ((0x3 << 2) >> 2)).unwrap();
        self.special = data[0] & ((0xF << 4) >> 4);
    }
}


/* Adapted from AX5043 Programming Manual, Table 22 */
#[derive(Debug)]
#[allow(non_snake_case)]
pub struct Registers<'a> {
    /* Revision & Interface Probing */
    pub REVISION:       ReadOnly <'a, 0x000, 1, u8>,  // Silicon Revision
    pub SCRATCH:        ReadWrite<'a, 0x001, 1, u8>,  // Scratch Register
    /* Operating Mode */
    pub PWRMODE:        ReadWrite<'a, 0x002, 1, PwrMode>, // Power Mode
    /* Voltage Regulator */
    pub POWSTAT:        ReadOnly <'a, 0x003, 1, PowStat>,    // Power Management Status
    pub POWSTICKYSTAT:  ReadOnly <'a, 0x004, 1, PowStat>,    // Power Management Sticky Status
    pub POWIRQMASK:     ReadWrite<'a, 0x005, 1, PowIRQMask>, // Power Management Interrupt Mask
    /* Interrupt Control */
    pub IRQMASK:        ReadWrite<'a, 0x006, 2, IRQ>,        // IRQ Mask
    pub RADIOEVENTMASK: ReadWrite<'a, 0x008, 2, RadioEvent>, // Radio Event Mask
    pub IRQINVERSION:   ReadWrite<'a, 0x00A, 2, IRQ>,        // IRQ Inversion
    pub IRQREQUEST:     ReadOnly <'a, 0x00C, 2, IRQ>,        // IRQ Request
    pub RADIOEVENTREQ:  ReadOnly <'a, 0x00E, 2, RadioEvent>, // Radio Event Request
    /* Modulation & Framing */
    pub MODULATION:     ReadWrite<'a, 0x010, 1, Modulation>, // Modulation
    pub ENCODING:       ReadWrite<'a, 0x011, 1, Encoding>,   // Encoder/Decoder Settings
    pub FRAMING:        ReadWrite<'a, 0x012, 1, Framing>,    // Framing settings
    pub CRCINIT:        ReadWrite<'a, 0x014, 4, u32>,        // CRC Initialisation Data
    /* Forward Error Correction */
    pub FEC:            ReadWrite<'a, 0x018, 1, u8>, // FEC (Viterbi) Configuration
    pub FECSYNC:        ReadWrite<'a, 0x019, 1, u8>, // Interleaver Synchronisation Threshold
    pub FECSTATUS:      ReadOnly <'a, 0x01A, 1, u8>, // FEC Status
    /* Status */
    pub RADIOSTATE:     ReadOnly <'a, 0x01C, 1, RadioState>, // Radio Controller State
    pub XTALSTATUS:     ReadOnly <'a, 0x01D, 1, XtalStatus>, // Crystal Oscillator Status
    /* Pin Configuration */
    pub PINSTATE:       ReadOnly <'a, 0x020, 1, PinState>, // Pinstate
    pub PINFUNCSYSCLK:  ReadWrite<'a, 0x021, 1, PFSysClk>, // SYSCLK Pin Function
    pub PINFUNCDCLK:    ReadWrite<'a, 0x022, 1, PFDClk>,   // DCLK Pin Function
    pub PINFUNCDATA:    ReadWrite<'a, 0x023, 1, PFData>,   // DATA Pin Function
    pub PINFUNCIRQ:     ReadWrite<'a, 0x024, 1, PFIRQ>,    // IRQ Pin Function
    pub PINFUNCANTSEL:  ReadWrite<'a, 0x025, 1, PFAntSel>, // ANTSEL Pin Function
    pub PINFUNCPWRAMP:  ReadWrite<'a, 0x026, 1, PFPwrAmp>, // PWRAMP Pin Function
    pub PWRAMP:         ReadWrite<'a, 0x027, 1, u8>,       // PWRAMP Control
    /* FIFO */
    pub FIFOSTAT:       ReadOnly <'a, 0x028, 1, FIFOStat>, // FIFO Control
    pub FIFOCMD:        WriteOnly<'a, 0x028, 1, FIFOCmd>,  // FIFO Control
    pub FIFODATA:       ReadWrite<'a, 0x029, 1, u8>,       // FIFO Data
    pub FIFOCOUNT:      ReadOnly <'a, 0x02A, 2, u16>,      // Number of Words currently in FIFO
    pub FIFOFREE:       ReadOnly <'a, 0x02C, 2, u16>,      // Number of Words that can be written to FIFO
    pub FIFOTHRESH:     ReadWrite<'a, 0x02E, 2, u16>,      // FIFO Threshold
    /* Synthesizer */
    pub PLLLOOP:        ReadWrite<'a, 0x030, 1, PLLLoop>,    // PLL Loop Filter Settings
    pub PLLCPI:         ReadWrite<'a, 0x031, 1, u8>,         // PLL Charge Pump Current
    pub PLLVCODIV:      ReadWrite<'a, 0x032, 1, PLLVCODiv>,  // PLL Divider Settings
    pub PLLRANGINGA:    ReadWrite<'a, 0x033, 1, PLLRanging>, // PLL Autoranging
    pub FREQA:          ReadWrite<'a, 0x034, 4, u32>,        // Synthesizer Frequency
    pub PLLLOOPBOOST:   ReadWrite<'a, 0x038, 1, PLLLoop>,    // PLL Loop Filter Settings (Boosted)
    pub PLLCPIBOOST:    ReadWrite<'a, 0x039, 1, u8>,         // PLL Charge Pump Current (Boosted)
    pub PLLRANGINGB:    ReadWrite<'a, 0x03B, 1, PLLRanging>, // PLL Autoranging
    pub FREQB:          ReadWrite<'a, 0x03C, 4, u32>,        // Synthesizer Frequency
    /* Signal Strength */
    pub RSSI:           ReadOnly <'a, 0x040, 1, u8>,        // Received Signal Strength Indicator
    pub BGNDRSSI:       ReadWrite<'a, 0x041, 1, u8>,        // Background RSSI
    pub DIVERSITY:      ReadWrite<'a, 0x042, 1, Diversity>, // Antenna Diversity Configuration
    pub AGCCOUNTER:     ReadWrite<'a, 0x043, 1, u8>,        // AGC Current Value
    /* Receiver Tracking */
    pub TRKDATARATE:    ReadOnly <'a, 0x045, 3, u32>, // Datarate Tracking
    pub TRKAMPL:        ReadOnly <'a, 0x048, 2, u16>, // Amplitude Tracking
    pub TRKPHASE:       ReadOnly <'a, 0x04A, 2, u16>, // Phase Tracking
    pub TRKRFFREQ:      ReadWrite<'a, 0x04D, 3, u32>, // RF Frequency Tracking
    pub TRKFREQ:        ReadWrite<'a, 0x050, 2, u16>, // Frequency Tracking
    pub TRKFSKDEMOD:    ReadOnly <'a, 0x052, 2, u16>, // FSK Demodulator Tracking
    /* Timer */
    pub TIMER2:         ReadOnly <'a, 0x059, 3, u32>, // 1 MHz Timer
    /* Wakeup Timer */
    pub WAKEUPTIMER:    ReadOnly <'a, 0x068, 2, u16>, // Wakeup Timer
    pub WAKEUP:         ReadWrite<'a, 0x06A, 2, u16>, // Wakeup Time
    pub WAKEUPFREQ:     ReadWrite<'a, 0x06C, 2, u16>, // Wakeup Frequency
    pub WAKEUPXOEARLY:  ReadWrite<'a, 0x06E, 1, u8>,  // Wakeup Crystal Oscillator Early
    /* Physical Layer Parameters */
    /* Receiver Parameters */
    pub IFFREQ:         ReadWrite<'a, 0x100, 2, u16>,         // 2nd LO / IF Frequency
    pub DECIMATION:     ReadWrite<'a, 0x102, 1, u8>,          // Decimation Factor
    pub RXDATARATE:     ReadWrite<'a, 0x103, 3, u32>,         // Receiver Datarate
    pub MAXDROFFSET:    ReadWrite<'a, 0x106, 3, u32>,         // Maximum Receiver Datarate Offset
    pub MAXRFOFFSET:    ReadWrite<'a, 0x109, 3, MaxRFOffset>, // Maximum Receiver RF Offset
    pub FSKDMAX:        ReadWrite<'a, 0x10C, 2, u16>,         // Four FSK Rx Deviation
    pub FSKDMIN:        ReadWrite<'a, 0x10E, 2, i16>,         // Four FSK Rx Deviation
    pub AFSKSPACE:      ReadWrite<'a, 0x110, 2, u16>,         // AFSK Space (0) Frequency
    pub AFSKMARK:       ReadWrite<'a, 0x112, 2, u16>,         // AFSK Mark (1) Frequency
    pub AFSKCTRL:       ReadWrite<'a, 0x114, 1, u8>,          // AFSK Control
    pub AMPLFILTER:     ReadWrite<'a, 0x115, 1, u8>,          // Amplitude Filter
    pub FREQUENCYLEAK:  ReadWrite<'a, 0x116, 1, u8>,          // Baseband Frequency Recovery Loop Leakiness
    pub RXPARAMSETS:    ReadWrite<'a, 0x117, 1, RxParamSets>,  // Receiver Parameter Set Indirection
    pub RXPARAMCURSET:  ReadOnly <'a, 0x118, 1, RxParamCurSet>, // Receiver Parameter Current Set
    /* Receiver Parameter Set 0 */
    pub AGCGAIN0:       ReadWrite<'a, 0x120, 1, AGCGain>,   // AGC Speed
    pub AGCTARGET0:     ReadWrite<'a, 0x121, 1, u8>,        // AGC Target
    pub AGCAHYST0:      ReadWrite<'a, 0x122, 1, AGCHyst>,   // AGC Digital Threshold Range
    pub AGCMINMAX0:     ReadWrite<'a, 0x123, 1, AGCMinMax>, // AGC Digital Minimum/Maximum Set Points
    pub TIMEGAIN0:      ReadWrite<'a, 0x124, 1, TimeGain>,  // Timing Gain
    pub DRGAIN0:        ReadWrite<'a, 0x125, 1, DRGain>,    // Data Rate Gain
    pub PHASEGAIN0:     ReadWrite<'a, 0x126, 1, PhaseGain>, // Filter Index, Phase Gain
    pub FREQGAINA0:     ReadWrite<'a, 0x127, 1, FreqGainA>, // Frequency Gain A
    pub FREQGAINB0:     ReadWrite<'a, 0x128, 1, FreqGainB>, // Frequency Gain B
    pub FREQGAINC0:     ReadWrite<'a, 0x129, 1, FreqGainC>, // Frequency Gain C
    pub FREQGAIND0:     ReadWrite<'a, 0x12A, 1, FreqGainD>, // Frequency Gain D
    pub AMPLGAIN0:      ReadWrite<'a, 0x12B, 1, AmplGain>,  // Amplitude Gain
    pub FREQDEV0:       ReadWrite<'a, 0x12C, 2, u16>,       // Receiver Frequency Deviation
    pub FOURFSK0:       ReadWrite<'a, 0x12E, 1, FourFSK>,   // Four FSK Control
    pub BBOFFSRES0:     ReadWrite<'a, 0x12F, 1, BBOffsRes>, // Baseband Offset Compensation Resistors
    /* Receiver Parameter Set 1 */
    pub AGCGAIN1:       ReadWrite<'a, 0x130, 1, AGCGain>,   // AGC Speed
    pub AGCTARGET1:     ReadWrite<'a, 0x131, 1, u8>,        // AGC Target
    pub AGCAHYST1:      ReadWrite<'a, 0x132, 1, AGCHyst>,   // AGC Digital Threshold Range
    pub AGCMINMAX1:     ReadWrite<'a, 0x133, 1, AGCMinMax>, // AGC Digital Minimum/Maximum Set Points
    pub TIMEGAIN1:      ReadWrite<'a, 0x134, 1, TimeGain>,  // Timing Gain
    pub DRGAIN1:        ReadWrite<'a, 0x135, 1, DRGain>,    // Data Rate Gain
    pub PHASEGAIN1:     ReadWrite<'a, 0x136, 1, PhaseGain>, // Filter Index, Phase Gain
    pub FREQGAINA1:     ReadWrite<'a, 0x137, 1, FreqGainA>, // Frequency Gain A
    pub FREQGAINB1:     ReadWrite<'a, 0x138, 1, FreqGainB>, // Frequency Gain B
    pub FREQGAINC1:     ReadWrite<'a, 0x139, 1, FreqGainC>, // Frequency Gain C
    pub FREQGAIND1:     ReadWrite<'a, 0x13A, 1, FreqGainD>, // Frequency Gain D
    pub AMPLGAIN1:      ReadWrite<'a, 0x13B, 1, AmplGain>,  // Amplitude Gain
    pub FREQDEV1:       ReadWrite<'a, 0x13C, 2, u16>,       // Receiver Frequency Deviation
    pub FOURFSK1:       ReadWrite<'a, 0x13E, 1, FourFSK>,   // Four FSK Control
    pub BBOFFSRES1:     ReadWrite<'a, 0x13F, 1, BBOffsRes>, // Baseband Offset Compensation Resistors
    /* Receiver Parameter Set 2 */
    pub AGCGAIN2:       ReadWrite<'a, 0x140, 1, AGCGain>,   // AGC Speed
    pub AGCTARGET2:     ReadWrite<'a, 0x141, 1, u8>,        // AGC Target
    pub AGCAHYST2:      ReadWrite<'a, 0x142, 1, AGCHyst>,   // AGC Digital Threshold Range
    pub AGCMINMAX2:     ReadWrite<'a, 0x143, 1, AGCMinMax>, // AGC Digital Minimum/Maximum Set Points
    pub TIMEGAIN2:      ReadWrite<'a, 0x144, 1, TimeGain>,  // Timing Gain
    pub DRGAIN2:        ReadWrite<'a, 0x145, 1, DRGain>,    // Data Rate Gain
    pub PHASEGAIN2:     ReadWrite<'a, 0x146, 1, PhaseGain>, // Filter Index, Phase Gain
    pub FREQGAINA2:     ReadWrite<'a, 0x147, 1, FreqGainA>, // Frequency Gain A
    pub FREQGAINB2:     ReadWrite<'a, 0x148, 1, FreqGainB>, // Frequency Gain B
    pub FREQGAINC2:     ReadWrite<'a, 0x149, 1, FreqGainC>, // Frequency Gain C
    pub FREQGAIND2:     ReadWrite<'a, 0x14A, 1, FreqGainD>, // Frequency Gain D
    pub AMPLGAIN2:      ReadWrite<'a, 0x14B, 1, AmplGain>,  // Amplitude Gain
    pub FREQDEV2:       ReadWrite<'a, 0x14C, 2, u16>,       // Receiver Frequency Deviation
    pub FOURFSK2:       ReadWrite<'a, 0x14E, 1, FourFSK>,   // Four FSK Control
    pub BBOFFSRES2:     ReadWrite<'a, 0x14F, 1, BBOffsRes>, // Baseband Offset Compensation Resistors
    /* Receiver Parameter Set 3 */
    pub AGCGAIN3:       ReadWrite<'a, 0x150, 1, AGCGain>,   // AGC Speed
    pub AGCTARGET3:     ReadWrite<'a, 0x151, 1, u8>,        // AGC Target
    pub AGCAHYST3:      ReadWrite<'a, 0x152, 1, AGCHyst>,   // AGC Digital Threshold Range
    pub AGCMINMAX3:     ReadWrite<'a, 0x153, 1, AGCMinMax>, // AGC Digital Minimum/Maximum Set Points
    pub TIMEGAIN3:      ReadWrite<'a, 0x154, 1, TimeGain>,  // Timing Gain
    pub DRGAIN3:        ReadWrite<'a, 0x155, 1, DRGain>,    // Data Rate Gain
    pub PHASEGAIN3:     ReadWrite<'a, 0x156, 1, PhaseGain>, // Filter Index, Phase Gain
    pub FREQGAINA3:     ReadWrite<'a, 0x157, 1, FreqGainA>, // Frequency Gain A
    pub FREQGAINB3:     ReadWrite<'a, 0x158, 1, FreqGainB>, // Frequency Gain B
    pub FREQGAINC3:     ReadWrite<'a, 0x159, 1, FreqGainC>, // Frequency Gain C
    pub FREQGAIND3:     ReadWrite<'a, 0x15A, 1, FreqGainD>, // Frequency Gain D
    pub AMPLGAIN3:      ReadWrite<'a, 0x15B, 1, AmplGain>,  // Amplitude Gain
    pub FREQDEV3:       ReadWrite<'a, 0x15C, 2, u16>,       // Receiver Frequency Deviation
    pub FOURFSK3:       ReadWrite<'a, 0x15E, 1, FourFSK>,   // Four FSK Control
    pub BBOFFSRES3:     ReadWrite<'a, 0x15F, 1, BBOffsRes>, // Baseband Offset Compensation Resistors
    /* Transmitter Parameters */
    pub MODCFGF:        ReadWrite<'a, 0x160, 1, ModCfgF>, // Modulator Configuration F
    pub FSKDEV:         ReadWrite<'a, 0x161, 3, u32>,     // FSK Frequency Deviation // TODO: Aliased
    pub MODCFGA:        ReadWrite<'a, 0x164, 1, ModCfgA>, // Modulator Configuration A
    pub TXRATE:         ReadWrite<'a, 0x165, 3, u32>,     // Transmitter Bitrate
    pub TXPWRCOEFFA:    ReadWrite<'a, 0x168, 2, u16>,     // Transmitter Predistortion Coefficient A
    pub TXPWRCOEFFB:    ReadWrite<'a, 0x16A, 2, u16>,     // Transmitter Predistortion Coefficient B
    pub TXPWRCOEFFC:    ReadWrite<'a, 0x16C, 2, u16>,     // Transmitter Predistortion Coefficient C
    pub TXPWRCOEFFD:    ReadWrite<'a, 0x16E, 2, u16>,     // Transmitter Predistortion Coefficient D
    pub TXPWRCOEFFE:    ReadWrite<'a, 0x170, 2, u16>,     // Transmitter Predistortion Coefficient E
    /* PLL Parameters */
    pub PLLVCOI:        ReadWrite<'a, 0x180, 1, PLLVCOI>,    // VCO Current
    pub PLLVCOIR:       ReadWrite<'a, 0x181, 1, u8>,         // VCO Current Readback
    pub PLLLOCKDET:     ReadWrite<'a, 0x182, 1, PLLLockDet>, // PLL Lock Detect Delay
    pub PLLRNGCLK:      ReadWrite<'a, 0x183, 1, PLLRngClk>,  // PLL Ranging Clock
    /* Crystal Oscillator */
    pub XTALCAP:        ReadWrite<'a, 0x184, 1, u8>, // Crystal Oscillator Load Capacitance Configuration
    /* Baseband */
    pub BBTUNE:         ReadWrite<'a, 0x188, 1, u8>, // Baseband Tuning
    pub BBOFFSCAP:      ReadWrite<'a, 0x189, 1, u8>, // Baseband Offset Compensation Capacitors
    /* MAC Layer Parameters */
    /* Packet Format */
    pub PKTADDRCFG:     ReadWrite<'a, 0x200, 1, u8>,   // Packet Address Config
    pub PKTLENCFG:      ReadWrite<'a, 0x201, 1, u8>,   // Packet Length Config
    pub PKTLENOFFSET:   ReadWrite<'a, 0x202, 1, u8>,   // Packet Length Offset
    pub PKTMAXLEN:      ReadWrite<'a, 0x203, 1, u8>,   // Packet Maximum Length
    pub PKTADDR:        ReadWrite<'a, 0x204, 4, u32>,  // Packet Address 3
    pub PKTADDRMASK:    ReadWrite<'a, 0x208, 4, u32>,  // Packet Address Mask 3
    /* Pattern Match */
    pub MATCH0PAT:      ReadWrite<'a, 0x210, 4, u32>, // Pattern Match Unit 0, Pattern
    pub MATCH0LEN:      ReadWrite<'a, 0x214, 1, u8>,  // Pattern Match Unit 0, Pattern Length
    pub MATCH0MIN:      ReadWrite<'a, 0x215, 1, u8>,  // Pattern Match Unit 0, Minimum Match
    pub MATCH0MAX:      ReadWrite<'a, 0x216, 1, u8>,  // Pattern Match Unit 0, Maximum Match
    pub MATCH1PAT:      ReadWrite<'a, 0x218, 2, u16>, // Pattern Match Unit 1, Pattern
    pub MATCH1LEN:      ReadWrite<'a, 0x21C, 1, u8>,  // Pattern Match Unit 1, Pattern Length
    pub MATCH1MIN:      ReadWrite<'a, 0x21D, 1, u8>,  // Pattern Match Unit 1, Minimum Match
    pub MATCH1MAX:      ReadWrite<'a, 0x21E, 1, u8>,  // Pattern Match Unit 1, Maximum Match
    /* Packet Controller */
    pub TMGTXBOOST:     ReadWrite<'a, 0x220, 1, u8>, // Transmit PLL Boost Time
    pub TMGTXSETTLE:    ReadWrite<'a, 0x221, 1, u8>, // Transmit PLL (post Boost) Settling Time
    pub TMGRXBOOST:     ReadWrite<'a, 0x223, 1, u8>, // Receive PLL Boost Time
    pub TMGRXSETTLE:    ReadWrite<'a, 0x224, 1, u8>, // Receive PLL (post Boost) Settling Time
    pub TMGRXOFFSACQ:   ReadWrite<'a, 0x225, 1, u8>, // Receive Baseband DC Offset Acquisition Time
    pub TMGRXCOARSEAGC: ReadWrite<'a, 0x226, 1, u8>, // Receive Coarse AGC Time
    pub TMGRXAGC:       ReadWrite<'a, 0x227, 1, u8>, // Receiver AGC Settling Time
    pub TMGRXRSSI:      ReadWrite<'a, 0x228, 1, u8>, // Receiver RSSI Settling Time
    pub TMGRXPREAMBLE1: ReadWrite<'a, 0x229, 1, u8>, // Receiver Preamble 1 Timeout
    pub TMGRXPREAMBLE2: ReadWrite<'a, 0x22A, 1, u8>, // Receiver Preamble 2 Timeout
    pub TMGRXPREAMBLE3: ReadWrite<'a, 0x22B, 1, u8>, // Receiver Preamble 3 Timeout
    pub RSSIREFERENCE:  ReadWrite<'a, 0x22C, 1, u8>, // RSSI Offset
    pub RSSIABSTHR:     ReadWrite<'a, 0x22D, 1, u8>, // RSSI Absolute Threshold
    pub BGNDRSSIGAIN:   ReadWrite<'a, 0x22E, 1, u8>, // Background RSSI Averaging Time Constant
    pub BGNDRSSITHR:    ReadWrite<'a, 0x22F, 1, u8>, // Background RSSI Relative Threshold
    pub PKTCHUNKSIZE:   ReadWrite<'a, 0x230, 1, u8>, // Packet Chunk Size
    pub PKTMISCFLAGS:   ReadWrite<'a, 0x231, 1, u8>, // Packet Controller Miscellaneous Flags
    pub PKTSTOREFLAGS:  ReadWrite<'a, 0x232, 1, u8>, // Packet Controller Store Flags
    pub PKTACCEPTFLAGS: ReadWrite<'a, 0x233, 1, u8>, // Packet Controller Accept Flags
    /* Special Functions */
    /* General Purpose ADC */
    pub GPADCCTRL:      ReadWrite<'a, 0x300, 1, u8>,  // General Purpose ADC Control
    pub GPADCPERIOD:    ReadWrite<'a, 0x301, 1, u8>,  // GPADC Sampling Period
    pub GPADC13VALUE:   ReadOnly <'a, 0x308, 2, u16>, // GPADC13 Value
    /* Low Power Oscillator Calibration */
    pub LPOSCCONFIG:    ReadWrite<'a, 0x310, 1, u8>,  // Low Power Oscillator Configuration
    pub LPOSCSTATUS:    ReadOnly <'a, 0x311, 1, u8>,  // Low Power Oscillator Status
    pub LPOSCKFILT:     ReadWrite<'a, 0x312, 2, u16>, // Low Power Oscillator Calibration Filter Constant
    pub LPOSCREF:       ReadWrite<'a, 0x314, 2, u16>, // Low Power Oscillator Calibration Reference
    pub LPOSCFREQ:      ReadWrite<'a, 0x316, 2, u16>, // Low Power Oscillator Calibration Frequency
    pub LPOSCPER:       ReadWrite<'a, 0x318, 2, u16>, // Low Power Oscillator Calibration Period
    /* DAC */
    pub DACVALUE:       ReadWrite<'a, 0x330, 2, u16>, // DAC Value
    pub DACCONFIG:      ReadWrite<'a, 0x332, 1, u8>,  // DAC Configuration
    /* Performance Tuning Registers */
    pub PERF_F00:       ReadWrite<'a, 0xF00, 1, u8>,
    pub PERF_F08:       ReadWrite<'a, 0xF08, 1, u8>,
    pub PERF_F0D:       ReadWrite<'a, 0xF0D, 1, u8>,
    pub PERF_F10:       ReadWrite<'a, 0xF10, 1, PerfF10>,
    pub PERF_F11:       ReadWrite<'a, 0xF11, 1, PerfF11>,
    pub PERF_F18:       ReadWrite<'a, 0xF18, 1, u8>,
    pub PERF_F1C:       ReadWrite<'a, 0xF1C, 1, u8>,
    pub PERF_F21:       ReadWrite<'a, 0xF21, 1, u8>,
    pub PERF_F22:       ReadWrite<'a, 0xF22, 1, u8>,
    pub PERF_F23:       ReadWrite<'a, 0xF23, 1, u8>,
    pub PERF_F26:       ReadWrite<'a, 0xF26, 1, u8>,
    pub PERF_F34:       ReadWrite<'a, 0xF34, 1, PerfF34>,
    pub PERF_F35:       ReadWrite<'a, 0xF35, 1, PerfF35>,
    pub PERF_F44:       ReadWrite<'a, 0xF44, 1, u8>,
}

pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Spidev> {
    let mut spi = Spidev::open(path)?;
    let options = SpidevOptions::new()
        .max_speed_hz(10_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options)?;
    Ok(spi)
}

pub fn registers<'a>(spi: &'a Spidev, on_status: &'a dyn Fn(Status)) -> Registers<'a> { // FIXME: new()?
    Registers {
        REVISION:       ReadOnly { spi, on_status, value: Default::default() },
        SCRATCH:        ReadWrite{ spi, on_status, value: Default::default() },
        PWRMODE:        ReadWrite{ spi, on_status, value: Default::default() },
        POWSTAT:        ReadOnly { spi, on_status, value: Default::default() },
        POWSTICKYSTAT:  ReadOnly { spi, on_status, value: Default::default() },
        POWIRQMASK:     ReadWrite{ spi, on_status, value: Default::default() },
        IRQMASK:        ReadWrite{ spi, on_status, value: Default::default() },
        RADIOEVENTMASK: ReadWrite{ spi, on_status, value: Default::default() },
        IRQINVERSION:   ReadWrite{ spi, on_status, value: Default::default() },
        IRQREQUEST:     ReadOnly { spi, on_status, value: Default::default() },
        RADIOEVENTREQ:  ReadOnly { spi, on_status, value: Default::default() },
        MODULATION:     ReadWrite{ spi, on_status, value: Default::default() },
        ENCODING:       ReadWrite{ spi, on_status, value: Default::default() },
        FRAMING:        ReadWrite{ spi, on_status, value: Default::default() },
        CRCINIT:        ReadWrite{ spi, on_status, value: Default::default() },
        FEC:            ReadWrite{ spi, on_status, value: Default::default() },
        FECSYNC:        ReadWrite{ spi, on_status, value: Default::default() },
        FECSTATUS:      ReadOnly { spi, on_status, value: Default::default() },
        RADIOSTATE:     ReadOnly { spi, on_status, value: Default::default() },
        XTALSTATUS:     ReadOnly { spi, on_status, value: Default::default() },
        PINSTATE:       ReadOnly { spi, on_status, value: Default::default() },
        PINFUNCSYSCLK:  ReadWrite{ spi, on_status, value: Default::default() },
        PINFUNCDCLK:    ReadWrite{ spi, on_status, value: Default::default() },
        PINFUNCDATA:    ReadWrite{ spi, on_status, value: Default::default() },
        PINFUNCIRQ:     ReadWrite{ spi, on_status, value: Default::default() },
        PINFUNCANTSEL:  ReadWrite{ spi, on_status, value: Default::default() },
        PINFUNCPWRAMP:  ReadWrite{ spi, on_status, value: Default::default() },
        PWRAMP:         ReadWrite{ spi, on_status, value: Default::default() },
        FIFOSTAT:       ReadOnly { spi, on_status, value: Default::default() },
        FIFOCMD:        WriteOnly{ spi, on_status, value: Default::default() },
        FIFODATA:       ReadWrite{ spi, on_status, value: Default::default() },
        FIFOCOUNT:      ReadOnly { spi, on_status, value: Default::default() },
        FIFOFREE:       ReadOnly { spi, on_status, value: Default::default() },
        FIFOTHRESH:     ReadWrite{ spi, on_status, value: Default::default() },
        PLLLOOP:        ReadWrite{ spi, on_status, value: Default::default() },
        PLLCPI:         ReadWrite{ spi, on_status, value: Default::default() },
        PLLVCODIV:      ReadWrite{ spi, on_status, value: Default::default() },
        PLLRANGINGA:    ReadWrite{ spi, on_status, value: Default::default() },
        FREQA:          ReadWrite{ spi, on_status, value: Default::default() },
        PLLLOOPBOOST:   ReadWrite{ spi, on_status, value: PLLLoop {
            filter: FLT::INTERNAL_x5,
            ..Default::default()
        }},
        PLLCPIBOOST:    ReadWrite{ spi, on_status, value: Default::default() },
        PLLRANGINGB:    ReadWrite{ spi, on_status, value: Default::default() },
        FREQB:          ReadWrite{ spi, on_status, value: Default::default() },
        RSSI:           ReadOnly { spi, on_status, value: Default::default() },
        BGNDRSSI:       ReadWrite{ spi, on_status, value: Default::default() },
        DIVERSITY:      ReadWrite{ spi, on_status, value: Default::default() },
        AGCCOUNTER:     ReadWrite{ spi, on_status, value: Default::default() },
        TRKDATARATE:    ReadOnly { spi, on_status, value: Default::default() },
        TRKAMPL:        ReadOnly { spi, on_status, value: Default::default() },
        TRKPHASE:       ReadOnly { spi, on_status, value: Default::default() },
        TRKRFFREQ:      ReadWrite{ spi, on_status, value: Default::default() },
        TRKFREQ:        ReadWrite{ spi, on_status, value: Default::default() },
        TRKFSKDEMOD:    ReadOnly { spi, on_status, value: Default::default() },
        TIMER2:         ReadOnly { spi, on_status, value: Default::default() },
        WAKEUPTIMER:    ReadOnly { spi, on_status, value: Default::default() },
        WAKEUP:         ReadWrite{ spi, on_status, value: Default::default() },
        WAKEUPFREQ:     ReadWrite{ spi, on_status, value: Default::default() },
        WAKEUPXOEARLY:  ReadWrite{ spi, on_status, value: Default::default() },
        IFFREQ:         ReadWrite{ spi, on_status, value: Default::default() },
        DECIMATION:     ReadWrite{ spi, on_status, value: Default::default() },
        RXDATARATE:     ReadWrite{ spi, on_status, value: Default::default() },
        MAXDROFFSET:    ReadWrite{ spi, on_status, value: Default::default() },
        MAXRFOFFSET:    ReadWrite{ spi, on_status, value: Default::default() },
        FSKDMAX:        ReadWrite{ spi, on_status, value: Default::default() },
        FSKDMIN:        ReadWrite{ spi, on_status, value: Default::default() },
        AFSKSPACE:      ReadWrite{ spi, on_status, value: Default::default() },
        AFSKMARK:       ReadWrite{ spi, on_status, value: Default::default() },
        AFSKCTRL:       ReadWrite{ spi, on_status, value: Default::default() },
        AMPLFILTER:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQUENCYLEAK:  ReadWrite{ spi, on_status, value: Default::default() },
        RXPARAMSETS:    ReadWrite{ spi, on_status, value: Default::default() },
        RXPARAMCURSET:  ReadOnly { spi, on_status, value: Default::default() },
        AGCGAIN0:       ReadWrite{ spi, on_status, value: Default::default() },
        AGCTARGET0:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCAHYST0:      ReadWrite{ spi, on_status, value: Default::default() },
        AGCMINMAX0:     ReadWrite{ spi, on_status, value: Default::default() },
        TIMEGAIN0:      ReadWrite{ spi, on_status, value: Default::default() },
        DRGAIN0:        ReadWrite{ spi, on_status, value: Default::default() },
        PHASEGAIN0:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINA0:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINB0:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINC0:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAIND0:     ReadWrite{ spi, on_status, value: Default::default() },
        AMPLGAIN0:      ReadWrite{ spi, on_status, value: Default::default() },
        FREQDEV0:       ReadWrite{ spi, on_status, value: Default::default() },
        FOURFSK0:       ReadWrite{ spi, on_status, value: Default::default() },
        BBOFFSRES0:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCGAIN1:       ReadWrite{ spi, on_status, value: Default::default() },
        AGCTARGET1:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCAHYST1:      ReadWrite{ spi, on_status, value: Default::default() },
        AGCMINMAX1:     ReadWrite{ spi, on_status, value: Default::default() },
        TIMEGAIN1:      ReadWrite{ spi, on_status, value: Default::default() },
        DRGAIN1:        ReadWrite{ spi, on_status, value: Default::default() },
        PHASEGAIN1:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINA1:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINB1:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINC1:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAIND1:     ReadWrite{ spi, on_status, value: Default::default() },
        AMPLGAIN1:      ReadWrite{ spi, on_status, value: Default::default() },
        FREQDEV1:       ReadWrite{ spi, on_status, value: Default::default() },
        FOURFSK1:       ReadWrite{ spi, on_status, value: Default::default() },
        BBOFFSRES1:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCGAIN2:       ReadWrite{ spi, on_status, value: Default::default() },
        AGCTARGET2:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCAHYST2:      ReadWrite{ spi, on_status, value: Default::default() },
        AGCMINMAX2:     ReadWrite{ spi, on_status, value: Default::default() },
        TIMEGAIN2:      ReadWrite{ spi, on_status, value: Default::default() },
        DRGAIN2:        ReadWrite{ spi, on_status, value: Default::default() },
        PHASEGAIN2:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINA2:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINB2:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINC2:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAIND2:     ReadWrite{ spi, on_status, value: Default::default() },
        AMPLGAIN2:      ReadWrite{ spi, on_status, value: Default::default() },
        FREQDEV2:       ReadWrite{ spi, on_status, value: Default::default() },
        FOURFSK2:       ReadWrite{ spi, on_status, value: Default::default() },
        BBOFFSRES2:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCGAIN3:       ReadWrite{ spi, on_status, value: Default::default() },
        AGCTARGET3:     ReadWrite{ spi, on_status, value: Default::default() },
        AGCAHYST3:      ReadWrite{ spi, on_status, value: Default::default() },
        AGCMINMAX3:     ReadWrite{ spi, on_status, value: Default::default() },
        TIMEGAIN3:      ReadWrite{ spi, on_status, value: Default::default() },
        DRGAIN3:        ReadWrite{ spi, on_status, value: Default::default() },
        PHASEGAIN3:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINA3:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINB3:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAINC3:     ReadWrite{ spi, on_status, value: Default::default() },
        FREQGAIND3:     ReadWrite{ spi, on_status, value: Default::default() },
        AMPLGAIN3:      ReadWrite{ spi, on_status, value: Default::default() },
        FREQDEV3:       ReadWrite{ spi, on_status, value: Default::default() },
        FOURFSK3:       ReadWrite{ spi, on_status, value: Default::default() },
        BBOFFSRES3:     ReadWrite{ spi, on_status, value: Default::default() },
        MODCFGF:        ReadWrite{ spi, on_status, value: Default::default() },
        FSKDEV:         ReadWrite{ spi, on_status, value: Default::default() },
        MODCFGA:        ReadWrite{ spi, on_status, value: Default::default() },
        TXRATE:         ReadWrite{ spi, on_status, value: Default::default() },
        TXPWRCOEFFA:    ReadWrite{ spi, on_status, value: Default::default() },
        TXPWRCOEFFB:    ReadWrite{ spi, on_status, value: Default::default() },
        TXPWRCOEFFC:    ReadWrite{ spi, on_status, value: Default::default() },
        TXPWRCOEFFD:    ReadWrite{ spi, on_status, value: Default::default() },
        TXPWRCOEFFE:    ReadWrite{ spi, on_status, value: Default::default() },
        PLLVCOI:        ReadWrite{ spi, on_status, value: Default::default() },
        PLLVCOIR:       ReadWrite{ spi, on_status, value: Default::default() },
        PLLLOCKDET:     ReadWrite{ spi, on_status, value: Default::default() },
        PLLRNGCLK:      ReadWrite{ spi, on_status, value: Default::default() },
        XTALCAP:        ReadWrite{ spi, on_status, value: Default::default() },
        BBTUNE:         ReadWrite{ spi, on_status, value: Default::default() },
        BBOFFSCAP:      ReadWrite{ spi, on_status, value: Default::default() },
        PKTADDRCFG:     ReadWrite{ spi, on_status, value: Default::default() },
        PKTLENCFG:      ReadWrite{ spi, on_status, value: Default::default() },
        PKTLENOFFSET:   ReadWrite{ spi, on_status, value: Default::default() },
        PKTMAXLEN:      ReadWrite{ spi, on_status, value: Default::default() },
        PKTADDR:        ReadWrite{ spi, on_status, value: Default::default() },
        PKTADDRMASK:    ReadWrite{ spi, on_status, value: Default::default() },
        MATCH0PAT:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH0LEN:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH0MIN:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH0MAX:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH1PAT:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH1LEN:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH1MIN:      ReadWrite{ spi, on_status, value: Default::default() },
        MATCH1MAX:      ReadWrite{ spi, on_status, value: Default::default() },
        TMGTXBOOST:     ReadWrite{ spi, on_status, value: Default::default() },
        TMGTXSETTLE:    ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXBOOST:     ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXSETTLE:    ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXOFFSACQ:   ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXCOARSEAGC: ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXAGC:       ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXRSSI:      ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXPREAMBLE1: ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXPREAMBLE2: ReadWrite{ spi, on_status, value: Default::default() },
        TMGRXPREAMBLE3: ReadWrite{ spi, on_status, value: Default::default() },
        RSSIREFERENCE:  ReadWrite{ spi, on_status, value: Default::default() },
        RSSIABSTHR:     ReadWrite{ spi, on_status, value: Default::default() },
        BGNDRSSIGAIN:   ReadWrite{ spi, on_status, value: Default::default() },
        BGNDRSSITHR:    ReadWrite{ spi, on_status, value: Default::default() },
        PKTCHUNKSIZE:   ReadWrite{ spi, on_status, value: Default::default() },
        PKTMISCFLAGS:   ReadWrite{ spi, on_status, value: Default::default() },
        PKTSTOREFLAGS:  ReadWrite{ spi, on_status, value: Default::default() },
        PKTACCEPTFLAGS: ReadWrite{ spi, on_status, value: Default::default() },
        GPADCCTRL:      ReadWrite{ spi, on_status, value: Default::default() },
        GPADCPERIOD:    ReadWrite{ spi, on_status, value: Default::default() },
        GPADC13VALUE:   ReadOnly { spi, on_status, value: Default::default() },
        LPOSCCONFIG:    ReadWrite{ spi, on_status, value: Default::default() },
        LPOSCSTATUS:    ReadOnly { spi, on_status, value: Default::default() },
        LPOSCKFILT:     ReadWrite{ spi, on_status, value: Default::default() },
        LPOSCREF:       ReadWrite{ spi, on_status, value: Default::default() },
        LPOSCFREQ:      ReadWrite{ spi, on_status, value: Default::default() },
        LPOSCPER:       ReadWrite{ spi, on_status, value: Default::default() },
        DACVALUE:       ReadWrite{ spi, on_status, value: Default::default() },
        DACCONFIG:      ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F00:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F08:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F0D:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F10:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F11:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F18:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F1C:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F21:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F22:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F23:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F26:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F34:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F35:       ReadWrite{ spi, on_status, value: Default::default() },
        PERF_F44:       ReadWrite{ spi, on_status, value: Default::default() },
    }
}
impl Registers<'_> {
    pub fn reset(&mut self) -> io::Result<()> {
        self.PWRMODE.write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags:  PwrFlags::RST,
        })?;

        self.PWRMODE.write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::empty(),
        })?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(4, 4);
    }
}

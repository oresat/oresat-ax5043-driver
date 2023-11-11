extern crate spidev;
use bitflags::bitflags;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::convert::TryFrom;
use std::path::Path;
use std::{fmt, fmt::Debug, io, collections::VecDeque};

use registers::*;

pub mod config;
pub mod registers;
pub mod tui;

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
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

// FIXME typealias for TryFrom + Into?
pub struct ReadOnly<'a, const ADDR: u16, const S: usize, R: TryFrom<Reg<S>> + Debug> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: R,
}

impl<const ADDR: u16, const S: usize, R: TryFrom<Reg<S>> + Copy + Debug> Debug
    for ReadOnly<'_, ADDR, S, R>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReadOnly")
            .field("spi", &self.spi)
            .field("value", &self.value)
            .finish()
    }
}

impl<const ADDR: u16, const S: usize, R: TryFrom<Reg<S>> + Copy + Debug> ReadOnly<'_, ADDR, S, R> {
    pub fn read(&mut self) -> io::Result<R>
    where
        <R as TryFrom<Reg<S>>>::Error: Debug,
    {
        let addr = (ADDR | 0x7000).to_be_bytes();
        let mut stat = [0; 2];

        let tx: [u8; S] = [0; S];
        let mut rx: [u8; S] = [0; S];
        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;

        //println!("{:?} Read  0x{:03X}: {:X?}", self.spi.inner(), ADDR, rx);
        self.value = Reg(rx).try_into().unwrap();
        (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
        Ok(self.value)
    }
}

pub struct ReadWrite<
    'a,
    const ADDR: u16,
    const S: usize,
    RW: TryFrom<Reg<S>> + Into<Reg<S>> + Copy + Debug,
> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: RW,
}

impl<const ADDR: u16, const S: usize, RW: TryFrom<Reg<S>> + Into<Reg<S>> + Copy + Debug> Debug
    for ReadWrite<'_, ADDR, S, RW>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReadWrite")
            .field("spi", &self.spi)
            .field("value", &self.value)
            .finish()
    }
}

impl<const ADDR: u16, const S: usize, RW: TryFrom<Reg<S>> + Into<Reg<S>> + Copy + Debug>
    ReadWrite<'_, ADDR, S, RW>
{
    pub fn read(&mut self) -> io::Result<RW>
    where
        <RW as TryFrom<Reg<S>>>::Error: Debug,
    {
        let addr = (ADDR | 0x7000).to_be_bytes();
        let mut stat = [0; 2];

        let tx: [u8; S] = [0; S];
        let mut rx: [u8; S] = [0; S];
        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;

        //println!("{:?} Read  0x{:03X}: {:X?}", self.spi.inner(), ADDR, rx);
        self.value = Reg(rx).try_into().unwrap();
        (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
        Ok(self.value)
    }

    pub fn writeval(&self) -> io::Result<()> {
        let addr = (ADDR | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = self.value.into().0;
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

pub struct WriteOnly<'a, const ADDR: u16, const S: usize, W: Into<Reg<S>> + Copy> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: W,
}

impl<const ADDR: u16, const S: usize, W: Into<Reg<S>> + Copy + Debug> Debug
    for WriteOnly<'_, ADDR, S, W>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WriteOnly")
            .field("spi", &self.spi)
            .field("value", &self.value)
            .finish()
    }
}

impl<const ADDR: u16, const S: usize, W: Into<Reg<S>> + Copy> WriteOnly<'_, ADDR, S, W> {
    pub fn writeval(&self) -> io::Result<()> {
        let addr = (ADDR | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = self.value.into().0;
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

pub struct ReadFIFO<'a, const ADDR: u16, const S: usize, R: TryFrom<Vec<u8>>> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: R,
}

impl<const ADDR: u16, const S: usize, R: TryFrom<Vec<u8>> + Debug> Debug
    for ReadFIFO<'_, ADDR, S, R>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReadFIFO")
            .field("spi", &self.spi)
            .field("value", &self.value)
            .finish()
    }
}

impl<const ADDR: u16, const S: usize, R: TryFrom<Vec<u8>>> ReadFIFO<'_, ADDR, S, R> {
    pub fn read(&mut self, len: usize) -> io::Result<Vec<R>>
    where
        <R as TryFrom<Vec<u8>>>::Error: Debug,
    {
        let addr = (ADDR | 0x7000).to_be_bytes();
        let mut stat = [0; 2];

        // All RX chunks are either a fixed length of at least 2 bytes or
        // a variable length with the length in the second byte.
        let mut tx = vec![0; len];
        let mut rx = vec![0; len];

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;
/*
        let len: usize = match FIFOChunkHeaderRX::try_from(rx[0]) {
            Ok(FIFOChunkHeaderRX::RSSI)       => 0,
            Ok(FIFOChunkHeaderRX::FREQOFFS)   => 1,
            Ok(FIFOChunkHeaderRX::ANTRSSI2)   => 1,
            Ok(FIFOChunkHeaderRX::TIMER)      => 2,
            Ok(FIFOChunkHeaderRX::RFFREQOFFS) => 2,
            Ok(FIFOChunkHeaderRX::DATARATE)   => 2,
            Ok(FIFOChunkHeaderRX::ANTRSSI3)   => 2,
            Ok(FIFOChunkHeaderRX::DATA)       => rx[1].into(),
            Err(_) => todo!()
        };
        tx.resize(2 + len, 0);
        rx.resize(2 + len, 0);

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&tx[2..], &mut rx[2..]),
        ])?;
*/
        let mut chunks: Vec<R> = Vec::new();
        let mut bytes = VecDeque::from(rx);
        while bytes.len() > 0 {
            let len: usize = match FIFOChunkHeaderRX::try_from(bytes[0]) {
                Ok(FIFOChunkHeaderRX::RSSI)       => 2,
                Ok(FIFOChunkHeaderRX::FREQOFFS)   => 3,
                Ok(FIFOChunkHeaderRX::ANTRSSI2)   => 3,
                Ok(FIFOChunkHeaderRX::TIMER)      => 4,
                Ok(FIFOChunkHeaderRX::RFFREQOFFS) => 4,
                Ok(FIFOChunkHeaderRX::DATARATE)   => 4,
                Ok(FIFOChunkHeaderRX::ANTRSSI3)   => 4,
                Ok(FIFOChunkHeaderRX::DATA)       => (bytes[1] + 2).into(),
                Err(_) => todo!()
            };

            let remaining = bytes.split_off(len);
            let chunk:Vec<u8> = bytes.into();
            chunks.push(chunk.try_into().unwrap());
            bytes = remaining;
        }
        //self.value = rx.try_into().unwrap();
        (self.on_status)(Status::from_bits(u16::from_be_bytes(stat)).unwrap());
        Ok(chunks)
    }
}

pub struct WriteFIFO<'a, const ADDR: u16, const S: usize, W: Into<Vec<u8>>> {
    pub spi: &'a Spidev,
    on_status: &'a dyn Fn(Status),
    pub value: W,
}

impl<const ADDR: u16, const S: usize, W: Into<Vec<u8>> + Debug> Debug
    for WriteFIFO<'_, ADDR, S, W>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WriteFIFO")
            .field("spi", &self.spi)
            .field("value", &self.value)
            .finish()
    }
}

impl<const ADDR: u16, const S: usize, W: Into<Vec<u8>> + Clone> WriteFIFO<'_, ADDR, S, W> {
    pub fn writeval(&self) -> io::Result<()> {
        let addr = (ADDR | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = &self.value.clone().into()[..];
        let rx: &mut [u8] = &mut vec![0; tx.len()];

        //println!("{:?} Write 0x{:03X}: {:X?}", self.spi.inner(), ADDR, tx);

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(tx, rx),
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

/* Adapted from AX5043 Programming Manual, Table 22 */
#[derive(Debug)]
#[allow(non_snake_case)]
#[rustfmt::skip]
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
    pub FEC:            ReadWrite<'a, 0x018, 1, FEC>,       // FEC (Viterbi) Configuration
    pub FECSYNC:        ReadWrite<'a, 0x019, 1, u8>,        // Interleaver Synchronisation Threshold
    pub FECSTATUS:      ReadOnly <'a, 0x01A, 1, FECStatus>, // FEC Status
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
    pub PWRAMP:         ReadWrite<'a, 0x027, 1, PwrAmp>,   // PWRAMP Control
    /* FIFO */
    pub FIFOSTAT:       ReadOnly <'a, 0x028, 1, FIFOStat>,    // FIFO Control
    pub FIFOCMD:        WriteOnly<'a, 0x028, 1, FIFOCmd>,     // FIFO Control
    pub FIFODATA:       ReadWrite<'a, 0x029, 1, u8>,          // FIFO Data
    pub FIFODATARX:     ReadFIFO <'a, 0x029, 1, FIFOChunkRX>, // FIFO Data
    pub FIFODATATX:     WriteFIFO<'a, 0x029, 1, FIFOChunkTX>, // FIFO Data
    pub FIFOCOUNT:      ReadOnly <'a, 0x02A, 2, u16>,         // Number of Words currently in FIFO
    pub FIFOFREE:       ReadOnly <'a, 0x02C, 2, u16>,         // Number of Words that can be written to FIFO
    pub FIFOTHRESH:     ReadWrite<'a, 0x02E, 2, u16>,         // FIFO Threshold
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
    pub RSSI:           ReadOnly <'a, 0x040, 1, i8>,        // Received Signal Strength Indicator
    pub BGNDRSSI:       ReadWrite<'a, 0x041, 1, u8>,        // Background RSSI
    pub DIVERSITY:      ReadWrite<'a, 0x042, 1, Diversity>, // Antenna Diversity Configuration
    pub AGCCOUNTER:     ReadWrite<'a, 0x043, 1, i8>,        // AGC Current Value
    /* Receiver Tracking */
    pub TRKDATARATE:    ReadOnly <'a, 0x045, 3, i32>,       // Datarate Tracking
    pub TRKAMPL:        ReadOnly <'a, 0x048, 2, u16>,       // Amplitude Tracking
    pub TRKPHASE:       ReadOnly <'a, 0x04A, 2, u16>,       // Phase Tracking
    pub TRKRFFREQ:      ReadWrite<'a, 0x04D, 3, TrkRFFreq>, // RF Frequency Tracking
    pub TRKFREQ:        ReadWrite<'a, 0x050, 2, i16>,       // Frequency Tracking
    pub TRKFSKDEMOD:    ReadOnly <'a, 0x052, 2, u16>,       // FSK Demodulator Tracking
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
    pub PKTADDRCFG:     ReadWrite<'a, 0x200, 1, PktAddrCfg>, // Packet Address Config
    pub PKTLENCFG:      ReadWrite<'a, 0x201, 1, PktLenCfg>,  // Packet Length Config
    pub PKTLENOFFSET:   ReadWrite<'a, 0x202, 1, u8>,         // Packet Length Offset
    pub PKTMAXLEN:      ReadWrite<'a, 0x203, 1, u8>,         // Packet Maximum Length
    pub PKTADDR:        ReadWrite<'a, 0x204, 4, u32>,        // Packet Address 3
    pub PKTADDRMASK:    ReadWrite<'a, 0x208, 4, u32>,        // Packet Address Mask 3
    /* Pattern Match */
    pub MATCH0PAT:      ReadWrite<'a, 0x210, 4, u32>,      // Pattern Match Unit 0, Pattern
    pub MATCH0LEN:      ReadWrite<'a, 0x214, 1, MatchLen>, // Pattern Match Unit 0, Pattern Length
    pub MATCH0MIN:      ReadWrite<'a, 0x215, 1, u8>,       // Pattern Match Unit 0, Minimum Match
    pub MATCH0MAX:      ReadWrite<'a, 0x216, 1, u8>,       // Pattern Match Unit 0, Maximum Match
    pub MATCH1PAT:      ReadWrite<'a, 0x218, 2, u16>,      // Pattern Match Unit 1, Pattern
    pub MATCH1LEN:      ReadWrite<'a, 0x21C, 1, MatchLen>, // Pattern Match Unit 1, Pattern Length
    pub MATCH1MIN:      ReadWrite<'a, 0x21D, 1, u8>,       // Pattern Match Unit 1, Minimum Match
    pub MATCH1MAX:      ReadWrite<'a, 0x21E, 1, u8>,       // Pattern Match Unit 1, Maximum Match
    /* Packet Controller */
    pub TMGTXBOOST:     ReadWrite<'a, 0x220, 1, TMG>,            // Transmit PLL Boost Time
    pub TMGTXSETTLE:    ReadWrite<'a, 0x221, 1, TMG>,            // Transmit PLL (post Boost) Settling Time
    pub TMGRXBOOST:     ReadWrite<'a, 0x223, 1, TMG>,            // Receive PLL Boost Time
    pub TMGRXSETTLE:    ReadWrite<'a, 0x224, 1, TMG>,            // Receive PLL (post Boost) Settling Time
    pub TMGRXOFFSACQ:   ReadWrite<'a, 0x225, 1, TMG>,            // Receive Baseband DC Offset Acquisition Time
    pub TMGRXCOARSEAGC: ReadWrite<'a, 0x226, 1, TMG>,            // Receive Coarse AGC Time
    pub TMGRXAGC:       ReadWrite<'a, 0x227, 1, TMG>,            // Receiver AGC Settling Time
    pub TMGRXRSSI:      ReadWrite<'a, 0x228, 1, TMG>,            // Receiver RSSI Settling Time
    pub TMGRXPREAMBLE1: ReadWrite<'a, 0x229, 1, TMG>,            // Receiver Preamble 1 Timeout
    pub TMGRXPREAMBLE2: ReadWrite<'a, 0x22A, 1, TMG>,            // Receiver Preamble 2 Timeout
    pub TMGRXPREAMBLE3: ReadWrite<'a, 0x22B, 1, TMG>,            // Receiver Preamble 3 Timeout
    pub RSSIREFERENCE:  ReadWrite<'a, 0x22C, 1, i8>,             // RSSI Offset
    pub RSSIABSTHR:     ReadWrite<'a, 0x22D, 1, u8>,             // RSSI Absolute Threshold
    pub BGNDRSSIGAIN:   ReadWrite<'a, 0x22E, 1, u8>,             // Background RSSI Averaging Time Constant
    pub BGNDRSSITHR:    ReadWrite<'a, 0x22F, 1, u8>,             // Background RSSI Relative Threshold
    pub PKTCHUNKSIZE:   ReadWrite<'a, 0x230, 1, u8>,             // Packet Chunk Size
    pub PKTMISCFLAGS:   ReadWrite<'a, 0x231, 1, PktMiscFlags>,   // Packet Controller Miscellaneous Flags
    pub PKTSTOREFLAGS:  ReadWrite<'a, 0x232, 1, PktStoreFlags>,  // Packet Controller Store Flags
    pub PKTACCEPTFLAGS: ReadWrite<'a, 0x233, 1, PktAcceptFlags>, // Packet Controller Accept Flags
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
    pub PERF_F72:       ReadWrite<'a, 0xF72, 1, u8>,
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

impl Registers<'_> {
    #[rustfmt::skip]
    pub fn new<'a>(spi: &'a Spidev, on_status: &'a dyn Fn(Status)) -> Registers<'a> {
        // Default vaules from PM Table 22
        Registers {
            REVISION:       ReadOnly { spi, on_status, value: 0b0101_0001 },
            SCRATCH:        ReadWrite{ spi, on_status, value: 0b1100_0101 },
            PWRMODE:        ReadWrite{ spi, on_status, value: PwrMode {
                mode: PwrModes::POWEROFF,
                flags: PwrFlags::REFEN | PwrFlags::XOEN,
            }},
            POWSTAT:        ReadOnly { spi, on_status, value: PowStat::empty() },
            POWSTICKYSTAT:  ReadOnly { spi, on_status, value: PowStat::empty() },
            POWIRQMASK:     ReadWrite{ spi, on_status, value: PowIRQMask::empty() },
            IRQMASK:        ReadWrite{ spi, on_status, value: IRQ::empty() },
            RADIOEVENTMASK: ReadWrite{ spi, on_status, value: RadioEvent::empty() },
            IRQINVERSION:   ReadWrite{ spi, on_status, value: IRQ::empty() },
            IRQREQUEST:     ReadOnly { spi, on_status, value: IRQ::empty() },
            RADIOEVENTREQ:  ReadOnly { spi, on_status, value: RadioEvent::empty() },
            MODULATION:     ReadWrite{ spi, on_status, value: Modulation {
                mode: ModulationMode::FSK,
                halfspeed: false,
            }},
            ENCODING:       ReadWrite{ spi, on_status, value: Encoding::DIFF },
            FRAMING:        ReadWrite{ spi, on_status, value: Framing {
                frmmode: FrameMode::RAW,
                crcmode: CRCMode::OFF,
                flags: FramingFlags::empty(),
            }},
            CRCINIT:        ReadWrite{ spi, on_status, value: 0xFFFF_FFFF },
            FEC:            ReadWrite{ spi, on_status, value: FEC {
                flags: FECFlags::empty(),
                inpshift: 0,
            }},
            FECSYNC:        ReadWrite{ spi, on_status, value: 0b0110_0010 },
            FECSTATUS:      ReadOnly { spi, on_status, value: FECStatus {
                max_metric: 0,
                inv: false,
            }},
            RADIOSTATE:     ReadOnly { spi, on_status, value: RadioState::IDLE },
            XTALSTATUS:     ReadOnly { spi, on_status, value: XtalStatus::empty() },
            PINSTATE:       ReadOnly { spi, on_status, value: PinState::empty() },
            PINFUNCSYSCLK:  ReadWrite{ spi, on_status, value: PFSysClk {
                mode: PFSysClkMode::F_XTAL_DIV_16,
                pullup: false,
            }},
            PINFUNCDCLK:    ReadWrite{ spi, on_status, value: PFDClk {
                mode: PFDClkMode::OUT,
                flags: PFFlags::empty()
            }},
            PINFUNCDATA:    ReadWrite{ spi, on_status, value: PFData {
                mode: PFDataMode::MODEM_OUT,
                flags: PFFlags::PULLUP,
            }},
            PINFUNCIRQ:     ReadWrite{ spi, on_status, value: PFIRQ {
                mode: PFIRQMode::IRQ,
                flags: PFFlags::empty(),
            }},
            PINFUNCANTSEL:  ReadWrite{ spi, on_status, value: PFAntSel {
                mode: PFAntSelMode::ANTSEL,
                flags: PFFlags::empty(),
            }},
            PINFUNCPWRAMP:  ReadWrite{ spi, on_status, value: PFPwrAmp {
                mode: PFPwrAmpMode::PWRAMP,
                flags: PFFlags::empty(),
            }},
            PWRAMP:         ReadWrite{ spi, on_status, value: PwrAmp::empty() },
            FIFOSTAT:       ReadOnly { spi, on_status, value: FIFOStat::empty() },
            FIFOCMD:        WriteOnly{ spi, on_status, value: FIFOCmd {
                mode: FIFOCmds::NOP,
                auto_commit: false,
            }},
            FIFODATA:       ReadWrite{ spi, on_status, value: 0 },
            FIFODATARX:     ReadFIFO { spi, on_status, value: FIFOChunkRX::RSSI(0)}, // FIXME: NOP?
            FIFODATATX:     WriteFIFO{ spi, on_status, value: FIFOChunkTX::NOP },
            FIFOCOUNT:      ReadOnly { spi, on_status, value: 0 },
            FIFOFREE:       ReadOnly { spi, on_status, value: 0x100 },
            FIFOTHRESH:     ReadWrite{ spi, on_status, value: 0 },
            PLLLOOP:        ReadWrite{ spi, on_status, value: PLLLoop {
                filter: FLT::INTERNAL_x1,
                flags: PLLLoopFlags::DIRECT,
                freqsel: FreqSel::A,
            }},
            PLLCPI:         ReadWrite{ spi, on_status, value: 8 },
            PLLVCODIV:      ReadWrite{ spi, on_status, value: PLLVCODiv {
                mode: PLLVCORefDiv::F_XTAL,
                flags: PLLVCODivFlags::empty(),
            }},
            PLLRANGINGA:    ReadWrite{ spi, on_status, value: PLLRanging {
                vcor: 8,
                flags: PLLRangingFlags::empty(),
            }},
            FREQA:          ReadWrite{ spi, on_status, value: 0x3934_CCCD },
            PLLLOOPBOOST:   ReadWrite{ spi, on_status, value: PLLLoop {
                filter: FLT::INTERNAL_x5,
                flags: PLLLoopFlags::DIRECT,
                freqsel: FreqSel::A,
            }},
            PLLCPIBOOST:    ReadWrite{ spi, on_status, value: 0xC8},
            PLLRANGINGB:    ReadWrite{ spi, on_status, value: PLLRanging {
                vcor: 8,
                flags: PLLRangingFlags::empty(),
            }},
            FREQB:          ReadWrite{ spi, on_status, value: 0x3934_CCCD },
            RSSI:           ReadOnly { spi, on_status, value: 0 },
            BGNDRSSI:       ReadWrite{ spi, on_status, value: 0 },
            DIVERSITY:      ReadWrite{ spi, on_status, value: Diversity::empty() },
            AGCCOUNTER:     ReadWrite{ spi, on_status, value: 0 },
            TRKDATARATE:    ReadOnly { spi, on_status, value: 0 },
            TRKAMPL:        ReadOnly { spi, on_status, value: 0 },
            TRKPHASE:       ReadOnly { spi, on_status, value: 0 },
            TRKRFFREQ:      ReadWrite{ spi, on_status, value: TrkRFFreq(0) },
            TRKFREQ:        ReadWrite{ spi, on_status, value: 0 },
            TRKFSKDEMOD:    ReadOnly { spi, on_status, value: 0 },
            TIMER2:         ReadOnly { spi, on_status, value: 0 },
            WAKEUPTIMER:    ReadOnly { spi, on_status, value: 0 },
            WAKEUP:         ReadWrite{ spi, on_status, value: 0 },
            WAKEUPFREQ:     ReadWrite{ spi, on_status, value: 0 },
            WAKEUPXOEARLY:  ReadWrite{ spi, on_status, value: 0 },
            IFFREQ:         ReadWrite{ spi, on_status, value: 0x1327 },
            DECIMATION:     ReadWrite{ spi, on_status, value: 0x0D },
            RXDATARATE:     ReadWrite{ spi, on_status, value: 0x3D8A },
            MAXDROFFSET:    ReadWrite{ spi, on_status, value: 0x9E },
            MAXRFOFFSET:    ReadWrite{ spi, on_status, value: MaxRFOffset{
                offset: 0x1687,
                correction: false,
            }},
            FSKDMAX:        ReadWrite{ spi, on_status, value: 0x80 },
            FSKDMIN:        ReadWrite{ spi, on_status, value: -0x80 },
            AFSKSPACE:      ReadWrite{ spi, on_status, value: 0x40 },
            AFSKMARK:       ReadWrite{ spi, on_status, value: 0x75 },
            AFSKCTRL:       ReadWrite{ spi, on_status, value: 0x04 },
            AMPLFILTER:     ReadWrite{ spi, on_status, value: 0 },
            FREQUENCYLEAK:  ReadWrite{ spi, on_status, value: 0 },
            RXPARAMSETS:    ReadWrite{ spi, on_status, value: RxParamSets(
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
            )},
            RXPARAMCURSET:  ReadOnly { spi, on_status, value: RxParamCurSet {
                index: 0,
                number: RxParamSet::Set0,
                special: 0
            }},
            AGCGAIN0:       ReadWrite{ spi, on_status, value: AGCGain {
                attack: 0x4,
                decay: 0xB,
            }},
            AGCTARGET0:     ReadWrite{ spi, on_status, value: 0x76 },
            AGCAHYST0:      ReadWrite{ spi, on_status, value: AGCHyst { hyst: 0 }},
            AGCMINMAX0:     ReadWrite{ spi, on_status, value: AGCMinMax {
                min: 0,
                max: 0,
            }},
            TIMEGAIN0:      ReadWrite{ spi, on_status, value: TimeGain {
                exponent: 0,
                mantissa: 0x1F,
            }},
            DRGAIN0:        ReadWrite{ spi, on_status, value: DRGain {
                exponent: 2,
                mantissa: 0x1E,
            }},
            PHASEGAIN0:     ReadWrite{ spi, on_status, value: PhaseGain {
                gain: 3,
                filter: 3,
            }},
            FREQGAINA0:     ReadWrite{ spi, on_status, value: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            }},
            FREQGAINB0:     ReadWrite{ spi, on_status, value: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            }},
            FREQGAINC0:     ReadWrite{ spi, on_status, value: FreqGainC {
                gain: 0xA,
            }},
            FREQGAIND0:     ReadWrite{ spi, on_status, value: FreqGainD {
                gain: 0xA,
                freeze: false,
            }},
            AMPLGAIN0:      ReadWrite{ spi, on_status, value: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            }},
            FREQDEV0:       ReadWrite{ spi, on_status, value: 0x0020 },
            FOURFSK0:       ReadWrite{ spi, on_status, value: FourFSK {
                decay: 6,
                update: true,
            }},
            BBOFFSRES0:     ReadWrite{ spi, on_status, value: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            }},
            AGCGAIN1:       ReadWrite{ spi, on_status, value: AGCGain {
                attack: 0x4,
                decay: 0xB,
            }},
            AGCTARGET1:     ReadWrite{ spi, on_status, value: 0x76 },
            AGCAHYST1:      ReadWrite{ spi, on_status, value: AGCHyst { hyst: 0 }},
            AGCMINMAX1:     ReadWrite{ spi, on_status, value: AGCMinMax {
                min: 0,
                max: 0,
            }},
            TIMEGAIN1:      ReadWrite{ spi, on_status, value: TimeGain {
                exponent: 0x6,
                mantissa: 0xF,
            }},
            DRGAIN1:        ReadWrite{ spi, on_status, value: DRGain {
                exponent: 0x1,
                mantissa: 0xF,
            }},
            PHASEGAIN1:     ReadWrite{ spi, on_status, value: PhaseGain {
                gain: 3,
                filter: 3,
            }},
            FREQGAINA1:     ReadWrite{ spi, on_status, value: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            }},
            FREQGAINB1:     ReadWrite{ spi, on_status, value: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            }},
            FREQGAINC1:     ReadWrite{ spi, on_status, value: FreqGainC {
                gain: 0xB,
            }},
            FREQGAIND1:     ReadWrite{ spi, on_status, value: FreqGainD {
                gain: 0xB,
                freeze: false,
            }},
            AMPLGAIN1:      ReadWrite{ spi, on_status, value: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            }},
            FREQDEV1:       ReadWrite{ spi, on_status, value: 0x20 },
            FOURFSK1:       ReadWrite{ spi, on_status, value: FourFSK {
                decay: 0,
                update: true,
            }},
            BBOFFSRES1:     ReadWrite{ spi, on_status, value: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            }},
            AGCGAIN2:       ReadWrite{ spi, on_status, value: AGCGain {
                attack: 0xF,
                decay: 0xF,
            }},
            AGCTARGET2:     ReadWrite{ spi, on_status, value: 0x76 },
            AGCAHYST2:      ReadWrite{ spi, on_status, value: AGCHyst { hyst: 0 }},
            AGCMINMAX2:     ReadWrite{ spi, on_status, value: AGCMinMax {
                min: 0,
                max: 0,
            }},
            TIMEGAIN2:      ReadWrite{ spi, on_status, value: TimeGain {
                exponent: 0x5,
                mantissa: 0xF,
            }},
            DRGAIN2:        ReadWrite{ spi, on_status, value: DRGain {
                exponent: 0x0,
                mantissa: 0xF,
            }},
            PHASEGAIN2:     ReadWrite{ spi, on_status, value: PhaseGain {
                gain: 3,
                filter: 3,
            }},
            FREQGAINA2:     ReadWrite{ spi, on_status, value: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            }},
            FREQGAINB2:     ReadWrite{ spi, on_status, value: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            }},
            FREQGAINC2:     ReadWrite{ spi, on_status, value: FreqGainC {
                gain: 0xD,
            }},
            FREQGAIND2:     ReadWrite{ spi, on_status, value: FreqGainD {
                gain: 0xD,
                freeze: false,
            }},
            AMPLGAIN2:      ReadWrite{ spi, on_status, value: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            }},
            FREQDEV2:       ReadWrite{ spi, on_status, value: 0x20 },
            FOURFSK2:       ReadWrite{ spi, on_status, value: FourFSK {
                decay: 0xA,
                update: true,
            }},
            BBOFFSRES2:     ReadWrite{ spi, on_status, value: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            }},
            AGCGAIN3:       ReadWrite{ spi, on_status, value: AGCGain {
                attack: 0xF,
                decay: 0xF,
            }},
            AGCTARGET3:     ReadWrite{ spi, on_status, value: 0x76 },
            AGCAHYST3:      ReadWrite{ spi, on_status, value: AGCHyst { hyst: 0 }},
            AGCMINMAX3:     ReadWrite{ spi, on_status, value: AGCMinMax {
                min: 0,
                max: 0,
            }},
            TIMEGAIN3:      ReadWrite{ spi, on_status, value: TimeGain {
                exponent: 0x5,
                mantissa: 0xF,
            }},
            DRGAIN3:        ReadWrite{ spi, on_status, value: DRGain {
                exponent: 0x0,
                mantissa: 0xF,
            }},
            PHASEGAIN3:     ReadWrite{ spi, on_status, value: PhaseGain {
                gain: 3,
                filter: 3,
            }},
            FREQGAINA3:     ReadWrite{ spi, on_status, value: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            }},
            FREQGAINB3:     ReadWrite{ spi, on_status, value: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            }},
            FREQGAINC3:     ReadWrite{ spi, on_status, value: FreqGainC {
                gain: 0xD,
            }},
            FREQGAIND3:     ReadWrite{ spi, on_status, value: FreqGainD {
                gain: 0xD,
                freeze: false,
            }},
            AMPLGAIN3:      ReadWrite{ spi, on_status, value: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            }},
            FREQDEV3:       ReadWrite{ spi, on_status, value: 0x20 },
            FOURFSK3:       ReadWrite{ spi, on_status, value: FourFSK {
                decay: 0xA,
                update: true,
            }},
            BBOFFSRES3:     ReadWrite{ spi, on_status, value: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            }},
            MODCFGF:        ReadWrite{ spi, on_status, value: ModCfgF::UNSHAPED },
            FSKDEV:         ReadWrite{ spi, on_status, value: 0x00_0A3D },
            MODCFGA:        ReadWrite{ spi, on_status, value: ModCfgA {
                slowramp: SlowRamp::STARTUP_1b,
                flags: ModCfgAFlags::AMPLSHAPE | ModCfgAFlags::TXDIFF,
            }},
            TXRATE:         ReadWrite{ spi, on_status, value: 0x00_28F6 },
            TXPWRCOEFFA:    ReadWrite{ spi, on_status, value: 0x0000 },
            TXPWRCOEFFB:    ReadWrite{ spi, on_status, value: 0x0FFF },
            TXPWRCOEFFC:    ReadWrite{ spi, on_status, value: 0x0000 },
            TXPWRCOEFFD:    ReadWrite{ spi, on_status, value: 0x0000 },
            TXPWRCOEFFE:    ReadWrite{ spi, on_status, value: 0x0000 },
            PLLVCOI:        ReadWrite{ spi, on_status, value: PLLVCOI {
                bias: 0x12,
                flags: PLLVCOIFlags::empty(),
            }},
            PLLVCOIR:       ReadWrite{ spi, on_status, value: 0 },
            PLLLOCKDET:     ReadWrite{ spi, on_status, value: PLLLockDet {
                delay: LockDetDly::d14ns,
                flags: LockDetFlags::AUTOMATIC,
                readback: LockDetDly::d6ns,
            }},
            PLLRNGCLK:      ReadWrite{ spi, on_status, value: PLLRngClk::XTAL_DIV_2pow11 },
            XTALCAP:        ReadWrite{ spi, on_status, value: 0 },
            BBTUNE:         ReadWrite{ spi, on_status, value: 9 },
            BBOFFSCAP:      ReadWrite{ spi, on_status, value: 0x77},
            PKTADDRCFG:     ReadWrite{ spi, on_status, value: PktAddrCfg {
                addr_pos: 0,
                flags: PktAddrCfgFlags::FEC_SYNC_DIS,
            }},
            PKTLENCFG:      ReadWrite{ spi, on_status, value: PktLenCfg {
                bits: 0,
                pos: 0,
            }},
            PKTLENOFFSET:   ReadWrite{ spi, on_status, value: 0 },
            PKTMAXLEN:      ReadWrite{ spi, on_status, value: 0 },
            PKTADDR:        ReadWrite{ spi, on_status, value: 0 },
            PKTADDRMASK:    ReadWrite{ spi, on_status, value: 0 },
            MATCH0PAT:      ReadWrite{ spi, on_status, value: 0 },
            MATCH0LEN:      ReadWrite{ spi, on_status, value: MatchLen { len: 0, raw: false }},
            MATCH0MIN:      ReadWrite{ spi, on_status, value: 0 },
            MATCH0MAX:      ReadWrite{ spi, on_status, value: 0x1F },
            MATCH1PAT:      ReadWrite{ spi, on_status, value: 0 },
            MATCH1LEN:      ReadWrite{ spi, on_status, value: MatchLen { len: 0, raw: false }},
            MATCH1MIN:      ReadWrite{ spi, on_status, value: 0 },
            MATCH1MAX:      ReadWrite{ spi, on_status, value: 0xF },
            TMGTXBOOST:     ReadWrite{ spi, on_status, value: TMG { e: 2, m: 0x12 }},
            TMGTXSETTLE:    ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0x0A }},
            TMGRXBOOST:     ReadWrite{ spi, on_status, value: TMG { e: 1, m: 0x12 }},
            TMGRXSETTLE:    ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0x14 }},
            TMGRXOFFSACQ:   ReadWrite{ spi, on_status, value: TMG { e: 3, m: 0x13 }},
            TMGRXCOARSEAGC: ReadWrite{ spi, on_status, value: TMG { e: 1, m: 0x19 }},
            TMGRXAGC:       ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0 }},
            TMGRXRSSI:      ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0 }},
            TMGRXPREAMBLE1: ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0 }},
            TMGRXPREAMBLE2: ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0 }},
            TMGRXPREAMBLE3: ReadWrite{ spi, on_status, value: TMG { e: 0, m: 0 }},
            RSSIREFERENCE:  ReadWrite{ spi, on_status, value: 0 },
            RSSIABSTHR:     ReadWrite{ spi, on_status, value: 0 },
            BGNDRSSIGAIN:   ReadWrite{ spi, on_status, value: 0 },
            BGNDRSSITHR:    ReadWrite{ spi, on_status, value: 0 },
            PKTCHUNKSIZE:   ReadWrite{ spi, on_status, value: 0 },
            PKTMISCFLAGS:   ReadWrite{ spi, on_status, value: PktMiscFlags::empty() },
            PKTSTOREFLAGS:  ReadWrite{ spi, on_status, value: PktStoreFlags::empty() },
            PKTACCEPTFLAGS: ReadWrite{ spi, on_status, value: PktAcceptFlags::empty() },
            GPADCCTRL:      ReadWrite{ spi, on_status, value: 0 },
            GPADCPERIOD:    ReadWrite{ spi, on_status, value: 0x3F },
            GPADC13VALUE:   ReadOnly { spi, on_status, value: 0 },
            LPOSCCONFIG:    ReadWrite{ spi, on_status, value: 0 },
            LPOSCSTATUS:    ReadOnly { spi, on_status, value: 0 },
            LPOSCKFILT:     ReadWrite{ spi, on_status, value: 0x20C7 },
            LPOSCREF:       ReadWrite{ spi, on_status, value: 0x61A8 },
            LPOSCFREQ:      ReadWrite{ spi, on_status, value: 0 },
            LPOSCPER:       ReadWrite{ spi, on_status, value: 0 },
            DACVALUE:       ReadWrite{ spi, on_status, value: 0 },
            DACCONFIG:      ReadWrite{ spi, on_status, value: 0 },
            PERF_F00:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F08:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F0D:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F10:       ReadWrite{ spi, on_status, value: PerfF10::XO },
            PERF_F11:       ReadWrite{ spi, on_status, value: PerfF11::Reset },
            PERF_F18:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F1C:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F21:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F22:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F23:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F26:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F34:       ReadWrite{ spi, on_status, value: PerfF34::Reset },
            PERF_F35:       ReadWrite{ spi, on_status, value: PerfF35::FreqLT24p8MHz },
            PERF_F44:       ReadWrite{ spi, on_status, value: 0 },
            PERF_F72:       ReadWrite{ spi, on_status, value: 0 },
        }
    }

    pub fn reset(&mut self) -> io::Result<()> {
        self.PWRMODE.write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::RST,
        })?;

        self.PWRMODE.write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::empty(),
        })?;

        Ok(())
    }
}

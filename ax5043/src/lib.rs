use bitflags::bitflags;
use serde::{Deserialize, Serialize};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{
    collections::VecDeque, convert::TryFrom, fmt::Debug, io::Read, marker::PhantomData, path::Path,
};
use thiserror::Error;

use registers::*;

pub mod config;
pub mod registers;
pub mod tui;

// TODO: repurpose for fs/ccsds?
// GOALS: device state tracking, bind transport to state tracker
//
// bits AsIs/Preserve/Update?, DontCare, RW, RO, WO, W1C, RC, S?
//
// idea:
// ax5043.write(PwrMode { POWEROFF | TXEN | REFEN });
// enum PwrMode: u8 {
//     7 => RST,
//     6 => REFEN,
//     5 => XOEN,
//     4 => WDS [ RO ],
//     0..3 => {
//         POWEROFF  = 0b0000,
//         DEEPSLEEP = 0b0001,
//         XOEN      = 0b0101,
//         FIFOEN    = 0b0111,
//         SYNTHRX   = 0b1000,
//         RX        = 0b1001,
//         WORRX     = 0b1011,
//         SYNTHTX   = 0b1100,
//         TX        = 0b1101,
//     },
// }
//
//TODO: write has all fields of register? write(flags, mode)?
//TODO: write(u8/u16/u32 for direct hex)?
//TODO:
//   write(val: T) where T: Reg
//   impl<PwrMode> write(val: PwrMode)
//   impl<PowStat> write(val: PowStat)
//   This has a dowwnside that we need a unique type per register, no sharing.
//
//   What about an address wrapper around the types then?
//    PWRMODE(PwrMode{ })
//   Kinda boilerplate.
//   Do write(PwrMode) where unambiguous, but fall back to write(PWRMODE(PwrMode))?
//
//    construct inner type from values?
//    POWERMODE {
//      const addr: 0x002_u16
//      val: PwrMode
//    }
//    impl Into<POWERMODE> for PwrMode {}
//    write(POWEROFF | TXEN | REFEN)
//
//    REG<const A: u16, T> {
//      const addr: A
//      val: T
//    }
//    impl Into<REG<0x002, PwrMode>> for PwrMode {}
//

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
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

#[derive(Error, Debug)]
pub enum Error {
    #[error("SPI communication failed")]
    Io(#[from] std::io::Error),
    #[error("Invalid values returned from device")]
    Decode, // FIXME: recover bytes
    #[error("Invalid values: {}, {0:?}", .0.len())]
    DecodeBytes(Vec<u8>),
    #[error("Invalid header: {0:?}")]
    FIFOHeader(Vec<u8>),
    #[error("Invalid status (should not happen)")]
    Status([u8; 2]),
    #[error("Autoranging failed")]
    Autorange, // TODO: A vs B
    #[error("Invalid config setting")]
    Invalid, // FIXME: this is a generic catchall, should always be made specific
}

type Result<T> = std::result::Result<T, Error>;

pub trait IO {
    fn spi(&self) -> &Spidev;
    fn addr(&self) -> u16;
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]);
}

pub trait RX<const S: usize>: IO {
    type Value: TryFrom<Reg<S>>;
    fn read(&mut self) -> Result<Self::Value> {
        let addr = (self.addr() | 0x7000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = [0; S];
        let mut rx = [0; S];
        self.spi().transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;

        let status = Status::from_bits(u16::from_be_bytes(stat)).ok_or(Error::Status(stat))?;
        let data = Reg(rx).try_into().map_err(|_| Error::Decode)?;

        self.on_status(u16::from_be_bytes(addr), status, &rx);
        Ok(data)
    }
}

pub trait TX<const S: usize>: IO {
    type Value: Into<Reg<S>>;
    fn write(&mut self, value: Self::Value) -> Result<()> {
        let addr = (self.addr() | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = value.into().0;
        let mut rx: [u8; S] = [0; S];

        self.spi().transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;
        //assert_eq!(rx, [0; S]); fails TODO: what does this return? Old value? check that it
        //matches our previous known state?
        let status = Status::from_bits(u16::from_be_bytes(stat)).ok_or(Error::Status(stat))?;
        self.on_status(u16::from_be_bytes(addr), status, &tx);
        Ok(())
    }
}

pub struct ReadWrite<'a, const S: usize, V: TryFrom<Reg<S>> + Into<Reg<S>>> {
    data: PhantomData<V>,
    spi: &'a Spidev,
    addr: u16,
    on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
}

impl<const S: usize, V: TryFrom<Reg<S>> + Into<Reg<S>>> IO for ReadWrite<'_, S, V> {
    fn spi(&self) -> &Spidev {
        self.spi
    }
    fn addr(&self) -> u16 {
        self.addr
    }
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]) {
        (self.on_status)(self.spi, addr, status, data);
    }
}

impl<const S: usize, V: TryFrom<Reg<S>> + Into<Reg<S>>> RX<S> for ReadWrite<'_, S, V> {
    type Value = V;
}

impl<const S: usize, V: TryFrom<Reg<S>> + Into<Reg<S>>> TX<S> for ReadWrite<'_, S, V> {
    type Value = V;
}

pub struct ReadOnly<'a, const S: usize, V: TryFrom<Reg<S>>> {
    data: PhantomData<V>,
    spi: &'a Spidev,
    addr: u16,
    on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
}

impl<const S: usize, V: TryFrom<Reg<S>>> IO for ReadOnly<'_, S, V> {
    fn spi(&self) -> &Spidev {
        self.spi
    }
    fn addr(&self) -> u16 {
        self.addr
    }
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]) {
        (self.on_status)(self.spi, addr, status, data);
    }
}

impl<const S: usize, V: TryFrom<Reg<S>>> RX<S> for ReadOnly<'_, S, V> {
    type Value = V;
}

pub struct WriteOnly<'a, const S: usize, V: Into<Reg<S>>> {
    data: PhantomData<V>,
    spi: &'a Spidev,
    addr: u16,
    on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
}

impl<const S: usize, V: Into<Reg<S>>> IO for WriteOnly<'_, S, V> {
    fn spi(&self) -> &Spidev {
        self.spi
    }
    fn addr(&self) -> u16 {
        self.addr
    }
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]) {
        (self.on_status)(self.spi, addr, status, data);
    }
}

impl<const S: usize, V: Into<Reg<S>>> TX<S> for WriteOnly<'_, S, V> {
    type Value = V;
}

pub struct ReadFIFO<'a, const S: usize, V: TryFrom<Vec<u8>>> {
    data: PhantomData<V>,
    spi: &'a Spidev,
    addr: u16,
    on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
}

impl<const S: usize, V: TryFrom<Vec<u8>>> ReadFIFO<'_, S, V> {
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]) {
        (self.on_status)(self.spi, addr, status, data);
    }

    pub fn read(&mut self, len: usize) -> Result<Vec<V>>
    where
        <V as TryFrom<Vec<u8>>>::Error: Debug,
    {
        let addr = (self.addr | 0x7000).to_be_bytes();
        let mut stat = [0; 2];

        // All RX chunks are either a fixed length of at least 2 bytes or
        // a variable length with the length in the second byte, so I'd
        // like to do a read of 2 and then read the remaining chunk, but the
        // FIFO returns about 30 bytes of strange data before returning the
        // remaining chunk. Instead this reads all the bytes that FIFOCOUNT
        // says are available and then breaks it into one or more chunks.
        // TODO: figure out why and if we can just read one chunk at a time.
        let tx = vec![0; len];
        let mut rx = vec![0; len];

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(&tx, &mut rx),
        ])?;
        let mut chunks: Vec<V> = Vec::new();

        let mut bytes = VecDeque::from(rx.clone());
        while !bytes.is_empty() {
            #[rustfmt::skip]
            let chunksize: usize = match FIFOChunkHeaderRX::try_from(bytes[0]) {
                Ok(FIFOChunkHeaderRX::RSSI)       => 2,
                Ok(FIFOChunkHeaderRX::FREQOFFS)   => 3,
                Ok(FIFOChunkHeaderRX::ANTRSSI2)   => 3,
                Ok(FIFOChunkHeaderRX::TIMER)      => 4,
                Ok(FIFOChunkHeaderRX::RFFREQOFFS) => 4,
                Ok(FIFOChunkHeaderRX::DATARATE)   => 4,
                Ok(FIFOChunkHeaderRX::ANTRSSI3)   => 4,
                Ok(FIFOChunkHeaderRX::DATA)       => usize::from(bytes[1]) + 2,
                Err(_) => return Err(Error::FIFOHeader(bytes.clone().into())),
            };

            if bytes.len() < chunksize {
                return Err(Error::DecodeBytes(bytes.clone().into()));
            }
            let mut chunk = vec![0; chunksize];
            bytes.read_exact(&mut chunk)?;
            chunks.push(chunk.clone().try_into().map_err(|_| Error::DecodeBytes(chunk.clone()))?);
        }
        let status = Status::from_bits(u16::from_be_bytes(stat)).ok_or(Error::Status(stat))?;
        self.on_status(u16::from_be_bytes(addr), status, &rx);
        Ok(chunks)
    }
}

pub struct WriteFIFO<'a, const S: usize, V: Into<Vec<u8>>> {
    data: PhantomData<V>,
    spi: &'a Spidev,
    addr: u16,
    on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
}

impl<const S: usize, V: Into<Vec<u8>>> WriteFIFO<'_, S, V> {
    fn on_status(&mut self, addr: u16, status: Status, data: &[u8]) {
        (self.on_status)(self.spi, addr, status, data);
    }

    pub fn write(&mut self, value: V) -> Result<()> {
        let addr = (self.addr | 0xF000).to_be_bytes();
        let mut stat = [0; 2];

        let tx = &value.into()[..];
        let rx: &mut [u8] = &mut vec![0; tx.len()];

        self.spi.transfer_multiple(&mut [
            SpidevTransfer::read_write(&addr, &mut stat),
            SpidevTransfer::read_write(tx, rx),
        ])?;
        //assert_eq!(rx, [0; S]); fails TODO: what does this return? Old value? check that it
        //matches our previous known state?
        let status = Status::from_bits(u16::from_be_bytes(stat)).ok_or(Error::Status(stat))?;
        self.on_status(u16::from_be_bytes(addr), status, tx);
        Ok(())
    }
}

// Name: Type [Addr, Width, Access],
macro_rules! registers {
    (
        $vis:vis struct $name:ident {
            $($reg:ident: $T:ty [ $addr:literal, $width:literal, $access:ident ],)*
        }
    ) => {

        #[allow(non_snake_case)]
        pub struct $name<'a> {
            spi: Spidev,
            on_status: &'a mut dyn FnMut(&Spidev, u16, Status, &[u8]),
            $(pub $reg: $T,)*
        }

        #[allow(non_snake_case)]
        impl $name<'_> {
            $(
                pub fn $reg(&mut self) -> $access<$width, $T> {
                    $access {
                        data: PhantomData,
                        spi: &self.spi,
                        addr: $addr,
                        on_status: &mut self.on_status,
                    }
                }
            )*
        }
    }
}

/* Adapted from AX5043 Programming Manual, Table 22 */
//#[derive(Debug)]
//#[allow(non_snake_case)]
//#[rustfmt::skip]

registers! {
    pub struct Registers {
        /* Revision & Interface Probing */
        REVISION:       u8          [0x000, 1, ReadOnly ], // Silicon Revision
        SCRATCH:        u8          [0x001, 1, ReadWrite], // Scratch Register
        /* Operating Mode */
        PWRMODE:        PwrMode     [0x002, 1, ReadWrite], // Power Mode
        /* Voltage Regulator */
        POWSTAT:        PowStat     [0x003, 1, ReadOnly ], // Power Management Status
        POWSTICKYSTAT:  PowStat     [0x004, 1, ReadOnly ], // Power Management Sticky Status
        POWIRQMASK:     PowIRQMask  [0x005, 1, ReadWrite], // Power Management Interrupt Mask
        /* Interrupt Control */
        IRQMASK:        IRQ         [0x006, 2, ReadWrite], // IRQ Mask
        RADIOEVENTMASK: RadioEvent  [0x008, 2, ReadWrite], // Radio Event Mask
        IRQINVERSION:   IRQ         [0x00A, 2, ReadWrite], // IRQ Inversion
        IRQREQUEST:     IRQ         [0x00C, 2, ReadOnly ], // IRQ Request
        RADIOEVENTREQ:  RadioEvent  [0x00E, 2, ReadOnly ], // Radio Event Request
        /* Modulation & Framing */
        MODULATION:     Modulation  [0x010, 1, ReadWrite], // Modulation
        ENCODING:       Encoding    [0x011, 1, ReadWrite], // Encoder/Decoder Settings
        FRAMING:        Framing     [0x012, 1, ReadWrite], // Framing settings
        CRCINIT:        u32         [0x014, 4, ReadWrite], // CRC Initialisation Data
        /* Forward Error Correction */
        FEC:            FEC         [0x018, 1, ReadWrite], // FEC (Viterbi) Configuration
        FECSYNC:        u8          [0x019, 1, ReadWrite], // Interleaver Synchronisation Threshold
        FECSTATUS:      FECStatus   [0x01A, 1, ReadOnly ], // FEC Status
        /* Status */
        RADIOSTATE:     RadioState  [0x01C, 1, ReadOnly ], // Radio Controller State
        XTALSTATUS:     XtalStatus  [0x01D, 1, ReadOnly ], // Crystal Oscillator Status
        /* Pin Configuration */
        PINSTATE:       PinState    [0x020, 1, ReadOnly ], // Pinstate
        PINFUNCSYSCLK:  PFSysClk    [0x021, 1, ReadWrite], // SYSCLK Pin Function
        PINFUNCDCLK:    PFDClk      [0x022, 1, ReadWrite], // DCLK Pin Function
        PINFUNCDATA:    PFData      [0x023, 1, ReadWrite], // DATA Pin Function
        PINFUNCIRQ:     PFIRQ       [0x024, 1, ReadWrite], // IRQ Pin Function
        PINFUNCANTSEL:  PFAntSel    [0x025, 1, ReadWrite], // ANTSEL Pin Function
        PINFUNCPWRAMP:  PFPwrAmp    [0x026, 1, ReadWrite], // PWRAMP Pin Function
        PWRAMP:         PwrAmp      [0x027, 1, ReadWrite], // PWRAMP Control
        /* FIFO */
        FIFOSTAT:       FIFOStat    [0x028, 1, ReadOnly ], // FIFO Control
        FIFOCMD:        FIFOCmd     [0x028, 1, WriteOnly], // FIFO Control
        FIFODATA:       u8          [0x029, 1, ReadWrite], // FIFO Data
        FIFODATARX:     FIFOChunkRX [0x029, 1, ReadFIFO ], // FIFO Data
        FIFODATATX:     FIFOChunkTX [0x029, 1, WriteFIFO], // FIFO Data
        FIFOCOUNT:      u16         [0x02A, 2, ReadOnly ], // Number of Words currently in FIFO
        FIFOFREE:       u16         [0x02C, 2, ReadOnly ], // Number of Words that can be written to FIFO
        FIFOTHRESH:     u16         [0x02E, 2, ReadWrite], // FIFO Threshold
        /* Synthesizer */
        PLLLOOP:        PLLLoop     [0x030, 1, ReadWrite], // PLL Loop Filter Settings
        PLLCPI:         u8          [0x031, 1, ReadWrite], // PLL Charge Pump Current
        PLLVCODIV:      PLLVCODiv   [0x032, 1, ReadWrite], // PLL Divider Settings
        PLLRANGINGA:    PLLRanging  [0x033, 1, ReadWrite], // PLL Autoranging
        FREQA:          u32         [0x034, 4, ReadWrite], // Synthesizer Frequency
        PLLLOOPBOOST:   PLLLoop     [0x038, 1, ReadWrite], // PLL Loop Filter Settings (Boosted)
        PLLCPIBOOST:    u8          [0x039, 1, ReadWrite], // PLL Charge Pump Current (Boosted)
        PLLRANGINGB:    PLLRanging  [0x03B, 1, ReadWrite], // PLL Autoranging
        FREQB:          u32         [0x03C, 4, ReadWrite], // Synthesizer Frequency
        /* Signal Strength */
        SIGNALSTR:      SignalStr   [0x040, 4, ReadOnly ], // Aggregate signal strength block
        RSSI:           i8          [0x040, 1, ReadOnly ], // Received Signal Strength Indicator
        BGNDRSSI:       u8          [0x041, 1, ReadWrite], // Background RSSI
        DIVERSITY:      Diversity   [0x042, 1, ReadWrite], // Antenna Diversity Configuration
        AGCCOUNTER:     i8          [0x043, 1, ReadWrite], // AGC Current Value
        /* Receiver Tracking */
        RXTRACKING:     RXTracking  [0x045,16, ReadOnly ], // Aggregate receiver tracking block
        TRKDATARATE:    i32         [0x045, 3, ReadOnly ], // Datarate Tracking
        TRKAMPL:        u16         [0x048, 2, ReadOnly ], // Amplitude Tracking - output of demodulator, this / AGCCOUNTER = RSSI
        TRKPHASE:       TrkPhase    [0x04A, 2, ReadOnly ], // Phase Tracking
        TRKRFFREQ:      TrkRFFreq   [0x04D, 3, ReadWrite], // RF Frequency Tracking
        TRKFREQ:        i16         [0x050, 2, ReadWrite], // Frequency Tracking
        TRKFSKDEMOD:    TrkFSKDemod [0x052, 2, ReadOnly ], // FSK Demodulator Tracking
        TRKAFSKDEMOD:   u16         [0x054, 2, ReadOnly ], // AFSK Demodulator Tracking
        /* Timer */
        TIMER2:         u32         [0x059, 3, ReadOnly], // 1 MHz Timer
        /* Wakeup Timer */
        WAKEUPTIMER:    u16         [0x068, 2, ReadOnly ], // Wakeup Timer
        WAKEUP:         u16         [0x06A, 2, ReadWrite], // Wakeup Time
        WAKEUPFREQ:     u16         [0x06C, 2, ReadWrite], // Wakeup Frequency
        WAKEUPXOEARLY:  u8          [0x06E, 1, ReadWrite], // Wakeup Crystal Oscillator Early
        /* Physical Layer Parameters */
        /* Receiver Parameters */
        IFFREQ:         u16         [0x100, 2, ReadWrite], // 2nd LO / IF Frequency
        DECIMATION:     u8          [0x102, 1, ReadWrite], // Decimation Factor
        RXDATARATE:     u32         [0x103, 3, ReadWrite], // Receiver Datarate
        MAXDROFFSET:    u32         [0x106, 3, ReadWrite], // Maximum Receiver Datarate Offset
        MAXRFOFFSET:    MaxRFOffset [0x109, 3, ReadWrite], // Maximum Receiver RF Offset
        FSKDMAX:        i16         [0x10C, 2, ReadWrite], // Four FSK Rx Deviation
        FSKDMIN:        i16         [0x10E, 2, ReadWrite], // Four FSK Rx Deviation
        AFSKSPACE:      u16         [0x110, 2, ReadWrite], // AFSK Space (0) Frequency
        AFSKMARK:       u16         [0x112, 2, ReadWrite], // AFSK Mark (1) Frequency
        AFSKCTRL:       u8          [0x114, 1, ReadWrite], // AFSK Control
        AMPLFILTER:     u8          [0x115, 1, ReadWrite], // Amplitude Filter
        FREQUENCYLEAK:  u8          [0x116, 1, ReadWrite], // Baseband Frequency Recovery Loop Leakiness
        RXPARAMSETS:    RxParamSets [0x117, 1, ReadWrite], // Receiver Parameter Set Indirection
        RXPARAMCURSET:  RxParamCurSet [0x118, 1, ReadOnly],// Receiver Parameter Current Set
        /* Receiver Parameter Set 0 */
        AGCGAIN0:       AGCGain     [0x120, 1, ReadWrite], // AGC Speed
        AGCTARGET0:     u8          [0x121, 1, ReadWrite], // AGC Target
        AGCAHYST0:      AGCHyst     [0x122, 1, ReadWrite], // AGC Digital Threshold Range
        AGCMINMAX0:     AGCMinMax   [0x123, 1, ReadWrite], // AGC Digital Minimum/Maximum Set Points
        TIMEGAIN0:      Float4      [0x124, 1, ReadWrite], // Timing Gain
        DRGAIN0:        Float4      [0x125, 1, ReadWrite], // Data Rate Gain
        PHASEGAIN0:     PhaseGain   [0x126, 1, ReadWrite], // Filter Index, Phase Gain
        FREQGAINA0:     FreqGainA   [0x127, 1, ReadWrite], // Frequency Gain A
        FREQGAINB0:     FreqGainB   [0x128, 1, ReadWrite], // Frequency Gain B
        FREQGAINC0:     FreqGainC   [0x129, 1, ReadWrite], // Frequency Gain C
        FREQGAIND0:     FreqGainD   [0x12A, 1, ReadWrite], // Frequency Gain D
        AMPLGAIN0:      AmplGain    [0x12B, 1, ReadWrite], // Amplitude Gain
        FREQDEV0:       u16         [0x12C, 2, ReadWrite], // Receiver Frequency Deviation
        FOURFSK0:       FourFSK     [0x12E, 1, ReadWrite], // Four FSK Control
        BBOFFSRES0:     BBOffsRes   [0x12F, 1, ReadWrite], // Baseband Offset Compensation Resistors
        /* Receiver Parameter Set 1 */
        AGCGAIN1:       AGCGain     [0x130, 1, ReadWrite], // AGC Speed
        AGCTARGET1:     u8          [0x131, 1, ReadWrite], // AGC Target
        AGCAHYST1:      AGCHyst     [0x132, 1, ReadWrite], // AGC Digital Threshold Range
        AGCMINMAX1:     AGCMinMax   [0x133, 1, ReadWrite], // AGC Digital Minimum/Maximum Set Points
        TIMEGAIN1:      Float4      [0x134, 1, ReadWrite], // Timing Gain
        DRGAIN1:        Float4      [0x135, 1, ReadWrite], // Data Rate Gain
        PHASEGAIN1:     PhaseGain   [0x136, 1, ReadWrite], // Filter Index, Phase Gain
        FREQGAINA1:     FreqGainA   [0x137, 1, ReadWrite], // Frequency Gain A
        FREQGAINB1:     FreqGainB   [0x138, 1, ReadWrite], // Frequency Gain B
        FREQGAINC1:     FreqGainC   [0x139, 1, ReadWrite], // Frequency Gain C
        FREQGAIND1:     FreqGainD   [0x13A, 1, ReadWrite], // Frequency Gain D
        AMPLGAIN1:      AmplGain    [0x13B, 1, ReadWrite], // Amplitude Gain
        FREQDEV1:       u16         [0x13C, 2, ReadWrite], // Receiver Frequency Deviation
        FOURFSK1:       FourFSK     [0x13E, 1, ReadWrite], // Four FSK Control
        BBOFFSRES1:     BBOffsRes   [0x13F, 1, ReadWrite], // Baseband Offset Compensation Resistors
        /* Receiver Parameter Set 2 */
        AGCGAIN2:       AGCGain     [0x140, 1, ReadWrite], // AGC Speed
        AGCTARGET2:     u8          [0x141, 1, ReadWrite], // AGC Target
        AGCAHYST2:      AGCHyst     [0x142, 1, ReadWrite], // AGC Digital Threshold Range
        AGCMINMAX2:     AGCMinMax   [0x143, 1, ReadWrite], // AGC Digital Minimum/Maximum Set Points
        TIMEGAIN2:      Float4      [0x144, 1, ReadWrite], // Timing Gain
        DRGAIN2:        Float4      [0x145, 1, ReadWrite], // Data Rate Gain
        PHASEGAIN2:     PhaseGain   [0x146, 1, ReadWrite], // Filter Index, Phase Gain
        FREQGAINA2:     FreqGainA   [0x147, 1, ReadWrite], // Frequency Gain A
        FREQGAINB2:     FreqGainB   [0x148, 1, ReadWrite], // Frequency Gain B
        FREQGAINC2:     FreqGainC   [0x149, 1, ReadWrite], // Frequency Gain C
        FREQGAIND2:     FreqGainD   [0x14A, 1, ReadWrite], // Frequency Gain D
        AMPLGAIN2:      AmplGain    [0x14B, 1, ReadWrite], // Amplitude Gain
        FREQDEV2:       u16         [0x14C, 2, ReadWrite], // Receiver Frequency Deviation
        FOURFSK2:       FourFSK     [0x14E, 1, ReadWrite], // Four FSK Control
        BBOFFSRES2:     BBOffsRes   [0x14F, 1, ReadWrite], // Baseband Offset Compensation Resistors
        /* Receiver Parameter Set 3 */
        AGCGAIN3:       AGCGain     [0x150, 1,ReadWrite],  // AGC Speed
        AGCTARGET3:     u8          [0x151, 1,ReadWrite],  // AGC Target
        AGCAHYST3:      AGCHyst     [0x152, 1,ReadWrite],  // AGC Digital Threshold Range
        AGCMINMAX3:     AGCMinMax   [0x153, 1,ReadWrite],  // AGC Digital Minimum/Maximum Set Points
        TIMEGAIN3:      Float4      [0x154, 1,ReadWrite],  // Timing Gain
        DRGAIN3:        Float4      [0x155, 1,ReadWrite],  // Data Rate Gain
        PHASEGAIN3:     PhaseGain   [0x156, 1,ReadWrite],  // Filter Index, Phase Gain
        FREQGAINA3:     FreqGainA   [0x157, 1,ReadWrite],  // Frequency Gain A
        FREQGAINB3:     FreqGainB   [0x158, 1,ReadWrite],  // Frequency Gain B
        FREQGAINC3:     FreqGainC   [0x159, 1,ReadWrite],  // Frequency Gain C
        FREQGAIND3:     FreqGainD   [0x15A, 1,ReadWrite],  // Frequency Gain D
        AMPLGAIN3:      AmplGain    [0x15B, 1,ReadWrite],  // Amplitude Gain
        FREQDEV3:       u16         [0x15C, 2,ReadWrite],  // Receiver Frequency Deviation
        FOURFSK3:       FourFSK     [0x15E, 1,ReadWrite],  // Four FSK Control
        BBOFFSRES3:     BBOffsRes   [0x15F, 1,ReadWrite],  // Baseband Offset Compensation Resistors
        /* Transmitter Parameters */
        MODCFGF:        ModCfgF     [0x160, 1, ReadWrite], // Modulator Configuration F
        FSKDEV:         u32         [0x161, 3, ReadWrite], // FSK Frequency Deviation // TODO: Aliased
        MODCFGA:        ModCfgA     [0x164, 1, ReadWrite], // Modulator Configuration A
        TXRATE:         u32         [0x165, 3, ReadWrite], // Transmitter Bitrate
        TXPWRCOEFFA:    u16         [0x168, 2, ReadWrite], // Transmitter Predistortion Coefficient A
        TXPWRCOEFFB:    u16         [0x16A, 2, ReadWrite], // Transmitter Predistortion Coefficient B
        TXPWRCOEFFC:    u16         [0x16C, 2, ReadWrite], // Transmitter Predistortion Coefficient C
        TXPWRCOEFFD:    u16         [0x16E, 2, ReadWrite], // Transmitter Predistortion Coefficient D
        TXPWRCOEFFE:    u16         [0x170, 2, ReadWrite], // Transmitter Predistortion Coefficient E
        /* PLL Parameters */
        PLLVCOI:        PLLVCOI     [0x180, 1, ReadWrite], // VCO Current
        PLLVCOIR:       u8          [0x181, 1, ReadWrite], // VCO Current Readback
        PLLLOCKDET:     PLLLockDet  [0x182, 1, ReadWrite], // PLL Lock Detect Delay
        PLLRNGCLK:      PLLRngClk   [0x183, 1, ReadWrite], // PLL Ranging Clock
        /* Crystal Oscillator */
        XTALCAP:        u8          [0x184, 1, ReadWrite], // Crystal Oscillator Load Capacitance Configuration
        /* Baseband */
        BBTUNE:         u8          [0x188, 1, ReadWrite], // Baseband Tuning
        BBOFFSCAP:      u8          [0x189, 1, ReadWrite], // Baseband Offset Compensation Capacitors
        /* MAC Layer Parameters */
        /* Packet Format */
        PKTADDRCFG:     PktAddrCfg  [0x200, 1, ReadWrite], // Packet Address Config
        PKTLENCFG:      PktLenCfg   [0x201, 1, ReadWrite], // Packet Length Config
        PKTLENOFFSET:   u8          [0x202, 1, ReadWrite], // Packet Length Offset
        PKTMAXLEN:      u8          [0x203, 1, ReadWrite], // Packet Maximum Length
        PKTADDR:        u32         [0x204, 4, ReadWrite], // Packet Address 3
        PKTADDRMASK:    u32         [0x208, 4, ReadWrite], // Packet Address Mask 3
        /* Pattern Match */
        MATCH0PAT:      u32         [0x210, 4, ReadWrite], // Pattern Match Unit 0, Pattern
        MATCH0LEN:      MatchLen    [0x214, 1, ReadWrite], // Pattern Match Unit 0, Pattern Length
        MATCH0MIN:      u8          [0x215, 1, ReadWrite], // Pattern Match Unit 0, Minimum Match
        MATCH0MAX:      u8          [0x216, 1, ReadWrite], // Pattern Match Unit 0, Maximum Match
        MATCH1PAT:      u16         [0x218, 2, ReadWrite], // Pattern Match Unit 1, Pattern
        MATCH1LEN:      MatchLen    [0x21C, 1, ReadWrite], // Pattern Match Unit 1, Pattern Length
        MATCH1MIN:      u8          [0x21D, 1, ReadWrite], // Pattern Match Unit 1, Minimum Match
        MATCH1MAX:      u8          [0x21E, 1, ReadWrite], // Pattern Match Unit 1, Maximum Match
        /* Packet Controller */
        TMGTXBOOST:     Float5      [0x220, 1, ReadWrite], // Transmit PLL Boost Time
        TMGTXSETTLE:    Float5      [0x221, 1, ReadWrite], // Transmit PLL (post Boost) Settling Time
        TMGRXBOOST:     Float5      [0x223, 1, ReadWrite], // Receive PLL Boost Time
        TMGRXSETTLE:    Float5      [0x224, 1, ReadWrite], // Receive PLL (post Boost) Settling Time
        TMGRXOFFSACQ:   Float5      [0x225, 1, ReadWrite], // Receive Baseband DC Offset Acquisition Time
        TMGRXCOARSEAGC: Float5      [0x226, 1, ReadWrite], // Receive Coarse AGC Time
        TMGRXAGC:       Float5      [0x227, 1, ReadWrite], // Receiver AGC Settling Time
        TMGRXRSSI:      Float5      [0x228, 1, ReadWrite], // Receiver RSSI Settling Time
        TMGRXPREAMBLE1: Float5      [0x229, 1, ReadWrite], // Receiver Preamble 1 Timeout
        TMGRXPREAMBLE2: Float5      [0x22A, 1, ReadWrite], // Receiver Preamble 2 Timeout
        TMGRXPREAMBLE3: Float5      [0x22B, 1, ReadWrite], // Receiver Preamble 3 Timeout
        RSSIREFERENCE:  i8          [0x22C, 1, ReadWrite], // RSSI Offset
        RSSIABSTHR:     u8          [0x22D, 1, ReadWrite], // RSSI Absolute Threshold
        BGNDRSSIGAIN:   u8          [0x22E, 1, ReadWrite], // Background RSSI Averaging Time Constant
        BGNDRSSITHR:    u8          [0x22F, 1, ReadWrite], // Background RSSI Relative Threshold
        PKTCHUNKSIZE:   u8          [0x230, 1, ReadWrite], // Packet Chunk Size
        PKTMISCFLAGS:   PktMiscFlags [0x231, 1, ReadWrite], // Packet Controller Miscellaneous Flags
        PKTSTOREFLAGS:  PktStoreFlags [0x232, 1, ReadWrite], // Packet Controller Store Flags
        PKTACCEPTFLAGS: PktAcceptFlags [0x233, 1, ReadWrite], // Packet Controller Accept Flags
        /* Special Functions */
        /* General Purpose ADC */
        GPADCCTRL:      u8          [0x300, 1, ReadWrite], // General Purpose ADC Control
        GPADCPERIOD:    u8          [0x301, 1, ReadWrite], // GPADC Sampling Period
        GPADC13VALUE:   u16         [0x308, 2, ReadOnly ], // GPADC13 Value
        /* Low Power Oscillator Calibration */
        LPOSCCONFIG:    u8          [0x310, 1, ReadWrite], // Low Power Oscillator Configuration
        LPOSCSTATUS:    u8          [0x311, 1, ReadOnly ], // Low Power Oscillator Status
        LPOSCKFILT:     u16         [0x312, 2, ReadWrite], // Low Power Oscillator Calibration Filter Constant
        LPOSCREF:       u16         [0x314, 2, ReadWrite], // Low Power Oscillator Calibration Reference
        LPOSCFREQ:      u16         [0x316, 2, ReadWrite], // Low Power Oscillator Calibration Frequency
        LPOSCPER:       u16         [0x318, 2, ReadWrite], // Low Power Oscillator Calibration Period
        /* DAC */
        DACVALUE:       u16         [0x330, 2, ReadWrite], // DAC Value
        DACCONFIG:      u8          [0x332, 1, ReadWrite], // DAC Configuration
        /* Performance Tuning Registers */
        PERF_F00:       u8          [0xF00, 1, ReadWrite],
        PERF_F08:       u8          [0xF08, 1, ReadWrite],
        PERF_F0D:       u8          [0xF0D, 1, ReadWrite],
        PERF_F10:       PerfF10     [0xF10, 1, ReadWrite],
        PERF_F11:       PerfF11     [0xF11, 1, ReadWrite],
        PERF_F18:       u8          [0xF18, 1, ReadWrite],
        PERF_F1C:       u8          [0xF1C, 1, ReadWrite],
        PERF_F21:       u8          [0xF21, 1, ReadWrite],
        PERF_F22:       u8          [0xF22, 1, ReadWrite],
        PERF_F23:       u8          [0xF23, 1, ReadWrite],
        PERF_F26:       u8          [0xF26, 1, ReadWrite],
        PERF_F34:       PerfF34     [0xF34, 1, ReadWrite],
        PERF_F35:       PerfF35     [0xF35, 1, ReadWrite],
        PERF_F44:       u8          [0xF44, 1, ReadWrite],
        PERF_F72:       u8          [0xF72, 1, ReadWrite],
    }
}

pub fn open<P: AsRef<Path>>(path: P) -> std::io::Result<Spidev> {
    let mut spi = Spidev::open(path)?;
    let options = SpidevOptions::new()
        .max_speed_hz(10_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options)?;
    Ok(spi)
}

impl Registers<'_> {
    pub fn new(
        spi: Spidev,
        on_status: &mut dyn FnMut(&Spidev, u16, Status, &[u8]),
    ) -> Registers<'_> {
        // Default vaules from PM Table 22
        Registers {
            spi,
            on_status,

            REVISION: 0b0101_0001,
            SCRATCH: 0b1100_0101,
            PWRMODE: PwrMode {
                mode: PwrModes::POWEROFF,
                flags: PwrFlags::REFEN | PwrFlags::XOEN,
            },
            POWSTAT: PowStat::empty(),
            POWSTICKYSTAT: PowStat::empty(),
            POWIRQMASK: PowIRQMask::empty(),
            IRQMASK: IRQ::empty(),
            RADIOEVENTMASK: RadioEvent::empty(),
            IRQINVERSION: IRQ::empty(),
            IRQREQUEST: IRQ::empty(),
            RADIOEVENTREQ: RadioEvent::empty(),
            MODULATION: Modulation {
                mode: ModulationMode::FSK,
                halfspeed: false,
            },
            ENCODING: Encoding::DIFF,
            FRAMING: Framing {
                frmmode: FrameMode::RAW,
                crcmode: CRCMode::OFF,
                flags: FramingFlags::empty(),
            },
            CRCINIT: 0xFFFF_FFFF,
            FEC: FEC {
                flags: FECFlags::empty(),
                inpshift: 0,
            },
            FECSYNC: 0b0110_0010,
            FECSTATUS: FECStatus {
                max_metric: 0,
                inv: false,
            },
            RADIOSTATE: RadioState::IDLE,
            XTALSTATUS: XtalStatus::empty(),
            PINSTATE: PinState::empty(),
            PINFUNCSYSCLK: PFSysClk {
                mode: PFSysClkMode::F_XTAL_DIV_16,
                pullup: false,
            },
            PINFUNCDCLK: PFDClk {
                mode: PFDClkMode::OUT,
                flags: PFFlags::empty(),
            },
            PINFUNCDATA: PFData {
                mode: PFDataMode::MODEM_OUT,
                flags: PFFlags::PULLUP,
            },
            PINFUNCIRQ: PFIRQ {
                mode: PFIRQMode::IRQ,
                flags: PFFlags::empty(),
            },
            PINFUNCANTSEL: PFAntSel {
                mode: PFAntSelMode::ANTSEL,
                flags: PFFlags::empty(),
            },
            PINFUNCPWRAMP: PFPwrAmp {
                mode: PFPwrAmpMode::PWRAMP,
                flags: PFFlags::empty(),
            },
            PWRAMP: PwrAmp::empty(),
            FIFOSTAT: FIFOStat::empty(),
            FIFOCMD: FIFOCmd {
                mode: FIFOCmds::NOP,
                auto_commit: false,
            },
            FIFODATA: 0,
            FIFODATARX: FIFOChunkRX::RSSI(0), // FIXME: NOP?
            FIFODATATX: FIFOChunkTX::NOP,
            FIFOCOUNT: 0,
            FIFOFREE: 0, // Documented as 0x100 but POWEROFF makes FIFO inaccessable
            FIFOTHRESH: 0,
            PLLLOOP: PLLLoop {
                filter: FLT::INTERNAL_x1,
                flags: PLLLoopFlags::DIRECT,
                freqsel: FreqSel::A,
            },
            PLLCPI: 8,
            PLLVCODIV: PLLVCODiv {
                mode: PLLVCORefDiv::F_XTAL,
                flags: PLLVCODivFlags::empty(),
            },
            PLLRANGINGA: PLLRanging {
                vcor: 8,
                flags: PLLRangingFlags::empty(),
            },
            FREQA: 0x3934_CCCD,
            PLLLOOPBOOST: PLLLoop {
                filter: FLT::INTERNAL_x5,
                flags: PLLLoopFlags::DIRECT,
                freqsel: FreqSel::A,
            },
            PLLCPIBOOST: 0xC8,
            PLLRANGINGB: PLLRanging {
                vcor: 8,
                flags: PLLRangingFlags::empty(),
            },
            FREQB: 0x3934_CCCD,
            SIGNALSTR: SignalStr {
                rssi: 0,
                bgndrssi: 0,
                diversity: Diversity::empty(),
                agccounter: 0,
            },
            RSSI: 0,
            BGNDRSSI: 0,
            DIVERSITY: Diversity::empty(),
            AGCCOUNTER: 0,
            RXTRACKING: RXTracking {
                datarate: 0,
                ampl: 0,
                phase: TrkPhase(0),
                rffreq: TrkRFFreq(0),
                freq: 0,
                fskdemod: TrkFSKDemod(0),
                afskdemod: 0,
            },
            TRKDATARATE: 0,
            TRKAMPL: 0,
            TRKPHASE: TrkPhase(0),
            TRKRFFREQ: TrkRFFreq(0),
            TRKFREQ: 0,
            TRKFSKDEMOD: TrkFSKDemod(0),
            TRKAFSKDEMOD: 0,
            TIMER2: 0,
            WAKEUPTIMER: 0,
            WAKEUP: 0,
            WAKEUPFREQ: 0,
            WAKEUPXOEARLY: 0,
            IFFREQ: 0x1327,
            DECIMATION: 0x0D,
            RXDATARATE: 0x3D8A,
            MAXDROFFSET: 0x9E,
            MAXRFOFFSET: MaxRFOffset {
                offset: 0x1687,
                correction: false,
            },
            FSKDMAX: 0x80,
            FSKDMIN: -0x80,
            AFSKSPACE: 0x40,
            AFSKMARK: 0x75,
            AFSKCTRL: 0x04,
            AMPLFILTER: 0,
            FREQUENCYLEAK: 0,
            RXPARAMSETS: RxParamSets(
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
            ),
            RXPARAMCURSET: RxParamCurSet {
                index: 0,
                number: RxParamSet::Set0,
                special: 0,
            },
            AGCGAIN0: AGCGain {
                attack: 0x4,
                decay: 0xB,
            },
            AGCTARGET0: 0x76,
            AGCAHYST0: AGCHyst { hyst: 0 },
            AGCMINMAX0: AGCMinMax { min: 0, max: 0 },
            TIMEGAIN0: Float4 { e: 0x8, m: 0xF },
            DRGAIN0: Float4 { e: 2, m: 0xF },
            PHASEGAIN0: PhaseGain { gain: 3, filter: 3 },
            FREQGAINA0: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            },
            FREQGAINB0: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            },
            FREQGAINC0: FreqGainC { gain: 0xA },
            FREQGAIND0: FreqGainD {
                gain: 0xA,
                freeze: false,
            },
            AMPLGAIN0: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            },
            FREQDEV0: 0x0020,
            FOURFSK0: FourFSK {
                decay: 6,
                update: true,
            },
            BBOFFSRES0: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            },
            AGCGAIN1: AGCGain {
                attack: 0x4,
                decay: 0xB,
            },
            AGCTARGET1: 0x76,
            AGCAHYST1: AGCHyst { hyst: 0 },
            AGCMINMAX1: AGCMinMax { min: 0, max: 0 },
            TIMEGAIN1: Float4 { e: 0x6, m: 0xF },
            DRGAIN1: Float4 { e: 0x1, m: 0xF },
            PHASEGAIN1: PhaseGain { gain: 3, filter: 3 },
            FREQGAINA1: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            },
            FREQGAINB1: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            },
            FREQGAINC1: FreqGainC { gain: 0xB },
            FREQGAIND1: FreqGainD {
                gain: 0xB,
                freeze: false,
            },
            AMPLGAIN1: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            },
            FREQDEV1: 0x20,
            FOURFSK1: FourFSK {
                decay: 0x8,
                update: true,
            },
            BBOFFSRES1: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            },
            AGCGAIN2: AGCGain {
                attack: 0xF,
                decay: 0xF,
            },
            AGCTARGET2: 0x76,
            AGCAHYST2: AGCHyst { hyst: 0 },
            AGCMINMAX2: AGCMinMax { min: 0, max: 0 },
            TIMEGAIN2: Float4 { e: 0x5, m: 0xF },
            DRGAIN2: Float4 { e: 0x0, m: 0xF },
            PHASEGAIN2: PhaseGain { gain: 3, filter: 3 },
            FREQGAINA2: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            },
            FREQGAINB2: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            },
            FREQGAINC2: FreqGainC { gain: 0xD },
            FREQGAIND2: FreqGainD {
                gain: 0xD,
                freeze: false,
            },
            AMPLGAIN2: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            },
            FREQDEV2: 0x20,
            FOURFSK2: FourFSK {
                decay: 0xA,
                update: true,
            },
            BBOFFSRES2: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            },
            AGCGAIN3: AGCGain {
                attack: 0xF,
                decay: 0xF,
            },
            AGCTARGET3: 0x76,
            AGCAHYST3: AGCHyst { hyst: 0 },
            AGCMINMAX3: AGCMinMax { min: 0, max: 0 },
            TIMEGAIN3: Float4 { e: 0x5, m: 0xF },
            DRGAIN3: Float4 { e: 0x0, m: 0xF },
            PHASEGAIN3: PhaseGain { gain: 3, filter: 3 },
            FREQGAINA3: FreqGainA {
                gain: 0xF,
                flags: FreqGainAFlags::empty(),
            },
            FREQGAINB3: FreqGainB {
                gain: 0x1F,
                flags: FreqGainBFlags::empty(),
            },
            FREQGAINC3: FreqGainC { gain: 0xD },
            FREQGAIND3: FreqGainD {
                gain: 0xD,
                freeze: false,
            },
            AMPLGAIN3: AmplGain {
                gain: 6,
                flags: AmplGainFlags::AGC,
            },
            FREQDEV3: 0x20,
            FOURFSK3: FourFSK {
                decay: 0xA,
                update: true,
            },
            BBOFFSRES3: BBOffsRes {
                res_int_a: 8,
                res_int_b: 8,
            },
            MODCFGF: ModCfgF::UNSHAPED,
            FSKDEV: 0x00_0A3D,
            MODCFGA: ModCfgA {
                slowramp: SlowRamp::STARTUP_1b,
                flags: ModCfgAFlags::AMPLSHAPE | ModCfgAFlags::TXDIFF,
            },
            TXRATE: 0x00_28F6,
            TXPWRCOEFFA: 0x0000,
            TXPWRCOEFFB: 0x0FFF,
            TXPWRCOEFFC: 0x0000,
            TXPWRCOEFFD: 0x0000,
            TXPWRCOEFFE: 0x0000,
            PLLVCOI: PLLVCOI {
                bias: 0x12,
                flags: PLLVCOIFlags::empty(),
            },
            PLLVCOIR: 0,
            PLLLOCKDET: PLLLockDet {
                delay: LockDetDly::d14ns,
                flags: LockDetFlags::AUTOMATIC,
                readback: LockDetDly::d6ns,
            },
            PLLRNGCLK: PLLRngClk::XTAL_DIV_2pow11,
            XTALCAP: 0,
            BBTUNE: 9,
            BBOFFSCAP: 0x77,
            PKTADDRCFG: PktAddrCfg {
                addr_pos: 0,
                flags: PktAddrCfgFlags::FEC_SYNC_DIS,
            },
            PKTLENCFG: PktLenCfg { bits: 0, pos: 0 },
            PKTLENOFFSET: 0,
            PKTMAXLEN: 0,
            PKTADDR: 0,
            PKTADDRMASK: 0,
            MATCH0PAT: 0,
            MATCH0LEN: MatchLen { len: 0, raw: false },
            MATCH0MIN: 0,
            MATCH0MAX: 0x1F,
            MATCH1PAT: 0,
            MATCH1LEN: MatchLen { len: 0, raw: false },
            MATCH1MIN: 0,
            MATCH1MAX: 0xF,
            TMGTXBOOST: Float5 { e: 1, m: 0x12 },
            TMGTXSETTLE: Float5 { e: 0, m: 0x0A },
            TMGRXBOOST: Float5 { e: 1, m: 0x12 },
            TMGRXSETTLE: Float5 { e: 0, m: 0x14 },
            TMGRXOFFSACQ: Float5 { e: 3, m: 0x13 },
            TMGRXCOARSEAGC: Float5 { e: 1, m: 0x19 },
            TMGRXAGC: Float5 { e: 0, m: 0 },
            TMGRXRSSI: Float5 { e: 0, m: 0 },
            TMGRXPREAMBLE1: Float5 { e: 0, m: 0 },
            TMGRXPREAMBLE2: Float5 { e: 0, m: 0 },
            TMGRXPREAMBLE3: Float5 { e: 0, m: 0 },
            RSSIREFERENCE: 0,
            RSSIABSTHR: 0,
            BGNDRSSIGAIN: 0,
            BGNDRSSITHR: 0,
            PKTCHUNKSIZE: 0,
            PKTMISCFLAGS: PktMiscFlags::empty(),
            PKTSTOREFLAGS: PktStoreFlags::empty(),
            PKTACCEPTFLAGS: PktAcceptFlags::empty(),
            GPADCCTRL: 0,
            GPADCPERIOD: 0x3F,
            GPADC13VALUE: 0,
            LPOSCCONFIG: 0,
            LPOSCSTATUS: 0,
            LPOSCKFILT: 0x20C4,
            LPOSCREF: 0x61A8,
            LPOSCFREQ: 0,
            LPOSCPER: 0,
            DACVALUE: 0,
            DACCONFIG: 0,
            PERF_F00: 0,
            PERF_F08: 4,
            PERF_F0D: 4,
            PERF_F10: PerfF10::TCXO,
            PERF_F11: PerfF11::Reset,
            PERF_F18: 6,
            PERF_F1C: 4,
            PERF_F21: 0x20,
            PERF_F22: 0x54,
            PERF_F23: 0x50,
            PERF_F26: 0x88,
            PERF_F34: PerfF34::Reset,
            PERF_F35: PerfF35::FreqLT24p8MHz,
            PERF_F44: 0x25,
            PERF_F72: 0,
        }
    }

    pub fn reset(&mut self) -> Result<()> {
        self.PWRMODE().write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::RST,
        })?;

        self.PWRMODE().write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::empty(),
        })?;

        Ok(())
    }
}

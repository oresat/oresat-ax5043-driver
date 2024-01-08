use crate::*;

#[derive(Copy, Clone, Default)]
pub enum SysClk {
    Zero,
    One,
    #[default]
    Z,
    XtalInvert,
    XtalDiv1,
    XtalDiv2,
    XtalDiv4,
    XtalDiv8,
    XtalDiv16,
    XtalDiv32,
    XtalDiv64,
    XtalDiv128,
    XtalDiv256,
    XtalDiv512,
    XtalDiv1024,
    LPO,
    Test,
}

#[rustfmt::skip]
impl From<SysClk> for PFSysClkMode {
    fn from(v: SysClk) -> PFSysClkMode {
        match v {
            SysClk::Zero        => PFSysClkMode::ZERO,
            SysClk::One         => PFSysClkMode::ONE,
            SysClk::Z           => PFSysClkMode::Z,
            SysClk::XtalInvert  => PFSysClkMode::F_XTAL_INVERT,
            SysClk::XtalDiv1    => PFSysClkMode::F_XTAL,
            SysClk::XtalDiv2    => PFSysClkMode::F_XTAL_DIV_2,
            SysClk::XtalDiv4    => PFSysClkMode::F_XTAL_DIV_4,
            SysClk::XtalDiv8    => PFSysClkMode::F_XTAL_DIV_8,
            SysClk::XtalDiv16   => PFSysClkMode::F_XTAL_DIV_16,
            SysClk::XtalDiv32   => PFSysClkMode::F_XTAL_DIV_32,
            SysClk::XtalDiv64   => PFSysClkMode::F_XTAL_DIV_64,
            SysClk::XtalDiv128  => PFSysClkMode::F_XTAL_DIV_128,
            SysClk::XtalDiv256  => PFSysClkMode::F_XTAL_DIV_256,
            SysClk::XtalDiv512  => PFSysClkMode::F_XTAL_DIV_512,
            SysClk::XtalDiv1024 => PFSysClkMode::F_XTAL_DIV_1024,
            SysClk::LPO         => PFSysClkMode::LPO,
            SysClk::Test        => PFSysClkMode::TEST,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub enum DClk {
    Zero,
    One,
    #[default]
    Z,
    In,
    Out,
    None,
    Test,
}

#[rustfmt::skip]
impl From<DClk> for PFDClkMode {
    fn from(v: DClk) -> PFDClkMode {
        match v {
            DClk::Zero => PFDClkMode::ZERO,
            DClk::One  => PFDClkMode::ONE,
            DClk::Z    => PFDClkMode::Z,
            DClk::In   => PFDClkMode::IN,
            DClk::Out  => PFDClkMode::OUT,
            DClk::None => PFDClkMode::NONE,
            DClk::Test => PFDClkMode::TEST,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub enum Data {
    Zero,
    One,
    #[default]
    Z,
    FrameIO,
    ModemIO,
    AsyncIO,
    ModemOut,
    Test,
}

#[rustfmt::skip]
impl From<Data> for PFDataMode {
    fn from(v: Data) -> PFDataMode {
        match v {
            Data::Zero     => PFDataMode::ZERO,
            Data::One      => PFDataMode::ONE,
            Data::Z        => PFDataMode::Z,
            Data::FrameIO  => PFDataMode::FRAME_IO,
            Data::ModemIO  => PFDataMode::MODEM_IO,
            Data::AsyncIO  => PFDataMode::ASYNC_IO,
            Data::ModemOut => PFDataMode::MODEM_OUT,
            Data::Test     => PFDataMode::TEST,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub enum PwrAmp {
    Zero,
    One,
    #[default]
    Z,
    DibitSyncIO,
    DibitSyncObs,
    DAC,
    PwrAmp,
    TCXO,
    Test,
}

#[rustfmt::skip]
impl From<PwrAmp> for PFPwrAmpMode {
    fn from(v: PwrAmp) -> PFPwrAmpMode {
        match v {
            PwrAmp::Zero         => PFPwrAmpMode::ZERO,
            PwrAmp::One          => PFPwrAmpMode::ONE,
            PwrAmp::Z            => PFPwrAmpMode::Z,
            PwrAmp::DibitSyncIO  => PFPwrAmpMode::DIBIT_SYNC_IO,
            PwrAmp::DibitSyncObs => PFPwrAmpMode::DIBIT_SYNC_OBS,
            PwrAmp::DAC          => PFPwrAmpMode::DAC,
            PwrAmp::PwrAmp       => PFPwrAmpMode::PWRAMP,
            PwrAmp::TCXO         => PFPwrAmpMode::EXTTCXO,
            PwrAmp::Test         => PFPwrAmpMode::TEST,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub enum IRQ {
    Zero,
    One,
    #[default]
    Z,
    IRQ,
    Test,
}

#[rustfmt::skip]
impl From<IRQ> for PFIRQMode {
    fn from(v: IRQ) -> PFIRQMode {
        match v {
            IRQ::Zero => PFIRQMode::ZERO,
            IRQ::One  => PFIRQMode::ONE,
            IRQ::Z    => PFIRQMode::Z,
            IRQ::IRQ  => PFIRQMode::IRQ,
            IRQ::Test => PFIRQMode::TEST,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub enum AntSel {
    Zero,
    One,
    #[default]
    Z,
    BBTuneClk,
    TCXO,
    DAC,
    AntSel,
    Test,
}

#[rustfmt::skip]
impl From<AntSel> for PFAntSelMode {
    fn from(v: AntSel) -> Self {
        match v {
            AntSel::Zero      => Self::ZERO,
            AntSel::One       => Self::ONE,
            AntSel::Z         => Self::Z,
            AntSel::BBTuneClk => Self::BBTUNECLK,
            AntSel::TCXO      => Self::EXTTCXO,
            AntSel::DAC       => Self::DAC,
            AntSel::AntSel    => Self::ANTSEL,
            AntSel::Test      => Self::TEST,
        }
    }
}
// TODO:
// - None/default = Z?
// - 0, 1, test are special (no inv/pull)?
// - configure pullup/invert in a different way?
// enum{ zero, one, z, test, func(T) }
#[derive(Copy, Clone, Default)]
pub struct Pin<T> {
    pub mode: T,
    pub pullup: bool,
    pub invert: bool,
}

impl<T> From<Pin<T>> for PFFlags {
    fn from(v: Pin<T>) -> Self {
        let mut f = PFFlags::empty();
        if v.pullup {
            f |= PFFlags::PULLUP;
        }
        if v.invert {
            f |= PFFlags::INVERT;
        }
        f
    }
}

#[derive(Default, Copy, Clone)]
pub enum XtalPin {
    #[default]
    None,
    PwrAmp,
    AntSel,
}

pub type Hz = u64;
#[allow(non_camel_case_types)]
pub type pF = f64; // TODO: newtype and XtalLoadCap::new(), TryFrom/From, make internal type u8

#[derive(Default, Copy, Clone)]
pub enum XtalKind {
    // FIXME rename to Oscillator? ExtOsc?
    XO {
        load_cap: pF,
    }, // limited to 3pF or 8.5pF - 40pF in 0.5pF steps // TODO LoadCap not needed for TCXO?
    #[default]
    TCXO,
}

#[derive(Default, Copy, Clone)]
pub struct Xtal {
    pub kind: XtalKind,
    pub freq: Hz,
    pub enable: XtalPin,
}

impl Xtal {
    #[must_use]
    pub fn div(&self) -> u64 {
        if self.freq < 24_800_000 {
            1
        } else {
            2
        }
    }
}

#[derive(Default, Copy, Clone)]
pub enum VCO {
    #[default]
    Internal, // VCO1
    Inductor, // VCO2
    External, // Bypassed
}

#[derive(Default, Copy, Clone)]
pub enum Filter {
    // TODO: values?
    #[default]
    Internal,
    External,
}

#[derive(Default, Copy, Clone)]
pub enum Antenna {
    #[default]
    SingleEnded,
    Differential,
}

#[derive(Default, Copy, Clone)]
pub enum DACPin {
    #[default]
    None,
    PwrAmp,
    AntSel,
}

#[derive(Default, Copy, Clone)]
pub struct DAC {
    pub pin: DACPin,
    // TODO: initial output?
}

#[derive(Default, Copy, Clone)]
pub enum ADC {
    #[default]
    None,
    ADC1,
    ADC2,
    Both,
}

#[derive(Default, Copy, Clone)]
#[rustfmt::skip]
pub struct Board {
    pub sysclk: Pin<SysClk>, // FIXME: sysclk doesn't have invert
    pub dclk:   Pin<DClk>,
    pub data:   Pin<Data>,
    pub pwramp: Pin<PwrAmp>,
    pub irq:    Pin<IRQ>,
    pub antsel: Pin<AntSel>,

    pub xtal: Xtal,
    pub vco: VCO,
    pub filter: Filter,
    pub dac: DAC,
    pub adc: ADC,
}

fn set_load_cap(radio: &mut Registers, load_cap: f64) -> Result<()> {
    // FIXME: move to new() on load_cap?
    assert!((3.0..=40.0).contains(&load_cap));
    let cap = (load_cap * 2.0).round() as u8;
    let mut val = 0;
    if cap >= 17 {
        val = cap - 16;
    }
    assert!(val < 0x40);

    radio.XTALCAP().write(val)?;
    Ok(())
}

impl Board {
    pub fn write(self, radio: &mut Registers) -> Result<Self> {
        // TODO: check that dac pin is set correctly
        // TODO: check that tcxo_en is set correctly
        // TODO: check SYSCLK inver
        radio.PINFUNCSYSCLK().write(PFSysClk {
            mode: self.sysclk.mode.into(),
            pullup: self.sysclk.pullup,
        })?;
        radio.PINFUNCDCLK().write(PFDClk {
            mode: self.dclk.mode.into(),
            flags: self.dclk.into(),
        })?;
        radio.PINFUNCDATA().write(PFData {
            mode: self.data.mode.into(),
            flags: self.data.into(),
        })?;
        radio.PINFUNCIRQ().write(PFIRQ {
            mode: self.irq.mode.into(),
            flags: self.irq.into(),
        })?;
        radio.PINFUNCANTSEL().write(PFAntSel {
            mode: self.antsel.mode.into(),
            flags: self.antsel.into(),
        })?;
        radio.PINFUNCPWRAMP().write(PFPwrAmp {
            mode: self.pwramp.mode.into(),
            flags: self.pwramp.into(),
        })?;

        if self.xtal.freq < 24_800_000 {
            radio.PERF_F35().write(PerfF35::FreqLT24p8MHz)?;
        } else {
            radio.PERF_F35().write(PerfF35::FreqGE24p8MHz)?;
        }
        match self.xtal.kind {
            XtalKind::XO { load_cap } => {
                set_load_cap(radio, load_cap)?;
                if self.xtal.freq > 43_000_000 {
                    radio.PERF_F10().write(PerfF10::FreqGT43MHz)?;
                } else {
                    radio.PERF_F10().write(PerfF10::XO)?;
                }
                radio.PERF_F11().write(PerfF11::XO)?;
            }
            XtalKind::TCXO => {
                // From DS Table 6 note 5:
                // if an external clock or TCXO is used, it should be input via an AC coupling at
                // pin CLK16P with the oscillator powered up and XTALCAP = 0x00.
                set_load_cap(radio, 3.0)?; // TCXO shouldn't be affected by load cap, set to minimum
                if self.xtal.freq > 43_000_000 {
                    radio.PERF_F10().write(PerfF10::FreqGT43MHz)?;
                } else {
                    radio.PERF_F10().write(PerfF10::TCXO)?;
                }
                radio.PERF_F11().write(PerfF11::TCXO)?;
            }
        }
        // Magic numbers! PM table 199 gives no indication of what these mean.
        // They were generated by AX_radiolab
        radio.PERF_F00().write(0x0F)?;
        radio.PERF_F08().write(0x04)?; // TODO where does this come from?
                                       // radio.PERF_F0C().write(0x00)?;
        radio.PERF_F0D().write(0x03)?;
        radio.PERF_F18().write(0x06)?; // TODO where does this come from?
        radio.PERF_F1C().write(0x07)?;
        radio.PERF_F21().write(0x68)?; // TODO PM recommends 0x5C
        radio.PERF_F22().write(0xFF)?; // TODO PM recommends 0x53
        radio.PERF_F23().write(0x84)?; // TODO PM recommends 0x76
        radio.PERF_F26().write(0x98)?; // TODO PM recommends 0x92
        radio.PERF_F44().write(0x25)?; // TODO PM recommends 0x24

        // PERF 0x30-0x33 depend on WOR

        // FIXME: Depends on register FRAIMING
        radio.PERF_F72().write(0x00)?;
        Ok(self)
    }
}
/*
 * Synthesizer configuration
 */

#[derive(Copy, Clone, Debug)]
pub enum LoopFilter {
    External,
    Internalx1,
    Internalx2,
    Internalx5,
}

#[rustfmt::skip]
impl From<LoopFilter> for FLT {
    fn from(v: LoopFilter) -> Self {
        match v {
            LoopFilter::External   => Self::EXTERNAL,
            LoopFilter::Internalx1 => Self::INTERNAL_x1,
            LoopFilter::Internalx2 => Self::INTERNAL_x2,
            LoopFilter::Internalx5 => Self::INTERNAL_x5,
        }
    }
}

pub struct PLL {
    // TODO: Can I time how long it takes to settle?
    pub filter_bandwidth: LoopFilter, //TODO: Hz, Depends on CPI, internal/external filt (PLLLOOP::FLT)
    // Enable and bypass external filter (PLLLOOP::{FILTEN, DIRECT}?
    pub charge_pump_current: u8, //TODO: uA, // PLL{LOOP}CPI, TODO: when filterBandwith is hz, calc?
                                 // TODO: filten/direct
                                 // Does FILTEN control Active/passive?
                                 // Does DIRECT control external/internal?
                                 // what would FILTEN 1, DIRECT 0 do?
}

#[derive(Copy, Clone, Debug)]
pub enum LockDetector {
    Delay6ns,
    Delay9ns,
    Delay12ns,
    Delay14ns,
}

#[rustfmt::skip]
impl From<LockDetector> for LockDetDly {
    fn from(v: LockDetector) -> Self {
        match v {
            LockDetector::Delay6ns  => Self::d6ns,
            LockDetector::Delay9ns  => Self::d9ns,
            LockDetector::Delay12ns => Self::d12ns,
            LockDetector::Delay14ns => Self::d14ns,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum RangingClock {
    XtalDiv256,
    XtalDiv512,
    XtalDiv1024,
    XtalDiv2048,
    XtalDiv4096,
    XtalDiv8192,
    XtalDiv16384,
    XtalDiv32687,
}

#[rustfmt::skip]
impl From<RangingClock> for PLLRngClk {
    fn from(v: RangingClock) -> Self {
        match v {
            RangingClock::XtalDiv256   => Self::XTAL_DIV_2pow8,
            RangingClock::XtalDiv512   => Self::XTAL_DIV_2pow9,
            RangingClock::XtalDiv1024  => Self::XTAL_DIV_2pow10,
            RangingClock::XtalDiv2048  => Self::XTAL_DIV_2pow11,
            RangingClock::XtalDiv4096  => Self::XTAL_DIV_2pow12,
            RangingClock::XtalDiv8192  => Self::XTAL_DIV_2pow13,
            RangingClock::XtalDiv16384 => Self::XTAL_DIV_2pow14,
            RangingClock::XtalDiv32687 => Self::XTAL_DIV_2pow15,
        }
    }
}

/* SYNTHBOOST and SYNTHSETTLE form the two
 * stage procedure to settle the synthesizer on the first LO
 * frequency. During SYNTHBOOST, the synthesizer is
 * operated at a higher loop bandwidth (register
 * PLLLOOPBOOST), while during SYNTHSETTLE, the
 * final settling is done at the nominal, lower noise, loop
 * bandwidth (register PLLLOOP)
 */

#[derive(Copy, Clone, Debug)]
pub enum FreqReg {
    A,
    B,
}

impl From<FreqReg> for FreqSel {
    fn from(v: FreqReg) -> Self {
        match v {
            FreqReg::A => Self::A,
            FreqReg::B => Self::B,
        }
    }
}

pub enum Control<T> {
    Automatic,
    Manual(T),
}

pub struct Synthesizer {
    pub freq_a: Hz,
    pub freq_b: Hz,
    pub active: FreqReg, // PLLLOOP::FREQSEL, shared between PLLLOOP and PLLLOOPBOOST
    // TODO: TMG{RX,TX} boost and settle times?
    pub pll: PLL,
    pub boost: PLL,

    // PLLVCODIV::REFDIV derived from xo freq, keep input below 24.8mhz
    // PLLVCODIV::RFDIV derived from carrier freq (also see F34). See blackmagic for range, also DS
    // Table 8. Depends also on vcosel
    // PLLVCODIV::{VCOSEL,VCO2INT} from phys layout
    // PLLRANGING::VCOR{A,B} saved, otherwise 8. also has status

    pub vco_current: Control<u8>, //depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
    pub lock_detector_delay: Control<LockDetector>, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
    pub ranging_clock: RangingClock, // less than one tenth the loop filter bandwidth. Derive?
}

fn to_freq(carrier: u64, xtal: u64) -> u32 {
    // PM Table 75: FREQA = fcarrier/fxtal * 2^24 + 1/2
    // It is not recommended to use an RF frequency that is an integer multiple of the reference
    // frequency, due to stray RF desensitizing the receiver.
    // TODO: is fcarrier RF freqency and fxtal reference? I think no, reference comes out of PLL
    //
    // It is strongly recommended to always set bit 0 to avoid spectral tones.
    // FIXME: does this math math right?
    (((carrier << 24) / xtal) | 0x01).try_into().unwrap()
}

impl Synthesizer {
    pub fn write(self, radio: &mut Registers, board: &Board) -> Result<Self> {
        // TODO
        // - FREQA
        // - FREQB
        // - PLLLOOP
        // - PLLCPI
        // - PLLLOOPBOOST
        // - PLLCPIBOOST
        // - PLLVCODIV TODO: has VCOI-MAN in table but not description? Bit 6
        // - F34
        // - PLLVCOI
        // - PLLLOCKDET
        // - PLLRNGCLK

        // TODO: cross check synth.{pll,boost}.filter_bandwidth with board.filter

        radio
            .FREQA()
            .write(to_freq(self.freq_a, board.xtal.freq))?;
        radio
            .FREQB()
            .write(to_freq(self.freq_b, board.xtal.freq))?;

        radio.PLLLOOP().write(PLLLoop {
            filter: self.pll.filter_bandwidth.into(),
            flags: PLLLoopFlags::DIRECT,
            freqsel: self.active.into(),
        })?;
        radio.PLLCPI().write(self.pll.charge_pump_current)?;

        radio.PLLLOOPBOOST().write(PLLLoop {
            filter: self.boost.filter_bandwidth.into(),
            flags: PLLLoopFlags::DIRECT,
            freqsel: self.active.into(),
        })?;
        radio.PLLCPIBOOST().write(self.boost.charge_pump_current)?;

        // TODO: Check VCO with center freqency, check blackmagic table
        // See Datasheet, table 8
        // FIXME: Also apply to freq_b if selected
        let mut flags = PLLVCODivFlags::empty();
        if self.freq_a > 400_000_000 && self.freq_a < 525_000_000 {
            flags |= PLLVCODivFlags::RFDIV;
        }

        let vco = match board.vco {
            // The way I read the datasheet, VCO2INT shouldn't matter if VCOSEL isn't set
            // but:
            // - Radiolab sets it anyway
            // - The recommended schematics have L1 and L2 shorted, which VCO2INT probably
            //   affects.
            // This makes me wonder if it has some kind of effect on VCO2 which might lead to
            // reduced noise or something.
            VCO::Internal => PLLVCODivFlags::VCO2INT, // PLLVCODivFlags::empty(),
            VCO::Inductor => PLLVCODivFlags::VCOSEL | PLLVCODivFlags::VCO2INT,
            VCO::External => PLLVCODivFlags::VCOSEL,
        };
        radio.PLLVCODIV().write(PLLVCODiv {
            mode: PLLVCORefDiv::F_XTAL, // FIXME how to config?
            flags: flags | vco,
        })?;
        if flags.contains(PLLVCODivFlags::RFDIV) {
            radio.PERF_F34().write(PerfF34::RFDIV_set)?;
        } else {
            radio.PERF_F34().write(PerfF34::RFDIV_unset)?;
        }

        radio.PLLVCOI().write(match self.vco_current {
            Control::Manual(x) => PLLVCOI {
                bias: x,
                flags: PLLVCOIFlags::MANUAL,
            },
            Control::Automatic => PLLVCOI {
                bias: 0,
                flags: PLLVCOIFlags::AUTOMATIC,
            },
        })?;

        let lock_default = PLLLockDet {
            delay: LockDetDly::d14ns,
            flags: LockDetFlags::AUTOMATIC,
            readback: LockDetDly::d6ns,
        };
        radio.PLLLOCKDET().write(match self.lock_detector_delay {
            Control::Manual(x) => PLLLockDet {
                flags: LockDetFlags::MANUAL,
                delay: x.into(),
                ..lock_default
            },
            Control::Automatic => PLLLockDet {
                flags: LockDetFlags::AUTOMATIC,
                ..lock_default
            },
        })?;
        radio.PLLRNGCLK().write(self.ranging_clock.into())?;
        Ok(self)
    }

    pub fn autorange(&self, radio: &mut Registers) -> Result<()> {
        /* If both frequency register sets FREQA and FREQB are used, then both
         * frequencies must be auto-ranged by first starting auto-ranging in
         * PLLRANGINGA, waiting for its completion, followed by starting auto-ranging in
         * PLLRANGINGB and waiting for its completion.
         */

        // TODO: try to merge both ranging in one powerup or one by one.

        /* PM Figure 8: Autoranging flow chart
         *    Set PWRMODE to STANDBY
         *          Enable TCXO
         *                |
         *                V
         *    Wait until oscillator is ready
         *                |
         *                V
         *    Set RNGSTART in PLLRANGINGA/B
         *                |
         *                V
         *    Wait until RNGSTART is set to 0
         *                |
         *                V
         *    Check if RNGERR is set, if yes error out
         *                |
         *                v
         *       Return to POWERDOWN
         *          Disable TCXO
         */

        // TODO: check PLL lock/Sticky lock
        radio.PWRMODE().write(PwrMode {
            mode: PwrModes::XOEN,
            flags: PwrFlags::XOEN | PwrFlags::REFEN,
        })?; // TODO REFEN corresponds to REF and VREF in POWSTAT. Is this the power
             // domain for the synth? I assume that means we need it for autoranigng then
             // but this should be tested.
        while radio.XTALSTATUS().read()? != XtalStatus::XTAL_RUN {} // TODO: IRQXTALREADY

        radio.PLLRANGINGA().write(PLLRanging {
            vcor: 0x08,
            flags: PLLRangingFlags::RNG_START,
        })?; // TODO: cache or pre-calc VCORA/B?

        while radio
            .PLLRANGINGA()
            .read()?
            .flags
            .contains(PLLRangingFlags::RNG_START)
        {} // TODO: IRQRNGDONE

        if radio
            .PLLRANGINGA()
            .read()?
            .flags
            .contains(PLLRangingFlags::RNGERR)
        {
            return Err(Error::Autorange);
        }

        //println!("\n{:?}", radio.PLLRANGINGA().read()?);
        //println!("PLLVCOIR: 0x{:x?}", radio.PLLVCOIR().read()?);
        //println!("PLLOCKDET: 0x{:x?}", radio.PLLLOCKDET().read()?);
        radio.PWRMODE().write(PwrMode {
            mode: PwrModes::POWEROFF,
            flags: PwrFlags::empty(),
        })?;
        Ok(())
    }
}

pub enum ADCKind {
    ADC13,
    ADC1,
    ADC2,
    ADC3,
}

pub struct ADCcfg {
    pub sext: bool,
    pub offs: bool,
    pub kind: ADCKind,
}

#[derive(Clone, Copy, Debug)]
pub enum SlowRamp {
    Bits1,
    Bits2,
    Bits4,
    Bits8,
}

impl From<SlowRamp> for crate::SlowRamp {
    fn from(x: SlowRamp) -> Self {
        match x {
            SlowRamp::Bits1 => Self::STARTUP_1b,
            SlowRamp::Bits2 => Self::STARTUP_2b,
            SlowRamp::Bits4 => Self::STARTUP_4b,
            SlowRamp::Bits8 => Self::STARTUP_8b,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BT(pub f32);

impl TryFrom<BT> for ModCfgF {
    type Error = ();
    fn try_from(v: BT) -> std::result::Result<Self, Self::Error> {
        match v {
            x if x == BT(0.3) => Ok(ModCfgF::GAUSSIAN_BT_0p3),
            x if x == BT(0.5) => Ok(ModCfgF::GAUSSIAN_BT_0p5),
            _ => Err(()),
        }
    }
}

pub enum Modulation {
    ASK,
    ASKCoherent, // FIXME part of ASK, relevent to detection only? has a fifo cmd
    PSK {
        ramp: u8,
    },
    OQPSK {
        ramp: u8,
    },
    MSK {
        ramp: SlowRamp,
    },
    GMSK {
        ramp: SlowRamp,
        bt: BT,
    },
    FSK {
        deviation: u64,
        ramp: SlowRamp,
    },
    GFSK {
        deviation: u64,
        ramp: SlowRamp,
        bt: BT,
    },
    FSK4 {
        deviation: u64,
        ramp: SlowRamp,
        nosync: bool,
    },
    AFSK {
        deviation: u64,
        ramp: SlowRamp,
    }, // Note: different formula than FSK
    FM {
        deviation: u64,
        ramp: SlowRamp,
        adc: ADCcfg,
    },
    //  AM?
}

// ENCODING NOSYNC only relevent in 4-fsk
// Encoding in general looks dependent on modulation?
// FEC only works with HDLC
// Manchester, FM0, and FM1 are not recommended for
// new systems, as they double the bitrate.
// In HDLC [1] mode, use NRZI, NRZI + Scrambler, or
// NRZ + Scrambler.
// In Raw modes, the choice depends on the legacy
// system to be implemented

pub struct FEC {
    // FIXME: stuff
}

pub enum Framing {
    Raw,
    RawSoft,
    HDLC { fec: FEC },
    RawPattern,
    MBus,
    MBus4t6,
}

#[rustfmt::skip]
pub enum CRC {
    None,
    CCITT {initial: u16},
    CRC16 {initial: u16},
    DNP   {initial: u16},
    CRC32 {initial: u32},
}

#[derive(PartialEq)]
pub enum BitOrder {
    LSBFirst,
    MSBFirst,
}

pub struct ChannelParameters {
    pub modulation: Modulation,
    // If BROWN GATE is set, the transmitter is disabled
    // whenever one (or more) of the SSVIO, SSBEVMODEM or
    // SSBEVANA bits of the POWSTICKYSTAT register is zero.
    // In order for this to work, the user must read the
    // POWSTICKYSTAT after setting the PWRMODE register
    // for transmission.
    pub encoding: Encoding,
    pub framing: Framing,
    pub crc: CRC,
    pub datarate: u64,
    pub bitorder: BitOrder,
}

impl ChannelParameters {
    pub fn write(self, radio: &mut Registers, board: &Board) -> Result<Self> {
        match self.modulation {
            Modulation::GFSK { deviation, .. } => {
                radio.MODULATION().write(crate::Modulation {
                    mode: ModulationMode::FSK,
                    halfspeed: false,
                })?;
                radio.FSKDEV().write(
                    (deviation * 2_u64.pow(24) / board.xtal.freq)
                        .try_into()
                        .unwrap(),
                )?;
            }
            Modulation::GMSK { .. } => {
                radio.MODULATION().write(crate::Modulation {
                    mode: ModulationMode::FSK,
                    halfspeed: false,
                })?;
                // m = 0.5, fskdev = 0.5 * f_dev, 1/(0.5*0.5) = 4
                radio.FSKDEV().write(
                    ((self.datarate / 4) * 2_u64.pow(24) / board.xtal.freq)
                        .try_into()
                        .unwrap(),
                )?;
            }
            Modulation::ASK => {
                radio.MODULATION().write(registers::Modulation {
                    mode: ModulationMode::ASK,
                    halfspeed: false,
                })?;
            }
            _ => unimplemented!(),
        }

        radio.ENCODING().write(self.encoding)?;
        radio.FRAMING().write(crate::Framing {
            frmmode: FrameMode::HDLC,
            crcmode: CRCMode::CCITT,
            flags: FramingFlags::empty(),
        })?;
        radio.CRCINIT().write(0xFFFF_FFFF)?;

        let flags = PktAddrCfgFlags::FEC_SYNC_DIS // FIXME config when FEC is ready
            | if self.bitorder == BitOrder::MSBFirst {
                PktAddrCfgFlags::MSB_FIRST
            } else {
                // LSB first is unset MSB
                PktAddrCfgFlags::empty()
            };

        radio.PKTADDRCFG().write(PktAddrCfg {
            addr_pos: 0,
            flags,
        })?;

        Ok(self)
    }
}

pub enum AmplitudeShaping {
    None {
        b: u16,
    }, // Datasheet says shaping should always be used? even in AM modes?
    RaisedCosine {
        a: u16,
        b: u16,
        c: u16,
        d: u16,
        e: u16,
    },
}
/* FIXME`
For conventional (non-predistorted output), α0 = α2 = α3
= α4 = 0 and 0 ≤ α1 ≤ 1 controls the output power. If hard
amplitude shaping is selected, both the raised cosine
amplitude shaper and the predistortion is bypassed, and α1
used.
*/
pub struct TXParameters {
    pub antenna: Antenna,
    pub amp: AmplitudeShaping,
    pub plllock_gate: bool,
    pub brownout_gate: bool,
}

impl TXParameters {
    pub fn write(
        self,
        radio: &mut Registers,
        board: &Board,
        channel: &ChannelParameters,
    ) -> Result<Self> {
        // MODULATION,
        // FSKDEV,
        // MODCFGA,
        // MODCFGF,
        match channel.modulation {
            Modulation::GFSK { ramp, bt, .. } | Modulation::GMSK { ramp, bt } => {
                radio.MODCFGF().write(bt.try_into().unwrap())?;
                let cfga = ModCfgA {
                    slowramp: ramp.into(),
                    flags: match self.antenna {
                        Antenna::SingleEnded => ModCfgAFlags::TXSE,
                        Antenna::Differential => ModCfgAFlags::TXDIFF,
                    } | match self.amp {
                        AmplitudeShaping::RaisedCosine { .. } => ModCfgAFlags::AMPLSHAPE,
                        AmplitudeShaping::None { .. } => ModCfgAFlags::empty(),
                    } | if self.plllock_gate {
                        ModCfgAFlags::PLLLCK_GATE
                    } else {
                        ModCfgAFlags::empty()
                    } | if self.brownout_gate {
                        ModCfgAFlags::BROWN_GATE
                    } else {
                        ModCfgAFlags::empty()
                    },
                };
                radio.MODCFGA().write(cfga)?;
            }
            Modulation::ASK => {
                let cfga = ModCfgA {
                    slowramp: SlowRamp::Bits1.into(),
                    flags: match self.antenna {
                        Antenna::SingleEnded => ModCfgAFlags::TXSE,
                        Antenna::Differential => ModCfgAFlags::TXDIFF,
                    } | match self.amp {
                        AmplitudeShaping::RaisedCosine { .. } => ModCfgAFlags::AMPLSHAPE,
                        AmplitudeShaping::None { .. } => ModCfgAFlags::empty(),
                    } | if self.plllock_gate {
                        ModCfgAFlags::PLLLCK_GATE
                    } else {
                        ModCfgAFlags::empty()
                    } | if self.brownout_gate {
                        ModCfgAFlags::BROWN_GATE
                    } else {
                        ModCfgAFlags::empty()
                    },
                };
                radio.MODCFGA().write(cfga)?;
            }
            _ => unimplemented!(),
        }

        radio.TXRATE().write(
            (channel.datarate * 2_u64.pow(24) / board.xtal.freq)
                .try_into()
                .unwrap(),
        )?;
        match self.amp {
            AmplitudeShaping::RaisedCosine { a, b, c, d, e } => {
                radio.TXPWRCOEFFA().write(a)?;
                radio.TXPWRCOEFFB().write(b)?;
                radio.TXPWRCOEFFC().write(c)?;
                radio.TXPWRCOEFFD().write(d)?;
                radio.TXPWRCOEFFE().write(e)?;
            }
            AmplitudeShaping::None { b } => {
                radio.TXPWRCOEFFA().write(0)?;
                radio.TXPWRCOEFFB().write(b)?;
                radio.TXPWRCOEFFC().write(0)?;
                radio.TXPWRCOEFFD().write(0)?;
                radio.TXPWRCOEFFE().write(0)?;
            }
        }

        Ok(self)
    }
}

pub enum RXParameters {
    MSK {
        // MODULATION::RX_HALFSPEED
        //max_dr_offset: u64,
        freq_offs_corr: bool,
        ampl_filter: u8,
        frequency_leak: u8,
    },
}

impl RXParameters {
    pub fn write(
        self,
        radio: &mut Registers,
        board: &config::Board,
        channel: &config::ChannelParameters,
    ) -> Result<Self> {
        match self {
            Self::MSK {
                freq_offs_corr,
                ampl_filter,
                frequency_leak,
            } => {
                // modulation index: m = 0.5;
                // bandwidth = (1+m) * datarate (Carson's rule)
                // TODO: Radiolab lists bandwidth always as 1.5*datarate, does not compensate for m
                let bandwidth = 3 * channel.datarate / 2;
                // FIXME PHASEGAIN::FILTERIDX but translated through table 116. Note that column
                // names are swapped. Label -3dB BW should be nominal BW.
                // Radiolab calculates -3dB BW as nominal BW * 1.1, but where does 1.1 come from? TODO
                // fcoeff = 0.221497
                // 1/fcoeff = 4.51...  ~=  9/2
                let fbaseband_min = bandwidth * 9 / 2; // FIXME: this math makes fbaseband_min slightly too small
                //pick decimation such that fbaseband is at least fbaseband_min
                //integer devision ensures that decimation value will always pick higher fbaseband
                //TODO: we can pick a filter such that this is closer
                let decimation = board.xtal.freq / (board.xtal.div() * 2_u64.pow(4) * fbaseband_min);
                radio.DECIMATION().write(decimation.try_into().unwrap())?; // TODO: 7bits max
                //Now that we have a fixed fbaseband, we can re-determine the bandwidth
                let fbaseband = board.xtal.freq / (board.xtal.div() * 2_u64.pow(4) * decimation);
                let bandwidth = fbaseband * 2 / 9;
                // NBM lists IF freq as half the bandwidth? Is there a way to find the optimal?
                // RadioLab does 1.25 * datarate = (4/5)*bandwidth below ~42.3 except for the weird
                // flat bands at 3.6, 7.2, 14.4, 28.2, 43.2, and 57.6. Above 42.3 (so really above
                // 57.6) it's datarate / 4.9 + 44.25 = bandwidth / 7.35 + 44.25
                let if_freq = if channel.datarate < 42_300 {
                    bandwidth * 4 / 5
                } else {
                    // bandwidth / 7.35 + 44.25 ~= (bandwidth + 325) * 5 / 37
                    (bandwidth + 325) * 5 / 37
                };
                radio.IFFREQ().write(
                    (if_freq * board.xtal.div() * 2_u64.pow(20) / board.xtal.freq)
                        .try_into()
                        .unwrap(),
                )?;

                // TODO: see note table 96
                radio.RXDATARATE().write(
                    (2u64.pow(7) * board.xtal.freq
                        / (channel.datarate * board.xtal.div() * decimation))
                        .try_into()
                        .unwrap(),
                )?;

                //let droff = (2u64.pow(7) * board.xtal.freq * *max_dr_offset) / (board.xtal.div() * channel.datarate.pow(2) * decimation);
                //radio.MAXDROFFSET.write(droff.try_into().unwrap())?;
                radio.MAXDROFFSET().write(0)?;

                let max_rf_offset = bandwidth/4 ; // bw/4 Upper bound - difference between tx and rx fcarriers. see note pm table 98
                //let max_rf_offset = 873; // From radiolab
                radio.MAXRFOFFSET().write(MaxRFOffset {
                    offset: (max_rf_offset * 2u64.pow(24) / board.xtal.freq)
                        .try_into()
                        .unwrap(),
                    correction: freq_offs_corr,
                })?;

                radio.AMPLFILTER().write(ampl_filter)?;
                radio.FREQUENCYLEAK().write(frequency_leak)?;
            }
        }
        Ok(self)
    }
}

// The note after table 109 has a honking big equation for calculating
// the 3db corner frequency of the AGC loop. We can't do that in integer math
// but the two values it eventually sets, attack and decay, only take the values
// 0 - 14, so we can compress the equation in a lookup table, and then scale it by
// the clock live when we need the actual values. This specifically is 1/stuff
// where stuff is everything that isn't fxtal and fxtaldiv. To get f3dB then
// fxtal / (fxtaldiv * AGCGAIN_LOOP_SCALE[attack|decay])
const AGCGAIN_LOOP_SCALE: [u64; 15] = [
    139,
    245,
    449,
    853,
    1_658,
    3_267,
    6_484,
    12_918,
    25_786,
    51_522,
    102_994,
    205_938,
    411_825,
    823_600,
    1_647_150,
];


pub struct RXParameterAGC {
    attack: u8,
    decay: u8,
    target: u8,
    ahyst: u8,
    min: u8,
    max: u8,
}

impl RXParameterAGC {
    pub fn new(board: &Board, channel: &ChannelParameters) -> Self {
        // Datasheet says attack f3dB should be ~= BITRATE
        //                 decay f3db should be ~= BITRATE/10
        //                 TODO: for (G)FSK/(G)MSK only
        // RadioLAB calculates these as at least instead of about
        let mut attack = 0xF;
        for a in 0..AGCGAIN_LOOP_SCALE.len() {
            let f3db = board.xtal.freq / (board.xtal.div() * AGCGAIN_LOOP_SCALE[a]);
            if f3db < channel.datarate {
                attack = a;
                break
            }
        }

        let mut decay = 0xF;
        for d in 0..AGCGAIN_LOOP_SCALE.len() {
            let f3db = board.xtal.freq / (board.xtal.div() * AGCGAIN_LOOP_SCALE[d]);
            if f3db * 10 < channel.datarate {
                decay = d;
                break
            }
        }

        Self {
            attack: u8::try_from(attack).unwrap(),
            decay: u8::try_from(decay).unwrap(),
            target: 0x84, // RadioLAB always picks this, seems reasonable?
            ahyst: 0,
            min: 0,
            max: 0,
        }
    }

    pub fn off() -> Self {
        Self { // attack/decay value F disables AGC
            attack: 0xF,
            decay: 0xF,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        }
    }
}

pub struct RXParameterFreq {
    pub phase: u8,
    pub freq: u8,
}

pub struct RXParameterGain {
    pub time: TimeGain,
    pub datarate: DRGain,
    pub phase: u8,
    pub filter: u8,
    pub baseband: RXParameterFreq,
    pub rf: RXParameterFreq,
    pub amplitude: u8,
    pub deviation_update: bool, // FIXME below decay?
    pub ampl_agc_jump_correction: bool,
    pub ampl_averaging: bool,
}

pub struct RXParameterBasebandOffset {
    pub a: u8,
    pub b: u8,
}

pub struct RXParameterSet {
    pub agc: RXParameterAGC,
    pub gain: RXParameterGain,
    pub freq_dev: u16,
    pub decay: u8,
    pub baseband_offset: RXParameterBasebandOffset,
}

impl RXParameterSet {
    pub fn write0(&self, radio: &mut Registers) -> Result<()> {
        radio.AGCGAIN0().write(AGCGain {
            attack: self.agc.attack,
            decay: self.agc.decay,
        })?;
        radio.AGCTARGET0().write(self.agc.target)?;
        radio.AGCAHYST0().write(AGCHyst {
            hyst: self.agc.ahyst,
        })?;
        radio.AGCMINMAX0().write(AGCMinMax {
            min: self.agc.min,
            max: self.agc.max,
        })?;
        radio.TIMEGAIN0().write(self.gain.time)?;
        radio.DRGAIN0().write(self.gain.datarate)?;
        radio.PHASEGAIN0().write(PhaseGain {
            gain: self.gain.phase,
            filter: self.gain.filter,
        })?;
        radio.FREQGAINA0().write(FreqGainA {
            gain: self.gain.baseband.phase,
            flags: FreqGainAFlags::empty(),
        })?;
        radio.FREQGAINB0().write(FreqGainB {
            gain: self.gain.baseband.freq,
            flags: FreqGainBFlags::empty(),
        })?;
        radio.FREQGAINC0().write(FreqGainC {
            gain: self.gain.rf.phase,
        })?;
        radio.FREQGAIND0().write(FreqGainD {
            gain: self.gain.rf.freq,
            freeze: false,
        })?;
        radio.AMPLGAIN0().write(AmplGain {
            gain: self.gain.amplitude,
            flags: if self.gain.ampl_agc_jump_correction {
                AmplGainFlags::AGC
            } else {
                AmplGainFlags::empty()
            } | if self.gain.ampl_averaging {
                AmplGainFlags::AVG
            } else {
                AmplGainFlags::empty()
            },
        })?;
        radio.FREQDEV0().write(self.freq_dev)?;
        radio.FOURFSK0().write(FourFSK {
            decay: self.decay,
            update: self.gain.deviation_update,
        })?;
        radio.BBOFFSRES0().write(BBOffsRes {
            res_int_a: self.baseband_offset.a,
            res_int_b: self.baseband_offset.b,
        })?;
        Ok(())
    }

    pub fn write1(&self, radio: &mut Registers) -> Result<()> {
        radio.AGCGAIN1().write(AGCGain {
            attack: self.agc.attack,
            decay: self.agc.decay,
        })?;
        radio.AGCTARGET1().write(self.agc.target)?;
        radio.AGCAHYST1().write(AGCHyst {
            hyst: self.agc.ahyst,
        })?;
        radio.AGCMINMAX1().write(AGCMinMax {
            min: self.agc.min,
            max: self.agc.max,
        })?;
        radio.TIMEGAIN1().write(self.gain.time)?;
        radio.DRGAIN1().write(self.gain.datarate)?;
        radio.PHASEGAIN1().write(PhaseGain {
            gain: self.gain.phase,
            filter: self.gain.filter,
        })?;
        radio.FREQGAINA1().write(FreqGainA {
            gain: self.gain.baseband.phase,
            flags: FreqGainAFlags::empty(),
        })?;
        radio.FREQGAINB1().write(FreqGainB {
            gain: self.gain.baseband.freq,
            flags: FreqGainBFlags::empty(),
        })?;
        radio.FREQGAINC1().write(FreqGainC {
            gain: self.gain.rf.phase,
        })?;
        radio.FREQGAIND1().write(FreqGainD {
            gain: self.gain.rf.freq,
            freeze: false,
        })?;
        radio.AMPLGAIN1().write(AmplGain {
            gain: self.gain.amplitude,
            flags: if self.gain.ampl_agc_jump_correction {
                AmplGainFlags::AGC
            } else {
                AmplGainFlags::empty()
            } | if self.gain.ampl_averaging {
                AmplGainFlags::AVG
            } else {
                AmplGainFlags::empty()
            },
        })?;
        radio.FREQDEV1().write(self.freq_dev)?;
        radio.FOURFSK1().write(FourFSK {
            decay: self.decay,
            update: self.gain.deviation_update,
        })?;
        radio.BBOFFSRES1().write(BBOffsRes {
            res_int_a: self.baseband_offset.a,
            res_int_b: self.baseband_offset.b,
        })?;
        Ok(())
    }

    pub fn write3(&self, radio: &mut Registers) -> Result<()> {
        radio.AGCGAIN3().write(AGCGain {
            attack: self.agc.attack,
            decay: self.agc.decay,
        })?;
        radio.AGCTARGET3().write(self.agc.target)?;
        radio.AGCAHYST3().write(AGCHyst {
            hyst: self.agc.ahyst,
        })?;
        radio.AGCMINMAX3().write(AGCMinMax {
            min: self.agc.min,
            max: self.agc.max,
        })?;
        radio.TIMEGAIN3().write(self.gain.time)?;
        radio.DRGAIN3().write(self.gain.datarate)?;
        radio.PHASEGAIN3().write(PhaseGain {
            gain: self.gain.phase,
            filter: self.gain.filter,
        })?;
        radio.FREQGAINA3().write(FreqGainA {
            gain: self.gain.baseband.phase,
            flags: FreqGainAFlags::empty(),
        })?;
        radio.FREQGAINB3().write(FreqGainB {
            gain: self.gain.baseband.freq,
            flags: FreqGainBFlags::empty(),
        })?;
        radio.FREQGAINC3().write(FreqGainC {
            gain: self.gain.rf.phase,
        })?;
        radio.FREQGAIND3().write(FreqGainD {
            gain: self.gain.rf.freq,
            freeze: false,
        })?;
        radio.AMPLGAIN3().write(AmplGain {
            gain: self.gain.amplitude,
            flags: if self.gain.ampl_agc_jump_correction {
                AmplGainFlags::AGC
            } else {
                AmplGainFlags::empty()
            } | if self.gain.ampl_averaging {
                AmplGainFlags::AVG
            } else {
                AmplGainFlags::empty()
            },
        })?;
        radio.FREQDEV3().write(self.freq_dev)?;
        radio.FOURFSK3().write(FourFSK {
            decay: self.decay,
            update: self.gain.deviation_update,
        })?;
        radio.BBOFFSRES3().write(BBOffsRes {
            res_int_a: self.baseband_offset.a,
            res_int_b: self.baseband_offset.b,
        })?;
        Ok(())
    }
}

/* Register type definitions
 *
 * Since one type can be used for multiple registers, defaults should not be
 * set here.
 * */
// FIXME: rustfmt enum_discrim_align_threshold when it gets out of nightly
//        instead of rustfmt::skip
use bitflags::bitflags;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use std::{fmt, ops::{Index, Range}};
#[cfg(test)] use proptest::prelude::*;
#[cfg(test)] use proptest_derive::Arbitrary;

// newtypes to placate the orphan rule
// registers are big endian
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reg<const S: usize>(pub [u8; S]);
pub type Reg8 = Reg<1>;
type Reg16 = Reg<2>;
type Reg24 = Reg<3>;
type Reg32 = Reg<4>;
// TODO? struct RegN(Vec<u8>);

impl<const S: usize> Index<usize> for Reg<S> {
    type Output = u8;
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const S: usize> Index<Range<usize>> for Reg<S> {
    type Output = [u8];
    fn index(&self, index: Range<usize>) -> &Self::Output {
        &self.0[index]
    }
}

impl<const S: usize> From<[u8; S]> for Reg<S> {
    fn from(item: [u8; S]) -> Self {
        Self(item)
    }
}

impl<const S: usize> From<Reg<S>> for [u8; S] {
    fn from(item: Reg<S>) -> Self {
        item.0
    }
}

impl TryFrom<Reg8> for u8 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl TryFrom<Reg8> for i8 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl TryFrom<Reg16> for u16 {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl TryFrom<Reg16> for i16 {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl TryFrom<Reg24> for u32 {
    type Error = Reg24;
    fn try_from(item: Reg24) -> Result<Self, Self::Error> {
        Ok(Self::from(item[0]) << 16 | Self::from(item[1]) << 8 | Self::from(item[2]))
    }
}

impl TryFrom<Reg24> for i32 {
    type Error = Reg24;
    fn try_from(item: Reg24) -> Result<Self, Self::Error> {
        // Shift down for sign extension
        Ok(
            (Self::from(item[0]) << 24
                | Self::from(item[1]) << 16
                | Self::from(item[2]) << 8)
                >> 8,
        )
    }
}

impl TryFrom<Reg32> for u32 {
    type Error = Reg32;
    fn try_from(item: Reg32) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl TryFrom<Reg32> for i32 {
    type Error = Reg32;
    fn try_from(item: Reg32) -> Result<Self, Self::Error> {
        Ok(Self::from_be_bytes(item.into()))
    }
}

impl From<u8> for Reg8 {
    fn from(item: u8) -> Self {
        Self(item.to_be_bytes())
    }
}

impl From<i8> for Reg8 {
    fn from(item: i8) -> Self {
        Self(item.to_be_bytes())
    }
}

impl From<u16> for Reg16 {
    fn from(item: u16) -> Self {
        Self(item.to_be_bytes())
    }
}

impl From<i16> for Reg16 {
    fn from(item: i16) -> Self {
        Self(item.to_be_bytes())
    }
}

impl From<u32> for Reg24 {
    fn from(item: u32) -> Self {
        Self(item.to_be_bytes()[1..4].try_into().unwrap())
    }
}

impl From<i32> for Reg24 {
    fn from(item: i32) -> Self {
        Self(item.to_be_bytes()[1..4].try_into().unwrap())
    }
}

impl From<u32> for Reg32 {
    fn from(item: u32) -> Self {
        Self(item.to_be_bytes())
    }
}

impl From<i32> for Reg32 {
    fn from(item: i32) -> Self {
        Self(item.to_be_bytes())
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn reg8_u8_inverse(n: u8) {
        assert_eq!(n, Reg8::from(n).try_into().unwrap());
    }

    #[test]
    fn reg8_i8_inverse(n: i8) {
        assert_eq!(n, Reg8::from(n).try_into().unwrap());
    }

    #[test]
    fn reg16_u16_inverse(n: u16) {
        assert_eq!(n, Reg16::from(n).try_into().unwrap());
    }

    #[test]
    fn reg16_i16_inverse(n: i16) {
        assert_eq!(n, Reg16::from(n).try_into().unwrap());
    }

    #[test] // FIXME: Test whole range, possibly fallible conversion?
    fn reg24_u32_inverse(n in 0..2_u32.pow(24)) {
        assert_eq!(n, Reg24::from(n).try_into().unwrap());
    }

    #[test] // FIXME: Test whole range, possibly fallible conversion?
    fn reg24_i32_inverse(n in -2_i32.pow(23)..2_i32.pow(23)) {
        assert_eq!(n, Reg24::from(n).try_into().unwrap());
    }

    #[test]
    fn reg32_u32_inverse(n: u32) {
        assert_eq!(n, Reg32::from(n).try_into().unwrap());
    }

    #[test]
    fn reg32_i32_inverse(n: i32) {
        assert_eq!(n, Reg32::from(n).try_into().unwrap());
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Float<const M: u8> {
    // m width/e width where m + e = 8
    // The ax5043 uses both 5/3 and 4/4 formats
    // 5/3 max value = 31 * 2^7 = 3968
    // 4/4 max value = 15 * 2^15 = 491520
    // 2/6 would require u64
    // 1/7 would require u128, but only represents powers of 2
    pub m: u8,
    pub e: u8,
}

impl<const M: u8> Float<M> {
    pub fn new(val: u64) -> Self {
        let msb: u8 = (u64::BITS - val.leading_zeros()).try_into().unwrap();
        let e = msb.saturating_sub(M); // FIXME: check if greater than e bits
        let m = (val >> e).try_into().unwrap();
        Self { m, e }
    }
}

impl<const M: u8> From<Float<M>> for u64 {
    fn from(value: Float<M>) -> Self {
        u64::from(value.m) * 2u64.pow(value.e.into())
    }
}

impl<const M: u8> fmt::Display for Float<M> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let power = ["⁰", "¹", "²", "³", "⁴", "⁵", "⁶", "⁷", "⁸", "⁹"];
        write!(f, "{}⋅2{}", self.m, power[usize::from(self.e)])
    }
}

pub type Float4 = Float<4>;
// The AX5043 does Mantissa high for Float4, but Exponent high for Float5
// so each type needs its own impl
impl TryFrom<Reg8> for Float4 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            e: item[0] & ((1 << 4) - 1),
            m: (item[0] >> 4) & ((1 << (8 - 4)) - 1),
        })
    }
}

impl From<Float4> for Reg8 {
    fn from(item: Float4) -> Self {
        (item.e | item.m << 4).into()
    }
}

pub type Float5 = Float<5>;

impl TryFrom<Reg8> for Float5 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            m: item[0] & ((1 << 5) - 1),
            e: (item[0] >> 5) & ((1 << (8 - 5)) - 1),
        })
    }
}

impl From<Float5> for Reg8 {
    fn from(item: Float5) -> Self {
        (item.m | item.e << 5).into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn float4_convert(n: u32) {
        let shift = (u32::BITS - n.leading_zeros()).saturating_sub(4);
        assert_eq!(n & 0xF << shift, Float4::new(n).into());
    }

    #[test]
    fn float5_convert(n: u32) {
        let shift = (u32::BITS - n.leading_zeros()).saturating_sub(5);
        assert_eq!(n & 0x1F << shift, Float5::new(n).into());
    }
}

#[test]
fn float4_zero() {
    assert_eq!(0u32, Float4::new(0).into());
}

#[test]
fn float5_zero() {
    assert_eq!(0u32, Float5::new(0).into());
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(test, derive(Arbitrary))]
#[repr(u8)]
#[rustfmt::skip]
pub enum PwrModes{
    POWEROFF  = 0b0000,
    DEEPSLEEP = 0b0001,
    XOEN      = 0b0101,
    FIFOEN    = 0b0111,
    SYNTHRX   = 0b1000,
    RX        = 0b1001,
    WORRX     = 0b1011,
    SYNTHTX   = 0b1100,
    TX        = 0b1101,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PwrFlags: u8 {
        const WDS   = 0x10; // RO
        const REFEN = 0x20;
        const XOEN  = 0x40;
        const RST   = 0x80;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwrMode {
    pub mode: PwrModes,
    pub flags: PwrFlags,
}

impl TryFrom<Reg8> for PwrMode {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PwrModes::try_from(item[0] & 0x0f).or(Err(item))?,
            flags: PwrFlags::from_bits(item[0] & 0xf0).ok_or(item)?,
        })
    }
}

impl From<PwrMode> for Reg8 {
    fn from(item: PwrMode) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn pwrmode_inverse(t in
        (any::<PwrModes>(),
        prop::bits::u8::masked(0xF0).prop_map(
            |x| PwrFlags::from_bits(x).unwrap())).prop_map(
            |(mode, flags)| PwrMode {mode, flags})) {
        assert_eq!(t, Reg8::from(t).try_into().unwrap());
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
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

impl TryFrom<Reg8> for PowStat {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn powstat_read(b in prop::bits::u8::ANY) {
        assert_eq!(b, PowStat::try_from(Reg::<1>([b])).unwrap().bits())
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
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

impl TryFrom<Reg8> for PowIRQMask {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<PowIRQMask> for Reg8 {
    fn from(item: PowIRQMask) -> Self {
        item.bits().into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn powirqmask_inverse(b in prop::bits::u8::ANY.prop_map(|x| PowIRQMask::from_bits(x).unwrap())) {
        assert_eq!(b, Reg8::from(b).try_into().unwrap())
    }
}

bitflags! {
    #[derive(Copy, Clone, Debug, PartialEq)]
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

impl TryFrom<Reg16> for IRQ {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Self::from_bits(u16::from_be_bytes(item.0)).ok_or(item)
    }
}

impl From<IRQ> for Reg16 {
    fn from(item: IRQ) -> Self {
        item.bits().into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn irq_inverse(b in prop::bits::u16::masked(0x1FFF).prop_map(|x| IRQ::from_bits(x).unwrap())) {
        assert_eq!(b, Reg16::from(b).try_into().unwrap())
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct RadioEvent: u16 {
        const DONE          = 1 << 0;
        const SETTLED       = 1 << 1;
        const RADIOSTATECHG = 1 << 2;
        const RXPARAMSETCHG = 1 << 3;
        const FRAMECLK      = 1 << 4;
    }
}

impl TryFrom<Reg16> for RadioEvent {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Self::from_bits(u16::from_be_bytes(item.0)).ok_or(item)
    }
}

impl From<RadioEvent> for Reg16 {
    fn from(item: RadioEvent) -> Self {
        item.bits().into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn radioevent_inverse(b in prop::bits::u16::masked(0x1F).prop_map(
            |x| RadioEvent::from_bits(x).unwrap())) {
        assert_eq!(b, Reg16::from(b).try_into().unwrap())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(test, derive(Arbitrary))]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
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

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(test, derive(Arbitrary))]
pub struct Modulation {
    pub mode: ModulationMode,
    pub halfspeed: bool,
}

impl TryFrom<Reg8> for Modulation {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: ModulationMode::try_from(item[0] & 0x0F).or(Err(item))?,
            halfspeed: item[0] & 0x10 > 0,
        })
    }
}

impl From<Modulation> for Reg8 {
    fn from(item: Modulation) -> Self {
        (u8::from(item.mode) | if item.halfspeed { 0x10 } else { 0 }).into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn modulation_inverse(
        b in
        (any::<ModulationMode>(), prop::bool::ANY).prop_map(
            |(mode, halfspeed)| Modulation {mode, halfspeed})) {
        assert_eq!(b, Reg8::from(b).try_into().unwrap())
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
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
        const NRZISCR = Self::NRZI.bits() | Self::SCRAM.bits();
    }
}

impl TryFrom<Reg8> for Encoding {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<Encoding> for Reg8 {
    fn from(item: Encoding) -> Self {
        item.bits().into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn encoding_inverse(b in prop::bits::u8::masked(0x1F).prop_map(
            |x| Encoding::from_bits(x).unwrap())) {
        assert_eq!(b, Reg8::from(b).try_into().unwrap())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(test, derive(Arbitrary))]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum FrameMode {
    RAW                = 0b000,
    RAW_SOFT_BITS      = 0b001,
    HDLC               = 0b010,
    RAW_PATTERN_MATCH  = 0b011,
    WIRELESS_MBUS      = 0b100,
    WIRELESS_MBUS_4TO6 = 0b101,
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(test, derive(Arbitrary))]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum CRCMode {
    OFF   = 0b000,
    CCITT = 0b001,
    CRC16 = 0b010,
    DNP   = 0b011,
    CRC32 = 0b110,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FramingFlags: u8 {
        const ABORT = 1 << 0;
        const FRMRX = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Framing {
    pub frmmode: FrameMode,
    pub crcmode: CRCMode,
    pub flags: FramingFlags,
}

impl TryFrom<Reg8> for Framing {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            frmmode: FrameMode::try_from((item[0] & 0x0e) >> 1).or(Err(item))?,
            crcmode: CRCMode::try_from((item[0] & 0x70) >> 4).or(Err(item))?,
            flags: FramingFlags::from_bits(item[0] & 0x81).ok_or(item)?,
        })
    }
}

impl From<Framing> for Reg8 {
    fn from(item: Framing) -> Self {
        (u8::from(item.frmmode) << 1 | u8::from(item.crcmode) << 4 | item.flags.bits()).into()
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn framing_inverse(
        b in
        (any::<FrameMode>(), any::<CRCMode>(), prop::bits::u8::masked(0x81).prop_map(
                |x| FramingFlags::from_bits(x).unwrap())).prop_map(
            |(frmmode, crcmode, flags)| Framing {frmmode, crcmode, flags})) {
        assert_eq!(b, Reg8::from(b).try_into().unwrap())
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FECFlags: u8 {
        const ENA         = 1 << 0;
        const POS         = 1 << 4;
        const NEG         = 1 << 5;
        const RST_VITERBI = 1 << 6;
        const SHORT_MEM   = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FEC {
    pub flags: FECFlags,
    pub inpshift: u8,
}

impl TryFrom<Reg8> for FEC {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            flags: FECFlags::from_bits(item[0] & 0xF1).ok_or(item)?,
            inpshift: item[0] & 0x0E,
        })
    }
}

impl From<FEC> for Reg8 {
    fn from(item: FEC) -> Self {
        ((item.inpshift & 0x7) << 1 | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FECStatus {
    pub max_metric: u8,
    pub inv: bool,
}

impl TryFrom<Reg8> for FECStatus {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            max_metric: item[0] & 0x7F,
            inv: item[0] & 0x80 > 0,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
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

impl TryFrom<Reg8> for RadioState {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct XtalStatus: u8 {
        const XTAL_RUN = 1 << 0;
    }
}

impl TryFrom<Reg8> for XtalStatus {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PinState: u8 {
        const SYSCLK = 1 << 0;
        const DCLK   = 1 << 1;
        const DATA   = 1 << 2;
        const IRQ    = 1 << 3;
        const ANTSEL = 1 << 4;
        const PWRAMP = 1 << 5;
    }
}

impl TryFrom<Reg8> for PinState {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum PFSysClkMode {
    ZERO            = 0b0_0000,
    ONE             = 0b0_0001,
    Z               = 0b0_0010,
    F_XTAL_INVERT   = 0b0_0011,
    F_XTAL          = 0b0_0100,
    F_XTAL_DIV_2    = 0b0_0101,
    F_XTAL_DIV_4    = 0b0_0110,
    F_XTAL_DIV_8    = 0b0_0111,
    F_XTAL_DIV_16   = 0b0_1000,
    F_XTAL_DIV_32   = 0b0_1001,
    F_XTAL_DIV_64   = 0b0_1010,
    F_XTAL_DIV_128  = 0b0_1011,
    F_XTAL_DIV_256  = 0b0_1100,
    F_XTAL_DIV_512  = 0b0_1101,
    F_XTAL_DIV_1024 = 0b0_1110,
    LPO             = 0b0_1111,
    TEST            = 0b1_1111,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFSysClk {
    pub mode: PFSysClkMode,
    pub pullup: bool,
}

impl TryFrom<Reg8> for PFSysClk {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFSysClkMode::try_from(item[0] & 0x1f).or(Err(item))?,
            pullup: item[0] & 0x80 > 0,
        })
    }
}

impl From<PFSysClk> for Reg8 {
    fn from(item: PFSysClk) -> Self {
        (u8::from(item.mode) | if item.pullup { 0x80 } else { 0 }).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PFFlags: u8 {
        const INVERT = 1 << 6;
        const PULLUP = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum PFDClkMode {
    ZERO = 0b000,
    ONE  = 0b001,
    Z    = 0b010,
    IN   = 0b011,
    OUT  = 0b100,
    NONE = 0b101,
    TEST = 0b111,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFDClk {
    pub mode: PFDClkMode,
    pub flags: PFFlags,
}

impl TryFrom<Reg8> for PFDClk {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFDClkMode::try_from(item[0] & 0x7).or(Err(item))?,
            flags: PFFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<PFDClk> for Reg8 {
    fn from(item: PFDClk) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFData {
    pub mode: PFDataMode,
    pub flags: PFFlags,
}

impl TryFrom<Reg8> for PFData {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFDataMode::try_from(item[0] & 0xF).or(Err(item))?,
            flags: PFFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<PFData> for Reg8 {
    fn from(item: PFData) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum PFIRQMode {
    ZERO = 0b000,
    ONE  = 0b001,
    Z    = 0b010,
    IRQ  = 0b011,
    TEST = 0b111,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFIRQ {
    pub mode: PFIRQMode,
    pub flags: PFFlags,
}

impl TryFrom<Reg8> for PFIRQ {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFIRQMode::try_from(item[0] & 0xF).or(Err(item))?,
            flags: PFFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<PFIRQ> for Reg8 {
    fn from(item: PFIRQ) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFAntSel {
    pub mode: PFAntSelMode,
    pub flags: PFFlags,
}

impl TryFrom<Reg8> for PFAntSel {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFAntSelMode::try_from(item[0] & 0xF).or(Err(item))?,
            flags: PFFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<PFAntSel> for Reg8 {
    fn from(item: PFAntSel) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PFPwrAmp {
    pub mode: PFPwrAmpMode,
    pub flags: PFFlags,
}

impl TryFrom<Reg8> for PFPwrAmp {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PFPwrAmpMode::try_from(item[0] & 0xF).or(Err(item))?,
            flags: PFFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<PFPwrAmp> for Reg8 {
    fn from(item: PFPwrAmp) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PwrAmp: u8 {
        const PWRAMP = 1 << 0;
    }
}

impl TryFrom<Reg8> for PwrAmp {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<PwrAmp> for Reg8 {
    fn from(item: PwrAmp) -> Self {
        item.bits().into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
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

impl TryFrom<Reg8> for FIFOStat {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        FIFOStat::from_bits(item[0]).ok_or(item)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum FIFOCmds {
    NOP          = 0b00_0000,
    ASK_COHERENT = 0b00_0001,
    CLEAR_ERROR  = 0b00_0010,
    CLEAR_DATA   = 0b00_0011,
    COMMIT       = 0b00_0100,
    ROLLBACK     = 0b00_0101,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FIFOCmd {
    pub mode: FIFOCmds,
    pub auto_commit: bool,
}

impl From<FIFOCmd> for Reg8 {
    fn from(item: FIFOCmd) -> Self {
        (u8::from(item.mode) | if item.auto_commit { 0x80 } else { 0 }).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum FIFOChunkHeaderRX {
    // NOP?
    RSSI       = 0b0011_0001, // RSSI
    FREQOFFS   = 0b0101_0010, // Frequency Offset
    ANTRSSI2   = 0b0101_0101, // Background Noise
    TIMER      = 0b0111_0000, // Timer
    RFFREQOFFS = 0b0111_0011, // RF Frequency Offset
    DATARATE   = 0b0111_0100, // Datarate
    ANTRSSI3   = 0b0111_0101, // Antenna Selection RSSI
    DATA       = 0b1110_0001, // Data
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FIFODataRXFlags: u8 {
        const ABORT    = 1 << 6;
        const SIZEFAIL = 1 << 5;
        const ADDRFAIL = 1 << 4;
        const CRCFAIL  = 1 << 3;
        const RESIDUE  = 1 << 2;
        const PKTEND   = 1 << 1;
        const PKTSTART = 1 << 0;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum FIFOChunkRX {
    // NOP, ?
    RSSI(i8),
    FREQOFFS(u16),
    ANTRSSI2 {
        rssi: i8,
        bgndnoise: u8,
    },
    TIMER(u32),
    RFFREQOFFS(i32),
    DATARATE(u32),
    ANTRSSI3 {
        ant1rssi: i8,
        ant2rssi: i8,
        bgndnoise: u8,
    },
    DATA {
        flags: FIFODataRXFlags,
        data: Vec<u8>,
    },
}

impl TryFrom<Vec<u8>> for FIFOChunkRX {
    type Error = Vec<u8>;
    fn try_from(item: Vec<u8>) -> Result<Self, Self::Error> {
        if item.is_empty() {
            return Err(item);
        }
        match FIFOChunkHeaderRX::try_from(item[0]).or(Err(item.to_vec()))? {
            FIFOChunkHeaderRX::RSSI => {
                if item.len() != 2 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::RSSI(i8::from_be_bytes(
                    item[1..2].try_into().unwrap(),
                )))
            }
            FIFOChunkHeaderRX::FREQOFFS => {
                if item.len() != 3 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::FREQOFFS(u16::from_be_bytes(
                    item[1..3].try_into().unwrap(),
                )))
            }
            FIFOChunkHeaderRX::ANTRSSI2 => {
                if item.len() != 3 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::ANTRSSI2 {
                    rssi: i8::from_be_bytes(item[1..2].try_into().unwrap()),
                    bgndnoise: item[2],
                })
            }
            FIFOChunkHeaderRX::TIMER => {
                if item.len() != 4 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::TIMER(
                    u32::from(item[1]) << 16 | u32::from(item[2]) << 8 | u32::from(item[3]),
                ))
            }
            FIFOChunkHeaderRX::RFFREQOFFS => {
                if item.len() != 4 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::RFFREQOFFS(
                    (i32::from(item[1]) << 24 | i32::from(item[2]) << 16 | i32::from(item[3]) << 8)
                        >> 8,
                ))
            }
            FIFOChunkHeaderRX::DATARATE => {
                if item.len() != 4 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::DATARATE(
                    u32::from(item[1]) << 16 | u32::from(item[2]) << 8 | u32::from(item[3]),
                ))
            }
            FIFOChunkHeaderRX::ANTRSSI3 => {
                if item.len() != 4 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::ANTRSSI3 {
                    // FIXME: PM mislables bytes, which are which?
                    ant1rssi: i8::from_be_bytes(item[1..2].try_into().unwrap()),
                    ant2rssi: i8::from_be_bytes(item[2..3].try_into().unwrap()),
                    bgndnoise: item[3],
                })
            }
            FIFOChunkHeaderRX::DATA => {
                if item.len() <= 4 {
                    return Err(item);
                }
                let length: usize = item[1].into();
                // length includes the flags byte
                if item.len() != length + 2 {
                    return Err(item);
                }
                Ok(FIFOChunkRX::DATA {
                    flags: FIFODataRXFlags::from_bits(item[2]).ok_or(item.to_vec())?,
                    data: item[3..].to_vec(),
                })
            }
        }
    }
}

#[derive(IntoPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum FIFOChunkHeaderTX {
    NOP        = 0b0000_0000, // No Operation
    TXCTRL     = 0b0011_1100, // Transmit Control
    REPEATDATA = 0b0110_0010, // Repeat Data
    DATA       = 0b1110_0001, // Data
    TXPWR      = 0b1111_1101, // Transmit Power
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct TXCtrl: u8 {
        const SETTX    = 1 << 6;
        const TXSE     = 1 << 5;
        const TXDIFF   = 1 << 4;
        const SETANT   = 1 << 3;
        const ANTSTATE = 1 << 2;
        const SETPA    = 1 << 1;
        const PASTATE  = 1 << 0;
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FIFODataTXFlags: u8 {
        const UNENC    = 1 << 5;
        const RAW      = 1 << 4;
        const NOCRC    = 1 << 3;
        const RESIDUE  = 1 << 2;
        const PKTEND   = 1 << 1;
        const PKTSTART = 1 << 0;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum FIFOChunkTX {
    NOP,
    TXCTRL(TXCtrl),
    REPEATDATA {
        flags: FIFODataTXFlags,
        count: u8,
        data: u8,
    },
    DATA {
        flags: FIFODataTXFlags,
        data: Vec<u8>,
    },
    TXPWR {
        a: u16,
        b: u16,
        c: u16,
        d: u16,
        e: u16,
    },
}

impl From<FIFOChunkTX> for Vec<u8> {
    fn from(item: FIFOChunkTX) -> Self {
        match item {
            FIFOChunkTX::NOP => vec![0],
            FIFOChunkTX::TXCTRL(flags) => vec![FIFOChunkHeaderTX::TXCTRL.into(), flags.bits()],
            FIFOChunkTX::REPEATDATA { flags, count, data } => {
                vec![
                    FIFOChunkHeaderTX::REPEATDATA.into(),
                    flags.bits(),
                    count,
                    data,
                ]
            }
            FIFOChunkTX::DATA { flags, mut data } => {
                let mut chunk = vec![
                    FIFOChunkHeaderTX::DATA.into(),
                    data.len().try_into().unwrap(),
                    flags.bits(),
                ];
                chunk[1] += 1; // length includes flag byte
                chunk.append(&mut data);
                chunk
            }
            FIFOChunkTX::TXPWR { a, b, c, d, e } => {
                let mut chunk = vec![FIFOChunkHeaderTX::TXPWR.into()];
                chunk.append(&mut a.to_be_bytes().to_vec());
                chunk.append(&mut b.to_be_bytes().to_vec());
                chunk.append(&mut c.to_be_bytes().to_vec());
                chunk.append(&mut d.to_be_bytes().to_vec());
                chunk.append(&mut e.to_be_bytes().to_vec());
                chunk
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum FLT {
    EXTERNAL    = 0b00,
    INTERNAL_x1 = 0b01,
    INTERNAL_x2 = 0b10,
    INTERNAL_x5 = 0b11,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PLLLoopFlags: u8 {
        const FILTEN = 1 << 2;
        const DIRECT = 1 << 3;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum FreqSel {
    A = 0,
    B = 1 << 7,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PLLLoop {
    pub filter: FLT,
    pub flags: PLLLoopFlags,
    pub freqsel: FreqSel,
}

impl TryFrom<Reg8> for PLLLoop {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            filter: FLT::try_from(item[0] & 0x3).or(Err(item))?,
            flags: PLLLoopFlags::from_bits(item[0] & 0xC).ok_or(item)?,
            freqsel: FreqSel::try_from(item[0] & 0x80).or(Err(item))?,
        })
    }
}

impl From<PLLLoop> for Reg8 {
    fn from(item: PLLLoop) -> Self {
        (u8::from(item.filter) | item.flags.bits() | u8::from(item.freqsel)).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum PLLVCORefDiv {
    F_XTAL = 0b00,
    F_XTAL_DIV_2 = 0b01,
    F_XTAL_DIV_4 = 0b10,
    F_XTAL_DIV_8 = 0b11,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PLLVCODivFlags: u8 {
        const RFDIV = 1 << 2;
        const VCOSEL = 1 << 4;
        const VCO2INT = 1 << 5;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PLLVCODiv {
    pub mode: PLLVCORefDiv,
    pub flags: PLLVCODivFlags,
}

impl TryFrom<Reg8> for PLLVCODiv {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            mode: PLLVCORefDiv::try_from(item[0] & 0x03).or(Err(item))?,
            flags: PLLVCODivFlags::from_bits(item[0] & 0xFC).ok_or(item)?,
        })
    }
}

impl From<PLLVCODiv> for Reg8 {
    fn from(item: PLLVCODiv) -> Self {
        (u8::from(item.mode) | item.flags.bits()).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PLLRangingFlags: u8 {
        const RNG_START = 1 << 4;
        const RNGERR = 1 << 5;
        const PLL_LOCK = 1 << 6;
        const STICKY_LOCK = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PLLRanging {
    pub vcor: u8,
    pub flags: PLLRangingFlags,
}

impl TryFrom<Reg8> for PLLRanging {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            vcor: item[0] & 0x0f,
            flags: PLLRangingFlags::from_bits(item[0] & 0xf0).ok_or(item)?,
        })
    }
}

impl From<PLLRanging> for Reg8 {
    fn from(item: PLLRanging) -> Self {
        (item.vcor & 0x0f | item.flags.bits()).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Diversity: u8 {
        const DIVENA = 1 << 0;
        const ANTSEL = 1 << 1;
    }
}

impl TryFrom<Reg8> for Diversity {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<Diversity> for Reg8 {
    fn from(item: Diversity) -> Self {
        item.bits().into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SignalStr {
    pub rssi: i8,
    pub bgndrssi: u8,
    pub diversity: Diversity,
    pub agccounter: i8,
}

impl TryFrom<Reg<4>> for SignalStr {
    type Error = Reg<4>;
    fn try_from(item: Reg<4>) -> Result<Self, Self::Error> {
        Ok(Self {
            rssi: Reg8::from(item[0]).try_into().map_err(|_| item)?,
            bgndrssi: Reg8::from(item[1]).try_into().map_err(|_| item)?,
            diversity: Reg8::from(item[2]).try_into().map_err(|_| item)?,
            agccounter: Reg8::from(item[3]).try_into().map_err(|_| item)?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrkPhase(pub i16); // TRKPHASE is a signed 12 bit value

impl TryFrom<Reg16> for TrkPhase {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Ok(Self(
            (((i16::from(item[0])) << 8 | i16::from(item[1]))
             << 4) // for sign extension. The register only sign extends
             >> 4, // out to 12 bits even though we can read 16
        ))
    }
}

impl From<TrkPhase> for Reg16 {
    fn from(item: TrkPhase) -> Self {
        item.0.into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrkRFFreq(pub i32); // TRKRFFREQ is a signed 20 bit value

impl TryFrom<Reg24> for TrkRFFreq {
    type Error = Reg24;
    fn try_from(item: Reg24) -> Result<Self, Self::Error> {
        Ok(Self(
            (((i32::from(item[0])) << 24 | (i32::from(item[1])) << 16 | (i32::from(item[2])) << 8)
             << 4)  // for sign extension. The register only sign extends
             >> 12, // out to 20 bits even though we can read 24
        ))
    }
}

impl From<TrkRFFreq> for Reg24 {
    fn from(item: TrkRFFreq) -> Self {
        item.0.into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrkFSKDemod(pub i16); // TRKFSKDEMOD is a signed 14 bit value

impl TryFrom<Reg16> for TrkFSKDemod {
    type Error = Reg16;
    fn try_from(item: Reg16) -> Result<Self, Self::Error> {
        Ok(Self(
            (((i16::from(item[0])) << 8 | i16::from(item[1]))
             << 2) // for sign extension. The register only sign extends
             >> 2, // out to 14 bits even though we can read 16
        ))
    }
}

impl From<TrkFSKDemod> for Reg16 {
    fn from(item: TrkFSKDemod) -> Self {
        item.0.into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RXTracking {
    pub datarate: i32,
    pub ampl: u16,
    pub phase: TrkPhase,
    pub rffreq: TrkRFFreq,
    pub freq: i16,
    pub fskdemod: TrkFSKDemod,
    pub afskdemod: u16,
}

impl TryFrom<Reg<16>> for RXTracking {
    type Error = Reg<16>;
    fn try_from(item: Reg<16>) -> Result<Self, Self::Error> {
        let datarate:  [u8; 3] = item[0..3].try_into().unwrap();
        let ampl:      [u8; 2] = item[3..5].try_into().unwrap();
        let phase:     [u8; 2] = item[5..7].try_into().unwrap();
        let rffreq:    [u8; 3] = item[7..10].try_into().unwrap();
        let freq:      [u8; 2] = item[10..12].try_into().unwrap();
        let fskdemod:  [u8; 2] = item[12..14].try_into().unwrap();
        let afskdemod: [u8; 2] = item[14..16].try_into().unwrap();

        Ok(Self {
            datarate: Reg24::from(datarate).try_into().map_err(|_| item)?,
            ampl: Reg16::from(ampl).try_into().map_err(|_| item)?,
            phase: Reg16::from(phase).try_into().map_err(|_| item)?,
            rffreq: Reg24::from(rffreq).try_into().map_err(|_| item)?,
            freq: Reg16::from(freq).try_into().map_err(|_| item)?,
            fskdemod: Reg16::from(fskdemod).try_into().map_err(|_| item)?,
            afskdemod: Reg16::from(afskdemod).try_into().map_err(|_| item)?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaxRFOffset {
    pub offset: u32,
    pub correction: bool,
}

impl TryFrom<Reg24> for MaxRFOffset {
    type Error = Reg24;
    fn try_from(item: Reg24) -> Result<Self, Self::Error> {
        let value = u32::from(item[0]) << 16 | u32::from(item[1]) << 8 | u32::from(item[2]);
        Ok(Self {
            offset: value & 0xF_FFFF,
            correction: (value & (1 << 23)) != 0,
        })
    }
}

impl From<MaxRFOffset> for Reg24 {
    fn from(item: MaxRFOffset) -> Self {
        (item.offset | if item.correction { 1 << 23 } else { 0 }).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum RxParamSet {
    Set0 = 0b00,
    Set1 = 0b01,
    Set2 = 0b10,
    Set3 = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RxParamSets(
    pub RxParamSet,
    pub RxParamSet,
    pub RxParamSet,
    pub RxParamSet,
);

impl TryFrom<Reg8> for RxParamSets {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self(
            RxParamSet::try_from(item[0] & 0x3).or(Err(item))?,
            RxParamSet::try_from(item[0] & ((0x3 << 2) >> 2)).or(Err(item))?,
            RxParamSet::try_from(item[0] & ((0x3 << 4) >> 4)).or(Err(item))?,
            RxParamSet::try_from(item[0] & ((0x3 << 6) >> 6)).or(Err(item))?,
        ))
    }
}

impl From<RxParamSets> for Reg8 {
    fn from(item: RxParamSets) -> Self {
        (u8::from(item.0) | u8::from(item.1) << 2 | u8::from(item.2) << 4 | u8::from(item.3) << 6)
            .into()
    }
}

//#[derive(Clone, Copy, Debug, PartialEq)] // FIXME:
//pub enum RxParamSpecial {
//    Normal,
//    CoarseAGC,
//    BBOffsetAcq,
//}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RxParamCurSet {
    pub index: u8,
    pub number: RxParamSet,
    pub special: u8,
}

impl TryFrom<Reg8> for RxParamCurSet {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            index: item[0] & 0x3,
            number: RxParamSet::try_from((item[0] & 0xC) >> 2).or(Err(item))?,
            special: (item[0] & 0xF0) >> 4,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AGCGain {
    pub attack: u8,
    pub decay: u8,
}

impl TryFrom<Reg8> for AGCGain {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            attack: item[0] & 0x0F,
            decay: (item[0] & 0xF0) >> 4,
        })
    }
}

impl From<AGCGain> for Reg8 {
    fn from(item: AGCGain) -> Self {
        (item.attack | item.decay << 4).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AGCHyst {
    pub hyst: u8,
}

impl TryFrom<Reg8> for AGCHyst {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            hyst: item[0] & 0x07,
        })
    }
}

impl From<AGCHyst> for Reg8 {
    fn from(item: AGCHyst) -> Self {
        item.hyst.into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AGCMinMax {
    pub min: u8,
    pub max: u8,
}

impl TryFrom<Reg8> for AGCMinMax {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            min: item[0] & 0x07,
            max: (item[0] >> 3) & 0x07,
        })
    }
}
impl From<AGCMinMax> for Reg8 {
    fn from(item: AGCMinMax) -> Self {
        (item.min | item.max << 3).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseGain {
    pub gain: u8,
    pub filter: u8,
}

impl TryFrom<Reg8> for PhaseGain {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x0F,
            filter: (item[0] >> 6) & 0x03,
        })
    }
}

impl From<PhaseGain> for Reg8 {
    fn from(item: PhaseGain) -> Self {
        (item.gain | item.filter << 6).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FreqGainAFlags: u8 {
        const AMPLGATE = 1 << 4;
        const HALFMOD  = 1 << 5;
        const MODULO   = 1 << 6;
        const LIM      = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FreqGainA {
    pub gain: u8,
    pub flags: FreqGainAFlags,
}

impl TryFrom<Reg8> for FreqGainA {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x0F,
            flags: FreqGainAFlags::from_bits(item[0] & 0xF0).ok_or(item)?,
        })
    }
}

impl From<FreqGainA> for Reg8 {
    fn from(item: FreqGainA) -> Self {
        (item.gain | item.flags.bits()).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FreqGainBFlags: u8 {
        const AVG    = 1 << 6;
        const FREEZE = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FreqGainB {
    pub gain: u8,
    pub flags: FreqGainBFlags,
}

impl TryFrom<Reg8> for FreqGainB {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x1F,
            flags: FreqGainBFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<FreqGainB> for Reg8 {
    fn from(item: FreqGainB) -> Self {
        (item.gain | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FreqGainC {
    pub gain: u8,
}

impl TryFrom<Reg8> for FreqGainC {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x1F,
        })
    }
}

impl From<FreqGainC> for Reg8 {
    fn from(item: FreqGainC) -> Self {
        item.gain.into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FreqGainD {
    pub gain: u8,
    pub freeze: bool,
}

impl TryFrom<Reg8> for FreqGainD {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x1F,
            freeze: item[0] & 0x80 > 0,
        })
    }
}

impl From<FreqGainD> for Reg8 {
    fn from(item: FreqGainD) -> Self {
        (item.gain | if item.freeze { 0x80 } else { 0 }).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct AmplGainFlags: u8 {
        const AGC = 1 << 6;
        const AVG = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AmplGain {
    pub gain: u8,
    pub flags: AmplGainFlags,
}

impl TryFrom<Reg8> for AmplGain {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            gain: item[0] & 0x0F,
            flags: AmplGainFlags::from_bits(item[0] & 0xC0).ok_or(item)?,
        })
    }
}

impl From<AmplGain> for Reg8 {
    fn from(item: AmplGain) -> Self {
        (item.gain | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FourFSK {
    pub decay: u8,
    pub update: bool,
}

impl TryFrom<Reg8> for FourFSK {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            decay: item[0] & 0x0F,
            update: item[0] & 0x10 > 0,
        })
    }
}

impl From<FourFSK> for Reg8 {
    fn from(item: FourFSK) -> Self {
        (item.decay | if item.update { 0x10 } else { 0 }).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BBOffsRes {
    pub res_int_a: u8,
    pub res_int_b: u8,
}

impl TryFrom<Reg8> for BBOffsRes {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            res_int_a: item[0] & 0x0F,
            res_int_b: (item[0] & 0xF0) >> 4,
        })
    }
}

impl From<BBOffsRes> for Reg8 {
    fn from(item: BBOffsRes) -> Self {
        (item.res_int_a | item.res_int_b << 4).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum ModCfgF {
    UNSHAPED        = 0b00,
    GAUSSIAN_BT_0p3 = 0b10,
    GAUSSIAN_BT_0p5 = 0b11,
}

impl TryFrom<Reg8> for ModCfgF {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        ModCfgF::try_from(item[0]).or(Err(item))
    }
}

impl From<ModCfgF> for Reg8 {
    fn from(item: ModCfgF) -> Self {
        u8::from(item).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum SlowRamp {
    STARTUP_1b = 0b00,
    STARTUP_2b = 0b01,
    STARTUP_4b = 0b10,
    STARTUP_8b = 0b11,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct ModCfgAFlags: u8 {
        const TXDIFF      = 1 << 0;
        const TXSE        = 1 << 1;
        const AMPLSHAPE   = 1 << 2;
        const PLLLCK_GATE = 1 << 6;
        const BROWN_GATE  = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModCfgA {
    pub slowramp: SlowRamp,
    pub flags: ModCfgAFlags,
}

impl TryFrom<Reg8> for ModCfgA {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            slowramp: SlowRamp::try_from((item[0] & 0x30) >> 4).or(Err(item))?,
            flags: ModCfgAFlags::from_bits(item[0] & 0xCF).ok_or(item)?,
        })
    }
}

impl From<ModCfgA> for Reg8 {
    fn from(item: ModCfgA) -> Self {
        (u8::from(item.slowramp) << 4 | item.flags.bits()).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PLLVCOIFlags: u8 {
        const AUTOMATIC = 0;
        const MANUAL = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PLLVCOI {
    pub bias: u8,
    pub flags: PLLVCOIFlags,
}

impl TryFrom<Reg8> for PLLVCOI {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            bias: item[0] & 0x3F,
            flags: PLLVCOIFlags::from_bits(item[0] & 0x80).ok_or(item)?,
        })
    }
}

impl From<PLLVCOI> for Reg8 {
    fn from(item: PLLVCOI) -> Self {
        (item.bias | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum LockDetDly {
    d6ns  = 0b00,
    d9ns  = 0b01,
    d12ns = 0b10,
    d14ns = 0b11,
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct LockDetFlags: u8 {
        const AUTOMATIC = 0;
        const MANUAL = 1 << 2;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PLLLockDet {
    pub delay: LockDetDly,
    pub flags: LockDetFlags,
    pub readback: LockDetDly,
}

impl TryFrom<Reg8> for PLLLockDet {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            delay: LockDetDly::try_from(item[0] & 0x03).or(Err(item))?,
            flags: LockDetFlags::from_bits(item[0] & 0x04).ok_or(item)?,
            readback: LockDetDly::try_from((item[0] & 0xc0) >> 6).or(Err(item))?,
        })
    }
}

impl From<PLLLockDet> for Reg8 {
    fn from(item: PLLLockDet) -> Self {
        (u8::from(item.delay) | item.flags.bits() | u8::from(item.readback) << 6).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum PLLRngClk {
    XTAL_DIV_2pow8  = 0b000,
    XTAL_DIV_2pow9  = 0b001,
    XTAL_DIV_2pow10 = 0b010,
    XTAL_DIV_2pow11 = 0b011,
    XTAL_DIV_2pow12 = 0b100,
    XTAL_DIV_2pow13 = 0b101,
    XTAL_DIV_2pow14 = 0b110,
    XTAL_DIV_2pow15 = 0b111,
}

impl TryFrom<Reg8> for PLLRngClk {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

impl From<PLLRngClk> for Reg8 {
    fn from(item: PLLRngClk) -> Self {
        u8::from(item).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PktAddrCfgFlags: u8 {
        const FEC_SYNC_DIS   = 1 << 5;
        const CRC_SKIP_FIRST = 1 << 6;
        const MSB_FIRST      = 1 << 7;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PktAddrCfg {
    pub addr_pos: u8,
    pub flags: PktAddrCfgFlags,
}

impl TryFrom<Reg8> for PktAddrCfg {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            addr_pos: item[0] & 0xF,
            flags: PktAddrCfgFlags::from_bits(item[0] & 0xF0).ok_or(item)?,
        })
    }
}

impl From<PktAddrCfg> for Reg8 {
    fn from(item: PktAddrCfg) -> Self {
        (item.addr_pos | item.flags.bits()).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PktLenCfg {
    pub pos: u8,
    pub bits: u8,
}

impl TryFrom<Reg8> for PktLenCfg {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            pos: item[0] & 0xF,
            bits: (item[0] & 0xF0) >> 4,
        })
    }
}

impl From<PktLenCfg> for Reg8 {
    fn from(item: PktLenCfg) -> Self {
        (item.pos | item.bits << 4).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MatchLen {
    pub len: u8, // FIXME 4bit/5bit
    pub raw: bool,
}

impl TryFrom<Reg8> for MatchLen {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Ok(Self {
            len: item[0] & 0x1F,
            raw: (item[0] & 0x80) > 0,
        })
    }
}

impl From<MatchLen> for Reg8 {
    fn from(item: MatchLen) -> Self {
        (item.len | if item.raw { 0x80 } else { 0 }).into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PktMiscFlags: u8 {
        const RX_RSSI_CLK   = 1 << 0;
        const RX_AGC_CLK    = 1 << 1;
        const BGND_RSSI     = 1 << 2;
        const AGC_SETTL_DET = 1 << 3;
        const WOR_MULTI_PKT = 1 << 4;
    }
}
impl TryFrom<Reg8> for PktMiscFlags {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<PktMiscFlags> for Reg8 {
    fn from(item: PktMiscFlags) -> Self {
        item.bits().into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PktStoreFlags: u8 {
        const TIMER    = 1 << 0;
        const FOFFS    = 1 << 1;
        const RFOFFS   = 1 << 2;
        const DR       = 1 << 3;
        const RSSI     = 1 << 4;
        const CRCB     = 1 << 5;
        const ANT_RSSI = 1 << 6;
    }
}

impl TryFrom<Reg8> for PktStoreFlags {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<PktStoreFlags> for Reg8 {
    fn from(item: PktStoreFlags) -> Self {
        item.bits().into()
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PktAcceptFlags: u8 {
        const RESIDUE = 1 << 0;
        const ABRT    = 1 << 1;
        const CRCF    = 1 << 2;
        const ADDRF   = 1 << 3;
        const SZF     = 1 << 4;
        const LRGP    = 1 << 5;
    }
}

impl TryFrom<Reg8> for PktAcceptFlags {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::from_bits(item[0]).ok_or(item)
    }
}

impl From<PktAcceptFlags> for Reg8 {
    fn from(item: PktAcceptFlags) -> Self {
        item.bits().into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum PerfF10 {
    XO          = 0x03,
    TCXO        = 0x04,
    FreqGT43MHz = 0x0D,
}

impl TryFrom<Reg8> for PerfF10 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

impl From<PerfF10> for Reg8 {
    fn from(item: PerfF10) -> Self {
        u8::from(item).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum PerfF11 {
    XO    = 0x07,
    TCXO  = 0x00,
    Reset = 0x80, // Undocumented, value on reset
}

impl TryFrom<Reg8> for PerfF11 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

impl From<PerfF11> for Reg8 {
    fn from(item: PerfF11) -> Self {
        u8::from(item).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
pub enum PerfF34 {
    RFDIV_unset = 0x08,
    RFDIV_set   = 0x28,
    Reset       = 0x0F, // Undocumented, value on reset
}

impl TryFrom<Reg8> for PerfF34 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

impl From<PerfF34> for Reg8 {
    fn from(item: PerfF34) -> Self {
        u8::from(item).into()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[rustfmt::skip]
pub enum PerfF35 {
    FreqLT24p8MHz = 0x10,
    FreqGE24p8MHz = 0x11,
}

impl TryFrom<Reg8> for PerfF35 {
    type Error = Reg8;
    fn try_from(item: Reg8) -> Result<Self, Self::Error> {
        Self::try_from(item[0]).or(Err(item))
    }
}

impl From<PerfF35> for Reg8 {
    fn from(item: PerfF35) -> Self {
        u8::from(item).into()
    }
}

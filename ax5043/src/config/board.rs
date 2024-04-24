use crate::config::*;

#[rustfmt::skip]
pub const C3_UHF: Board = Board {
    sysclk: Pin { mode: SysClk::Z,      pullup: true,  invert: false, },
    dclk:   Pin { mode: DClk::Z,        pullup: true,  invert: false, },
    data:   Pin { mode: Data::Z,        pullup: true,  invert: false, },
    pwramp: Pin { mode: PwrAmp::PwrAmp, pullup: false, invert: false, },
    irq:    Pin { mode: IRQ::IRQ,       pullup: false, invert: false, },
    antsel: Pin { mode: AntSel::Z,      pullup: true,  invert: false, },
    xtal: Xtal {
        kind: XtalKind::TCXO,
        freq: 16_000_000,
        enable: XtalPin::None,
    },
    vco: VCO::Internal,
    filter: Filter::Internal,
    dac: DAC { pin: DACPin::None },
    adc: ADC::None,
};

#[rustfmt::skip]
pub const C3_LBAND: Board = Board {
    sysclk: Pin { mode: SysClk::XtalDiv1, pullup: false, invert: false, },
    dclk:   Pin { mode: DClk::Z,          pullup: true,  invert: false, },
    data:   Pin { mode: Data::Z,          pullup: true,  invert: false, },
    pwramp: Pin { mode: PwrAmp::PwrAmp,   pullup: false, invert: false, },
    irq:    Pin { mode: IRQ::IRQ,         pullup: false, invert: false, },
    antsel: Pin { mode: AntSel::Z,        pullup: true,  invert: false, },
    xtal: Xtal {
        kind: XtalKind::TCXO,
        freq: 16_000_000,
        enable: XtalPin::None,
    },
    vco: VCO::Internal,
    filter: Filter::Internal,
    dac: DAC { pin: DACPin::None },
    adc: ADC::None,
};

#[rustfmt::skip]
pub const RPI: Board = Board {
    sysclk: Pin { mode: config::SysClk::Z,    pullup: true,  invert: false, },
    dclk:   Pin { mode: config::DClk::Z,      pullup: true,  invert: false, },
    data:   Pin { mode: config::Data::Z,      pullup: true,  invert: false, },
    pwramp: Pin { mode: config::PwrAmp::TCXO, pullup: false, invert: false, },
    irq:    Pin { mode: config::IRQ::IRQ,     pullup: false, invert: false, },
    antsel: Pin { mode: config::AntSel::Z,    pullup: true,  invert: false, },
    xtal: Xtal {
        kind: XtalKind::TCXO,
        freq: 48_000_000,
        enable: XtalPin::AntSel,
    },
    vco: VCO::Internal,
    filter: Filter::Internal,
    dac: DAC {
        pin: DACPin::PwrAmp,
    },
    adc: ADC::ADC1,
};


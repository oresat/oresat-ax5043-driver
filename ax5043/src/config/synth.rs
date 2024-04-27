use crate::config::*;

pub const UHF_436_5: Synthesizer = Synthesizer {
    freq_a: 436_500_000,
    freq_b: 0,
    active: FreqReg::A,
    pll: PLL {
        charge_pump_current: 0x02, // From spreadsheet
        filter_bandwidth: LoopFilter::Internalx1,
    },
    boost: PLL {
        charge_pump_current: 0xC8,                // Default value
        filter_bandwidth: LoopFilter::Internalx5, // Default value
    },
    //vco_current: Manual(0x13), // depends on VCO, readback VCOIR, see AND9858/D for manual cal
    vco_current: Control::Automatic,
    lock_detector_delay: Control::Automatic, // readback PLLLOCKDET::LOCKDETDLYR
    ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
};

pub const LBAND_DC_457: Synthesizer = Synthesizer {
    freq_a: 457_000_000,
    freq_b: 0,
    active: FreqReg::A,
    pll: PLL {
        charge_pump_current: 0x02, // From spreadsheet
        filter_bandwidth: LoopFilter::Internalx1,
    },
    boost: PLL {
        charge_pump_current: 0xC8,                // Default value
        filter_bandwidth: LoopFilter::Internalx5, // Default value
    },
    //vco_current: Manual(0x16), // depends on VCO, readback VCOIR, see AND9858/D for manual cal
    vco_current: Control::Automatic,
    lock_detector_delay: Control::Automatic, // readback PLLLOCKDET::LOCKDETDLYR
    ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
};

pub const LBAND_DC_505: Synthesizer = Synthesizer {
    freq_a: 505_000_000,
    freq_b: 0,
    active: FreqReg::A,
    pll: PLL {
        charge_pump_current: 0x02, // From spreadsheet
        filter_bandwidth: LoopFilter::Internalx1,
    },
    boost: PLL {
        charge_pump_current: 0xC8,                // Default value
        filter_bandwidth: LoopFilter::Internalx5, // Default value
    },
    //vco_current: Manual(0x16), // depends on VCO, readback VCOIR, see AND9858/D for manual cal
    vco_current: Control::Automatic,
    lock_detector_delay: Control::Automatic, // readback PLLLOCKDET::LOCKDETDLYR
    ranging_clock: RangingClock::XtalDiv8192, // less than one tenth the loop filter bandwidth. Derive?
};

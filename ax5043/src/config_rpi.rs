use crate::{
    config::{Framing, Modulation, SlowRamp, FEC, *},
    *,
};

use anyhow::Result;

fn config(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    #[rustfmt::skip]
    let board = Board {
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
    }.write(radio)?;

    let synth = Synthesizer {
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
    }.write(radio, &board)?;

    let channel = ChannelParameters {
        modulation: Modulation::GMSK {
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },
        datarate: 9_600,
        bitorder: BitOrder::LSBFirst,
    }.write(radio, &board)?;

    synth.autorange(radio)?;

    Ok((board, channel))
}

pub fn configure_radio_tx(radio: &mut Registers) -> Result<config::Board> {
    let (board, channel) = config(radio)?;

    TXParameters {
        antenna: Antenna::SingleEnded,
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        plllock_gate: true,
        brownout_gate: true,
    }.write(radio, &board, &channel)?;

    // As far as I can tell PLLUNLOCK and PLLRNGDONE have no way to clear/are level triggered
    // radio.IRQMASK.write(registers::IRQ::XTALREADY | registers::IRQ::RADIOCTRL)?;
    // radio.RADIOEVENTMASK.write(registers::RadioEvent::all())?;

    Ok(board)
}

/*
first SYNTHBOOST SYNTHSETTLE
second IFINIT COARSEAGC AGC RSSI

preamble1: PS0
    TMGRXPREAMBLE1 to reset to second?

preamble2: PS1
    MATCH1
    TMGRXPREAMBLE2

preamble3: PS2
    MATCH0
    TMGRXPREAMBLE3

packet: PS3
    SFD
*/

pub fn configure_radio_rx(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    let (board, channel) = config(radio)?;

    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;
    radio.PLLLOOP().write(PLLLoop {
        filter: FLT::INTERNAL_x5,
        flags: PLLLoopFlags::DIRECT,
        freqsel: FreqSel::A,
    })?;
    radio.PLLCPI().write(0x10)?;

    RXParameters::MSK {
        //max_dr_offset: 0, // TODO derived from what?
        freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    }.write(radio, &board, &channel)?;

    // Set 0
    RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time: Float4 { m: 0xF, e: 8 },
            datarate: Float4 { m: 0xF, e: 2 },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1010,
                freq: 0b0_1010,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    }.write0(radio)?;

    // Set 1
    RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time: Float4 { m: 0xF, e: 6 },
            datarate: Float4 { m: 0xF, e: 1 },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1010,
                freq: 0b0_1010,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0x32,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    }.write1(radio)?;

    // Set 3
    RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time: Float4 { m: 0xF, e: 5 },
            datarate: Float4 { m: 0xF, e: 0 },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1101,
                freq: 0b0_1101,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0x32,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    }.write3(radio)?;

    RXParameterStages {
        preamble1: Some(Preamble1 {
            pattern: PatternMatch1 {
                pat: 0x1111,
                len: 15,
                raw: false,
                min: 0,
                max: 15,
            },
            timeout: Float5 { m: 0x17, e: 5 },
            set: RxParamSet::Set0,
        }),
        preamble2: Some(Preamble2 {
            pattern: PatternMatch0 {
                pat: 0x1111_1111,
                len: 31,
                raw: false,
                min: 0,
                max: 31,
            },
            timeout: Float5 { m: 0x17, e: 0 },
            set: RxParamSet::Set1,
        }),
        preamble3: None,
        packet: RxParamSet::Set3,
    }.write(radio)?;


    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x09)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(64)?;

    Ok((board, channel))
}

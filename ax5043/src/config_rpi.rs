use crate::{
    config::{Framing, Modulation, SlowRamp, FEC, *},
    *,
};

use anyhow::Result;

fn config(radio: &mut Registers, antenna: config::Antenna) -> Result<(Board, ChannelParameters)> {
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
        antenna,
        dac: DAC {
            pin: DACPin::PwrAmp,
        },
        adc: ADC::ADC1,
    };
    configure(radio, &board)?;

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
        //vco_current: Some(0x13), // depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
        vco_current: None,
        lock_detector_delay: None, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };
    configure_synth(radio, &board, &synth)?;

    let channel = ChannelParameters {
        modulation: Modulation::GMSK {
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },
        datarate: 9_600,
    };
    configure_channel(radio, &board, &channel)?;

    autorange(radio)?;

    Ok((board, channel))
}

pub fn configure_radio_tx(radio: &mut Registers) -> Result<config::Board> {
    let (board, channel) = config(radio, config::Antenna::SingleEnded)?;

    let txparams = TXParameters {
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        plllock_gate: true,
        brownout_gate: true,
    };
    configure_tx(radio, &board, &channel, &txparams)?;

    // As far as I can tell PLLUNLOCK and PLLRNGDONE have no way to clear/are level triggered
    // radio.IRQMASK.write(registers::IRQ::XTALREADY | registers::IRQ::RADIOCTRL)?;
    // radio.RADIOEVENTMASK.write(registers::RadioEvent::all())?;

    Ok(board)
}

enum RXParameters {
    Msk {
        //max_dr_offset: u64,
        //freq_offs_corr: bool,
        ampl_filter: u8,
        frequency_leak: u8,
    },
}

impl RXParameters {
    fn write(
        &self,
        radio: &mut Registers,
        _board: &config::Board,
        _channel: &config::ChannelParameters,
    ) -> Result<()> {
        match self {
            Self::Msk {
                //max_dr_offset,
                //freq_offs_corr,
                ampl_filter,
                frequency_leak,
            } => {
                // m = 0.5;
                // bandwidth  = (1+m) * bitrate; // Carson's rule
                //let bandwidth = 3 * channel.datarate / 2;
                //let fcoeff = 0.25; // FIXME PHASEGAIN::FILTERIDX but translated through table 116
                //let fcoeff_inv = 4; // 1/fcoeff

                //let if_freq = bandwidth * 4 / 5;
                //radio.IFFREQ.write((if_freq * board.xtal.div() * 2_u64.pow(20) / board.xtal.freq).try_into().unwrap())?;
                radio.IFFREQ().write(0x20C)?;

                //let fbaseband = bandwidth * (1 + fcoeff_inv);
                //let decimation = board.xtal.freq / (fbaseband * 2u64.pow(4) * board.xtal.div());
                //radio.DECIMATION.write(decimation.try_into().unwrap())?; // TODO: 7bits max
                radio.DECIMATION().write(0x14)?;

                // TODO: see note table 96
                //radio.RXDATARATE.write((2u64.pow(7) * board.xtal.freq / (channel.datarate * board.xtal.div() * decimation)).try_into().unwrap())?;
                radio.RXDATARATE().write(0x3E80)?;

                //let droff = (2u64.pow(7) * board.xtal.freq * *max_dr_offset)
                //    / (board.xtal.div() * channel.datarate.pow(2) * decimation);
                //radio.MAXDROFFSET.write(droff.try_into().unwrap())?;

                radio.MAXDROFFSET().write(0)?;

                //let max_rf_offset = bandwidth / 4; // bw/4 Upper bound - difference between tx and rx fcarriers. see note pm table 98
                                                   //radio.MAXRFOFFSET.write(MaxRFOffset {
                                                   //    offset: (max_rf_offset * 2u64.pow(24) / board.xtal.freq).try_into().unwrap(),
                                                   //    correction: *freq_offs_corr,
                                                   //})?;

                radio.MAXRFOFFSET().write(MaxRFOffset {
                    offset: 0x131,
                    correction: true,
                })?;
                radio.AMPLFILTER().write(*ampl_filter)?;
                radio.FREQUENCYLEAK().write(*frequency_leak)?;
            }
        }
        Ok(())
    }
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

// FIXME need to know:
// h/m?
// fbaseband
// fif/bandwidth?

pub fn configure_radio_rx(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    let (board, channel) = config(radio, config::Antenna::Differential)?;

    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;
    radio.PLLLOOP().write(PLLLoop {
        filter: FLT::INTERNAL_x5,
        flags: PLLLoopFlags::DIRECT,
        freqsel: FreqSel::A,
    })?;
    radio.PLLCPI().write(0x10)?;

    let params = RXParameters::Msk {
        //max_dr_offset: 0, // TODO derived from what?
        //freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    };
    params.write(radio, &board, &channel)?;

    let set0 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0x5,
            decay: 0xC,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 8,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 2,
            },
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
    };
    set0.write0(radio)?;

    let set1 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0x5,
            decay: 0xC,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 6,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 1,
            },
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
    };
    set1.write1(radio)?;

    let set3 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0xF,
            decay: 0xF,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 5,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 0,
            },
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
    };
    set3.write3(radio)?;

    radio.RXPARAMSETS().write(RxParamSets(
        RxParamSet::Set0,
        RxParamSet::Set1,
        RxParamSet::Set3,
        RxParamSet::Set3,
    ))?;

    radio.MATCH1PAT().write(0x7E7E)?;
    radio.MATCH1LEN().write(MatchLen {
        len: 0xA,
        raw: false,
    })?;
    radio.MATCH1MAX().write(0xA)?;
    radio.TMGRXPREAMBLE2().write(TMG { m: 0x17, e: 0 })?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x09)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(64)?;

    Ok((board, channel))
}

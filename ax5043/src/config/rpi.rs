use crate::config::*;
use anyhow::Result;

fn config(radio: &mut Registers) -> Result<(Board, Synthesizer, ChannelParameters)> {
    let board = board::RPI.write(radio)?;
    let synth = synth::UHF_436_5.write(radio, &board)?;
    let channel = channel::GMSK_60000.write(radio, &board)?;

    synth.autorange(radio)?;

    Ok((board, synth, channel))
}

pub fn configure_radio_tx(radio: &mut Registers) -> Result<config::Board> {
    let (board, _, channel) = config(radio)?;

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
    }
    .write(radio, &board, &channel)?;

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
    let (board, synth, channel) = config(radio)?;

    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;

    let rxp = RXParameters::MSK {
        //max_dr_offset: 50, // TODO derived from what?
        max_dr_offset: 0,
        freq_offs_corr: FreqOffsetCorrection::AtFirstLO,
        ampl_filter: 0,
        frequency_leak: 0,
    }
    .write(radio, &board, &synth, &channel)?;

    // Set 0
    RXParameterSet {
        agc: Control::Automatic,
        gain: RXParameterGain {
            time_corr_frac: 4,
            datarate_corr_frac: 255,
            phase: 0b0011,
            filter: 0b11,
            //baseband: None,
            baseband: Some(RXParameterFreq {
                phase: 0x0A,
                freq: 0x0A,
            }),
            rf: None,
            //rf: Some(RXParameterFreq {
            //   phase: 0x0A,
            //   freq: 0x0A,
            //}),
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: None,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    }
    .write0(radio, &board, &channel, &rxp)?;

    // Set 3
    RXParameterSet {
        agc: Control::Automatic,
        gain: RXParameterGain {
            time_corr_frac: 32,
            datarate_corr_frac: 1024,
            phase: 0b0011,
            filter: 0b11,
            //baseband: None,
            baseband: Some(RXParameterFreq {
                phase: 0x0D,
                freq: 0x0D,
            }),
            rf: None,
            //rf: Some(RXParameterFreq {
            //    phase: 0x0D,
            //    freq: 0x0D,
            //}),
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: Some(0x32),
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    }
    .write3(radio, &board, &channel, &rxp)?;

    RXParameterStages {
        preamble1: Some(Preamble1 {
            timeout: Float5 { m: 0x17, e: 5 },
            set: RxParamSet::Set0,
        }),
        preamble2: None,
        preamble3: None,
        packet: RxParamSet::Set3,
    }
    .write(radio)?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x00)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(0)?;

    Ok((board, channel))
}

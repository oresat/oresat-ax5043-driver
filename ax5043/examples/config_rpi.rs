use std::io;
use ax5043::{
    config::{Framing, Modulation, SlowRamp, FEC, *},
    *,
    registers::*
};

fn config(radio: &mut Registers, antenna: config::Antenna) -> io::Result<(Board, ChannelParameters)> {
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

pub fn configure_radio_tx(radio: &mut Registers) -> io::Result<config::Board> {
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

struct RXParams {
    if_freq: u16,         // 16bit
    decimation: u8,       // 7bit
    rx_data_rate: u32,    // 24bit
    max_dr_offset: u32,   // 24bit
    max_rf_offset: u32,   // 20bit
    freq_offs_corr: bool, // flag

    fsk_dev_max: u16, // 16bit
    fsk_dev_min: i16, // 16bit

    afsk_space: u16, // 16bit
    afsk_mark: u16,  // 16bit
    afsk_shift: u8,  // 5bit

    ampl_filter: u8,    // 4bit
    frequency_leak: u8, // 4bit
}

impl Default for RXParams {
    fn default() -> Self {
        Self {
            if_freq: 0x1327,
            decimation: 0x0D,
            rx_data_rate: 0x3D8A,
            max_dr_offset: 0x9E,
            max_rf_offset: 0x1687,
            freq_offs_corr: false,

            fsk_dev_max: 0x80,
            fsk_dev_min: 0x80, // 0xFF80 but twos complement

            afsk_space: 0x40,
            afsk_mark: 0x75,
            afsk_shift: 0x4,

            ampl_filter: 0x0,
            frequency_leak: 0x0,
        }
    }
}

/*
fn param_set() {
    /*
    AGC Gain
    AGC Target
    AGC AHyst
    AGC MinMax
    Gain - Time
    Gain - Data Rate
    Gain - Phase
    Gain - Freq A
    Gain - Freq B
    Gain - Freq C
    Gain - Freq D
    Gain - Ampl
    Freq Deviaiton
    FourFSK
    BB Offset Resistor
    */
}
*/
/* Max RF Offset
 * FSK Dev Max
 * FSK Dev Min
 * AFSK Space
 * AFSK Mark
 * AFSK Ctrl
 * Ampl Filt
 * Freq Leak
 *
 * RX Param Set
 * RX Param Cur
 */


fn rx_set_params(radio: &mut Registers, _board: &config::Board, params: &RXParams) -> std::io::Result<()> {
    // General RX parameters
    //radio.IFFREQ.write((params.if_freq * 2) / board.xtal.freq * 2_i32.pow(20))?; // FIXME fxtaldiv instead of 2
    radio.IFFREQ.write(params.if_freq)?;
    radio.DECIMATION.write(params.decimation)?;
    radio.RXDATARATE.write(params.rx_data_rate)?;
    radio.MAXDROFFSET.write(params.max_dr_offset)?;
    radio.MAXRFOFFSET.write(MaxRFOffset {
        offset: params.max_rf_offset,
        correction: params.freq_offs_corr,
    })?;
    radio.FSKDMAX.write(params.fsk_dev_max)?;
    radio.FSKDMIN.write(params.fsk_dev_min)?;
    radio.AFSKSPACE.write(params.afsk_space)?;
    radio.AFSKMARK.write(params.afsk_mark)?;
    radio.AFSKCTRL.write(params.afsk_shift)?;
    radio.AMPLFILTER.write(params.ampl_filter)?;
    radio.FREQUENCYLEAK.write(params.frequency_leak)?;

    // RX Parameter set 0
    radio.AGCGAIN0.write(AGCGain {
        attack: 3,
        decay: 9,
    })?;
    radio.AGCTARGET0.write(0x89)?;
    radio.TIMEGAIN0.write(TimeGain {
        exponent: 9,
        mantissa: 8,
    })?;
    radio.DRGAIN0.write(DRGain {
        exponent: 3,
        mantissa: 8,
    })?;
    radio.FREQDEV0.write(0x0000)?;

    Ok(())

}

pub fn configure_radio_rx(radio: &mut Registers) -> io::Result<(Board, ChannelParameters)> {
    let (board, channel) = config(radio, config::Antenna::Differential)?;

    let params = RXParams {
        if_freq: 0x09A5,
        decimation: 0x03,
        rx_data_rate: 0x0042AA,
        max_dr_offset: 0x00,
        max_rf_offset: 0x800161,
        ..Default::default()
    };
    rx_set_params(radio, &board, &params)?;
    radio.RXPARAMSETS.write(RxParamSets(
        RxParamSet::Set0,
        RxParamSet::Set1,
        RxParamSet::Set2,
        RxParamSet::Set3,
    ))?;


    Ok((board, channel))
}


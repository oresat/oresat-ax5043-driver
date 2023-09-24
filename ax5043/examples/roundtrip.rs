extern crate ax5043;
use std::{
    cell::Cell,
    io,
    thread, time,
};
use ax5043::{config::Framing, config::Modulation, config::SlowRamp, config::*, *};

fn print_state(radio: &mut Registers, step: &str) -> io::Result<()> {
    println!("\nstep: {}", step);
    println!("IRQREQ     {:?}", radio.IRQREQUEST.read()?);
    println!("XTALST     {:?}", radio.XTALSTATUS.read()?);
    println!("PLLRANGING {:?}", radio.PLLRANGINGA.read()?); // sticky lock bit ~ IRQPLLUNLIOCK, gate
    println!("RADIOEVENT {:?}", radio.RADIOEVENTREQ.read()?);
    println!("POWSTAT    {:?}", radio.POWSTAT.read()?);
    println!("POWSTAT    {:?}", radio.POWSTICKYSTAT.read()?); // affects irq/spi status, gate
    println!("RADIOSTATE {:?}", radio.RADIOSTATE.read()?);
    println!("FIFO | count | free | thresh | stat");
    println!(
        "     | {:5} | {:4} | {:6} | {:?}",
        radio.FIFOCOUNT.read()?,
        radio.FIFOFREE.read()?,
        radio.FIFOTHRESH.read()?,
        radio.FIFOSTAT.read()?
    );
    Ok(())
}

fn ax5043_transmit(radio: &mut Registers, data: &[u8]) -> io::Result<()> {
    // DS Table 25
    // FULLTX:
    // Synthesizer and transmitter are running. Do not switch into this mode before the synthesiz-
    // er has completely settled on the transmit frequency (in SYNTHTX mode), otherwise spuri-
    // ous spectral transmissions will occur.

    // this conflicts with how the radio woks and the PM figure describes the process.

    // PM Figure 9
    // pwrmode = FULLTX
    // Write preamble to fifo
    // write data to fifo
    // check XTAL_RUN
    // commit fifo
    // wait until tx is done
    // pwrmode = POWERDOWN

    //print_state(radio, "pre-synthtx")?;
    // pll not locked
    //radio.PWRMODE.write(PwrMode { flags: PwrFlags::XOEN | PwrFlags::REFEN, mode: PwrModes::SYNTHTX })?;
    //thread::sleep(time::Duration::from_millis(10));

    //print_state(radio, "post-synthtx")?;
    // I'm getting Radio event frame clock - should I see pll settled here instead?
    //pll locked
    //while !radio
    //    .PLLRANGINGA
    //    .read()?
    //    .flags
    //    .contains(PLLRangingFlags::PLL_LOCK)
    //{} // TODO: IRQRQPLLUNLOCK?
    //print_state(radio, "pre-fulltx")?;
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;
    //print_state(radio, "fulltx")?;
    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio,
    };
    // FIXME: FIFOSTAT CLEAR?
    fifo.write(data, fifo::TXDataFlags::PKTSTART | fifo::TXDataFlags::PKTEND)?;
    fifo.commit()?;

    //print_state(radio, "commit")?;

    while radio.RADIOSTATE.read()? as u8 != RadioState::IDLE as u8 {} // TODO: Interrupt of some sort
    //print_state(radio, "tx complete")?;
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::XOEN,
    })?;
    //print_state(radio, "idle mode")?;

    Ok(())
}

// TODO: Synth state:
// - PLLVCOIR
// - PLLLOCKDET
// - PLLRANGING{A,B}

// TODO: Channel state:
// - MODULATION::REVRDONE
// - FRAMING::FRMRX
// TODO: Action
// - FRAMING::FABORT

fn configure_radio_tx(radio: &mut Registers) -> io::Result<()> {
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
        antenna: Antenna::SingleEnded,
        dac: DAC {
            pin: DACPin::PwrAmp,
        },
        adc: ADC::ADC1,
    };

    radio.IRQMASK.write(ax5043::IRQ::XTALREADY)?;

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
        vco_current: Some(0x13), // depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
        lock_detector_delay: None, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };

    configure_synth(radio, &board, &synth)?;

    let channel = ChannelParameters {
        modulation: Modulation::GFSK {
            deviation: 20_000,
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },

        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },

    };

    configure_channel(radio, &board, &channel)?;

    let parameters = TXParameters {
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        txrate: 60_000,
        plllock_gate: true,
        brownout_gate: true,
    };

    configure_tx(radio, &board, &channel, &parameters)?;
    // TMGTX{BOOST,SETTLE} in Packet controller

    autorange(radio)?;
    Ok(())
}

fn tx_main() -> io::Result<()> {
    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            //println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio_tx = ax5043::registers(&spi0, &callback);

    radio_tx.reset()?;

    configure_radio_tx(&mut radio_tx)?;

    loop {
        // Example transmission from PM table 16
        ax5043_transmit(&mut radio_tx, &[0xAA, 0xAA, 0x1A])?;
        thread::sleep(time::Duration::from_millis(500));
    }

    //radio_tx.reset()?;
    //Ok(())
}

struct RXParams {
    if_freq: u16,         // 16bit
    decimation: u8,       // 7bit
    rx_data_rate: u32,    // 24bit
    max_dr_offset: u32,   // 24bit
    max_rf_offset: u32,   // 20bit
    freq_offs_corr: bool, // flag

    fsk_dev_max: u16, // 16bit
    fsk_dev_min: u16, // 16bit

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
            fsk_dev_min: 0xFF80,

            afsk_space: 0x40,
            afsk_mark: 0x75,
            afsk_shift: 0x4,

            ampl_filter: 0x0,
            frequency_leak: 0x0,
        }
    }
}

fn rx_set_params(radio: &mut Registers, params: &RXParams) -> io::Result<()> {
    // General RX parameters
    radio.IFFREQ.write(params.if_freq)?; // = if_freq * f_xtal_div/ (f_xtal) * 2_u64.pow(20) + 1/2
    radio.DECIMATION.write(params.decimation)?; // TODO value 0 is illegal = f_xtal/(f_baseband * 2_u64.pow(4) * f_xtal_div)
    radio.RXDATARATE.write(params.rx_data_rate)?; // RXDATARATE - TIMEGAINx ≥ 2 12 should be ensured when programming. = 2_u64.pow(7) * f_xtal / (f_xtal_div * bitrate * decimation)
    radio.MAXDROFFSET.write(params.max_dr_offset)?; // larger -> longer preamble, If the bitrate offset is less than approximately ±1%, receiver bitrate tracking should be
                                                    //switched off completely by setting MAXDROFFSET to zero, to ensure minimum preamble length.
                                                    // = 2_u64.pow(7) * f_xtal * dBITRATE / (f_xtal_div * BITRATE^2 * DECIMATION)
    radio.MAXRFOFFSET.write(MaxRFOffset {
        offset: params.max_rf_offset, // max 1/4 fitler bandwidth, = df_carrier / f_xtal * 2_u64.pow(24)
        freq_offset_correction: params.freq_offs_corr,
    })?;
    radio.FSKDMAX.write(params.fsk_dev_max)?; // DEVUPDATE (in fourfsk*) selects automatic mode, recommended
    radio.FSKDMIN.write(params.fsk_dev_min)?; // DEVUPDATE
    radio.AFSKSPACE.write(params.afsk_space)?; // only AFSK mode
    radio.AFSKMARK.write(params.afsk_mark)?;   // only AFSK mode
    radio.AFSKCTRL.write(params.afsk_shift)?;  // only AFSK mode
    radio.AMPLFILTER.write(params.ampl_filter)?; // 0 bypass, very complicated expr for exact
    radio.FREQUENCYLEAK.write(params.frequency_leak)?; // 0 off, "leakiness of the baseband freq recovery loop". no formula given

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

pub fn ax5043_listen(radio: &mut Registers) -> io::Result<()> {

    // pll not locked
    print_state(radio, "pre-synthrx")?;
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::SYNTHRX,
    })?;
    print_state(radio, "post-synthrx")?;
    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio,
    };
    fifo.reset()?;
    print_state(radio, "post-fifo-clear")?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    Ok(())
}

fn print_signal(radio: &mut Registers) -> io::Result<()> {
/* TODO: tracking Register Resets
    Writes to TRKAMPL, TRKPHASE, TRKDATARATE
    cause the following action:
    DTRKRESET 3 W − Writing 1 clears the Datarate Tracking Register
    ATRKRESET 4 W − Writing 1 clears the Amplitude Tracking Register
    PTRKRESET 5 W − Writing 1 clears the Phase Tracking Register
    RTRKRESET 6 W − Writing 1 clears the RF Frequency Tracking Register
    FTRKRESET 7 W − Writing 1 clears the Frequency Tracking Register
*/
/* TODO: TRKFREQ, TRKRFFREQ should be frozen before attempting to write to.
    To freeze, set the RFFREQFREEZE bit in the appropriate
    FREQGAIND0, FREQGAIND1,FREQGAIND2, or FREQGAIND3 register, then wait for
    1/(4 * BAUDRATE) for the freeze to take effect.
*/
// TODO: what happens when TRKFREQ/TRKRFFREQ are written to?
    println!(
        "RSSI:{}dB BGNDRSSI:{}dB AGCCOUNTER:{}dB",
        radio.RSSI.read()?,
        radio.BGNDRSSI.read()?,
        (radio.AGCCOUNTER.read()? as u32 * 4) / 3
    );

    let rate = radio.TRKDATARATE.read()? as u64;
    println!(
        "RATE:{}Hz AMPL:{}dB PHASE:{}",
        rate,
        radio.TRKAMPL.read()?,
        radio.TRKPHASE.read()?
    );
    println!(
        "RFFREQ:{}Hz FREQ:{}Hz DEMOD:{}",
        radio.TRKRFFREQ.read()?,
        (radio.TRKFREQ.read()? as u64 * rate) / 2_u64.pow(16),
        radio.TRKFSKDEMOD.read()?
    );
    Ok(())
}

pub fn ax5043_receive(radio: &mut Registers) -> io::Result<()> {
    loop {
        print_signal(radio)?;
        print_state(radio, "pre-receive")?;
        if !radio.FIFOSTAT.read()?.contains(FIFOStat::EMPTY) {
            break;
        }
        print_signal(radio)?;
        thread::sleep(time::Duration::from_millis(250));
    }
    print_state(radio, "receive")?;

    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio: radio,
    };
    fifo.read()?;
    // TODO: figure out if we need to read more or if it's multiple
    Ok(())
}

fn configure_radio_rx(radio: &mut Registers) -> io::Result<()> {
    let board = config::Board {
        sysclk: Pin { mode: config::SysClk::Z,    pullup: true,  invert: false, },
        dclk:   Pin { mode: config::DClk::Z,      pullup: true,  invert: false, },
        data:   Pin { mode: config::Data::Z,      pullup: true,  invert: false, },
        pwramp: Pin { mode: config::PwrAmp::TCXO, pullup: false, invert: false, },
        irq:    Pin { mode: config::IRQ::IRQ,     pullup: false, invert: false, },
        antsel: Pin { mode: config::AntSel::Z,    pullup: true,  invert: false, },

        xtal: config::Xtal {
            kind: config::XtalKind::TCXO,
            freq: 48_000_000,
            enable: config::XtalPin::AntSel,
        },
        vco: config::VCO::Internal,
        filter: config::Filter::Internal,
        antenna: config::Antenna::Differential,
        dac: DAC {
            pin: config::DACPin::PwrAmp,
        },
        adc: config::ADC::ADC1,
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
        vco_current: Some(0x13), // depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
        lock_detector_delay: None, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };

    configure_synth(radio, &board, &synth)?;

    let channel = ChannelParameters {
        modulation: Modulation::GFSK {
            deviation: 20_000,
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },
    };

    configure_channel(radio, &board, &channel)?;

    let params = RXParams {
        if_freq: 0x09A5,
        decimation: 0x03,
        rx_data_rate: 0x0042AA,
        max_dr_offset: 0x00,
        max_rf_offset: 0x800161,
        ..Default::default()
    };
    rx_set_params(radio, &params)?;
    radio.RXPARAMSETS.write(0xF4)?;


    autorange(radio)?;
    Ok(())
}

fn main() -> io::Result<()> {
    thread::spawn(tx_main);

    let spi1 = ax5043::open("/dev/spidev1.0")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("RX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio_rx = ax5043::registers(&spi1, &callback);

    radio_rx.reset()?;

    configure_radio_rx(&mut radio_rx)?;
    loop {
        ax5043_listen(&mut radio_rx)?;
        ax5043_receive(&mut radio_rx)?;
    }
    //radio_rx.reset()?;
    //Ok(())
}

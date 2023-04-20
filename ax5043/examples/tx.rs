extern crate ax5043;
use std::{
    cell::Cell,
    io,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread, time,
};
//use gpiod::{Chip, Options, Masked, AsValuesMut};
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
    while !radio
        .PLLRANGINGA
        .read()?
        .flags
        .contains(PLLRangingFlags::PLL_LOCK)
    {} // TODO: IRQRQPLLUNLOCK?
    print_state(radio, "pre-fulltx")?;
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;
    print_state(radio, "fulltx")?;
    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio,
    };
    // FIXME: FIFOSTAT CLEAR?
    fifo.write(data, fifo::TXDataFlags::UNENC | fifo::TXDataFlags::RESIDUE)?;
    fifo.commit()?;

    print_state(radio, "commit")?;

    while radio.RADIOSTATE.read()? as u8 != RadioState::IDLE as u8 {} // TODO: Interrupt of some sort
    print_state(radio, "tx complete")?;
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::XOEN,
    })?;
    print_state(radio, "idle mode")?;

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

fn configure_radio(radio: &mut Registers) -> io::Result<()> {
    let board = Board {
        sysclk: Pin {
            mode: config::SysClk::Z,
            pullup: false,
            invert: false,
        },
        dclk: Pin {
            mode: config::DClk::Z,
            pullup: false,
            invert: false,
        },
        data: Pin {
            mode: config::Data::Z,
            pullup: false,
            invert: false,
        },
        pwramp: Pin {
            mode: config::PwrAmp::TCXO,
            pullup: false,
            invert: false,
        },
        irq: Pin {
            mode: config::IRQ::IRQ,
            pullup: false,
            invert: false,
        },
        antsel: Pin {
            mode: config::AntSel::Z,
            pullup: false,
            invert: false,
        },
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

    let parameters = TXParameters {
        modulation: Modulation::GFSK {
            deviation: 20_000,
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },
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
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },
    };

    configure_tx(radio, &board, &parameters)?;
    // Transmit parameters
    // TMGTX{BOOST,SETTLE} in Packet controller
    // Packet format?
    // Modulation, Encoding, Framing, CRC, FEC

    autorange(radio)?;
    Ok(())
}

fn main() -> io::Result<()> {
    //    thread::spawn(|| {
    //
    //
    //    });

    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio_tx = ax5043::registers(&spi0, &callback);

    radio_tx.reset()?;

    let stop = Arc::new(AtomicBool::new(false));
    let handlerstop = stop.clone();
    ctrlc::set_handler(move || {
        handlerstop.store(true, Ordering::SeqCst);
        println!("received Ctrl+C!");
    })
    .expect("Error setting Ctrl-C handler");

    configure_radio(&mut radio_tx)?;

    loop {
        // Example transmission from PM table 16
        ax5043_transmit(&mut radio_tx, &[0xAA, 0xAA, 0x1A])?;
        thread::sleep(time::Duration::from_millis(500));
        if stop.load(Ordering::SeqCst) {
            break;
        }
    }

    radio_tx.reset()?;
    Ok(())
}

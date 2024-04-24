extern crate ax5043;
use anyhow::Result;
use ax5043::{registers::*, *};
use bitflags::Flags;
use gpiod::{Chip, EdgeDetect, Options};
use mio::{unix::SourceFd, Events, Interest, Poll, Token, Waker};
use mio_signals::{Signal, Signals};
use std::{
    fmt::{Debug, Display},
    os::fd::AsRawFd,
    sync::Arc,
    thread,
    time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

use ax5043::config::rpi::configure_radio_tx;

fn print_diff<S: AsRef<str> + Display, T: Flags + PartialEq + Debug + Copy>(
    name: S,
    new: T,
    old: T,
) -> T {
    if new != old {
        print!("{:14}: ", name);
        let added = new.difference(old);
        if !added.is_empty() {
            print!("+{:?} ", added);
        }
        let removed = old.difference(new);
        if !removed.is_empty() {
            print!("-{:?} ", removed);
        }
        if !old.intersection(new).is_empty() {
            print!("={:?}", old.intersection(new));
        }
        println!();
    }
    new
}

struct AXState {
    irq: IRQ,
    xtal: XtalStatus,
    //pllranging: PLLRanging,
    radioevent: RadioEvent,
    radiostate: RadioState,
    powstat: PowStat,
    powsstat: PowStat,
}

#[rustfmt::skip]
fn print_state(radio: &mut Registers, step: &str, state: &mut AXState) -> Result<()> {
    println!("\nstep: {}", step);
    state.irq        = print_diff("IRQREQ    ", radio.IRQREQUEST().read()?, state.irq);
    state.xtal       = print_diff("XTALST    ", radio.XTALSTATUS().read()?, state.xtal);

    println!("{:14}: {:?}", "PLLRANGING", radio.PLLRANGINGA().read()?); // sticky lock bit ~ IRQPLLUNLIOCK, gate
    //state.pllranging = print_diff("PLLRANGING", radio.PLLRANGINGA.read()?, state.pllranging); // sticky lock bit ~ IRQPLLUNLIOCK, gate
    state.radioevent = print_diff("RADIOEVENT", radio.RADIOEVENTREQ().read()?, state.radioevent);
    state.powstat    = print_diff("POWSTAT   ", radio.POWSTAT().read()?, state.powstat);
    state.powsstat   = print_diff("POWSSTAT  ", radio.POWSTICKYSTAT().read()?, state.powsstat); // affects irq/spi status, gate
    let new = radio.RADIOSTATE().read()?;
    if state.radiostate != new {
        println!("{:14}: {:?}", "RADIOSTATE", new);
        state.radiostate = new;
    }
    println!("FIFO | count | free | thresh | stat");
    println!(
        "     | {:5} | {:4} | {:6} | {:?}",
        radio.FIFOCOUNT().read()?,
        radio.FIFOFREE().read()?,
        radio.FIFOTHRESH().read()?,
        radio.FIFOSTAT().read()?
    );
    Ok(())
}

fn ax5043_transmit(radio: &mut Registers, data: &[u8], state: &mut AXState) -> Result<()> {
    // DS Table 25
    // FULLTX:
    // Synthesizer and transmitter are running. Do not switch into this mode before the synthesiz-
    // er has completely settled on the transmit frequency (in SYNTHTX mode), otherwise spuri-
    // ous spectral transmissions will occur.

    // this conflicts with how the radio works and the PM figure describes the process.

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
    //{}

    // IMPORTANT: PLL_STICKY_LOCK needs to be set to coutner PLLLOCK_GATE
    //            Sticky SSBEVMODEM and SSBEVANA needs to be clear to counter BROWNOUT_GATE

    print_state(radio, "pre-fulltx", state)?;
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;
    print_state(radio, "fulltx", state)?;
    // FIXME: FIFOSTAT CLEAR?
    //fifo.write(data, fifo::TXDataFlags::UNENC | fifo::TXDataFlags::RESIDUE)?;
    // Preamble - see PM p16

    let preamble = FIFOChunkTX::REPEATDATA {
        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
        count: 1,
        data: 0x11,
    };
    let packet = FIFOChunkTX::DATA {
        flags: FIFODataTXFlags::PKTSTART | FIFODataTXFlags::PKTEND,
        data: data.to_vec(),
    };
    radio.FIFODATATX().write(preamble)?;
    radio.FIFODATATX().write(packet)?;

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::COMMIT,
        auto_commit: false,
    })?;

    print_state(radio, "commit", state)?;

    while radio.RADIOSTATE().read()? as u8 != RadioState::IDLE as u8 {} // TODO: Interrupt of some sort
    print_state(radio, "tx complete", state)?;
    /*
        radio.PWRMODE.write(PwrMode {
            flags: PwrFlags::XOEN | PwrFlags::REFEN,
            mode: PwrModes::XOEN,
        })?;
        print_state(radio, "idle mode", state)?;
    */
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

#[derive(Debug)]
struct IRQState {
    irq: IRQ,
}

fn service_irq(radio: &mut Registers, state: &mut IRQState) -> Result<()> {
    state.irq = print_diff("GPIO IRQ: ", radio.IRQREQUEST().read()?, state.irq);
    Ok(())
}

fn main() -> Result<()> {
    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, new, _: &_| {
        status = print_diff("Status change", new, status);
    };
    let mut radio_tx = Registers::new(spi0, &mut callback);

    radio_tx.reset()?;

    const IRQ: Token = Token(1);
    let irq_waker = Arc::new(Waker::new(poll.registry(), IRQ)?);
    let irq_thread_waker = irq_waker.clone();

    thread::spawn(move || -> Result<()> {
        let chip = Chip::new("gpiochip0")?;
        let opts = Options::input([17])
            .edge(EdgeDetect::Both)
            .consumer("ax5043");
        let mut inputs = chip.request_lines(opts)?;
        loop {
            let _event = inputs.read_event()?;
            irq_thread_waker.wake().unwrap();
        }
    });

    configure_radio_tx(&mut radio_tx)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(100),
        },
        SetTimeFlags::Default,
    );
    const BEACON: Token = Token(2);
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), BEACON, Interest::READABLE)?;

    let mut irqstate = IRQState { irq: IRQ::empty() };
    let mut state = AXState {
        irq: IRQ::empty(),
        xtal: XtalStatus::empty(),
        radioevent: RadioEvent::empty(),
        radiostate: RadioState::IDLE,
        powstat: PowStat::empty(),
        powsstat: PowStat::empty(),
    };

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();
                    // Example transmission from PM table 16
                    ax5043_transmit(&mut radio_tx, &[0xAA, 0xAA, 0x1A], &mut state)?;
                }
                IRQ => service_irq(&mut radio_tx, &mut irqstate)?,
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio_tx.reset()?;
    Ok(())
}

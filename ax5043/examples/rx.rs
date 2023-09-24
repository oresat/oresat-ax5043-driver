extern crate ax5043;
use std::{
    cell::Cell,
    io,
    time::Duration,
    os::fd::AsRawFd,
};
use timerfd::{TimerFd, TimerState, SetTimeFlags};
use mio::{Events, Poll, unix::SourceFd, Token, Interest};
use mio_signals::{Signal, Signals};

use ax5043::*;
mod config_rpi;
use crate::config_rpi::configure_radio_rx;


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
    println!(
        "RSSI:{}dB BGNDRSSI:{}dB AGCCOUNTER:{}dB",
        radio.RSSI.read()?,
        radio.BGNDRSSI.read()?,
        (radio.AGCCOUNTER.read()? as u32 * 4) / 3
    );
    println!(
        "RATE:{} AMPL:{} PHASE:{}",
        radio.TRKDATARATE.read()?,
        radio.TRKAMPL.read()?,
        radio.TRKPHASE.read()?
    );

    let mut demod: i32 = radio.TRKFSKDEMOD.read()?.into();
    if demod > 2_i32.pow(13) {
        demod = demod - 2_i32.pow(14)
    }
    println!(
        "RFFREQ:{} FREQ:{} DEMOD:{}",
        radio.TRKRFFREQ.read()?,
        radio.TRKFREQ.read()?,
        demod
    );
    println!("{:?}", radio.RXPARAMCURSET.read()?);
    Ok(())
}

pub fn ax5043_receive(radio: &mut Registers) -> io::Result<()> {
    print_signal(radio)?;
    //print_state(radio, "pre-receive")?;

    //let timeout = time::Instant::now() + time::Duration::new(1, 0);
    //while radio.FIFOSTAT.read()?.contains(FIFOStat::EMPTY) || time::Instant::now() <= timeout {}
    print_signal(radio)?;

    //print_state(radio, "receive")?;

    //let mut fifo = fifo::FIFO {
    //    threshold: 0,
    //    autocommit: false,
    //    radio: radio,
    //};
    //fifo.read()?;
    // TODO: figure out if we need to read more or if it's multiple
    Ok(())
}


fn main() -> io::Result<()> {
    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let spi1 = ax5043::open("/dev/spidev1.0")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio_rx = ax5043::registers(&spi1, &callback);

    radio_rx.reset()?;

    configure_radio_rx(&mut radio_rx)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(TimerState::Periodic{current: Duration::from_millis(100), interval: Duration::from_millis(100)}, SetTimeFlags::Default);
    const BEACON: Token = Token(2);
    registry.register(
        &mut SourceFd(&tfd.as_raw_fd()),
        BEACON,
        Interest::READABLE)?;

    //let mut irqstate = IRQState::default();
    //let mut state = AXState::default();


    ax5043_listen(&mut radio_rx)?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();
                    ax5043_receive(&mut radio_rx)?;
                },
                //IRQ => service_irq(&mut radio_tx, &mut irqstate)?,
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }
    radio_rx.reset()?;
    Ok(())
}

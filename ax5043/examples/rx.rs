extern crate ax5043;
use anyhow::{anyhow, Result};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{os::fd::AsRawFd, time::Duration};
use timerfd::{SetTimeFlags, TimerFd, TimerState};
use gpiod::{Chip, EdgeDetect, Options};

use ax5043::config_rpi::configure_radio_rx;
use ax5043::registers::*;
use ax5043::*;

fn print_state(radio: &mut Registers, step: &str) -> Result<()> {
    println!("\nstep: {}", step);
    println!("IRQREQ     {:?}", radio.IRQREQUEST().read()?);
    println!("XTALST     {:?}", radio.XTALSTATUS().read()?);
    println!("PLLRANGING {:?}", radio.PLLRANGINGA().read()?); // sticky lock bit ~ IRQPLLUNLIOCK, gate
    println!("RADIOEVENT {:?}", radio.RADIOEVENTREQ().read()?);
    println!("POWSTAT    {:?}", radio.POWSTAT().read()?);
    println!("POWSTAT    {:?}", radio.POWSTICKYSTAT().read()?); // affects irq/spi status, gate
    println!("RADIOSTATE {:?}", radio.RADIOSTATE().read()?);
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

pub fn ax5043_listen(radio: &mut Registers) -> Result<()> {
    // pll not locked
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::SYNTHRX,
    })?;

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    Ok(())
}

fn print_signal(radio: &mut Registers) -> Result<()> {
    println!(
        "RSSI:{}dB BGNDRSSI:{}dB AGCCOUNTER:{}dB",
        radio.RSSI().read()?,
        radio.BGNDRSSI().read()?,
        (i32::from(radio.AGCCOUNTER().read()?) * 4) / 3
    );
    println!(
        "RATE:{} AMPL:{} PHASE:{}",
        radio.TRKDATARATE().read()?,
        radio.TRKAMPL().read()?,
        radio.TRKPHASE().read()?
    );
    println!(
        "RFFREQ:{:?} FREQ:{:?} DEMOD:{:?}",
        radio.TRKRFFREQ().read()?,
        radio.TRKFREQ().read()?,
        radio.TRKFSKDEMOD().read()?.0
    );
    println!("{:?}", radio.RXPARAMCURSET().read()?);
    Ok(())
}

pub fn ax5043_receive(radio: &mut Registers) -> Result<()> {
    //print_signal(radio)?;
    let stat = radio.FIFOSTAT().read()?;
    println!("{:?}", stat);
    if stat.contains(FIFOStat::EMPTY) {
        return Err(anyhow!("Empty IRQ"));
    }

    let len = radio.FIFOCOUNT().read()?;
    if len > 0 {
        for packet in radio.FIFODATARX().read(len.into())? {
            match packet {
                FIFOChunkRX::DATA{flags, ref data} => {
                    println!("{}, {} {:02X?}", len, data.len(), packet)
                }
                _ => println!("{:02X?}", packet)
            }
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let chip = Chip::new("gpiochip0")?;
    let opts = Options::input([16])
        .edge(EdgeDetect::Rising);
    let mut inputs = chip.request_lines(opts)?;


    let spi1 = ax5043::open("/dev/spidev1.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, s, _: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
    };
    let mut radio_rx = Registers::new(spi1, &mut callback);

    radio_rx.reset()?;

    configure_radio_rx(&mut radio_rx)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::from_millis(100),
            interval: Duration::from_millis(100),
        },
        SetTimeFlags::Default,
    );
    const BEACON: Token = Token(2);
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), BEACON, Interest::READABLE)?;

    const IRQ: Token = Token(3);
    registry.register(&mut SourceFd(&inputs.as_raw_fd()), IRQ, Interest::READABLE)?;

    radio_rx.IRQMASK().write(IRQ::FIFONOTEMPTY)?;
    println!("{:?}", radio_rx.IRQMASK().read()?);

    ax5043_listen(&mut radio_rx)?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => { tfd.read(); },
                IRQ => {
                    println!("IRQ!: {:?}", inputs.read_event()?);
                    ax5043_receive(&mut radio_rx)?;
                },
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }
    radio_rx.reset()?;
    Ok(())
}

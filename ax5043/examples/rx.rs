extern crate ax5043;
use anyhow::{anyhow, Result};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{io::Write, os::fd::AsRawFd, time::Duration};
use std::net::{IpAddr, Ipv6Addr, SocketAddr, UdpSocket};
use timerfd::{SetTimeFlags, TimerFd, TimerState};
use gpiod::{Chip, EdgeDetect, Options};

use ax5043::config_rpi::configure_radio_rx;
use ax5043::registers::*;
use ax5043::*;

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

pub fn ax5043_receive(radio: &mut Registers, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    let stat = radio.FIFOSTAT().read()?;
    println!("{:?}", stat);
    if stat.contains(FIFOStat::EMPTY) {
        return Err(anyhow!("Empty IRQ"));
    }

    let len = radio.FIFOCOUNT().read()?;
    if len <= 0 {
        return Ok(())
    }

    for chunk in radio.FIFODATARX().read(len.into())? {
        match chunk {
            FIFOChunkRX::DATA{flags, ref data} => {
                if flags.intersects(FIFODataRXFlags::ABORT | FIFODataRXFlags::SIZEFAIL | FIFODataRXFlags::ADDRFAIL | FIFODataRXFlags::CRCFAIL | FIFODataRXFlags::RESIDUE) {
                    packet.clear();
                    continue;
                }
                if flags.contains(FIFODataRXFlags::PKTSTART) {
                    packet.clear();
                    packet.write(data)?;
                }
                if flags.is_empty() {
                    packet.write(data)?;
                }
                if flags.contains(FIFODataRXFlags::PKTEND) {
                    packet.write(data)?;
                    uplink.send(&packet[..packet.len()-2])?;
                    println!("{:02X?}", packet);
                    // indicate end
                }
            }
            _ => println!("{:02X?}", packet)
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

    let src = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 0);
    //let dest = SocketAddr::new(IpAddr::V6(Ipv6Addr::LOCALHOST), 10025);
    let mut uplink = UdpSocket::bind(src)?;
    uplink.connect("169.254.209.33:10025")?;


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

    let mut packet = Vec::new();
    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => { tfd.read(); },
                IRQ => {
                    println!("IRQ!: {:?}", inputs.read_event()?);
                    ax5043_receive(&mut radio_rx, &mut packet, &mut uplink)?;
                },
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }
    radio_rx.reset()?;
    Ok(())
}

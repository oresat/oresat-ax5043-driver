use anyhow::{anyhow, Result};
use ax5043::registers::*;
use ax5043::tui::*;
use ax5043::*;
use gpiod::{Chip, EdgeDetect, Options};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::{fs::read_to_string, os::fd::AsRawFd, time::Duration};
use timerfd::{SetTimeFlags, TimerFd, TimerState};
use toml;

pub fn ax5043_listen(radio: &mut Registers) -> Result<()> {
    // pll not locked
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;
    /*
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    */
    Ok(())
}

// TODO: FRAMING::FRMRX

fn read_packet(radio: &mut Registers, uplink: &UdpSocket) -> Result<()> {
    let stat = radio.FIFOSTAT().read()?;
    if stat.contains(FIFOStat::OVER) {
        radio.FIFOCMD().write(FIFOCmd {
            mode: FIFOCmds::CLEAR_DATA,
            auto_commit: false,
        })?;
        return Err(anyhow!("Overflow: {:?}", stat));
    }

    if !stat.contains(FIFOStat::EMPTY) {
        let len = radio.FIFOCOUNT().read()?;
        for data in radio.FIFODATARX().read(len.into())? {
            CommState::RX(data).send(uplink)?;
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 244)), 10035);
    let uplink = UdpSocket::bind(src)?;
    uplink.connect(dest)?;

    let spi0 = ax5043::open("/dev/spidev1.0")?;

    let mut status = Status::empty();
    let mut callback = |_: &_, _, new, _: &_| {
        if new == status {
            return;
        }
        status = new;
        CommState::STATUS(new).send(&uplink).unwrap();
    };
    let mut radio = ax5043::Registers::new(spi0, &mut callback);

    radio.reset()?;
    CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;

    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(50),
        },
        SetTimeFlags::Default,
    );
    const TELEMETRY: Token = Token(2);
    registry.register(
        &mut SourceFd(&tfd.as_raw_fd()),
        TELEMETRY,
        Interest::READABLE,
    )?;

    let file_path = "rpi-uhf-60000.toml";
    let contents = read_to_string(file_path)?;
    let config: config::Config = toml::from_str(&contents)?;
    config.write(&mut radio)?;
    radio.RADIOEVENTMASK().write(RadioEvent::all())?;

    CommState::BOARD(config.board.clone()).send(&uplink)?;

    CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;
    ax5043_listen(&mut radio)?;
    CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;

    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD

    //radio.PKTCHUNKSIZE.write(0b1011)?;
    //radio.PKTMISCFLAGS.write(PktMiscFlags::BGND_RSSI)?;
    //radio.PKTACCEPTFLAGS.write(
    //      PktAcceptFlags::RESIDUE
    //    | PktAcceptFlags::ABRT
    //    | PktAcceptFlags::CRCF
    //    | PktAcceptFlags::ADDRF
    //    | PktAcceptFlags::SZF
    //    | PktAcceptFlags::LRGP
    //)?;
    CommState::CONFIG(Config {
        rxparams: RXParams::new(&mut radio, &config.board)?,
        set0: RXParameterSet::set0(&mut radio)?,
        set1: RXParameterSet::set1(&mut radio)?,
        set2: RXParameterSet::set2(&mut radio)?,
        set3: RXParameterSet::set3(&mut radio)?,
        synthesizer: Synthesizer::new(&mut radio, &config.board)?,
        packet_controller: PacketController::new(&mut radio)?,
        packet_format: PacketFormat::new(&mut radio)?,
    })
    .send(&uplink)?;

    let chip = Chip::new("gpiochip0")?;
    let opts = Options::input([16]).edge(EdgeDetect::Rising);
    let mut irq = chip.request_lines(opts)?;

    const IRQ: Token = Token(3);
    registry.register(&mut SourceFd(&irq.as_raw_fd()), IRQ, Interest::READABLE)?;

    radio.IRQMASK().write(IRQ::FIFONOTEMPTY)?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                TELEMETRY => {
                    tfd.read();
                    CommState::STATE(RXState::new(&mut radio, &config.channel[0])?)
                        .send(&uplink)?;
                    CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;
                }
                IRQ => {
                    irq.read_event()?;
                    while irq.get_values(0u8)? > 0 {
                        let r = read_packet(&mut radio, &uplink);
                        if r.is_err() {
                            println!("{:?}", r);
                            //uplink.send(CommState::ERR(r))?;
                        }
                    }
                    //radio.FIFOCMD().write(FIFOCmd {
                    //    mode: FIFOCmds::CLEAR_DATA,
                    //    auto_commit: false,
                    //})?;
                }
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;

    Ok(())
}

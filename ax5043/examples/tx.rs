extern crate ax5043;
use anyhow::Result;
use ax5043::{registers::*, tui::*, *};
use gpiocdev::{line::EdgeDetection, Request};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    fs::read_to_string,
    net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket},
    os::fd::AsRawFd,
    time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

fn transmit(radio: &mut Registers, uplink: &UdpSocket) -> Result<()> {
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

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;

    radio.FIFOTHRESH().write(128)?;
    // Preamble - see PM p16
    let preamble = FIFOChunkTX::REPEATDATA {
        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
        count: 44,
        data: 0x7E,
    };
    let packets = [
        FIFOChunkTX::REPEATDATA {
            flags: FIFODataTXFlags::PKTSTART,
            count: 128,
            data: 0x41,
        },
        FIFOChunkTX::REPEATDATA {
            flags: FIFODataTXFlags::empty(),
            count: 128,
            data: 0x43,
        },
        FIFOChunkTX::REPEATDATA {
            flags: FIFODataTXFlags::empty(),
            count: 128,
            data: 0x45,
        },
        FIFOChunkTX::REPEATDATA {
            flags: FIFODataTXFlags::PKTEND,
            count: 128,
            data: 0x47,
        },
    ];

    radio.FIFODATATX().write(preamble)?;

    for packet in packets {
        radio.FIFODATATX().write(packet)?;
    }
    // FIXME: for packet data that isn't repeat data we'll need a more complicated
    // solution of feeding the fifo
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::COMMIT,
        auto_commit: false,
    })?;
    CommState::REGISTERS(StatusRegisters::new(radio)?).send(uplink)?;
    while !radio.FIFOSTAT().read()?.contains(FIFOStat::FREE_THR) {}
    // FIXME: FIFOSTAT CLEAR?
    CommState::REGISTERS(StatusRegisters::new(radio)?).send(uplink)?;
    Ok(())
}

fn main() -> Result<()> {
    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    const BEACON: Token = Token(2);
    let mut tfd = TimerFd::new()?;
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(1000),
        },
        SetTimeFlags::Default,
    );
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), BEACON, Interest::READABLE)?;

    let irq = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(17)
        .with_edge_detection(EdgeDetection::RisingEdge)
        .request()?;

    const IRQ: Token = Token(3);
    registry.register(&mut SourceFd(&irq.as_raw_fd()), IRQ, Interest::READABLE)?;

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 247)), 10036);
    //let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 42, 0, 1)), 10036);
    let uplink = UdpSocket::bind(src)?;
    uplink.connect(dest)?;

    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, new, _: &_| {
        if new == status {
            return;
        }
        status = new;
        CommState::STATUS(new).send(&uplink).unwrap();
    };
    let mut radio = Registers::new(spi0, &mut callback);
    radio.reset()?;

    let file_path = "rpi-uhf-60000.toml";
    let contents = read_to_string(file_path)?;
    let config: config::Config = toml::from_str(&contents)?;
    config.write(&mut radio)?;
    radio.RADIOEVENTMASK().write(RadioEvent::DONE)?;

    CommState::BOARD(config.board.clone()).send(&uplink)?;
    CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;
    CommState::CONFIG(Config {
        txparams: TXParameters::new(&mut radio, &config.board)?,
        rxparams: RXParams::new(&mut radio, &config.board)?,
        set0: RXParameterSet::set0(&mut radio)?,
        set1: RXParameterSet::set1(&mut radio)?,
        set2: RXParameterSet::set2(&mut radio)?,
        set3: RXParameterSet::set3(&mut radio)?,
        synthesizer: Synthesizer::new(&mut radio, &config.board)?,
        packet_controller: PacketController::new(&mut radio)?,
        packet_format: PacketFormat::new(&mut radio)?,
        channel: ChannelParameters::new(&mut radio)?,
    })
    .send(&uplink)?;


    radio.IRQMASK().write(IRQ::RADIOCTRL)?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();
                    transmit(&mut radio, &uplink)?;
                }
                IRQ => {
                    while irq.has_edge_event()? {
                        irq.read_edge_event()?;
                        let status = StatusRegisters::new(&mut radio)?;
                        if status.radio_event.contains(RadioEvent::DONE) {
                            continue;
                        }
                        CommState::REGISTERS(status).send(&uplink)?;

                        _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                        _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD
                        radio.PWRMODE().write(PwrMode {
                            flags: PwrFlags::XOEN | PwrFlags::REFEN,
                            mode: PwrModes::POWEROFF,
                        })?;

                        CommState::REGISTERS(StatusRegisters::new(&mut radio)?).send(&uplink)?;
                    }
                }
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;
    Ok(())
}

// Intended to be run on the C3v6, takes data from UDP port 10015
// and transmits it through the UHF AX5043
use anyhow::{bail, ensure, Context, Result};
use ax5043::{config, registers, registers::*, tui, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiocdev::{
    line::{EdgeDetection, Value},
    Request,
};
use mio::net::UdpSocket;
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    fs::read_to_string,
    io::{ErrorKind, Write},
    net::{IpAddr, Ipv4Addr, SocketAddr},
    os::fd::AsRawFd,
    time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

fn process_chunk(chunk: FIFOChunkRX, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    if let FIFOChunkRX::DATA { flags, ref data } = chunk {
        //println!("{:02X?}", chunk);
        if flags.intersects(
            FIFODataRXFlags::ABORT
                | FIFODataRXFlags::SIZEFAIL
                | FIFODataRXFlags::ADDRFAIL
                | FIFODataRXFlags::CRCFAIL
                | FIFODataRXFlags::RESIDUE,
        ) {
            println!(
                "UHF REJECTED {:?} {:02X?} ...+{}",
                flags,
                data[0],
                data.len()
            );
            packet.clear();
            return Ok(());
        }

        if flags.contains(FIFODataRXFlags::PKTSTART) {
            if !packet.is_empty() {
                println!(
                    "UHF PKT RESTART rejecting {:02X?} ...+{}",
                    data[0],
                    data.len()
                );
            }
            packet.clear();
        }

        if !flags.contains(FIFODataRXFlags::PKTSTART) && packet.is_empty() {
            println!("Invalid continued chunk {:02X?}", chunk);
            return Ok(());
        }

        packet.write_all(data)?;
        if flags.contains(FIFODataRXFlags::PKTEND) {
            let bytes = packet.split_off(packet.len() - 2);
            let checksum = u16::from_be_bytes([bytes[0], bytes[1]]);
            let ccitt = Crc::<u16>::new(&CRC_16_GENIBUS);
            let mut digest = ccitt.digest();
            digest.update(packet);
            let calculated = digest.finalize();

            if calculated == checksum {
                uplink.send(packet)?;
                println!("UHF RX PACKET: {:02X?}", packet);
            } else {
                println!(
                    "Rejected CRC: received 0x{:x}, calculated 0x{:x}",
                    checksum, calculated
                );
            }
            packet.clear();
        }
    }
    Ok(())
}

fn read_packet(radio: &mut Registers, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    let len = radio.FIFOCOUNT().read()?;
    if len == 0 {
        return Ok(());
    }

    match radio.FIFODATARX().read(len.into()) {
        Ok(chunks) => {
            for chunk in chunks {
                process_chunk(chunk, packet, uplink)?;
            }
        }
        Err(e) => {
            // FIFO Errors are usually just overflow, non-fatal
            println!("{}", e);
            packet.clear();
        }
    }
    Ok(())
}

fn transmit(radio: &mut Registers, buf: &[u8], src: SocketAddr) -> Result<()> {
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate

    // FIXME: I experienced some crashes that probably occured because FIFOTHRESH returned the
    // wrong value. Once 0, once too big (but not measured). Lets hard code it for now to be safe
    // for flight.
    //let thresh: usize = radio.FIFOTHRESH().read()?.into();
    let thresh: usize = 128;

    let pa_on = FIFOChunkTX::TXCTRL(TXCtrl::SETPA | TXCtrl::PASTATE);
    /* FIXME: this is the recommended preamble
    let preamble = FIFOChunkTX::DATA {
        flags: FIFODataTXFlags::RAW,
        data: vec![0x11],
    };
    */
    let preamble = FIFOChunkTX::REPEATDATA {
        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
        count: 0x50,
        data: 0x7E,
    };

    let postamble = FIFOChunkTX::REPEATDATA {
        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
        count: 0x5,
        data: 0x7E,
    };

    // TODO: integrate recv
    // TODO: maybe FIFOChunkTX::to_data -> Vec? It knows how big it should be
    let header_size = 3; // size of FIFOChunkTX::DATA header

    // I witnessed thresh read as 0 once, which crashed the driver. Having thresh (FIFOTHRESH)
    // return 0 makes no sense, but lets guard against it anyway.
    let Some(chunksize) = thresh.checked_sub(header_size) else {
        radio.PWRMODE().write(PwrMode {
            flags: PwrFlags::XOEN | PwrFlags::REFEN,
            mode: PwrModes::POWEROFF,
        })?;
        bail!("FIFOTHRESH returned 0. Weird");
    };
    let mut packet: Vec<FIFOChunkTX> = buf
        .chunks(chunksize)
        .map(|x| FIFOChunkTX::DATA {
            flags: FIFODataTXFlags::empty(),
            data: x.to_vec(),
        })
        .collect();
    if let Some(FIFOChunkTX::DATA { ref mut flags, .. }) = packet.first_mut() {
        *flags |= FIFODataTXFlags::PKTSTART;
    }
    if let Some(FIFOChunkTX::DATA { ref mut flags, .. }) = packet.last_mut() {
        *flags |= FIFODataTXFlags::PKTEND;
    }
    let pa_off = FIFOChunkTX::TXCTRL(TXCtrl::SETPA);

    radio.FIFODATATX().write(pa_on)?;
    radio.FIFODATATX().write(preamble)?;

    println!("UHF SEND {} from {:?}: {:X?}", buf.len(), src, packet);

    'outer: for chunk in packet {
        radio.FIFODATATX().write(chunk)?;
        radio.FIFOCMD().write(FIFOCmd {
            mode: FIFOCmds::COMMIT,
            auto_commit: false,
        })?;
        // FIXME interrupt?
        loop {
            let stat = radio.FIFOSTAT().read()?;
            if stat.contains(FIFOStat::OVER) || stat.contains(FIFOStat::UNDER) {
                println!("chunk: {:?}", stat);
                // FIXME: I saw this happen once and then hang? We should probably abandon ship
                // here. Possibly set the abort bit?
                radio.FIFOCMD().write(FIFOCmd {
                    mode: FIFOCmds::CLEAR_ERROR,
                    auto_commit: false,
                })?;
                radio.FIFOCMD().write(FIFOCmd {
                    mode: FIFOCmds::CLEAR_DATA,
                    auto_commit: false,
                })?;
                break 'outer;
            }
            if stat.contains(FIFOStat::FREE_THR) {
                break;
            }
        }
    }

    radio.FIFODATATX().write(postamble)?;

    radio.FIFODATATX().write(pa_off)?;
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::COMMIT,
        auto_commit: false,
    })?;

    while radio.RADIOSTATE().read()? != RadioState::IDLE {} // TODO: Interrupt of some sort
    radio.PWRAMP().write(registers::PwrAmp::empty())?; // FIXME why isn't pa_off doing this?
    while radio.PWRAMP().read()?.contains(registers::PwrAmp::PWRAMP) {} // TODO: Interrupt of some sort

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::POWEROFF,
    })?;
    Ok(())
}

#[derive(Parser, Debug)]
/// Try it out: `socat STDIO UDP:localhost:10015`
///             `socat UDP-LISTEN:10025 STDOUT`
struct Args {
    #[arg(short, long, default_value = "10015")]
    beacon: u16,
    #[arg(short, long, default_value = "10016")]
    downlink: u16,
    #[arg(short, long, default_value = "10025")]
    uplink: u16,
    #[arg(short, long, default_value = "/dev/spidev0.0")]
    spi: String,
    /// For example 10.18.17.6:10035
    #[arg(short, long)]
    telemetry: Option<String>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut poll = Poll::new()?;
    let registry = poll.registry();
    let mut events = Events::with_capacity(128);

    let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), args.beacon);
    let mut beacon = UdpSocket::bind(addr)?;
    const BEACON: Token = Token(0);
    registry.register(&mut beacon, BEACON, Interest::READABLE)?;

    let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), args.downlink);
    let mut downlink = UdpSocket::bind(addr)?;
    const DOWNLINK: Token = Token(1);
    registry.register(&mut downlink, DOWNLINK, Interest::READABLE)?;

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), args.uplink);
    let mut uplink = UdpSocket::bind(src)?;
    uplink.connect(dest)?;

    let mut telemetry: Option<std::net::UdpSocket> = None;
    if let Some(addr) = args.telemetry {
        let dest: SocketAddr = addr.parse().unwrap();
        let socket = std::net::UdpSocket::bind(src)?;
        socket.connect(dest)?;
        telemetry = Some(socket);
    }

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let pa_enable = Request::builder()
        .on_chip("/dev/gpiochip1")
        .with_line(27)
        .as_output(Value::Inactive)
        .request()?;

    let uhf_irq = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(30)
        .with_edge_detection(EdgeDetection::RisingEdge)
        .request()?;

    const IRQ: Token = Token(4);
    registry.register(&mut SourceFd(&uhf_irq.as_raw_fd()), IRQ, Interest::READABLE)?;

    let mut tfd = TimerFd::new().unwrap();
    if telemetry.is_some() {
        tfd.set_state(
            TimerState::Periodic {
                current: Duration::new(1, 0),
                interval: Duration::from_millis(25),
            },
            SetTimeFlags::Default,
        );
    } else {
        tfd.set_state(
            TimerState::Periodic {
                current: Duration::new(0, 0),
                interval: Duration::new(0, 0),
            },
            SetTimeFlags::Default,
        );
    }
    const TELEMETRY: Token = Token(5);
    registry.register(
        &mut SourceFd(&tfd.as_raw_fd()),
        TELEMETRY,
        Interest::READABLE,
    )?;

    let spi0 = ax5043::open(args.spi)?;
    let mut status = ax5043::Status::empty();
    let mut callback = |_: &_, _addr, s, _data: &_| {
        //println!("{:03X}: {:02X?}", addr, data);
        if s != status {
            if let Some(ref socket) = telemetry {
                tui::CommState::STATUS(s).send(socket).unwrap();
            }
            status = s;
        }
    };

    let mut radio = ax5043::Registers::new(spi0, &mut callback);
    radio.reset()?;

    let rev = radio.REVISION().read()?;
    ensure!(
        rev == 0x51,
        "Unexpected revision {}, expected {}",
        rev,
        0x51
    );

    let file_path = "c3-uhf-96000.toml";
    let contents = read_to_string(file_path)?;
    let config: config::Config = toml::from_str(&contents)?;
    config.write(&mut radio)?;
    let config_tx = config.tx.expect("Section [tx] required");
    let channel_edl = config
        .channel
        .first()
        .expect("Missing first [channel] (edl)");
    let channel_beacon = config
        .channel
        .get(1)
        .expect("Missing second [channel] (beacon)");

    radio.FIFOTHRESH().write(128)?; // Half the FIFO size

    radio.RSSIREFERENCE().write(32)?;

    pa_enable.set_value(27, Value::Active)?;

    if let Some(ref socket) = telemetry {
        tui::CommState::BOARD(config.board).send(socket)?;
        tui::CommState::REGISTERS(tui::StatusRegisters::new(&mut radio)?).send(socket)?;
        tui::CommState::CONFIG(tui::Config {
            txparams: tui::TXParameters::new(&mut radio, &config.board)?,
            rxparams: tui::RXParams::new(&mut radio, &config.board)?,
            set0: tui::RXParameterSet::set0(&mut radio)?,
            set1: tui::RXParameterSet::set1(&mut radio)?,
            set2: tui::RXParameterSet::set2(&mut radio)?,
            set3: tui::RXParameterSet::set3(&mut radio)?,
            synthesizer: tui::Synthesizer::new(&mut radio, &config.board)?,
            packet_controller: tui::PacketController::new(&mut radio)?,
            packet_format: tui::PacketFormat::new(&mut radio)?,
            channel: tui::ChannelParameters::new(&mut radio)?,
        })
        .send(socket)?;
    }

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
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

    radio
        .IRQMASK()
        .write(ax5043::registers::IRQ::FIFONOTEMPTY)?;

    let mut packet = Vec::new();

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                TELEMETRY => {
                    tfd.read();
                    if let Some(ref socket) = telemetry {
                        tui::CommState::STATE(tui::RXState::new(&mut radio, channel_edl)?)
                            .send(socket)?;
                        tui::CommState::REGISTERS(tui::StatusRegisters::new(&mut radio)?)
                            .send(socket)?;
                    }
                }
                BEACON => {
                    radio.IRQMASK().write(ax5043::registers::IRQ::empty())?;
                    // PM p. 12: The FIFO should be emptied before the PWRMODE is set to POWERDOWN
                    radio.FIFOCMD().write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_ERROR,
                        auto_commit: false,
                    })?;
                    radio.FIFOCMD().write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_DATA,
                        auto_commit: false,
                    })?;
                    // See errata - PWRMODE must transition through off for FIFO to work
                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::POWEROFF,
                    })?;

                    channel_beacon.write(&mut radio, &config.board)?;
                    config_tx.write(&mut radio, &config.board, channel_beacon)?;

                    let mut buf = [0; 2048];
                    loop {
                        match beacon.recv_from(&mut buf) {
                            Ok((amt, src)) => transmit(&mut radio, &buf[..amt], src)?,
                            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                            Err(e) => return Err(e).context("Ping socket read failed"),
                        }
                    }

                    channel_edl.write(&mut radio, &config.board)?;

                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::RX,
                    })?;
                    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD
                    radio
                        .IRQMASK()
                        .write(ax5043::registers::IRQ::FIFONOTEMPTY)?;
                }
                DOWNLINK => {
                    radio.IRQMASK().write(ax5043::registers::IRQ::empty())?;
                    // PM p. 12: The FIFO should be emptied before the PWRMODE is set to POWERDOWN
                    radio.FIFOCMD().write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_ERROR,
                        auto_commit: false,
                    })?;
                    radio.FIFOCMD().write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_DATA,
                        auto_commit: false,
                    })?;
                    // See errata - PWRMODE must transition through off for FIFO to work
                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::POWEROFF,
                    })?;

                    channel_edl.write(&mut radio, &config.board)?;
                    config_tx.write(&mut radio, &config.board, channel_edl)?;

                    let mut buf = [0; 2048];
                    loop {
                        match downlink.recv_from(&mut buf) {
                            Ok((amt, src)) => transmit(&mut radio, &buf[..amt], src)?,
                            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                            Err(e) => return Err(e).context("Downlink socket read failed"),
                        }
                    }

                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::RX,
                    })?;
                    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD
                    radio
                        .IRQMASK()
                        .write(ax5043::registers::IRQ::FIFONOTEMPTY)?;
                }
                IRQ => {
                    while uhf_irq.has_edge_event()? {
                        uhf_irq.read_edge_event()?;
                        read_packet(&mut radio, &mut packet, &mut uplink)?;
                    }
                }
                SIGINT => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    pa_enable.set_value(27, Value::Inactive)?;
    radio.reset()?;
    Ok(())
}

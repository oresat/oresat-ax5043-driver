use anyhow::{ensure, Result};
use ax5043::{config, registers::*, tui, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiocdev::{line::EdgeDetection, Request};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::{fs::read_to_string, io::Write, os::fd::AsRawFd, time::Duration};
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
                "LBAND REJECTED {:?} {:02X?} ...+{}",
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
                    "LBAND PKT RESTART rejecting {:02X?} ...+{}",
                    packet[0],
                    packet.len(),
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
                println!("LBAND RX PACKET: {:02X?}", packet);
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

#[derive(Parser, Debug)]
/// Try it out: `socat UDP-LISTEN:10025 STDOUT`
struct Args {
    #[arg(short, long, default_value = "10025")]
    uplink: u16,
    #[arg(short, long, default_value = "/dev/spidev1.1")]
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

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), args.uplink);
    let mut uplink = UdpSocket::bind(src)?;
    uplink.connect(dest)?;

    let mut telemetry: Option<UdpSocket> = None;
    if let Some(addr) = args.telemetry {
        let dest: SocketAddr = addr.parse().unwrap();
        let socket = UdpSocket::bind(src)?;
        socket.connect(dest)?;
        telemetry = Some(socket);
    }

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let lband_irq = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(30)
        .with_edge_detection(EdgeDetection::RisingEdge)
        .request()?;

    const IRQ: Token = Token(4);
    registry.register(
        &mut SourceFd(&lband_irq.as_raw_fd()),
        IRQ,
        Interest::READABLE,
    )?;

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
    let mut callback = |_: &_, _addr, s, _val: &[u8]| {
        //println!("{:03X}: {:02X?}", addr, val);
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
        0x51,
    );

    let file_path = "c3-lband-60000.toml";
    let contents = read_to_string(file_path)?;
    let config: config::Config = toml::from_str(&contents)?;
    config.write(&mut radio)?;

    radio.FIFOTHRESH().write(128)?; // Half the FIFO size
    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x00)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(32)?;

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
                        tui::CommState::STATE(tui::RXState::new(&mut radio, &config.channel[0])?)
                            .send(socket)?;
                        tui::CommState::REGISTERS(tui::StatusRegisters::new(&mut radio)?)
                            .send(socket)?;
                    }
                }
                IRQ => {
                    while lband_irq.has_edge_event()? {
                        lband_irq.read_edge_event()?;
                        read_packet(&mut radio, &mut packet, &mut uplink)?;
                    }
                }
                SIGINT => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;
    Ok(())
}

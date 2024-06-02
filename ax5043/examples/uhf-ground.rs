use anyhow::Result;
use ax5043::{config, Status};
use ax5043::{registers::*, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiod::{Chip, EdgeDetect, Options};
use mio::net::UdpSocket;
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    fs::read_to_string,
    io::Write,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    os::fd::AsRawFd,
};
use toml;

fn read_packet(
    radio: &mut Registers,
    packet: &mut Vec<u8>,
    downlink: &mut UdpSocket,
) -> Result<()> {
    let len = radio.FIFOCOUNT().read()?;
    if len <= 0 {
        return Ok(());
    }

    for chunk in radio.FIFODATARX().read(len.into())? {
        if let FIFOChunkRX::DATA { flags, ref data } = chunk {
            println!("{:02X?}", chunk);
            if flags.intersects(
                FIFODataRXFlags::ABORT
                    | FIFODataRXFlags::SIZEFAIL
                    | FIFODataRXFlags::ADDRFAIL
                    | FIFODataRXFlags::CRCFAIL
                    | FIFODataRXFlags::RESIDUE,
            ) {
                packet.clear();
                continue;
            }

            if flags.contains(FIFODataRXFlags::PKTSTART) {
                packet.clear();
            }
            packet.write(&data)?;
            if flags.contains(FIFODataRXFlags::PKTEND) {
                let bytes = packet.split_off(packet.len() - 2);
                let checksum = u16::from_be_bytes([bytes[0], bytes[1]]);
                let ccitt = Crc::<u16>::new(&CRC_16_GENIBUS);
                let mut digest = ccitt.digest();
                digest.update(packet);
                let calculated = digest.finalize();

                if calculated == checksum {
                    downlink.send(packet)?;
                    println!("UHF RX PACKET: {:02X?}", packet);
                } else {
                    println!(
                        "Rejected CRC: received 0x{:x}, calculated 0x{:x}",
                        checksum, calculated
                    );
                }
            }
        }
    }
    Ok(())
}

#[derive(Parser, Debug)]
/// Try it out: `socat STDIO UDP:localhost:10015`
///             `socat UDP-LISTEN:10025 STDOUT`
struct Args {
    #[arg(short, long, default_value = "10016")]
    downlink: u16,
    #[arg(short, long, default_value = "/dev/spidev1.0")]
    spi: String,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut poll = Poll::new()?;
    let registry = poll.registry();
    let mut events = Events::with_capacity(128);

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 16, 0, 5)), args.downlink);
    let mut downlink = UdpSocket::bind(src)?;
    downlink.connect(dest)?;

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let chip0 = Chip::new("gpiochip0")?;
    let opts = Options::input([16]).edge(EdgeDetect::Rising);
    let mut uhf_irq = chip0.request_lines(opts)?;

    const IRQ: Token = Token(4);
    registry.register(&mut SourceFd(&uhf_irq.as_raw_fd()), IRQ, Interest::READABLE)?;

    let spi0 = ax5043::open(args.spi)?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, s, _: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
    };
    let mut radio = ax5043::Registers::new(spi0, &mut callback);
    radio.reset()?;

    let rev = radio.REVISION().read()?;
    if rev != 0x51 {
        println!("Unexpected revision {}, expected {}", rev, 0x51);
        return Ok(());
    }

    let file_path = "rpi-uhf-96000.toml";
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
                IRQ => {
                    uhf_irq.read_event()?;
                    while uhf_irq.get_values(0u8)? > 0 {
                        read_packet(&mut radio, &mut packet, &mut downlink)?;
                    }
                }
                //TIMER => {
                //    tfd.read();
                //    println!("RSSI: {}", radio.RSSI().read()?);
                //}
                SIGINT => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;
    Ok(())
}

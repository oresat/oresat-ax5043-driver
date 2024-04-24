use anyhow::Result;
use ax5043::{config, config::*, Status};
use ax5043::{registers::*, Registers, RX, TX};
use clap::Parser;
use mio::net::UdpSocket;
use mio::{Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    net::{IpAddr, Ipv6Addr, SocketAddr},
};

fn configure_radio(radio: &mut Registers) -> Result<Board> {
    let board = config::board::RPI.write(radio)?;
    let synth = config::synth::LBAND_DC_505.write(radio, &board)?;
    let channel = config::channel::GMSK_60000.write(radio, &board)?;

    TXParameters {
        antenna: Antenna::SingleEnded,
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        plllock_gate: true,
        brownout_gate: true,
    }.write(radio, &board, &channel)?;

    radio.FIFOTHRESH().write(128)?; // Half the FIFO size

    synth.autorange(radio)?;
    Ok(board)
}

fn transmit(radio: &mut Registers, buf: &[u8], amt: usize) -> Result<()> {
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    _ = radio.POWSTICKYSTAT().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    let thresh = radio.FIFOTHRESH().read()?.into();

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
    let mut packet = Vec::new();
    for i in (0..amt).step_by(thresh) {
        packet.push(FIFOChunkTX::DATA {
            flags: if i == 0 {
                FIFODataTXFlags::PKTSTART
            } else {
                FIFODataTXFlags::empty()
            } | if i + thresh > amt {
                FIFODataTXFlags::PKTEND
            } else {
                FIFODataTXFlags::empty()
            },
            data: buf[i..std::cmp::min(i + thresh, amt)].to_vec(),
        })
    }


    radio.FIFODATATX().write(preamble)?;

    println!("sending {} chunks", packet.len());
    println!("{:X?}", packet);

    for chunk in packet {
        println!("chunk");
        radio.FIFODATATX().write(chunk)?;
        radio.FIFOCMD().write(FIFOCmd {
            mode: FIFOCmds::COMMIT,
            auto_commit: false,
        })?;
        while !radio.FIFOSTAT().read()?.contains(FIFOStat::FREE_THR) {} // TODO: Interrupt
    }

    radio.FIFODATATX().write(postamble)?;

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::COMMIT,
        auto_commit: false,
    })?;

    while radio.RADIOSTATE().read()? != RadioState::IDLE {} // TODO: Interrupt of some sort

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::POWEROFF,
    })?;
    Ok(())
}

#[derive(Parser, Debug)]
/// Try it out: `socat STDIO UDP:localhost:10025`
struct Args {
    #[arg(short, long, default_value = "10025")]
    uplink: u16,
    #[arg(short, long, default_value = "/dev/spidev0.0")]
    spi: String,
    #[arg(short, long, default_value_t = 0x700)]
    power: u16,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut poll = Poll::new()?;
    let registry = poll.registry();
    let mut events = Events::with_capacity(128);

    let addr = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), args.uplink);
    let mut uplink = UdpSocket::bind(addr)?;
    const UPLINK: Token = Token(0);
    registry.register(&mut uplink, UPLINK, Interest::READABLE)?;

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let spi0 = ax5043::open(args.spi)?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _addr, s, _data: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
        //println!("{addr:03X}: {data:02X?}");
    };
    let mut radio = ax5043::Registers::new(spi0, &mut callback);
    radio.reset()?;

    let rev = radio.REVISION().read()?;
    if rev != 0x51 {
        println!("Unexpected revision {}, expected {}", rev, 0x51);
        return Ok(());
    }
    _ = configure_radio(&mut radio)?;

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

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                UPLINK => {
                    // See errata - PWRMODE must transition through off for FIFO to work
                    let mut buf = [0; 2048];
                    let (amt, src) = uplink.recv_from(&mut buf)?;
                    println!("Recv {} from {}: {:X?}", amt, src, &buf[..amt]);

                    transmit(&mut radio, &buf, amt)?;
                }
                SIGINT => break 'outer,
                _ => unreachable!(),
            }
        }
    }
    radio.reset()?;
    Ok(())
}

use clap::Parser;
use mio::net::UdpSocket;
use mio::{Events, Interest, Poll, Token};
use std::{
    cell::Cell,
    io,
    net::{IpAddr, Ipv6Addr, SocketAddr},
};

use ax5043::{config::*, fifo, Encoding, PwrFlags, PwrMode, PwrModes, Registers, Status};

// Try it out: `socat STDIO UDP:localhost:10015`

fn configure_radio(radio: &mut Registers) -> io::Result<()> {
    let board = Board {
        sysclk: Pin { mode: SysClk::Z,    pullup: false, invert: false, },
        dclk:   Pin { mode: DClk::Z,      pullup: false, invert: false, },
        data:   Pin { mode: Data::Z,      pullup: false, invert: false, },
        pwramp: Pin { mode: PwrAmp::TCXO, pullup: false, invert: false, },
        irq:    Pin { mode: IRQ::IRQ,     pullup: false, invert: false, },
        antsel: Pin { mode: AntSel::Z,    pullup: false, invert: false, },
        xtal: Xtal {
            kind: XtalKind::TCXO,
            freq: 48_000_000,
            enable: XtalPin::AntSel,
        },
        vco: VCO::Internal,
        filter: Filter::Internal,
        antenna: Antenna::SingleEnded,
        dac: DAC {
            pin: DACPin::PwrAmp,
        },
        adc: ADC::ADC1,
    };

    configure(radio, &board)?;

    let synth = Synthesizer {
        freq_a: 436_500_000,
        freq_b: 0,
        active: FreqReg::A,
        pll: PLL {
            charge_pump_current: 0x02, // From spreadsheet
            filter_bandwidth: LoopFilter::Internalx1,
        },
        boost: PLL {
            charge_pump_current: 0xC8,                // Default value
            filter_bandwidth: LoopFilter::Internalx5, // Default value
        },
        vco_current: Some(0x13), // depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
        lock_detector_delay: None, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };

    configure_synth(radio, &board, &synth)?;

    let channel = ChannelParameters {
        modulation: Modulation::GFSK {
            deviation: 20_000,
            ramp: SlowRamp::Bits1,
            bt: BT(0.3),
        },
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: Framing::HDLC { fec: FEC {} },
        crc: CRC::CCITT { initial: 0xFFFF },
    };

    configure_channel(radio, &board, &channel)?;

    let parameters = TXParameters {
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        txrate: 60_000,
        plllock_gate: true,
        brownout_gate: true,
    };

    configure_tx(radio, &board, &channel, &parameters)?;

    autorange(radio)
}

#[derive(Parser, Debug)]
struct Args {
    #[arg(short, long, default_value = "10015")]
    beacon: u16,
    // #[arg(short, long, default_value="10016")]
    // edl: u16,
    // #[arg(short, long, default_value="::")]
    // source: String,
    // #[arg(short, long, default_value="::1")]
    // destination: String,
    // #[arg(short, long, default_value="spidev0.0")]
    // spi: String
}

fn main() -> io::Result<()> {
    let args = Args::parse();

    let addr = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), args.beacon);
    let mut beacon = UdpSocket::bind(addr)?;
    const BEACON: Token = Token(0);

    let mut poll = Poll::new()?;

    poll.registry()
        .register(&mut beacon, BEACON, Interest::READABLE)?;
    let mut events = Events::with_capacity(128);

    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio = ax5043::registers(&spi0, &callback);
    radio.reset()?;
    configure_radio(&mut radio)?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;
    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio: &mut radio,
    };

    loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    let mut buf = [0; 512];
                    let (amt, src) = beacon.recv_from(&mut buf)?;
                    println!("Recv {} from {}: {:?}", amt, src, &buf[..amt]);
                    fifo.write(
                        &buf[..amt],
                        fifo::TXDataFlags::PKTSTART | fifo::TXDataFlags::PKTEND,
                    )?;
                    fifo.commit()?;
                }
                _ => unreachable!(),
            }
        }
    }
}

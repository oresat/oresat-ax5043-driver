// Intended to be run on the C3v6, produces just a carrier signal
use anyhow::Result;
use ax5043::{config, config::PwrAmp, config::IRQ, config::*, Status};
use ax5043::{registers::*, Registers, RX, TX};
use clap::Parser;
use gpiod::{Chip, Options};
use mio::net::UdpSocket;
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    net::{IpAddr, Ipv6Addr, SocketAddr},
    os::fd::AsRawFd,
    time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

fn configure_radio(radio: &mut Registers, power: u16) -> Result<()> {
    #[rustfmt::skip]
    let board = Board {
        sysclk: Pin { mode: SysClk::Z,      pullup: true,  invert: false, },
        dclk:   Pin { mode: DClk::Z,        pullup: true,  invert: false, },
        data:   Pin { mode: Data::Z,        pullup: true,  invert: false, },
        pwramp: Pin { mode: PwrAmp::PwrAmp, pullup: false, invert: false, },
        irq:    Pin { mode: IRQ::IRQ,       pullup: false, invert: false, },
        antsel: Pin { mode: AntSel::Z,      pullup: true,  invert: false, },
        xtal: Xtal {
            kind: XtalKind::TCXO,
            freq: 16_000_000,
            enable: XtalPin::None,
        },
        vco: VCO::Internal,
        filter: Filter::Internal,
        dac: DAC { pin: DACPin::None },
        adc: ADC::None,
    };

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
        //vco_current: Manual(0x16), // depends on VCO, readback VCOIR, see AND9858/D for manual cal
        vco_current: Control::Automatic,
        lock_detector_delay: Control::Automatic, // readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };

    let channel = ChannelParameters {
        modulation: config::Modulation::ASK,
        encoding: Encoding::NRZ,
        framing: config::Framing::Raw,
        crc: CRC::None,
        datarate: 9_600,
        bitorder: BitOrder::LSBFirst,
    };
    let parameters = TXParameters {
        antenna: Antenna::SingleEnded,
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: power,
            c: 0,
            d: 0,
            e: 0,
        },
        plllock_gate: true,
        brownout_gate: true,
    };

    configure(radio, &board)?;
    configure_synth(radio, &board, &synth)?;
    configure_channel(radio, &board, &channel)?;
    configure_tx(radio, &board, &channel, &parameters)?;

    radio.FIFOTHRESH().write(128)?; // Half the FIFO size

    autorange(radio)?;
    Ok(())
}

fn carrier(radio: &mut Registers) -> Result<()> {
    // TODO: run on threshold, pause for tot
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate

    let pa_on = FIFOChunkTX::TXCTRL(TXCtrl::SETPA | TXCtrl::PASTATE);
    let carrier = FIFOChunkTX::REPEATDATA {
        flags: FIFODataTXFlags::UNENC,
        count: 0xFF,
        data: 0xFF,
    };

    let pa_off = FIFOChunkTX::TXCTRL(TXCtrl::SETPA);

    radio.FIFODATATX().write(pa_on)?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    radio.FIFODATATX().write(carrier.clone())?;
    // TODO: figure out exactly how long the tot is/limit carrier copies to tot
    // 800ms tot
    radio.FIFODATATX().write(pa_off)?;
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
/// Try it out: `socat STDIO UDP:localhost:10015`
struct Args {
    #[arg(short, long, default_value = "10015")]
    beacon: u16,
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

    let addr = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), args.beacon);
    let mut beacon = UdpSocket::bind(addr)?;
    const BEACON: Token = Token(0);
    registry.register(&mut beacon, BEACON, Interest::READABLE)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(500),
        },
        SetTimeFlags::Default,
    );
    const TIMER: Token = Token(2);
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), TIMER, Interest::READABLE)?;

    const CTRLC: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let chip = Chip::new("gpiochip1")?;
    let opts = Options::output([27]).values([false]);
    let pa_enable = chip.request_lines(opts)?;

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

    configure_radio(&mut radio, args.power)?;
    pa_enable.set_values([true])?;

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                TIMER => {
                    tfd.read();
                    carrier(&mut radio)?
                }
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    pa_enable.set_values([false])?;
    radio.reset()?;
    Ok(())
}

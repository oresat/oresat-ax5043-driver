use ax5043::{config, config::PwrAmp, config::IRQ, config::*, Status};
use ax5043::{registers::*, Registers};
use mio::{Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    cell::Cell,
    io,
};

fn configure_radio(radio: &mut Registers) -> io::Result<()> {
    #[rustfmt::skip]
    let board = Board {
        sysclk: Pin { mode: SysClk::XtalDiv1, pullup: false,  invert: false, },
        dclk:   Pin { mode: DClk::Z,          pullup: true,  invert: false, },
        data:   Pin { mode: Data::Z,          pullup: true,  invert: false, },
        pwramp: Pin { mode: PwrAmp::PwrAmp,   pullup: false, invert: false, },
        irq:    Pin { mode: IRQ::IRQ,         pullup: false, invert: false, },
        antsel: Pin { mode: AntSel::Z,        pullup: true,  invert: false, },
        xtal: Xtal {
            kind: XtalKind::TCXO,
            freq: 16_000_000,
            enable: XtalPin::None,
        },
        vco: VCO::Internal,
        filter: Filter::Internal,
        antenna: Antenna::SingleEnded,
        dac: DAC { pin: DACPin::None },
        adc: ADC::None,
    };

    let synth = Synthesizer {
        freq_a: 457_000_000,
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
        //vco_current: Some(0x16), // depends on VCO, auto or manual, readback VCOIR, see AND9858/D for manual cal
        vco_current: None,                        // FIXME: label Auto
        lock_detector_delay: None, // auto or manual, readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    };

    let channel = ChannelParameters {
        modulation: config::Modulation::GMSK {
            ramp: config::SlowRamp::Bits1,
            bt: BT(0.5),
        },
        encoding: Encoding::NRZI | Encoding::SCRAM,
        framing: config::Framing::HDLC {
            fec: config::FEC {},
        }, // FIXME PKTADDRCFG bit order
        crc: CRC::CCITT { initial: 0xFFFF },
    };
    let parameters = TXParameters {
        amp: AmplitudeShaping::RaisedCosine {
            a: 0,
            b: 0x700,
            c: 0,
            d: 0,
            e: 0,
        },
        txrate: 120_000,
        plllock_gate: true,
        brownout_gate: true,
    };

    configure(radio, &board)?;
    configure_synth(radio, &board, &synth)?;
    configure_channel(radio, &board, &channel)?;
    configure_tx(radio, &board, &channel, &parameters)?;

    radio.FIFOTHRESH.write(128)?; // Half the FIFO size

    autorange(radio)
}

fn main() -> io::Result<()> {
    let mut poll = Poll::new()?;
    let registry = poll.registry();
    let mut events = Events::with_capacity(128);

    const CTRLC: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    let spi0 = ax5043::open("/dev/spidev1.1")?;
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio = ax5043::Registers::new(&spi0, &callback);
    radio.reset()?;

    let rev = radio.REVISION.read()?;
    if rev != 0x51 {
        println!("Unexpected revision {}, expected {}", rev, 0x51);
        return Ok(());
    }

    configure_radio(&mut radio)?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::XOEN,
    })?;

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()
}


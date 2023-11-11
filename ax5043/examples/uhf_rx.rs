use ax5043::{config, config::PwrAmp, config::IRQ, config::*, Status};
use ax5043::{registers::*, Registers};
use clap::Parser;
use gpiod::{Chip, Options};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    cell::Cell,
    io,
    os::fd::AsRawFd,
    time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};



fn configure_radio(radio: &mut Registers) -> io::Result<(Board, ChannelParameters)> {
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
        antenna: Antenna::Differential,
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
        },
        crc: CRC::CCITT { initial: 0xFFFF },
        datarate: 9_600,
    };

    configure(radio, &board)?;
    configure_synth(radio, &board, &synth)?;
    configure_channel(radio, &board, &channel)?;

    radio.FIFOTHRESH.write(128)?; // Half the FIFO size

    autorange(radio)?;
    Ok((board, channel))
}

enum RXParameters {
    MSK {
        max_dr_offset: u64,
        freq_offs_corr: bool,
        ampl_filter: u8,
        frequency_leak: u8,
    }
}

impl RXParameters {
    fn write(&self, radio: &mut Registers, board: &config::Board, channel: &config::ChannelParameters) -> io::Result<()> {
        match self {
            Self::MSK {freq_offs_corr, ampl_filter, frequency_leak, .. } => {
                // m = 0.5;
                // bandwidth  = (1+m) * bitrate; // Carson's rule
                //let bandwidth = 3 * channel.datarate / 2;
                //let fcoeff = 0.25; // FIXME PHASEGAIN::FILTERIDX but translated through table 116
                //let fcoeff_inv = 4; // 1/fcoeff

                let if_freq = 12_000; // From radiolab
                radio.IFFREQ.write((if_freq * board.xtal.div() * 2_u64.pow(20) / board.xtal.freq).try_into().unwrap())?;

                //let fbaseband = bandwidth * (1+fcoeff_inv);
                let fbaseband = 75000; // From radiolab
                let decimation = board.xtal.freq / (fbaseband * 2u64.pow(4) * board.xtal.div());
                radio.DECIMATION.write(decimation.try_into().unwrap())?; // TODO: 7bits max

                // TODO: see note table 96
                radio.RXDATARATE.write((2u64.pow(7) * board.xtal.freq / (channel.datarate * board.xtal.div() * decimation)).try_into().unwrap())?;

                //let droff = (2u64.pow(7) * board.xtal.freq * *max_dr_offset) / (board.xtal.div() * channel.datarate.pow(2) * decimation);
                //radio.MAXDROFFSET.write(droff.try_into().unwrap())?;

                radio.MAXDROFFSET.write(0)?;

                //let max_rf_offset = bandwidth/4 ; // bw/4 Upper bound - difference between tx and rx fcarriers. see note pm table 98
                let max_rf_offset = 873; // From radiolab
                radio.MAXRFOFFSET.write(MaxRFOffset {
                    offset: (max_rf_offset * 2u64.pow(24) / board.xtal.freq).try_into().unwrap(),
                    correction: *freq_offs_corr,
                })?;

                radio.MAXRFOFFSET.write(MaxRFOffset {
                    offset: 0x131,
                    correction: true,

                })?;
                radio.AMPLFILTER.write(*ampl_filter)?;
                radio.FREQUENCYLEAK.write(*frequency_leak)?;

            }
            _ => unimplemented!()
        }
        Ok(())
    }
}

/*
first SYNTHBOOST SYNTHSETTLE
second IFINIT COARSEAGC AGC RSSI

preamble1: PS0
    TMGRXPREAMBLE1 to reset to second?

preamble2: PS1
    MATCH1
    TMGRXPREAMBLE2

preamble3: PS2
    MATCH0
    TMGRXPREAMBLE3

packet: PS3
    SFD
*/



// FIXME need to know:
// h/m?
// fbaseband
// fif/bandwidth?

pub fn configure_radio_rx(radio: &mut Registers) -> io::Result<(Board, ChannelParameters)> {
    let (board, channel) = configure_radio(radio)?;

    radio.PERF_F18.write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26.write(0x96)?;
    radio.PLLLOOP.write(PLLLoop {
        filter: FLT::INTERNAL_x5,
        flags: PLLLoopFlags::DIRECT,
        freqsel: FreqSel::A,
    })?;
    radio.PLLCPI.write(0x10)?;

    let params = RXParameters::MSK {
        max_dr_offset: 0, // TODO derived from what?
        freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    };
    params.write(radio, &board, &channel)?;

    let set0 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0x5,
            decay: 0xC,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 8,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 2,
            },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1010,
                freq: 0b0_1010,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset {
            a: 0,
            b: 0,
        },
    };
    set0.write0(radio)?;

    let set1 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0x5,
            decay: 0xC,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 6,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 1,
            },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1010,
                freq: 0b0_1010,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0x32,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset {
            a: 0,
            b: 0,
        },
    };
    set1.write1(radio)?;

    let set3 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0xF,
            decay: 0xF,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xF,
                exponent: 5,
            },
            datarate: DRGain {
                mantissa: 0xF,
                exponent: 0,
            },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1101,
                freq: 0b0_1101,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0x32,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset {
            a: 0,
            b: 0,
        },
    };
    set3.write3(radio)?;

    radio.RXPARAMSETS.write(RxParamSets(
        RxParamSet::Set0,
        RxParamSet::Set1,
        RxParamSet::Set3,
        RxParamSet::Set3,
    ))?;

    radio.MATCH1PAT.write(0x1111)?;
    radio.MATCH1LEN.write(MatchLen { len: 0xA, raw: false, })?;
    radio.MATCH1MAX.write(0xA)?;
    radio.TMGRXPREAMBLE2.write(TMG { m: 0x17, e: 0, })?;

    radio.PKTMAXLEN.write(0xC8)?;
    radio.PKTLENCFG.write(PktLenCfg {
        pos: 0,
        bits: 0x0
    })?;
    radio.PKTLENOFFSET.write(0x09)?;

    radio.PKTCHUNKSIZE.write(0x0D)?;
    radio.PKTACCEPTFLAGS.write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE.write(64)?;

    Ok((board, channel))
}






#[derive(Parser, Debug)]
/// Try it out: `socat STDIO UDP:localhost:10015`
struct Args {
    #[arg(short, long, default_value = "10015")]
    beacon: u16,
    #[arg(short, long, default_value = "/dev/spidev0.0")]
    spi: String,
}

fn main() -> io::Result<()> {
    let args = Args::parse();

    let mut poll = Poll::new()?;
    let registry = poll.registry();
    let mut events = Events::with_capacity(128);

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(50),
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
    let status = Cell::new(Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("RX Status change: {:?}", s);
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

    configure_radio_rx(&mut radio)?;
    pa_enable.set_values([true])?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;

    radio.FIFOCMD.write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD.write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;



    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                TIMER => {
                    tfd.read();
                    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT.read()?; // clear sticky power flags for PWR_GOOD

                    //println!("RSSI {}", radio.RSSI.read()?);
                    let len  = radio.FIFOCOUNT.read()?;
                    if len > 0 {
                        let data = radio.FIFODATARX.read(len.into())?;
                        println!("{:02X?}", data);
                    }

                }
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    pa_enable.set_values([false])?;
    radio.reset()
}

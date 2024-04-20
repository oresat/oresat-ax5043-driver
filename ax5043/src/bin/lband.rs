use anyhow::{ensure, Result};
use ax5043::{config, config::PwrAmp, config::IRQ, config::*};
use ax5043::{registers::*, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiod::{Chip, Options, EdgeDetect};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{io::Write, os::fd::AsRawFd};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};

fn configure_radio(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    #[rustfmt::skip]
    let board = Board {
        sysclk: Pin { mode: SysClk::XtalDiv1, pullup: false, invert: false, },
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
        dac: DAC { pin: DACPin::None },
        adc: ADC::None,
    }.write(radio)?;

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
        //vco_current: Manual(0x16), // depends on VCO, readback VCOIR, see AND9858/D for manual cal
        vco_current: Control::Automatic,
        lock_detector_delay: Control::Automatic, // readback PLLLOCKDET::LOCKDETDLYR
        ranging_clock: RangingClock::XtalDiv1024, // less than one tenth the loop filter bandwidth. Derive?
    }.write(radio, &board)?;

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
        datarate: 60_000,
        bitorder: BitOrder::MSBFirst,
    }.write(radio, &board)?;

    radio.FIFOTHRESH().write(128)?; // Half the FIFO size

    synth.autorange(radio)?;
    Ok((board, channel))
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

pub fn configure_radio_rx(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    let (board, channel) = configure_radio(radio)?;

    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;
    radio.PLLLOOP().write(PLLLoop {
        filter: FLT::INTERNAL_x5,
        flags: PLLLoopFlags::DIRECT,
        freqsel: FreqSel::A,
    })?;
    radio.PLLCPI().write(0x10)?;

    RXParameters::MSK {
        //max_dr_offset: 0, // TODO derived from what?
        freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    }.write(radio, &board, &channel)?;

    let set0 = RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0x8,
                exponent: 9,
            },
            datarate: DRGain {
                mantissa: 0x8,
                exponent: 3,
            },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_0110,
                freq: 0b0_0110,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set0.write0(radio)?;

    let set1 = RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0x8,
                exponent: 7,
            },
            datarate: DRGain {
                mantissa: 0x8,
                exponent: 2,
            },
            phase: 0b0011,
            filter: 0b11,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_0110,
                freq: 0b0_0110,
            },
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: 0x32,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set1.write1(radio)?;

    let set3 = RXParameterSet {
        agc: RXParameterAGC::off(),
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0x8,
                exponent: 6,
            },
            datarate: DRGain {
                mantissa: 0x8,
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
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set3.write3(radio)?;

    radio.RXPARAMSETS().write(RxParamSets(
        RxParamSet::Set0,
        RxParamSet::Set1,
        RxParamSet::Set3,
        RxParamSet::Set3,
    ))?;

    radio.MATCH1PAT().write(0x7E7E)?;
    radio.MATCH1LEN().write(MatchLen {
        len: 0xA,
        raw: false,
    })?;
    radio.MATCH1MAX().write(0xA)?;
    radio.TMGRXPREAMBLE2().write(TMG { m: 0x17, e: 1 })?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x09)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(64)?;

    Ok((board, channel))
}

fn process_chunk(chunk: FIFOChunkRX, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    if let FIFOChunkRX::DATA{flags, ref data} = chunk {
        //println!("{:02X?}", chunk);
        if flags.intersects(FIFODataRXFlags::ABORT | FIFODataRXFlags::SIZEFAIL | FIFODataRXFlags::ADDRFAIL | FIFODataRXFlags::CRCFAIL | FIFODataRXFlags::RESIDUE) {
            println!("LBAND REJECTED {:?} {:02X?} ...+{}", flags, data[0], data.len());
            packet.clear();
            return Ok(());
        }

        if flags.contains(FIFODataRXFlags::PKTSTART) {
            if !packet.is_empty() {
                println!("LBAND PKT RESTART {:?} {:02X?} ...+{}", flags, data[0], data.len());
            }
            packet.clear();
        }

        if !flags.contains(FIFODataRXFlags::PKTSTART) && packet.is_empty() {
            println!("Invalid continued chunk {:02X?}", chunk);
            return Ok(());
        }

        packet.write_all(data)?;
        if flags.contains(FIFODataRXFlags::PKTEND) {
            let bytes = packet.split_off(packet.len()-2);
            let checksum = u16::from_be_bytes([bytes[0], bytes[1]]);
            let ccitt = Crc::<u16>::new(&CRC_16_GENIBUS);
            let mut digest = ccitt.digest();
            digest.update(packet);
            let calculated = digest.finalize();

            if calculated == checksum {
                uplink.send(packet)?;
                println!("LBAND RX PACKET: {:02X?}", packet);
            } else {
                println!("Rejected CRC: received 0x{:x}, calculated 0x{:x}", checksum, calculated);
            }
            packet.clear();
        }
    }
    Ok(())
}

fn read_packet(radio: &mut Registers, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    let len = radio.FIFOCOUNT().read()?;
    if len == 0 {
        return Ok(())
    }

    match radio.FIFODATARX().read(len.into()) {
        Ok(chunks) => for chunk in chunks {
            process_chunk(chunk, packet, uplink)?;
        },
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

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let chip = Chip::new("gpiochip1")?;
    let opts = Options::output([27]).values([false]);
    let pa_enable = chip.request_lines(opts)?;

    let chip0 = Chip::new("gpiochip0")?;
    let opts = Options::input([31]).edge(EdgeDetect::Rising);
    let mut lband_irq = chip0.request_lines(opts)?;

    const IRQ: Token = Token(4);
    registry.register(&mut SourceFd(&lband_irq.as_raw_fd()), IRQ, Interest::READABLE)?;

    let spi0 = ax5043::open(args.spi)?;
    //let mut status = ax5043::Status::empty();
    //let mut callback = |_: &_, _, s, _: &_| {
    //    if s != status {
    //        println!("RX Status change: {:?}", s);
    //        status = s;
    //    }
    //};
    let mut callback = |_: &_, _, _, _: &_| {};

    let mut radio = ax5043::Registers::new(spi0, &mut callback);
    radio.reset()?;

    let rev = radio.REVISION().read()?;
    ensure!(rev == 0x51, "Unexpected revision {}, expected {}", rev, 0x51);

    configure_radio_rx(&mut radio)?;
    pa_enable.set_values([true])?;

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

    radio.IRQMASK().write(ax5043::registers::IRQ::FIFONOTEMPTY)?;
    let mut packet = Vec::new();

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                IRQ => {
                    lband_irq.read_event()?;
                    while lband_irq.get_values(0u8)? > 0 {
                        read_packet(&mut radio, &mut packet, &mut uplink)?;
                    }
                }
                SIGINT => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    pa_enable.set_values([false])?;
    radio.reset()?;
    Ok(())
}

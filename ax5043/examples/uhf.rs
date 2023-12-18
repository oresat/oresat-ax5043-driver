// Intended to be run on the C3v6, takes data from UDP port 10015
// and transmits it through the UHF AX5043
use anyhow::Result;
use ax5043::{config, config::PwrAmp, config::IRQ, config::*, Status};
use ax5043::{registers, registers::*, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiod::{Chip, Options, EdgeDetect};
use mio::net::UdpSocket;
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    io::Write,
    net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr},
    os::fd::AsRawFd,
};

fn configure_radio(radio: &mut Registers, power: u16) -> Result<(Board, ChannelParameters)> {
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
        //datarate: 9_600, // FIXME: beacon datarate vs edl datarate?
        datarate: 96_000,
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
    Ok((board, channel))
     // FIXME LSB First only for beacon

}

enum RXParameters {
    MSK {
        max_dr_offset: u64,
        freq_offs_corr: bool,
        ampl_filter: u8,
        frequency_leak: u8,
    },
}

impl RXParameters {
    fn write(
        &self,
        radio: &mut Registers,
        board: &config::Board,
        channel: &config::ChannelParameters,
    ) -> Result<()> {
        match self {
            Self::MSK {
                freq_offs_corr,
                ampl_filter,
                frequency_leak,
                ..
            } => {
                // m = 0.5;
                // bandwidth  = (1+m) * bitrate; // Carson's rule
                //let bandwidth = 3 * channel.datarate / 2;
                //let fcoeff = 0.25; // FIXME PHASEGAIN::FILTERIDX but translated through table 116
                //let fcoeff_inv = 4; // 1/fcoeff

                //let if_freq = 12_000; // From radiolab
                //radio.IFFREQ.write((if_freq * board.xtal.div() * 2_u64.pow(20) / board.xtal.freq).try_into().unwrap())?;
                radio.IFFREQ().write(0x1058)?;

                //let fbaseband = bandwidth * (1+fcoeff_inv);
                //let fbaseband = 75000; // From radiolab
                //let decimation = board.xtal.freq / (fbaseband * 2u64.pow(4) * board.xtal.div());
                //radio.DECIMATION.write(decimation.try_into().unwrap())?; // TODO: 7bits max
                radio.DECIMATION().write(1)?;
                // TODO: see note table 96
                //radio.RXDATARATE.write((2u64.pow(7) * board.xtal.freq / (channel.datarate * board.xtal.div() * decimation)).try_into().unwrap())?;
                radio.RXDATARATE().write(0x5355)?;

                //let droff = (2u64.pow(7) * board.xtal.freq * *max_dr_offset) / (board.xtal.div() * channel.datarate.pow(2) * decimation);
                //radio.MAXDROFFSET.write(droff.try_into().unwrap())?;

                radio.MAXDROFFSET().write(0)?;

                //let max_rf_offset = bandwidth/4 ; // bw/4 Upper bound - difference between tx and rx fcarriers. see note pm table 98
                let max_rf_offset = 873; // From radiolab
                radio.MAXRFOFFSET().write(MaxRFOffset {
                    offset: (max_rf_offset * 2u64.pow(24) / board.xtal.freq)
                        .try_into()
                        .unwrap(),
                    correction: *freq_offs_corr,
                })?;

                radio.MAXRFOFFSET().write(MaxRFOffset {
                    offset: 0x131,
                    correction: true,
                })?;
                radio.AMPLFILTER().write(*ampl_filter)?;
                radio.FREQUENCYLEAK().write(*frequency_leak)?;
            }
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

pub fn configure_radio_rx(radio: &mut Registers, power: u16) -> Result<(Board, ChannelParameters)> {
    let (board, channel) = configure_radio(radio, power)?;

    radio.PERF_F18().write(0x02)?; // TODO set by radiolab during RX
    radio.PERF_F26().write(0x96)?;
    radio.PLLLOOP().write(PLLLoop {
        filter: FLT::INTERNAL_x5,
        flags: PLLLoopFlags::DIRECT,
        freqsel: FreqSel::A,
    })?;
    radio.PLLCPI().write(0x10)?;

    let params = RXParameters::MSK {
        max_dr_offset: 0, // TODO derived from what?
        freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    };
    params.write(radio, &board, &channel)?;

    let set0 = RXParameterSet {
        agc: RXParameterAGC {
            attack: 0x3,
            decay: 0x8,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xA,
                exponent: 9,
            },
            datarate: DRGain {
                mantissa: 0xA,
                exponent: 3,
            },
            phase: 0b0011,
            filter: 0b10,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_0101,
                freq: 0b0_0101,
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
        agc: RXParameterAGC {
            attack: 0x3,
            decay: 0x8,
            target: 0x84,
            ahyst: 0,
            min: 0,
            max: 0,
        },
        gain: RXParameterGain {
            time: TimeGain {
                mantissa: 0xA,
                exponent: 7,
            },
            datarate: DRGain {
                mantissa: 0xA,
                exponent: 2,
            },
            phase: 0b0011,
            filter: 0b10,
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
                mantissa: 0xA,
                exponent: 6,
            },
            datarate: DRGain {
                mantissa: 0xA,
                exponent: 1,
            },
            phase: 0b0011,
            filter: 0b10,
            baseband: RXParameterFreq {
                phase: 0b1111,
                freq: 0b1_1111,
            },
            rf: RXParameterFreq {
                phase: 0b0_1001,
                freq: 0b0_1001,
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

    radio.MATCH1PAT().write(0x7E)?;
    radio.MATCH1LEN().write(MatchLen {
        len: 0xA,
        raw: false,
    })?;
    radio.MATCH1MAX().write(0xA)?;
    radio.TMGRXPREAMBLE2().write(TMG { m: 0x17, e: 0 })?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x09)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.PKTADDRCFG().write(PktAddrCfg {
        addr_pos: 0,
        flags: PktAddrCfgFlags::MSB_FIRST | PktAddrCfgFlags::FEC_SYNC_DIS,
    })?;

    radio.RSSIREFERENCE().write(64)?;

    Ok((board, channel))
}

fn read_packet(radio: &mut Registers, packet: &mut Vec<u8>, uplink: &mut UdpSocket) -> Result<()> {
    let len = radio.FIFOCOUNT().read()?;
    if len <= 0 {
        return Ok(())
    }

    for chunk in radio.FIFODATARX().read(len.into())? {
        if let FIFOChunkRX::DATA{flags, ref data} = chunk {
            println!("{:02X?}", chunk);
            if flags.intersects(FIFODataRXFlags::ABORT | FIFODataRXFlags::SIZEFAIL | FIFODataRXFlags::ADDRFAIL | FIFODataRXFlags::CRCFAIL | FIFODataRXFlags::RESIDUE) {
                packet.clear();
                continue;
            }

            if flags.contains(FIFODataRXFlags::PKTSTART) {
                packet.clear();
            }
            packet.write(&data)?;
            if flags.contains(FIFODataRXFlags::PKTEND) {
                let bytes = packet.split_off(packet.len()-2);
                let checksum = u16::from_be_bytes([bytes[0], bytes[1]]);
                let ccitt = Crc::<u16>::new(&CRC_16_GENIBUS);
                let mut digest = ccitt.digest();
                digest.update(packet);
                let calculated = digest.finalize();

                if calculated == checksum {
                    uplink.send(packet)?;
                    println!("{:02X?}", packet);
                } else {
                    println!("Rejected CRC: received 0x{:x}, calculated 0x{:x}", checksum, calculated);
                }
            }
        }
    }
    Ok(())
}


fn transmit(radio: &mut Registers, buf: &[u8], amt: usize) -> Result<()> {

    // TODO Datarate
    // TODO bitorder

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    let thresh = radio.FIFOTHRESH().read()?.into();

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

    let pa_off = FIFOChunkTX::TXCTRL(TXCtrl::SETPA);

    radio.FIFODATATX().write(pa_on)?;
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

    let addr = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), args.beacon);
    let mut beacon = UdpSocket::bind(addr)?;
    const BEACON: Token = Token(0);
    registry.register(&mut beacon, BEACON, Interest::READABLE)?;

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
    let dest = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), args.uplink);
    let mut uplink = UdpSocket::bind(src)?;
    uplink.connect(dest)?;

    const SIGINT: Token = Token(3);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, SIGINT, Interest::READABLE)?;

    let chip1 = Chip::new("gpiochip1")?;
    let opts = Options::output([27]).values([false]);
    let pa_enable = chip1.request_lines(opts)?;

    let chip0 = Chip::new("gpiochip0")?;
    let opts = Options::input([30]).edge(EdgeDetect::Rising);
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

    configure_radio_rx(&mut radio, args.power)?;
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
                BEACON => {

                    radio.IRQMASK().write(ax5043::registers::IRQ::empty())?;
                    // See errata - PWRMODE must transition through off for FIFO to work
                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::POWEROFF,
                    })?;

                    let mut buf = [0; 2048];
                    let (amt, src) = beacon.recv_from(&mut buf)?;
                    println!("Recv {} from {}: {:X?}", amt, src, &buf[..amt]);
                    transmit(&mut radio, &buf, amt)?;
                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::RX,
                    })?;
                    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD
                    radio.IRQMASK().write(ax5043::registers::IRQ::FIFONOTEMPTY)?;
                }
                IRQ => {
                    uhf_irq.read_event()?;
                    while uhf_irq.get_values(0u8)? > 0 {
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

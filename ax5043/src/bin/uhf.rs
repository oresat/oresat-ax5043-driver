// Intended to be run on the C3v6, takes data from UDP port 10015
// and transmits it through the UHF AX5043
use anyhow::{bail, ensure, Context, Result};
use ax5043::{config, config::*};
use ax5043::{registers, registers::*, Registers, RX, TX};
use clap::Parser;
use crc::{Crc, CRC_16_GENIBUS}; // TODO: this CRC works but is it correct?
use gpiod::{Chip, EdgeDetect, Options};
use mio::net::UdpSocket;
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use std::{
    io::{ErrorKind, Write},
    net::{IpAddr, Ipv4Addr, SocketAddr},
    os::fd::AsRawFd,
};

fn configure_radio(radio: &mut Registers) -> Result<(Board, ChannelParameters)> {
    let board = config::board::C3_UHF.write(radio)?;
    let synth = config::synth::UHF_436_5.write(radio, &board)?;
    let channel = config::channel::GMSK_96000.write(radio, &board)?;

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

    let rxp = RXParameters::MSK {
        max_dr_offset: 0,
        freq_offs_corr: true,
        ampl_filter: 0,
        frequency_leak: 0,
    }
    .write(radio, &board, &channel)?;

    // TODO: see note table 96: RXDATARATE - TIMEGAINx ≥ 2^12 should be ensured
    let set0 = RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time_corr_frac: 4,
            datarate_corr_frac: 255,
            phase: 0b0011,
            filter: 0b10,
            baseband: Some(RXParameterFreq {
                phase: 0x05,
                freq: 0x05,
            }),
            rf: None,
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: None,
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set0.write0(radio, &board, &channel, &rxp)?;

    let set1 = RXParameterSet {
        agc: RXParameterAGC::new(&board, &channel),
        gain: RXParameterGain {
            time_corr_frac: 16,
            datarate_corr_frac: 512,
            phase: 0b0011,
            filter: 0b10,
            baseband: Some(RXParameterFreq {
                phase: 0x05,
                freq: 0x05,
            }),
            rf: None,
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: Some(0x32),
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set1.write1(radio, &board, &channel, &rxp)?;

    let set3 = RXParameterSet {
        agc: RXParameterAGC::off(),
        gain: RXParameterGain {
            time_corr_frac: 32,
            datarate_corr_frac: 1024,
            phase: 0b0011,
            filter: 0b10,
            baseband: Some(RXParameterFreq {
                phase: 0x09,
                freq: 0x09,
            }),
            rf: None,
            amplitude: 0b0110,
            deviation_update: true,
            ampl_agc_jump_correction: false,
            ampl_averaging: false,
        },
        freq_dev: Some(0x32),
        decay: 0b0110,
        baseband_offset: RXParameterBasebandOffset { a: 0, b: 0 },
    };
    set3.write3(radio, &board, &channel, &rxp)?;

    // TODO: set timeout (TMGRXPREAMBLEx) off of expected bitrate + preamble length?
    RXParameterStages {
        preamble1: Some(Preamble1 {
            pattern: PatternMatch1 {
                pat: 0x1111,
                len: 15,
                raw: false,
                min: 0,
                max: 15,
            },
            timeout: Float5 { m: 0x17, e: 5 },
            set: RxParamSet::Set0,
        }),
        preamble2: Some(Preamble2 {
            pattern: PatternMatch0 {
                pat: 0x1111_1111,
                len: 31,
                raw: false,
                min: 0,
                max: 31,
            },
            timeout: Float5 { m: 0x17, e: 2 },
            set: RxParamSet::Set1,
        }),
        preamble3: None,
        packet: RxParamSet::Set3,
    }
    .write(radio)?;

    radio.PKTMAXLEN().write(0xFF)?;
    radio.PKTLENCFG().write(PktLenCfg { pos: 0, bits: 0xF })?;
    radio.PKTLENOFFSET().write(0x09)?;

    radio.PKTCHUNKSIZE().write(0x09)?;
    radio.PKTACCEPTFLAGS().write(PktAcceptFlags::LRGP)?;

    radio.RSSIREFERENCE().write(32)?;

    Ok((board, channel))
}

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
    #[arg(short, long, default_value_t = 0x700)]
    power: u16,
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
    //let mut status = ax5043::Status::empty();
    //let mut callback = |_: &_, addr, s, data: &_| {
    //    if s != status {
    //        println!("TX Status change: {:?}", s);
    //        status = s;
    //    }
    //    println!("{:04X}: {:?}", addr, data);
    //};
    let mut callback = |_: &_, _, _, _: &_| {};

    let mut radio = ax5043::Registers::new(spi0, &mut callback);
    radio.reset()?;

    let rev = radio.REVISION().read()?;
    ensure!(
        rev == 0x51,
        "Unexpected revision {}, expected {}",
        rev,
        0x51
    );

    let (board, _) = configure_radio_rx(&mut radio)?;
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

    radio
        .IRQMASK()
        .write(ax5043::registers::IRQ::FIFONOTEMPTY)?;

    let mut packet = Vec::new();

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
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

                    let channel = config::channel::GMSK_9600_LSB.write(&mut radio, &board)?;

                    TXParameters {
                        antenna: Antenna::SingleEnded,
                        amp: AmplitudeShaping::RaisedCosine {
                            a: 0,
                            b: args.power,
                            //b: 0xFFF,
                            c: 0,
                            d: 0,
                            e: 0,
                        },
                        plllock_gate: true,
                        brownout_gate: true,
                    }
                    .write(&mut radio, &board, &channel)?;

                    let mut buf = [0; 2048];
                    loop {
                        match beacon.recv_from(&mut buf) {
                            Ok((amt, src)) => transmit(&mut radio, &buf[..amt], src)?,
                            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                            Err(e) => return Err(e).context("Ping socket read failed"),
                        }
                    }

                    config::channel::GMSK_96000.write(&mut radio, &board)?;

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

                    let channel = config::channel::GMSK_96000.write(&mut radio, &board)?;

                    TXParameters {
                        antenna: Antenna::SingleEnded,
                        amp: AmplitudeShaping::RaisedCosine {
                            a: 0,
                            b: args.power,
                            c: 0,
                            d: 0,
                            e: 0,
                        },
                        plllock_gate: true,
                        brownout_gate: true,
                    }
                    .write(&mut radio, &board, &channel)?;

                    let mut buf = [0; 2048];
                    loop {
                        match downlink.recv_from(&mut buf) {
                            Ok((amt, src)) => transmit(&mut radio, &buf[..amt], src)?,
                            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                            Err(e) => return Err(e).context("Downlink socket read failed"),
                        }
                    }

                    config::channel::GMSK_96000.write(&mut radio, &board)?;
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

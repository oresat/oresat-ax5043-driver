use crate::{config, registers::*, Registers, Status, RX};
use anyhow::Result;
use bitflags::Flags;
use ciborium;
use ratatui::{
    prelude::*,
    style::Style,
    widgets::{Block, Borders, Cell, Row, Table},
};
use serde::{Deserialize, Serialize};
use std::{io::ErrorKind, net::UdpSocket};

#[derive(Debug, Serialize, Deserialize)]
pub enum CommState {
    RX(FIFOChunkRX),
    //ERR(Result<()>),
    STATUS(Status),
    STATE(RXState),
    REGISTERS(StatusRegisters),
    BOARD(config::Board),
    CONFIG(Config),
}

impl CommState {
    pub fn send(&self, socket: &UdpSocket) -> Result<()> {
        let mut buf = Vec::<u8>::new();
        ciborium::ser::into_writer(self, &mut buf)?;
        if let Err(e) = socket.send(&buf) {
            match e.kind() {
                ErrorKind::ConnectionRefused => Ok(()),
                _ => Err(e)
            }?
        }
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct StatusRegisters {
    pub ranginga: PLLRanging,
    pub pwrmode: PwrMode,
    pub powstat: PowStat,
    pub irq: IRQ,
    pub radio_event: RadioEvent,
    pub radio_state: RadioState,
}

impl StatusRegisters {
    pub fn new(radio: &mut Registers) -> Result<Self> {
        Ok(Self {
            ranginga: radio.PLLRANGINGA().read()?, // sticky lock bit ~ IRQPLLUNLIOCK, gate
            pwrmode: radio.PWRMODE().read()?,
            powstat: radio.POWSTAT().read()?,
            irq: radio.IRQREQUEST().read()?,
            radio_event: radio.RADIOEVENTREQ().read()?,
            radio_state: radio.RADIOSTATE().read()?,
        })
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub rxparams: RXParams,
    pub set0: RXParameterSet,
    pub set1: RXParameterSet,
    pub set2: RXParameterSet,
    pub set3: RXParameterSet,
    pub synthesizer: Synthesizer,
    pub packet_controller: PacketController,
    pub packet_format: PacketFormat,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RXState {
    pub rssi: f64,
    pub agccounter: f64,
    pub datarate: f64,
    pub ampl: f64,
    pub phase: f64,
    pub fskdemod: f64,
    pub rffreq: f64,
    pub freq: f64,
    pub paramcurset: RxParamCurSet,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PacketFormat {
    addrcfg: PktAddrCfg,
    lencfg: PktLenCfg,
    lenoffset: u8,
    maxlen: u8,
    addr: u32,
    addrmask: u32,
}

impl Default for PacketFormat {
    fn default() -> Self {
        Self {
            addrcfg: PktAddrCfg {
                addr_pos: 0,
                flags: PktAddrCfgFlags::empty(),
            },
            lencfg: PktLenCfg { pos: 0, bits: 0 },
            lenoffset: 0,
            maxlen: 0,
            addr: 0,
            addrmask: 0,
        }
    }
}

impl PacketFormat {
    pub fn new(radio: &mut Registers) -> Result<PacketFormat> {
        Ok(PacketFormat {
            addrcfg: radio.PKTADDRCFG().read()?,
            lencfg: radio.PKTLENCFG().read()?,
            lenoffset: radio.PKTLENOFFSET().read()?,
            maxlen: radio.PKTMAXLEN().read()?,
            addr: radio.PKTADDR().read()?,
            addrmask: radio.PKTADDRMASK().read()?,
        })
    }

    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(
            vec![
                Row::new(vec![
                    Cell::from("Addr pos"),
                    Cell::from("Addr flags"),
                    Cell::from("len pos"),
                    Cell::from("len bits"),
                ]),
                Row::new(vec![
                    Cell::from(format!("{}", self.addrcfg.addr_pos)),
                    Cell::from(format!("{:?}", self.addrcfg.flags)),
                    Cell::from(self.lencfg.pos.to_string()),
                    Cell::from(self.lencfg.bits.to_string()),
                ]),
                Row::new(vec![
                    Cell::from("Addr"),
                    Cell::from("Mask"),
                    Cell::from("len off"),
                    Cell::from("len max"),
                ]),
                Row::new(vec![
                    Cell::from(self.addr.to_string()),
                    Cell::from(self.addrmask.to_string()),
                    Cell::from(self.lenoffset.to_string()),
                    Cell::from(self.maxlen.to_string()),
                ]),
            ],
            [
                Constraint::Max(10),
                Constraint::Min(30),
                Constraint::Min(10),
                Constraint::Min(10),
            ],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title("Packet Format"),
        )
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PacketController {
    tmg_tx_boost: Float5,
    tmg_tx_settle: Float5,
    tmg_rx_boost: Float5,
    tmg_rx_settle: Float5,
    tmg_rx_offsacq: Float5,
    tmg_rx_coarseagc: Float5,
    tmg_rx_agc: Float5,
    tmg_rx_rssi: Float5,
    tmg_rx_preamble1: Float5,
    tmg_rx_preamble2: Float5,
    tmg_rx_preamble3: Float5,
    rssi_reference: i8,
    rssi_abs_thr: u8,
    bgnd_rssi_gain: u8,
    bgnd_rssi_thr: u8,
    pkt_chunk_size: u8,
    pkt_misc_flags: PktMiscFlags,
    pkt_store_flags: PktStoreFlags,
    pkt_accept_flags: PktAcceptFlags,
}

impl Default for PacketController {
    fn default() -> Self {
        Self {
            tmg_tx_boost: Float5::new(0),
            tmg_tx_settle: Float5::new(0),
            tmg_rx_boost: Float5::new(0),
            tmg_rx_settle: Float5::new(0),
            tmg_rx_offsacq: Float5::new(0),
            tmg_rx_coarseagc: Float5::new(0),
            tmg_rx_agc: Float5::new(0),
            tmg_rx_rssi: Float5::new(0),
            tmg_rx_preamble1: Float5::new(0),
            tmg_rx_preamble2: Float5::new(0),
            tmg_rx_preamble3: Float5::new(0),
            rssi_reference: 0,
            rssi_abs_thr: 0,
            bgnd_rssi_gain: 0,
            bgnd_rssi_thr: 0,
            pkt_chunk_size: 0,
            pkt_misc_flags: PktMiscFlags::empty(),
            pkt_store_flags: PktStoreFlags::empty(),
            pkt_accept_flags: PktAcceptFlags::empty(),
        }
    }
}

impl PacketController {
    pub fn new(radio: &mut Registers) -> Result<PacketController> {
        Ok(PacketController {
            tmg_tx_boost: radio.TMGTXBOOST().read()?,
            tmg_tx_settle: radio.TMGTXSETTLE().read()?,
            tmg_rx_boost: radio.TMGRXBOOST().read()?,
            tmg_rx_settle: radio.TMGRXSETTLE().read()?,
            tmg_rx_offsacq: radio.TMGRXOFFSACQ().read()?,
            tmg_rx_coarseagc: radio.TMGRXCOARSEAGC().read()?,
            tmg_rx_agc: radio.TMGRXAGC().read()?,
            tmg_rx_rssi: radio.TMGRXRSSI().read()?,
            tmg_rx_preamble1: radio.TMGRXPREAMBLE1().read()?,
            tmg_rx_preamble2: radio.TMGRXPREAMBLE2().read()?,
            tmg_rx_preamble3: radio.TMGRXPREAMBLE3().read()?,
            rssi_reference: radio.RSSIREFERENCE().read()?,
            rssi_abs_thr: radio.RSSIABSTHR().read()?,
            bgnd_rssi_gain: radio.BGNDRSSIGAIN().read()?,
            bgnd_rssi_thr: radio.BGNDRSSITHR().read()?,
            pkt_chunk_size: radio.PKTCHUNKSIZE().read()?,
            pkt_misc_flags: radio.PKTMISCFLAGS().read()?,
            pkt_store_flags: radio.PKTSTOREFLAGS().read()?,
            pkt_accept_flags: radio.PKTACCEPTFLAGS().read()?,
        })
    }

    pub fn widget(&self, f: &mut Frame, area: Rect) {
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .margin(0)
            .constraints([Constraint::Min(4), Constraint::Min(2)].as_ref())
            .split(area);

        let upper = Table::new(
            vec![
                Row::new(vec![
                    Cell::from("TX Boost"),
                    Cell::from("Settle"),
                    Cell::from("RX Boost"),
                    Cell::from("Settle"),
                    Cell::from("ACQ Offs"),
                    Cell::from("Coarse AGC"),
                    Cell::from("AGC"),
                    Cell::from("RSSI"),
                    Cell::from("Pmbl 1"),
                    Cell::from("2"),
                    Cell::from("3"),
                ]),
                Row::new(vec![
                    Cell::from(self.tmg_tx_boost.to_string()),
                    Cell::from(self.tmg_tx_settle.to_string()),
                    Cell::from(self.tmg_rx_boost.to_string()),
                    Cell::from(self.tmg_rx_settle.to_string()),
                    Cell::from(self.tmg_rx_offsacq.to_string()),
                    Cell::from(self.tmg_rx_coarseagc.to_string()),
                    Cell::from(self.tmg_rx_agc.to_string()),
                    Cell::from(self.tmg_rx_rssi.to_string()),
                    Cell::from(self.tmg_rx_preamble1.to_string()),
                    Cell::from(self.tmg_rx_preamble2.to_string()),
                    Cell::from(self.tmg_rx_preamble3.to_string()),
                ]),
                Row::new(vec![
                    Cell::from("RSSI Ref"),
                    Cell::from("Abs Thr"),
                    Cell::from("BGND Gain"),
                    Cell::from("BGND thr"),
                    Cell::from("Chunk"),
                ]),
                Row::new(vec![
                    Cell::from(self.rssi_reference.to_string()),
                    Cell::from(self.rssi_abs_thr.to_string()),
                    Cell::from(self.bgnd_rssi_gain.to_string()),
                    Cell::from(self.bgnd_rssi_thr.to_string()),
                    Cell::from(self.pkt_chunk_size.to_string()),
                ]),
            ],
            [
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(10),
                Constraint::Max(6),
                Constraint::Max(6),
                Constraint::Max(6),
            ],
        );

        let lower = Table::new(
            vec![
                Row::new(vec![
                    Cell::from("Misc Flag"),
                    Cell::from("Store Flg"),
                    Cell::from("Acc Flags"),
                ]),
                Row::new(vec![
                    Cell::from(format!("{:?}", self.pkt_misc_flags)),
                    Cell::from(format!("{:?}", self.pkt_store_flags)),
                    Cell::from(format!("{:?}", self.pkt_accept_flags)),
                ]),
            ],
            [
                Constraint::Max(30),
                Constraint::Max(30),
                Constraint::Max(50),
            ],
        );

        f.render_widget(upper, layout[0]);
        f.render_widget(lower, layout[1]);
        //FIXME: border?
        // .block(
        //      Block::default()
        //          .borders(Borders::ALL)
        //          .title("Packet Controller"),
        //  )
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Synthesizer {
    freqa: u64,
    freqb: u64,
    pllloop: PLLLoop,
    pllloopboost: PLLLoop,
    cpi: u8,
    cpiboost: u8,
    vcodiv: PLLVCODiv,
    ranginga: PLLRanging,
    rangingb: PLLRanging,
}

impl Default for Synthesizer {
    fn default() -> Self {
        Self {
            freqa: 0,
            freqb: 0,
            pllloop: PLLLoop {
                filter: FLT::EXTERNAL,
                flags: PLLLoopFlags::empty(),
                freqsel: FreqSel::A,
            },
            pllloopboost: PLLLoop {
                filter: FLT::EXTERNAL,
                flags: PLLLoopFlags::empty(),
                freqsel: FreqSel::A,
            },
            cpi: 0,
            cpiboost: 0,
            vcodiv: PLLVCODiv {
                mode: PLLVCORefDiv::F_XTAL,
                flags: PLLVCODivFlags::empty(),
            },
            ranginga: PLLRanging {
                vcor: 0,
                flags: PLLRangingFlags::empty(),
            },
            rangingb: PLLRanging {
                vcor: 0,
                flags: PLLRangingFlags::empty(),
            },
        }
    }
}

impl Synthesizer {
    pub fn new(radio: &mut Registers, board: &config::Board) -> Result<Synthesizer> {
        Ok(Synthesizer {
            pllloop: radio.PLLLOOP().read()?,
            pllloopboost: radio.PLLLOOPBOOST().read()?,
            cpi: radio.PLLCPI().read()?,
            cpiboost: radio.PLLCPIBOOST().read()?,
            vcodiv: radio.PLLVCODIV().read()?,
            ranginga: radio.PLLRANGINGA().read()?,
            rangingb: radio.PLLRANGINGB().read()?,
            freqa: u64::from(radio.FREQA().read()?) * board.xtal.freq / 2_u64.pow(24),
            freqb: u64::from(radio.FREQB().read()?) * board.xtal.freq / 2_u64.pow(24),
        })
    }

    pub fn widget<'a>(&self) -> Table<'a> {
        /* pllloop / boost [ b direct filten filt ]
         * pllcpi / boost [ cpi ]
         * pllvcodiv [ VCOI_MAN VCO2INT VCOSEL  RFDIV REFDIV ]
         * pllranging A/B [ sticky lock err start vcor ]
         * freq a/b
         */

        Table::new(
            vec![
                Row::new(vec![
                    Cell::from(""),
                    Cell::from("Freq"),
                    Cell::from("Ranging"),
                ]),
                Row::new(vec![
                    Cell::from("A"),
                    Cell::from(format!("{} Hz", self.freqa)),
                    Cell::from(format!("{:?}", self.ranginga)),
                ]),
                Row::new(vec![
                    Cell::from("B"),
                    Cell::from(format!("{} Hz", self.freqb)),
                    Cell::from(format!("{:?}", self.rangingb)),
                ]),
                Row::new(vec![Cell::from(""), Cell::from("CPI"), Cell::from("Loop")]),
                Row::new(vec![
                    Cell::from("PLL"),
                    Cell::from(format!("{:?}", self.cpi)),
                    Cell::from(format!("{:?}", self.pllloop)),
                ]),
                Row::new(vec![
                    Cell::from("Boost"),
                    Cell::from(format!("{:?}", self.cpiboost)),
                    Cell::from(format!("{:?}", self.pllloopboost)),
                ]),
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(format!("{:?}", self.vcodiv)),
                ]),
            ],
            [Constraint::Max(5), Constraint::Max(13), Constraint::Max(80)],
        )
        .block(Block::default().borders(Borders::ALL).title("Synthesizer"))
    }
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct RXParameterAGC {
    attack: u8,
    decay: u8,
    target: u8,
    ahyst: u8,
    min: u8,
    max: u8,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct RXParameterFreq {
    phase: u8,
    freq: u8,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct RXParameterGain {
    time: u8,
    rate: u8,
    phase: u8,
    filter: u8,
    baseband: RXParameterFreq,
    rf: RXParameterFreq,
    amplitude: u8,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct RXParameterBasebandOffset {
    a: u8,
    b: u8,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct RXParameterSet {
    agc: RXParameterAGC,
    gain: RXParameterGain,
    freq_dev: u16,
    decay: u8,
    baseband_offset: RXParameterBasebandOffset,
}

impl RXParameterSet {
    pub fn set0(radio: &mut Registers) -> Result<RXParameterSet> {
        let agcgain = radio.AGCGAIN0().read()?;
        let agcmina = radio.AGCMINMAX0().read()?;
        let timegan = radio.TIMEGAIN0().read()?;
        let dargain = radio.DRGAIN0().read()?;
        let phasegn = radio.PHASEGAIN0().read()?;
        let bboffsr = radio.BBOFFSRES0().read()?;
        Ok(RXParameterSet {
            agc: RXParameterAGC {
                attack: agcgain.attack,
                decay: agcgain.decay,
                target: radio.AGCTARGET0().read()?,
                ahyst: radio.AGCAHYST0().read()?.hyst,
                min: agcmina.min,
                max: agcmina.max,
            },
            gain: RXParameterGain {
                time: timegan.e,
                rate: dargain.e,
                phase: phasegn.gain,
                filter: phasegn.filter,
                baseband: RXParameterFreq {
                    phase: radio.FREQGAINA0().read()?.gain,
                    freq: radio.FREQGAINB0().read()?.gain,
                },
                rf: RXParameterFreq {
                    phase: radio.FREQGAINC0().read()?.gain,
                    freq: radio.FREQGAIND0().read()?.gain,
                },
                amplitude: radio.AMPLGAIN0().read()?.gain,
            },
            freq_dev: radio.FREQDEV0().read()?,
            decay: radio.FOURFSK0().read()?.decay,
            baseband_offset: RXParameterBasebandOffset {
                a: bboffsr.res_int_a,
                b: bboffsr.res_int_b,
            },
        })
    }

    pub fn set1(radio: &mut Registers) -> Result<RXParameterSet> {
        let agcgain = radio.AGCGAIN1().read()?;
        let agcmina = radio.AGCMINMAX1().read()?;
        let timegan = radio.TIMEGAIN1().read()?;
        let dargain = radio.DRGAIN1().read()?;
        let phasegn = radio.PHASEGAIN1().read()?;
        let bboffsr = radio.BBOFFSRES1().read()?;
        Ok(RXParameterSet {
            agc: RXParameterAGC {
                attack: agcgain.attack,
                decay: agcgain.decay,
                target: radio.AGCTARGET1().read()?,
                ahyst: radio.AGCAHYST1().read()?.hyst,
                min: agcmina.min,
                max: agcmina.max,
            },
            gain: RXParameterGain {
                time: timegan.e,
                rate: dargain.e,
                phase: phasegn.gain,
                filter: phasegn.filter,
                baseband: RXParameterFreq {
                    phase: radio.FREQGAINA1().read()?.gain,
                    freq: radio.FREQGAINB1().read()?.gain,
                },
                rf: RXParameterFreq {
                    phase: radio.FREQGAINC1().read()?.gain,
                    freq: radio.FREQGAIND1().read()?.gain,
                },
                amplitude: radio.AMPLGAIN1().read()?.gain,
            },
            freq_dev: radio.FREQDEV1().read()?,
            decay: radio.FOURFSK1().read()?.decay,
            baseband_offset: RXParameterBasebandOffset {
                a: bboffsr.res_int_a,
                b: bboffsr.res_int_b,
            },
        })
    }

    pub fn set2(radio: &mut Registers) -> Result<RXParameterSet> {
        let agcgain = radio.AGCGAIN2().read()?;
        let agcmina = radio.AGCMINMAX2().read()?;
        let timegan = radio.TIMEGAIN2().read()?;
        let dargain = radio.DRGAIN2().read()?;
        let phasegn = radio.PHASEGAIN2().read()?;
        let bboffsr = radio.BBOFFSRES2().read()?;
        Ok(RXParameterSet {
            agc: RXParameterAGC {
                attack: agcgain.attack,
                decay: agcgain.decay,
                target: radio.AGCTARGET2().read()?,
                ahyst: radio.AGCAHYST2().read()?.hyst,
                min: agcmina.min,
                max: agcmina.max,
            },
            gain: RXParameterGain {
                time: timegan.e,
                rate: dargain.e,
                phase: phasegn.gain,
                filter: phasegn.filter,
                baseband: RXParameterFreq {
                    phase: radio.FREQGAINA2().read()?.gain,
                    freq: radio.FREQGAINB2().read()?.gain,
                },
                rf: RXParameterFreq {
                    phase: radio.FREQGAINC2().read()?.gain,
                    freq: radio.FREQGAIND2().read()?.gain,
                },
                amplitude: radio.AMPLGAIN2().read()?.gain,
            },
            freq_dev: radio.FREQDEV2().read()?,
            decay: radio.FOURFSK2().read()?.decay,
            baseband_offset: RXParameterBasebandOffset {
                a: bboffsr.res_int_a,
                b: bboffsr.res_int_b,
            },
        })
    }

    pub fn set3(radio: &mut Registers) -> Result<RXParameterSet> {
        let agcgain = radio.AGCGAIN3().read()?;
        let agcmina = radio.AGCMINMAX3().read()?;
        let timegan = radio.TIMEGAIN3().read()?;
        let dargain = radio.DRGAIN3().read()?;
        let phasegn = radio.PHASEGAIN3().read()?;
        let bboffsr = radio.BBOFFSRES3().read()?;
        Ok(RXParameterSet {
            agc: RXParameterAGC {
                attack: agcgain.attack,
                decay: agcgain.decay,
                target: radio.AGCTARGET3().read()?,
                ahyst: radio.AGCAHYST3().read()?.hyst,
                min: agcmina.min,
                max: agcmina.max,
            },
            gain: RXParameterGain {
                time: timegan.e,
                rate: dargain.e,
                phase: phasegn.gain,
                filter: phasegn.filter,
                baseband: RXParameterFreq {
                    phase: radio.FREQGAINA3().read()?.gain,
                    freq: radio.FREQGAINB3().read()?.gain,
                },
                rf: RXParameterFreq {
                    phase: radio.FREQGAINC3().read()?.gain,
                    freq: radio.FREQGAIND3().read()?.gain,
                },
                amplitude: radio.AMPLGAIN3().read()?.gain,
            },
            freq_dev: radio.FREQDEV3().read()?,
            decay: radio.FOURFSK3().read()?.decay,
            baseband_offset: RXParameterBasebandOffset {
                a: bboffsr.res_int_a,
                b: bboffsr.res_int_b,
            },
        })
    }

    pub fn widget<'a>(&self, active: &Style) -> Table<'a> {
        Table::new(
            vec![
                Row::new(vec![
                    "AGC", "attack", "decay", "target", "ahyst", "min", "max",
                ]),
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(self.agc.attack.to_string()),
                    Cell::from(self.agc.decay.to_string()),
                    Cell::from(self.agc.target.to_string()),
                    Cell::from(self.agc.ahyst.to_string()),
                    Cell::from(self.agc.min.to_string()),
                    Cell::from(self.agc.max.to_string()),
                ]),
                Row::new(vec![
                    "Gain",
                    "time",
                    "rate",
                    "phase",
                    "filt",
                    "BB gain phase",
                    "BB gain freq",
                    "RF gain freq",
                    "RF gain phase",
                    "ampl",
                ]),
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(self.gain.time.to_string()),
                    Cell::from(self.gain.rate.to_string()),
                    Cell::from(self.gain.phase.to_string()),
                    Cell::from(self.gain.filter.to_string()),
                    Cell::from(self.gain.baseband.freq.to_string()),
                    Cell::from(self.gain.baseband.phase.to_string()),
                    Cell::from(self.gain.rf.freq.to_string()),
                    Cell::from(self.gain.rf.phase.to_string()),
                    Cell::from(self.gain.amplitude.to_string()),
                ]),
                Row::new(vec!["", "freq dev", "decay", "BBR block A", "BBR block B"]),
                Row::new(vec![
                    Cell::from(""),
                    Cell::from(self.freq_dev.to_string()),
                    Cell::from(self.decay.to_string()),
                    Cell::from(self.baseband_offset.a.to_string()),
                    Cell::from(self.baseband_offset.b.to_string()),
                ]),
            ],
            [
                Constraint::Max(5),
                Constraint::Max(8),
                Constraint::Max(9),
                Constraint::Max(10),
                Constraint::Max(14),
                Constraint::Max(15),
                Constraint::Max(9),
                Constraint::Max(10),
                Constraint::Max(8),
                Constraint::Max(13),
                Constraint::Max(11),
                Constraint::Max(16),
                Constraint::Max(16),
                Constraint::Max(15),
                Constraint::Max(15),
            ],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title("RX Parameter Set"),
        )
        .style(*active)
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RXParams {
    pub iffreq: u64,
    pub baseband: u64,
    pub bitrate: u64,
    pub maxdroffset: u64,
    pub maxrfoffset: u64,
    pub fskdevmax: u64,
    pub fskdevmin: i64,
    pub afskspace: u16,
    pub afskmark: u16,
    pub afskctrl: u8,
    pub amplfilt: u8,
    pub freqleak: u8,
    pub rxparamset: RxParamSets,
}

impl Default for RXParams {
    fn default() -> Self {
        Self {
            iffreq: 0,
            baseband: 0,
            bitrate: 0,
            maxdroffset: 0,
            maxrfoffset: 0,
            fskdevmax: 0,
            fskdevmin: 0,
            afskspace: 0,
            afskmark: 0,
            afskctrl: 0,
            amplfilt: 0,
            freqleak: 0,
            rxparamset: RxParamSets(
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
                RxParamSet::Set0,
            ),
        }
    }
}

impl RXParams {
    pub fn new(radio: &mut Registers, board: &config::Board) -> Result<RXParams> {
        let decimation = u64::from(radio.DECIMATION().read()?);
        let rxdatarate = u64::from(radio.RXDATARATE().read()?);
        let bitrate = board.xtal.freq * 2_u64.pow(7) / (rxdatarate * decimation * board.xtal.div());

        Ok(RXParams {
            iffreq: u64::from(radio.IFFREQ().read()?) * board.xtal.freq
                / board.xtal.div()
                / 2_u64.pow(20),
            baseband: board.xtal.freq / (2_u64.pow(4) * board.xtal.div() * decimation),
            bitrate,
            maxdroffset: u64::from(radio.MAXDROFFSET().read()?)
                * board.xtal.div()
                * bitrate
                * bitrate
                * decimation
                / (2_u64.pow(7) * board.xtal.freq),
            maxrfoffset: u64::from(radio.MAXRFOFFSET().read()?.offset) * board.xtal.freq
                / 2_u64.pow(24), // TODO: xtal.div not part of this?
            fskdevmax: u64::from(radio.FSKDMAX().read()?) * bitrate / (3 * 512), // TODO baudrate?
            fskdevmin: i64::from(radio.FSKDMIN().read()?) * i64::try_from(bitrate).unwrap()
                / (-3 * 512), // TODO baudrate?
            afskspace: radio.AFSKSPACE().read()?,
            afskmark: radio.AFSKMARK().read()?,
            afskctrl: radio.AFSKCTRL().read()?,
            amplfilt: radio.AMPLFILTER().read()?,
            freqleak: radio.FREQUENCYLEAK().read()?,
            rxparamset: radio.RXPARAMSETS().read()?,
        })
    }

    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(
            vec![
                Row::new(vec!["IF Freq", "Max Δ", "Baseband", "Bitrate", "Max Δ"]),
                Row::new(vec![
                    Cell::from(format!("{} Hz", self.iffreq)),
                    Cell::from(format!("{} Hz", self.maxrfoffset)),
                    Cell::from(format!("{} Hz", self.baseband)),
                    Cell::from(format!("{} bit/s", self.bitrate)),
                    Cell::from(format!("{} bit/s", self.maxdroffset)),
                ]),
                Row::new(vec!["FSK Dev Max", "Min", "AFSK Space", "Mark", "Ctrl"]),
                Row::new(vec![
                    Cell::from(format!("{} Hz", self.fskdevmax)),
                    Cell::from(format!("{} Hz", self.fskdevmin)),
                    Cell::from(self.afskspace.to_string()),
                    Cell::from(self.afskmark.to_string()),
                    Cell::from(self.afskctrl.to_string()),
                ]),
                Row::new(vec![
                    "Ampl Filt",
                    "Freq Leak",
                    "RX 0",
                    "RX 1",
                    "RX 2",
                    "RX 3",
                ]),
                Row::new(vec![
                    Cell::from(self.amplfilt.to_string()),
                    Cell::from(self.freqleak.to_string()),
                    Cell::from(format!("{:?}", self.rxparamset.0)),
                    Cell::from(format!("{:?}", self.rxparamset.1)),
                    Cell::from(format!("{:?}", self.rxparamset.2)),
                    Cell::from(format!("{:?}", self.rxparamset.3)),
                ]),
            ],
            [
                Constraint::Max(11),
                Constraint::Max(10),
                Constraint::Max(11),
                Constraint::Max(10),
                Constraint::Max(14),
                Constraint::Max(15),
            ],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title("RX Parameters"),
        )
    }
}

// TODO: trait?
fn onoff<F: Flags>(field: &F, flag: F) -> Style {
    if field.contains(flag) {
        Style::default().bg(Color::Gray).fg(Color::Black)
    } else {
        Style::default().fg(Color::DarkGray)
    }
}

impl PwrMode {
    #[rustfmt::skip]
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from(format!("{:?}", self.mode)),
                Cell::from("REFEN").style(onoff(&self.flags, PwrFlags::REFEN)),
                Cell::from("XOEN" ).style(onoff(&self.flags, PwrFlags::XOEN)),
            ]),
        ], [
            Constraint::Length(8),
            Constraint::Length(5),
            Constraint::Length(4),
        ])
        .block(Block::default().borders(Borders::ALL).title("PWRMODE"))
        .style(Style::default().fg(Color::White))
    }
}

impl PowStat {
    #[rustfmt::skip]
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("vIO"     ).style(onoff(self, PowStat::VIO)),
                Cell::from("BEvMODEM").style(onoff(self, PowStat::BEVMODEM)),
                Cell::from("BEvANA"  ).style(onoff(self, PowStat::BEVANA)),
                Cell::from("vMODEM"  ).style(onoff(self, PowStat::VMODEM)),
                Cell::from("vANA"    ).style(onoff(self, PowStat::VANA)),
                Cell::from("vREF"    ).style(onoff(self, PowStat::VREF)),
                Cell::from("REF"     ).style(onoff(self, PowStat::REF)),
                Cell::from("SUM"     ).style(onoff(self, PowStat::SUM))
            ]),
        ], [
            Constraint::Max(3),
            Constraint::Max(8),
            Constraint::Max(6),
            Constraint::Max(6),
            Constraint::Max(4),
            Constraint::Max(4),
            Constraint::Max(3),
            Constraint::Max(3),
        ])
        .block(Block::default().borders(Borders::ALL).title("POWSTAT"))
    }
}

impl IRQ {
    #[rustfmt::skip]
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("PLLUNLOCK"  ).style(onoff(self, IRQ::PLLUNLOCK)),
                Cell::from("RADIOCTRL"  ).style(onoff(self, IRQ::RADIOCTRL)),
                Cell::from("POWER"      ).style(onoff(self, IRQ::POWER)),
                Cell::from("XTALREADY"  ).style(onoff(self, IRQ::XTALREADY)),
                Cell::from("WAKEUPTIMER").style(onoff(self, IRQ::WAKEUPTIMER)),
                Cell::from("LPOSC"      ).style(onoff(self, IRQ::LPOSC)),
                Cell::from("GPADC"      ).style(onoff(self, IRQ::GPADC)),
                Cell::from("PLLRNGDONE" ).style(onoff(self, IRQ::PLLRNGDONE)),
            ]),
            Row::new(vec![
                Cell::from("FIFONOTEMPTY").style(onoff(self, IRQ::FIFONOTEMPTY)),
                Cell::from("FIFONOTFULL" ).style(onoff(self, IRQ::FIFONOTFULL)),
                Cell::from("FIFOTHRCNT"  ).style(onoff(self, IRQ::FIFOTHRCNT)),
                Cell::from("FIFOTHRFREE" ).style(onoff(self, IRQ::FIFOTHRFREE)),
                Cell::from("FIFOERROR"   ).style(onoff(self, IRQ::FIFOERROR)),
            ]),
        ], [
            Constraint::Max(12),
            Constraint::Max(11),
            Constraint::Max(10),
            Constraint::Max(11),
            Constraint::Max(11),
            Constraint::Max(5),
            Constraint::Max(5),
            Constraint::Max(10),
        ])
        .block(Block::default().borders(Borders::ALL).title("IRQ"))
    }
}

impl Status {
    #[rustfmt::skip]
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("READY"           ).style(onoff(self, Status::READY)),
                Cell::from("PLL_LOCK"        ).style(onoff(self, Status::PLL_LOCK)),
                Cell::from("FIFO_OVER"       ).style(onoff(self, Status::FIFO_OVER)),
                Cell::from("FIFO_UNDER"      ).style(onoff(self, Status::FIFO_UNDER)),
                Cell::from("THRESHOLD_FREE"  ).style(onoff(self, Status::THRESHOLD_FREE)),
                Cell::from("THRESHOLD_COUNT" ).style(onoff(self, Status::THRESHOLD_COUNT)),
                Cell::from("FIFO_FULL"       ).style(onoff(self, Status::FIFO_FULL)),
                Cell::from("FIFO_EMPTY"      ).style(onoff(self, Status::FIFO_EMPTY)),
                Cell::from("PWR_GOOD"        ).style(onoff(self, Status::PWR_GOOD)),
                Cell::from("PWR_INTERRUPT"   ).style(onoff(self, Status::PWR_INTERRUPT)),
                Cell::from("RADIO_EVENT"     ).style(onoff(self, Status::RADIO_EVENT)),
                Cell::from("XTAL_OSC_RUNNING").style(onoff(self, Status::XTAL_OSC_RUNNING)),
                Cell::from("WAKEUP_INTERRUPT").style(onoff(self, Status::WAKEUP_INTERRUPT)),
                Cell::from("LPOSC_INTERRUPT" ).style(onoff(self, Status::LPOSC_INTERRUPT)),
                Cell::from("GPADC_INTERRUPT" ).style(onoff(self, Status::GPADC_INTERRUPT)),
            ]),
        ], [
            Constraint::Max(5),
            Constraint::Max(8),
            Constraint::Max(9),
            Constraint::Max(10),
            Constraint::Max(14),
            Constraint::Max(15),
            Constraint::Max(9),
            Constraint::Max(10),
            Constraint::Max(8),
            Constraint::Max(13),
            Constraint::Max(11),
            Constraint::Max(16),
            Constraint::Max(16),
            Constraint::Max(15),
            Constraint::Max(15),
        ])
        .block(Block::default().borders(Borders::ALL).title("STATUS"))
    }
}

impl RadioEvent {
    #[rustfmt::skip]
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("DONE"    ).style(onoff(self, RadioEvent::DONE)),
                Cell::from("SETTLED" ).style(onoff(self, RadioEvent::SETTLED)),
                Cell::from("STATE"   ).style(onoff(self, RadioEvent::RADIOSTATECHG)),
                Cell::from("PARAMSET").style(onoff(self, RadioEvent::RXPARAMSETCHG)),
                Cell::from("FRAMECLK").style(onoff(self, RadioEvent::FRAMECLK)),
            ]),
        ], [
            Constraint::Max(4),
            Constraint::Max(7),
            Constraint::Max(5),
            Constraint::Max(8),
            Constraint::Max(8),
        ])
        .block(Block::default().borders(Borders::ALL).title("Radio Event"))
    }
}

impl RadioState {
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(
            vec![Row::new([Cell::from(format!("{self:?}"))])],
            [Constraint::Max(13)],
        )
        .block(Block::default().borders(Borders::ALL).title("Radio State"))
    }
}

pub struct TXParameters {
    pub modcfgf: ModCfgF,
    pub fskdev: u64,
    pub modcfga: ModCfgA,
    pub txrate: u64,
    pub a: u16,
    pub b: u16,
    pub c: u16,
    pub d: u16,
    pub e: u16,
}

impl TXParameters {
    pub fn new(radio: &mut Registers, board: &config::Board) -> Result<Self> {
        Ok(Self {
            modcfgf: radio.MODCFGF().read()?,
            fskdev: u64::from(radio.FSKDEV().read()?) * board.xtal.freq / 2u64.pow(24),
            modcfga: radio.MODCFGA().read()?,
            txrate: u64::from(radio.TXRATE().read()?) * board.xtal.freq / 2u64.pow(24),
            a: radio.TXPWRCOEFFA().read()?,
            b: radio.TXPWRCOEFFB().read()?,
            c: radio.TXPWRCOEFFC().read()?,
            d: radio.TXPWRCOEFFD().read()?,
            e: radio.TXPWRCOEFFE().read()?,
        })
    }
    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(
            vec![
                Row::new([
                    Cell::from(format!("{:?}", self.modcfgf)),
                    Cell::from(format!("Deviation {:?} Hz", self.fskdev)),
                ]),
                Row::new([
                    Cell::from(format!("{:?}", self.modcfga)),
                    Cell::from(format!("{:?} bits/s", self.txrate)),
                ]),
                Row::new([
                    Cell::from("tx power coef b:"),
                    Cell::from(format!("{:X?}", self.b)),
                ]),
            ],
            [Constraint::Max(100), Constraint::Max(60)],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title("TX Parameters"),
        )
    }
}

impl Default for TXParameters {
    fn default() -> Self {
        Self {
            modcfgf: ModCfgF::UNSHAPED,
            fskdev: 0,
            modcfga: ModCfgA {
                slowramp: SlowRamp::STARTUP_1b,
                flags: ModCfgAFlags::empty(),
            },
            txrate: 0,
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
        }
    }
}

pub struct ChannelParameters {
    pub modulation: Modulation,
    pub encoding: Encoding,
    pub framing: Framing,
    pub crcinit: u32,
}

impl ChannelParameters {
    pub fn new(radio: &mut Registers) -> Result<Self> {
        Ok(Self {
            modulation: radio.MODULATION().read()?,
            encoding: radio.ENCODING().read()?,
            framing: radio.FRAMING().read()?,
            crcinit: radio.CRCINIT().read()?,
        })
    }

    pub fn widget<'a>(&self) -> Table<'a> {
        Table::new(
            vec![
                Row::new([
                    Cell::from(format!("{:?}", self.modulation)),
                    Cell::from(format!("{:?}", self.encoding)),
                ]),
                Row::new([
                    Cell::from(format!("{:?}", self.framing)),
                    Cell::from(format!("{:X?}", self.crcinit)),
                ]),
            ],
            [Constraint::Max(100), Constraint::Max(60)],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title("Channel Parameters"),
        )
    }
}

impl Default for ChannelParameters {
    fn default() -> Self {
        Self {
            modulation: Modulation {
                mode: ModulationMode::ASK,
                halfspeed: false,
            },
            encoding: Encoding::empty(),
            framing: Framing {
                frmmode: FrameMode::RAW,
                crcmode: CRCMode::OFF,
                flags: FramingFlags::empty(),
            },
            crcinit: 0,
        }
    }
}

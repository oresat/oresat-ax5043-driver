use std::{io, cell, time::Duration, os::fd::AsRawFd, collections::VecDeque, panic, backtrace::Backtrace};
use mio::{Events, Poll, unix::SourceFd, Token, Interest};
use mio_signals::{Signal, Signals};
use timerfd::{TimerFd, TimerState, SetTimeFlags};
use ratatui::{
    prelude::*,
    backend::CrosstermBackend,
    widgets::{Block, Borders, Table, Row, Cell, Sparkline},
    Terminal
};
use crossterm::{
    event::{Event, KeyEvent, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};

use bitflags::Flags;
use ax5043::registers::*;
use ax5043::*;
mod config_rpi;
use crate::config_rpi::configure_radio_rx;

// FIXME: Default isn't really the way to go, maybe ::new()?

#[derive(Default)]
struct RXParameterAGC {
    attack: u8,
    decay: u8,
    target: u8,
    ahyst: u8,
    min: u8,
    max: u8,
}

#[derive(Default)]
struct RXParameterFreq {
    phase: u8,
    freq: u8,
}

#[derive(Default)]
struct RXParameterGain {
    time: u8,
    rate: u8,
    phase: u8,
    filter: u8,
    baseband: RXParameterFreq,
    rf: RXParameterFreq,
    amplitude: u8,
}

#[derive(Default)]
struct RXParameterBasebandOffset {
    a: u8,
    b: u8
}

#[derive(Default)]
struct RXParameterSet {
    agc: RXParameterAGC,
    gain: RXParameterGain,
    freq_dev: u16,
    decay: u8,
    baseband_offset: RXParameterBasebandOffset,
}

struct RXParams {
    iffreq: u64,
    baseband: u64,
    bitrate: u64,
    maxdroffset: u64,
    maxrfoffset: u64,
    fskdevmax: u64,
    fskdevmin: i64,
    afskspace: u16,
    afskmark: u16,
    afskctrl: u8,
    amplfilt: u8,
    freqleak: u8,
    rxparamset: RxParamSets,
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

struct UIState {
    on: Style,
    off: Style,
    spark: Style,
    board: config::Board,
    rx: VecDeque<RXState>,
    status: Status,
    pwrmode: PwrMode,
    powstat: PowStat,
    irq: IRQ,
    radio_event: RadioEvent,
    radio_state: RadioState,
    rxparams: RXParams,
    set0 : RXParameterSet,
    set1 : RXParameterSet,
    set2 : RXParameterSet,
    set3 : RXParameterSet,
    synthesizer: Synthesizer,
    packet_controller: PacketController,
    packet_format: PacketFormat,
}

impl Default for UIState {
    fn default() -> Self {
        Self {
            on: Style::default().bg(Color::Gray).fg(Color::Black),
            off: Style::default().fg(Color::DarkGray),
            spark: Style::default().fg(Color::Red).bg(Color::White),
            board: config::Board::default(),
            rx: VecDeque::<RXState>::default(),
            status: Status::empty(),
            pwrmode: PwrMode {
                mode: PwrModes::POWEROFF,
                flags: PwrFlags::empty(),
            },
            powstat: PowStat::empty(),
            irq: IRQ::empty(),
            radio_event: RadioEvent::empty(),
            radio_state: RadioState::IDLE,
            rxparams: RXParams::default(),
            set0 : RXParameterSet::default(),
            set1 : RXParameterSet::default(),
            set2 : RXParameterSet::default(),
            set3 : RXParameterSet::default(),
            synthesizer: Synthesizer::default(),
            packet_controller: PacketController::default(),
            packet_format: PacketFormat::default(),
        }
    }
}

impl UIState {
    fn onoff<F: Flags>(&self, field: F, flag: F) -> Style {
        if field.contains(flag) {
            self.on
        } else {
            self.off
        }
    }

    fn pwr_mode<'a>(&self) -> Table<'a> {
        let flags = self.pwrmode.flags;
        Table::new(vec![
            Row::new(vec![
                Cell::from(format!("{:?}", self.pwrmode.mode)),
                Cell::from("REFEN").style(self.onoff(flags, PwrFlags::REFEN)),
                Cell::from("XOEN" ).style(self.onoff(flags, PwrFlags::XOEN)),
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("PWRMODE"))
        .widths(&[Constraint::Length(8), Constraint::Length(5), Constraint::Length(4)])
        .style(Style::default().fg(Color::White))
    }

    fn pow_stat<'a>(&self) -> Table<'a> {
        let flags = self.powstat;
        Table::new(vec![
            Row::new(vec![
                Cell::from("vIO"     ).style(self.onoff(flags, PowStat::VIO)),
                Cell::from("BEvMODEM").style(self.onoff(flags, PowStat::BEVMODEM)),
                Cell::from("BEvANA"  ).style(self.onoff(flags, PowStat::BEVANA)),
                Cell::from("vMODEM"  ).style(self.onoff(flags, PowStat::VMODEM)),
                Cell::from("vANA"    ).style(self.onoff(flags, PowStat::VANA)),
                Cell::from("vREF"    ).style(self.onoff(flags, PowStat::VREF)),
                Cell::from("REF"     ).style(self.onoff(flags, PowStat::REF)),
                Cell::from("SUM"     ).style(self.onoff(flags, PowStat::SUM))
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("POWSTAT"))
        .widths(&[Constraint::Max(3), Constraint::Max(8), Constraint::Max(6), Constraint::Max(6),
                Constraint::Max(4), Constraint::Max(4), Constraint::Max(3), Constraint::Max(3)])
    }

    fn irq<'a>(&self) -> Table<'a> {
        let flags = self.irq;
        Table::new(vec![
            Row::new(vec![
                Cell::from("PLLUNLOCK"  ).style(self.onoff(flags, IRQ::PLLUNLOCK)),
                Cell::from("RADIOCTRL"  ).style(self.onoff(flags, IRQ::RADIOCTRL)),
                Cell::from("POWER"      ).style(self.onoff(flags, IRQ::POWER)),
                Cell::from("XTALREADY"  ).style(self.onoff(flags, IRQ::XTALREADY)),
                Cell::from("WAKEUPTIMER").style(self.onoff(flags, IRQ::WAKEUPTIMER)),
                Cell::from("LPOSC"      ).style(self.onoff(flags, IRQ::LPOSC)),
                Cell::from("GPADC"      ).style(self.onoff(flags, IRQ::GPADC)),
                Cell::from("PLLRNGDONE" ).style(self.onoff(flags, IRQ::PLLRNGDONE)),
            ]),
            Row::new(vec![
                Cell::from("FIFONOTEMPTY").style(self.onoff(flags, IRQ::FIFONOTEMPTY)),
                Cell::from("FIFONOTFULL" ).style(self.onoff(flags, IRQ::FIFONOTFULL)),
                Cell::from("FIFOTHRCNT"  ).style(self.onoff(flags, IRQ::FIFOTHRCNT)),
                Cell::from("FIFOTHRFREE" ).style(self.onoff(flags, IRQ::FIFOTHRFREE)),
                Cell::from("FIFOERROR"   ).style(self.onoff(flags, IRQ::FIFOERROR)),
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("IRQ"))
        .widths(&[Constraint::Max(12), Constraint::Max(11), Constraint::Max(10), Constraint::Max(11),
                Constraint::Max(11), Constraint::Max(5), Constraint::Max(5), Constraint::Max(10)])
    }

    fn spi_status<'a>(&self) -> Table<'a> {
        let flags = self.status;
        Table::new(vec![
            Row::new(vec![
                Cell::from("READY"           ).style(self.onoff(flags, Status::READY)),
                Cell::from("PLL_LOCK"        ).style(self.onoff(flags, Status::PLL_LOCK)),
                Cell::from("FIFO_OVER"       ).style(self.onoff(flags, Status::FIFO_OVER)),
                Cell::from("FIFO_UNDER"      ).style(self.onoff(flags, Status::FIFO_UNDER)),
                Cell::from("THRESHOLD_FREE"  ).style(self.onoff(flags, Status::THRESHOLD_FREE)),
                Cell::from("THRESHOLD_COUNT" ).style(self.onoff(flags, Status::THRESHOLD_COUNT)),
                Cell::from("FIFO_FULL"       ).style(self.onoff(flags, Status::FIFO_FULL)),
                Cell::from("FIFO_EMPTY"      ).style(self.onoff(flags, Status::FIFO_EMPTY)),
                Cell::from("PWR_GOOD"        ).style(self.onoff(flags, Status::PWR_GOOD)),
                Cell::from("PWR_INTERRUPT"   ).style(self.onoff(flags, Status::PWR_INTERRUPT)),
                Cell::from("RADIO_EVENT"     ).style(self.onoff(flags, Status::RADIO_EVENT)),
                Cell::from("XTAL_OSC_RUNNING").style(self.onoff(flags, Status::XTAL_OSC_RUNNING)),
                Cell::from("WAKEUP_INTERRUPT").style(self.onoff(flags, Status::WAKEUP_INTERRUPT)),
                Cell::from("LPOSC_INTERRUPT" ).style(self.onoff(flags, Status::LPOSC_INTERRUPT)),
                Cell::from("GPADC_INTERRUPT" ).style(self.onoff(flags, Status::GPADC_INTERRUPT)),
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("STATUS"))
        .widths(&[Constraint::Max(5), Constraint::Max(8), Constraint::Max(9), Constraint::Max(10),
                  Constraint::Max(14), Constraint::Max(15), Constraint::Max(9), Constraint::Max(10),
                  Constraint::Max(8), Constraint::Max(13), Constraint::Max(11), Constraint::Max(16),
                  Constraint::Max(16), Constraint::Max(15), Constraint::Max(15)])
    }

    fn radio_event<'a>(&self) -> Table<'a> {
        let flags = self.radio_event;
        Table::new(vec![
            Row::new(vec![
                Cell::from("DONE"    ).style(self.onoff(flags, RadioEvent::DONE)),
                Cell::from("SETTLED" ).style(self.onoff(flags, RadioEvent::SETTLED)),
                Cell::from("STATE"   ).style(self.onoff(flags, RadioEvent::RADIOSTATECHG)),
                Cell::from("PARAMSET").style(self.onoff(flags, RadioEvent::RXPARAMSETCHG)),
                Cell::from("FRAMECLK").style(self.onoff(flags, RadioEvent::FRAMECLK)),
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("Radio Event"))
        .widths(&[Constraint::Max(4), Constraint::Max(7), Constraint::Max(5), Constraint::Max(8), Constraint::Max(8)])
    }

    fn radio_state<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new([
                Cell::from(format!("{:?}", self.radio_state)),
            ])
        ])
        .block(Block::default().borders(Borders::ALL).title("Radio State"))
        .widths(&[Constraint::Max(10)])
    }

    fn sparkline<'a, T>(&self, name: T, unit: T, data: &'a [u64]) -> Sparkline<'a> where T: AsRef<str> + std::fmt::Display {
        let min = data.iter().min().unwrap_or(&0);
        let max = data.iter().max().unwrap_or(&0);
        let name: &str = name.as_ref();
        Sparkline::default()
            .block(Block::default().title(format!("{name} ({min} {unit} - {max} {unit})")).borders(Borders::ALL))
            .data(data)
            .style(self.spark)
    }

    fn spark_signed<'a, T>(&self, name: T, unit: T, data: &'a [u64], min: i64, max: i64) -> Sparkline<'a> where T: AsRef<str> + std::fmt::Display {
        Sparkline::default()
            .block(Block::default().title(format!("{name} ({min} {unit} - {max} {unit})")).borders(Borders::ALL))
            .data(data)
            .style(self.spark)
    }

    fn rx_params<'a>(&self, data: &'a [u64], last: RxParamCurSet) -> Sparkline<'a> {
        Sparkline::default()
        .block(Block::default().borders(Borders::ALL).title(format!("Current RX Parameters - stage {} ({:?}) special {}", last.index, last.number, last.special)))
        .data(data)
        .style(self.spark)
    }

    fn receiver_parameter_set<'a>(&self, set: &RXParameterSet, active: bool) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                "AGC", "attack", "decay", "target", "ahyst", "min", "max",
            ]),
            Row::new(vec![
                Cell::from(""),
                Cell::from(set.agc.attack.to_string()),
                Cell::from(set.agc.decay.to_string()),
                Cell::from(set.agc.target.to_string()),
                Cell::from(set.agc.ahyst.to_string()),
                Cell::from(set.agc.min.to_string()),
                Cell::from(set.agc.max.to_string()),
            ]),
            Row::new(vec![
                "Gain", "time", "rate", "phase", "filt", "BB gain phase", "BB gain freq", "RF gain freq", "RF gain phase", "ampl",
            ]),
            Row::new(vec![
                 Cell::from(""),
                 Cell::from(set.gain.time.to_string()),
                 Cell::from(set.gain.rate.to_string()),
                 Cell::from(set.gain.phase.to_string()),
                 Cell::from(set.gain.filter.to_string()),
                 Cell::from(set.gain.baseband.freq.to_string()),
                 Cell::from(set.gain.baseband.phase.to_string()),
                 Cell::from(set.gain.rf.freq.to_string()),
                 Cell::from(set.gain.rf.phase.to_string()),
                 Cell::from(set.gain.amplitude.to_string()),
            ]),
            Row::new(vec![
                "", "freq dev", "decay", "BBR block A", "BBR block B",
            ]),
            Row::new(vec![
                Cell::from(""),
                Cell::from(set.freq_dev.to_string()),
                Cell::from(set.decay.to_string()),
                Cell::from(set.baseband_offset.a.to_string()),
                Cell::from(set.baseband_offset.b.to_string()),
            ]),
        ])
        .block(Block::default().borders(Borders::ALL).title("RX Parameter Set"))
        .widths(&[Constraint::Max(5), Constraint::Max(8), Constraint::Max(9), Constraint::Max(10),
               Constraint::Max(14), Constraint::Max(15), Constraint::Max(9), Constraint::Max(10),
               Constraint::Max(8), Constraint::Max(13), Constraint::Max(11), Constraint::Max(16),
               Constraint::Max(16), Constraint::Max(15), Constraint::Max(15)])
        .style(if active { self.on } else { self.off })
    }

    fn receiver_parameters<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                "IF Freq", "Max Δ", "Baseband", "Bitrate", "Max Δ"
            ]),
            Row::new(vec![
                Cell::from(format!("{} Hz", self.rxparams.iffreq)),
                Cell::from(format!("{} Hz", self.rxparams.maxrfoffset)),
                Cell::from(format!("{} Hz", self.rxparams.baseband)),
                Cell::from(format!("{} bit/s", self.rxparams.bitrate)),
                Cell::from(format!("{} bit/s", self.rxparams.maxdroffset)),
            ]),
            Row::new(vec![
                "FSK Dev Max", "Min", "AFSK Space", "Mark", "Ctrl"
            ]),
            Row::new(vec![
                Cell::from(format!("{} Hz", self.rxparams.fskdevmax)),
                Cell::from(format!("{} Hz", self.rxparams.fskdevmin)),
                Cell::from(self.rxparams.afskspace.to_string()),
                Cell::from(self.rxparams.afskmark.to_string()),
                Cell::from(self.rxparams.afskctrl.to_string()),
            ]),
            Row::new(vec![
                "Ampl Filt", "Freq Leak", "RX 0", "RX 1", "RX 2", "RX 3"
            ]),
            Row::new(vec![
                Cell::from(self.rxparams.amplfilt.to_string()),
                Cell::from(self.rxparams.freqleak.to_string()),
                Cell::from(format!("{:?}", self.rxparams.rxparamset.0)),
                Cell::from(format!("{:?}", self.rxparams.rxparamset.1)),
                Cell::from(format!("{:?}", self.rxparams.rxparamset.2)),
                Cell::from(format!("{:?}", self.rxparams.rxparamset.3)),
            ]),
        ])
        .block(Block::default().borders(Borders::ALL).title("RX Parameters"))
        .widths(&[Constraint::Max(11), Constraint::Max(10), Constraint::Max(11), Constraint::Max(10),
               Constraint::Max(14), Constraint::Max(15)])
    }

    fn synth<'a>(&self) -> Table<'a> {
        /* pllloop / boost [ b direct filten filt ]
         * pllcpi / boost [ cpi ]
         * pllvcodiv [ VCOI_MAN VCO2INT VCOSEL  RFDIV REFDIV ]
         * pllranging A/B [ sticky lock err start vcor ]
         * freq a/b
         */

        Table::new(vec![
            Row::new(vec![
                Cell::from(""),
                Cell::from("Freq"),
                Cell::from("Ranging"),
            ]),
            Row::new(vec![
                Cell::from("A"),
                Cell::from(format!("{} Hz", self.synthesizer.freqa)),
                Cell::from(format!("{:?}", self.synthesizer.ranginga)),

            ]),
            Row::new(vec![
                Cell::from("B"),
                Cell::from(format!("{} Hz", self.synthesizer.freqb)),
                Cell::from(format!("{:?}", self.synthesizer.rangingb)),
            ]),
            Row::new(vec![
                Cell::from(""),
                Cell::from("CPI"),
                Cell::from("Loop"),
            ]),
            Row::new(vec![
                Cell::from("PLL"),
                Cell::from(format!("{:?}", self.synthesizer.cpi)),
                Cell::from(format!("{:?}", self.synthesizer.pllloop)),
            ]),
            Row::new(vec![
                Cell::from("Boost"),
                Cell::from(format!("{:?}", self.synthesizer.cpiboost)),
                Cell::from(format!("{:?}", self.synthesizer.pllloopboost)),
            ]),

            Row::new(vec![
                Cell::from(""),
                Cell::from(""),
                Cell::from(format!("{:?}", self.synthesizer.vcodiv)),
            ]),
        ])
        .block(Block::default().borders(Borders::ALL).title("Synthesizer"))
        .widths(&[Constraint::Max(5), Constraint::Max(13), Constraint::Max(80)])
    }

    fn packet_controller<'a>(&self) -> Table<'a> {
        Table::new(vec![
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
                Cell::from(self.packet_controller.tmg_tx_boost.to_string()),
                Cell::from(self.packet_controller.tmg_tx_settle.to_string()),
                Cell::from(self.packet_controller.tmg_rx_boost.to_string()),
                Cell::from(self.packet_controller.tmg_rx_settle.to_string()),
                Cell::from(self.packet_controller.tmg_rx_offsacq.to_string()),
                Cell::from(self.packet_controller.tmg_rx_coarseagc.to_string()),
                Cell::from(self.packet_controller.tmg_rx_agc.to_string()),
                Cell::from(self.packet_controller.tmg_rx_rssi.to_string()),
                Cell::from(self.packet_controller.tmg_rx_preamble1.to_string()),
                Cell::from(self.packet_controller.tmg_rx_preamble2.to_string()),
                Cell::from(self.packet_controller.tmg_rx_preamble3.to_string()),
            ]),
            Row::new(vec![
                Cell::from("RSSI Ref"),
                Cell::from("Abs Thr"),
                Cell::from("BGND Gain"),
                Cell::from("BGND thr"),
                Cell::from("Chunk"),
            ]),
            Row::new(vec![
                Cell::from(self.packet_controller.rssi_reference.to_string()),
                Cell::from(self.packet_controller.rssi_abs_thr.to_string()),
                Cell::from(self.packet_controller.bgnd_rssi_gain.to_string()),
                Cell::from(self.packet_controller.bgnd_rssi_thr.to_string()),
                Cell::from(self.packet_controller.pkt_chunk_size.to_string()),
            ]),
        ])
        .block(Block::default().borders(Borders::ALL).title("Packet Controller"))
        .widths(&[Constraint::Max(10), Constraint::Max(10), Constraint::Max(10), Constraint::Max(10),
            Constraint::Max(10), Constraint::Max(10), Constraint::Max(10), Constraint::Max(10),
            Constraint::Max(10), Constraint::Max(10), Constraint::Max(10), Constraint::Max(10),
        ])
    }

    fn packet_controller_flags<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("Misc Flag"),
                Cell::from("Store Flg"),
                Cell::from("Acc Flags"),
            ]),
            Row::new(vec![
                Cell::from(format!("{:?}", self.packet_controller.pkt_misc_flags)),
                Cell::from(format!("{:?}", self.packet_controller.pkt_store_flags)),
                Cell::from(format!("{:?}", self.packet_controller.pkt_accept_flags)),
           ]),
        ])
        .widths(&[Constraint::Max(30), Constraint::Max(30), Constraint::Max(50)])
    }

    /*
    lencfg: PktLenCfg,
    lenoffset: u8,
    maxlen: u8,
    addr: u32,
    addrmask: u32,
*/

    fn packet_format<'a>(&self) -> Table<'a> {
        Table::new(vec![
            Row::new(vec![
                Cell::from("Addr pos"),
                Cell::from("Addr flags"),
                Cell::from("len pos"),
                Cell::from("len bits"),
            ]),
            Row::new(vec![
                Cell::from(format!("{}", self.packet_format.addrcfg.addr_pos)),
                Cell::from(format!("{:?}", self.packet_format.addrcfg.flags)),
                Cell::from(self.packet_format.lencfg.pos.to_string()),
                Cell::from(self.packet_format.lencfg.bits.to_string()),
            ]),
            Row::new(vec![
                Cell::from("Addr"),
                Cell::from("Mask"),
                Cell::from("len off"),
                Cell::from("len max"),
            ]),
            Row::new(vec![
                Cell::from(self.packet_format.addr.to_string()),
                Cell::from(self.packet_format.addrmask.to_string()),
                Cell::from(self.packet_format.lenoffset.to_string()),
                Cell::from(self.packet_format.maxlen.to_string()),
            ]),
        ])
        .block(Block::default().borders(Borders::ALL).title("Packet Format"))
        .widths(&[Constraint::Max(10), Constraint::Min(30), Constraint::Min(10), Constraint::Min(10)])
    }
}

struct PacketController {
    tmg_tx_boost: TMG,
    tmg_tx_settle: TMG,
    tmg_rx_boost: TMG,
    tmg_rx_settle: TMG,
    tmg_rx_offsacq: TMG,
    tmg_rx_coarseagc: TMG,
    tmg_rx_agc: TMG,
    tmg_rx_rssi: TMG,
    tmg_rx_preamble1: TMG,
    tmg_rx_preamble2: TMG,
    tmg_rx_preamble3: TMG,
    rssi_reference: u8,
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
            tmg_tx_boost: TMG{ e: 0, m: 0 },
            tmg_tx_settle: TMG{ e: 0, m: 0 },
            tmg_rx_boost: TMG{ e: 0, m: 0 },
            tmg_rx_settle: TMG{ e: 0, m: 0 },
            tmg_rx_offsacq: TMG{ e: 0, m: 0 },
            tmg_rx_coarseagc: TMG{ e: 0, m: 0 },
            tmg_rx_agc: TMG{ e: 0, m: 0 },
            tmg_rx_rssi: TMG{ e: 0, m: 0 },
            tmg_rx_preamble1: TMG{ e: 0, m: 0 },
            tmg_rx_preamble2: TMG{ e: 0, m: 0 },
            tmg_rx_preamble3: TMG{ e: 0, m: 0 },
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



fn ui<B: Backend>(f: &mut Frame<B>, state: &UIState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints(
            [
                Constraint::Percentage(10),
                Constraint::Percentage(80),
                Constraint::Percentage(10),
            ].as_ref()
        )
        .split(f.size());


    let power = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Min(21),
            Constraint::Min(46),
            Constraint::Min(85),
            Constraint::Min(40),
            Constraint::Min(0),
        ].as_ref())
        .split(chunks[0]);

    f.render_widget(state.pwr_mode(), power[0]);
    f.render_widget(state.pow_stat(), power[1]);
    f.render_widget(state.irq(), power[2]);
    f.render_widget(state.radio_event(), power[3]);
    f.render_widget(state.radio_state(), power[4]);

    let rx = Layout::default()
        .direction(Direction::Horizontal)
        .margin(1)
        .constraints([
            Constraint::Percentage(50),
            Constraint::Percentage(50),
        ].as_ref())
        .split(chunks[1]);

    let parameters = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints([
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
        ].as_ref())
        .split(rx[1]);


    f.render_widget(state.synth(), parameters[0]);
    f.render_widget(state.packet_controller(), parameters[1]);
    f.render_widget(state.packet_controller_flags(), parameters[2]);
    f.render_widget(state.packet_format(), parameters[3]);
/*
    f.render_widget(state.receiver_parameters(), parameters[1]);
    f.render_widget(state.receiver_parameter_set(&state.set0, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set0), parameters[2]);
    f.render_widget(state.receiver_parameter_set(&state.set1, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set1), parameters[2]);
    f.render_widget(state.receiver_parameter_set(&state.set2, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set2), parameters[3]);
    f.render_widget(state.receiver_parameter_set(&state.set3, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set3), parameters[3]);
*/
    let sparks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints([
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Percentage(10),
            Constraint::Min(0),
        ].as_ref())
        .split(rx[0]);
    let data = &state.rx.iter().map(|r| i64::from(r.rssi)).collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data.iter().map(|x| u64::try_from(x - min).unwrap()).collect::<Vec<u64>>();
    f.render_widget(state.spark_signed("RSSI", "dB", data, min, max), sparks[0]);
    f.render_widget(state.sparkline("Background RSSI", "dB",  &state.rx.iter().map(|r| u64::from(r.bgndrssi)).collect::<Vec<u64>>()), sparks[1]);

    let data = &state.rx.iter().map(|r| i64::from(r.agccounter)).collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data.iter().map(|x| u64::try_from(x - min).unwrap()).collect::<Vec<u64>>();
    f.render_widget(state.spark_signed("AGC Counter", "dB", data, min, max), sparks[2]);
    f.render_widget(state.sparkline("Data Rate", "bits/s", &state.rx.iter().map(|r| u64::from(r.datarate)).collect::<Vec<u64>>()), sparks[3]);
    f.render_widget(state.sparkline("Amplitude", "", &state.rx.iter().map(|r| u64::from(r.ampl)).collect::<Vec<u64>>()), sparks[4]);
    f.render_widget(state.sparkline("Phase", "", &state.rx.iter().map(|r| u64::from(r.phase)).collect::<Vec<u64>>()), sparks[5]);
    let data = &state.rx.iter().map(|r| i64::from(r.fskdemod)).collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data.iter().map(|x| u64::try_from(x - min).unwrap()).collect::<Vec<u64>>();
    f.render_widget(state.spark_signed("FSK Demodulation", "", data, min, max), sparks[6]);
    let data = &state.rx.iter().map(|r| i64::from(r.rffreq)).collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data.iter().map(|x| u64::try_from(x - min).unwrap()).collect::<Vec<u64>>();
    f.render_widget(state.spark_signed("RF Frequency", "Hz", data, min, max), sparks[7]);
    f.render_widget(state.sparkline("Frequency", "Hz", &state.rx.iter().map(|r| u64::from(r.freq)).collect::<Vec<u64>>()), sparks[8]);
    f.render_widget(state.rx_params(&state.rx.iter().map(|r| u64::from(r.paramcurset.index)).collect::<Vec<u64>>(), state.rx.back().unwrap_or(&RXState::default()).paramcurset), sparks[9]);
    f.render_widget(state.spi_status(), chunks[2]);
}

pub fn ax5043_listen(radio: &mut Registers) -> io::Result<()> {

    // pll not locked
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::SYNTHRX,
    })?;

    radio.FIFOCMD.write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD.write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    Ok(())
}

struct RXState {
    rssi: i8,
    bgndrssi: u8,
    agccounter: i32,
    datarate: u32,
    ampl: u16,
    phase: u16,
    fskdemod: i32,
    rffreq: i32,
    freq: u16,
    paramcurset: RxParamCurSet,
}

impl Default for RXState {
    fn default() -> Self {
        Self {
            rssi: 0,
            bgndrssi: 0,
            agccounter: 0,
            datarate: 0,
            ampl: 0,
            phase: 0,
            fskdemod: 0,
            rffreq: 0,
            freq: 0,
            paramcurset: RxParamCurSet {
                index: 0,
                number: RxParamSet::Set0,
                special: 0,
            },
        }
    }
}

fn get_signal(radio: &mut Registers) -> io::Result<RXState> {
    Ok(RXState {
        rssi: radio.RSSI.read()?,
        bgndrssi: radio.BGNDRSSI.read()?,
        agccounter: (i32::from(radio.AGCCOUNTER.read()?) * 4) / 3,
        datarate: radio.TRKDATARATE.read()?,
        ampl: radio.TRKAMPL.read()?,
        phase: radio.TRKPHASE.read()?,
        fskdemod: {
            let mut demod: i32 = radio.TRKFSKDEMOD.read()?.into();
            if demod > 2_i32.pow(13) {
                demod = demod - 2_i32.pow(14)
            }
            demod
        },
        rffreq: radio.TRKRFFREQ.read()?.0,
        freq: radio.TRKFREQ.read()?, // trkfreq / 2^16 * bitrate
        paramcurset: radio.RXPARAMCURSET.read()?,
    })
}

fn get_rx_parameter_set0(radio: &mut Registers) -> io::Result<RXParameterSet> {
    let agcgain = radio.AGCGAIN0.read()?;
    let agcmina = radio.AGCMINMAX0.read()?;
    let timegan = radio.TIMEGAIN0.read()?;
    let dargain = radio.DRGAIN0.read()?;
    let phasegn = radio.PHASEGAIN0.read()?;
    let bboffsr = radio.BBOFFSRES0.read()?;
    Ok(RXParameterSet {
        agc: RXParameterAGC {
            attack: agcgain.attack,
            decay: agcgain.decay,
            target: radio.AGCTARGET0.read()?,
            ahyst: radio.AGCAHYST0.read()?.hyst,
            min: agcmina.min,
            max: agcmina.max,
        },
        gain: RXParameterGain {
            time: timegan.exponent,
            rate: dargain.exponent,
            phase: phasegn.gain,
            filter: phasegn.filter,
            baseband: RXParameterFreq {
                phase: radio.FREQGAINA0.read()?.gain,
                freq: radio.FREQGAINB0.read()?.gain,
            },
            rf: RXParameterFreq {
                phase: radio.FREQGAINC0.read()?.gain,
                freq: radio.FREQGAIND0.read()?.gain,
            },
            amplitude: radio.AMPLGAIN0.read()?.gain,
        },
        freq_dev: radio.FREQDEV0.read()?,
        decay: radio.FOURFSK0.read()?.decay,
        baseband_offset: RXParameterBasebandOffset {
            a: bboffsr.res_int_a,
            b: bboffsr.res_int_b,
        },
    })
}

fn get_rx_parameter_set1(radio: &mut Registers) -> io::Result<RXParameterSet> {
    let agcgain = radio.AGCGAIN1.read()?;
    let agcmina = radio.AGCMINMAX1.read()?;
    let timegan = radio.TIMEGAIN1.read()?;
    let dargain = radio.DRGAIN1.read()?;
    let phasegn = radio.PHASEGAIN1.read()?;
    let bboffsr = radio.BBOFFSRES1.read()?;
    Ok(RXParameterSet {
        agc: RXParameterAGC {
            attack: agcgain.attack,
            decay: agcgain.decay,
            target: radio.AGCTARGET1.read()?,
            ahyst: radio.AGCAHYST1.read()?.hyst,
            min: agcmina.min,
            max: agcmina.max,
        },
        gain: RXParameterGain {
            time: timegan.exponent,
            rate: dargain.exponent,
            phase: phasegn.gain,
            filter: phasegn.filter,
            baseband: RXParameterFreq {
                phase: radio.FREQGAINA1.read()?.gain,
                freq: radio.FREQGAINB1.read()?.gain,
            },
            rf: RXParameterFreq {
                phase: radio.FREQGAINC1.read()?.gain,
                freq: radio.FREQGAIND1.read()?.gain,
            },
            amplitude: radio.AMPLGAIN1.read()?.gain,
        },
        freq_dev: radio.FREQDEV1.read()?,
        decay: radio.FOURFSK1.read()?.decay,
        baseband_offset: RXParameterBasebandOffset {
            a: bboffsr.res_int_a,
            b: bboffsr.res_int_b,
        },
    })
}

fn get_rx_parameter_set2(radio: &mut Registers) -> io::Result<RXParameterSet> {
    let agcgain = radio.AGCGAIN2.read()?;
    let agcmina = radio.AGCMINMAX2.read()?;
    let timegan = radio.TIMEGAIN2.read()?;
    let dargain = radio.DRGAIN2.read()?;
    let phasegn = radio.PHASEGAIN2.read()?;
    let bboffsr = radio.BBOFFSRES2.read()?;
    Ok(RXParameterSet {
        agc: RXParameterAGC {
            attack: agcgain.attack,
            decay: agcgain.decay,
            target: radio.AGCTARGET2.read()?,
            ahyst: radio.AGCAHYST2.read()?.hyst,
            min: agcmina.min,
            max: agcmina.max,
        },
        gain: RXParameterGain {
            time: timegan.exponent,
            rate: dargain.exponent,
            phase: phasegn.gain,
            filter: phasegn.filter,
            baseband: RXParameterFreq {
                phase: radio.FREQGAINA2.read()?.gain,
                freq: radio.FREQGAINB2.read()?.gain,
            },
            rf: RXParameterFreq {
                phase: radio.FREQGAINC2.read()?.gain,
                freq: radio.FREQGAIND2.read()?.gain,
            },
            amplitude: radio.AMPLGAIN2.read()?.gain,
        },
        freq_dev: radio.FREQDEV2.read()?,
        decay: radio.FOURFSK2.read()?.decay,
        baseband_offset: RXParameterBasebandOffset {
            a: bboffsr.res_int_a,
            b: bboffsr.res_int_b,
        },
    })
}

fn get_rx_parameter_set3(radio: &mut Registers) -> io::Result<RXParameterSet> {
    let agcgain = radio.AGCGAIN3.read()?;
    let agcmina = radio.AGCMINMAX3.read()?;
    let timegan = radio.TIMEGAIN3.read()?;
    let dargain = radio.DRGAIN3.read()?;
    let phasegn = radio.PHASEGAIN3.read()?;
    let bboffsr = radio.BBOFFSRES3.read()?;
    Ok(RXParameterSet {
        agc: RXParameterAGC {
            attack: agcgain.attack,
            decay: agcgain.decay,
            target: radio.AGCTARGET3.read()?,
            ahyst: radio.AGCAHYST3.read()?.hyst,
            min: agcmina.min,
            max: agcmina.max,
        },
        gain: RXParameterGain {
            time: timegan.exponent,
            rate: dargain.exponent,
            phase: phasegn.gain,
            filter: phasegn.filter,
            baseband: RXParameterFreq {
                phase: radio.FREQGAINA3.read()?.gain,
                freq: radio.FREQGAINB3.read()?.gain,
            },
            rf: RXParameterFreq {
                phase: radio.FREQGAINC3.read()?.gain,
                freq: radio.FREQGAIND3.read()?.gain,
            },
            amplitude: radio.AMPLGAIN3.read()?.gain,
        },
        freq_dev: radio.FREQDEV3.read()?,
        decay: radio.FOURFSK3.read()?.decay,
        baseband_offset: RXParameterBasebandOffset {
            a: bboffsr.res_int_a,
            b: bboffsr.res_int_b,
        },
    })
}

fn get_rx_params(radio: &mut Registers, board: &config::Board) -> io::Result<RXParams> {
    let decimation = u64::from(radio.DECIMATION.read()?);
    let rxdatarate = u64::from(radio.RXDATARATE.read()?);
    let bitrate = board.xtal.freq * 2_u64.pow(7) / (rxdatarate * decimation * board.xtal.div());

    Ok(RXParams {
        iffreq: u64::from(radio.IFFREQ.read()?) * board.xtal.freq / board.xtal.div() / 2_u64.pow(20) ,
        baseband: board.xtal.freq / (2_u64.pow(4) * board.xtal.div() * decimation),
        bitrate: bitrate,
        maxdroffset: u64::from(radio.MAXDROFFSET.read()?) * board.xtal.div() * bitrate * bitrate * decimation / (2_u64.pow(7) * board.xtal.freq),
        maxrfoffset: u64::from(radio.MAXRFOFFSET.read()?.offset) * board.xtal.freq / 2_u64.pow(24), // TODO: xtal.div not part of this?
        fskdevmax: u64::from(radio.FSKDMAX.read()?) * bitrate / (3 * 512),  // TODO baudrate?
        fskdevmin: i64::from(radio.FSKDMIN.read()?) * i64::try_from(bitrate).unwrap() / (-3 * 512), // TODO baudrate?
        afskspace: radio.AFSKSPACE.read()?,
        afskmark: radio.AFSKMARK.read()?,
        afskctrl: radio.AFSKCTRL.read()?,
        amplfilt: radio.AMPLFILTER.read()?,
        freqleak: radio.FREQUENCYLEAK.read()?,
        rxparamset: radio.RXPARAMSETS.read()?,
    })
}

struct Synthesizer {
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
                flags: PLLVCODivFlags::empty()
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

fn synthesizer(radio: &mut Registers, board: &config::Board) -> io::Result<Synthesizer> {
    Ok(Synthesizer {
        pllloop: radio.PLLLOOP.read()?,
        pllloopboost: radio.PLLLOOPBOOST.read()?,
        cpi: radio.PLLCPI.read()?,
        cpiboost: radio.PLLCPIBOOST.read()?,
        vcodiv: radio.PLLVCODIV.read()?,
        ranginga: radio.PLLRANGINGA.read()?,
        rangingb: radio.PLLRANGINGB.read()?,
        freqa: u64::from(radio.FREQA.read()?) * board.xtal.freq / 2_u64.pow(24),
        freqb: u64::from(radio.FREQB.read()?) * board.xtal.freq / 2_u64.pow(24),
    })
}


fn packet_controller(radio: &mut Registers) -> io::Result<PacketController> {
    Ok(PacketController {
        tmg_tx_boost: radio.TMGTXBOOST.read()?,
        tmg_tx_settle: radio.TMGTXSETTLE.read()?,
        tmg_rx_boost: radio.TMGRXBOOST.read()?,
        tmg_rx_settle: radio.TMGRXSETTLE.read()?,
        tmg_rx_offsacq: radio.TMGRXOFFSACQ.read()?,
        tmg_rx_coarseagc: radio.TMGRXCOARSEAGC.read()?,
        tmg_rx_agc: radio.TMGRXAGC.read()?,
        tmg_rx_rssi: radio.TMGRXRSSI.read()?,
        tmg_rx_preamble1: radio.TMGRXPREAMBLE1.read()?,
        tmg_rx_preamble2: radio.TMGRXPREAMBLE2.read()?,
        tmg_rx_preamble3: radio.TMGRXPREAMBLE3.read()?,
        rssi_reference: radio.RSSIREFERENCE.read()?,
        rssi_abs_thr: radio.RSSIABSTHR.read()?,
        bgnd_rssi_gain: radio.BGNDRSSIGAIN.read()?,
        bgnd_rssi_thr: radio.BGNDRSSITHR.read()?,
        pkt_chunk_size: radio.PKTCHUNKSIZE.read()?,
        pkt_misc_flags: radio.PKTMISCFLAGS.read()?,
        pkt_store_flags: radio.PKTSTOREFLAGS.read()?,
        pkt_accept_flags: radio.PKTACCEPTFLAGS.read()?,
    })
}

struct PacketFormat {
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
            addrcfg: PktAddrCfg { addr_pos: 0, flags: PktAddrCfgFlags::empty()},
            lencfg: PktLenCfg {pos: 0, bits: 0},
            lenoffset: 0,
            maxlen: 0,
            addr: 0,
            addrmask: 0,
        }
    }
}

fn packet_format(radio: &mut Registers) -> io::Result<PacketFormat> {
    Ok(PacketFormat {
        addrcfg: radio.PKTADDRCFG.read()?,
        lencfg: radio.PKTLENCFG.read()?,
        lenoffset: radio.PKTLENOFFSET.read()?,
        maxlen: radio.PKTMAXLEN.read()?,
        addr: radio.PKTADDR.read()?,
        addrmask: radio.PKTADDRMASK.read()?,
    })
}

fn main() -> Result<(), io::Error> {
    panic::set_hook(Box::new(|panic| {
        disable_raw_mode().unwrap();
        execute!(
            io::stdout(),
            LeaveAlternateScreen,
        ).unwrap();

        println!("{}", panic);
        println!("{}", Backtrace::capture());
    }));

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(
        stdout,
        EnterAlternateScreen,
    )?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut uistate = UIState::default();
    terminal.draw(|f| {
        ui(f, &uistate);
    })?;

    let spi0 = ax5043::open("/dev/spidev1.0")?;
    let state = cell::RefCell::new(&mut uistate);
    let term = cell::RefCell::new(&mut terminal);
    let callback = |new: Status| {
        if new == state.borrow().status {
            return
        }
        state.borrow_mut().status = new;
        let _ = term.borrow_mut().draw(|f| {
            ui(f, &state.borrow());
        });
    };
    let mut radio = ax5043::Registers::new(&spi0, &callback);

    radio.reset()?;

    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
    state.borrow_mut().powstat = radio.POWSTAT.read()?;
    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });


    let mut poll = Poll::new()?;
    let registry = poll.registry();

    const CTRLC: Token = Token(0);
    let mut signals = Signals::new(Signal::Interrupt.into())?;
    registry.register(&mut signals, CTRLC, Interest::READABLE)?;

    const STDIN: Token = Token(1);
    registry.register(
        &mut SourceFd(&io::stdin().as_raw_fd()),
        STDIN,
        Interest::READABLE)?;

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(TimerState::Periodic{current: Duration::new(1, 0), interval: Duration::from_millis(50)}, SetTimeFlags::Default);
    const BEACON: Token = Token(2);
    registry.register(
        &mut SourceFd(&tfd.as_raw_fd()),
        BEACON,
        Interest::READABLE)?;

    let board = configure_radio_rx(&mut radio)?;
    radio.RADIOEVENTMASK.write(RadioEvent::all())?;

    state.borrow_mut().board = board.clone();
    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
    state.borrow_mut().powstat = radio.POWSTAT.read()?;
    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });


    ax5043_listen(&mut radio)?;
    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
    state.borrow_mut().powstat = radio.POWSTAT.read()?;
    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT.read()?; // clear sticky power flags for PWR_GOOD

    //radio.PKTCHUNKSIZE.write(0b1011)?;
    //radio.PKTMISCFLAGS.write(PktMiscFlags::BGND_RSSI)?;
    //radio.PKTACCEPTFLAGS.write(
    //      PktAcceptFlags::RESIDUE
    //    | PktAcceptFlags::ABRT
    //    | PktAcceptFlags::CRCF
    //    | PktAcceptFlags::ADDRF
    //    | PktAcceptFlags::SZF
    //    | PktAcceptFlags::LRGP
    //)?;
    state.borrow_mut().rxparams = get_rx_params(&mut radio, &board)?;
    state.borrow_mut().set0 = get_rx_parameter_set0(&mut radio)?;
    state.borrow_mut().set1 = get_rx_parameter_set1(&mut radio)?;
    state.borrow_mut().set2 = get_rx_parameter_set2(&mut radio)?;
    state.borrow_mut().set3 = get_rx_parameter_set3(&mut radio)?;
    state.borrow_mut().synthesizer = synthesizer(&mut radio, &board)?;
    state.borrow_mut().packet_controller = packet_controller(&mut radio)?;
    state.borrow_mut().packet_format = packet_format(&mut radio)?;


    // Expect 32 bit preamble
    radio.MATCH0PAT.write(0x1111_1111)?;
    radio.MATCH0LEN.write(MatchLen { len: 32, raw: true})?;
    radio.TMGRXPREAMBLE1.write(TMG { m: 1, e: 6, })?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();
                    // Example transmission from PM table 16
                    //ax5043_transmit(&mut radio_tx, &[0xAA, 0xAA, 0x1A])?;
                    let signal = get_signal(&mut radio)?;
                    state.borrow_mut().rx.push_back(signal);
                    if state.borrow().rx.len() > 100 {
                        state.borrow_mut().rx.pop_front();
                    }

                    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
                    state.borrow_mut().powstat = radio.POWSTAT.read()?;
                    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
                    state.borrow_mut().radio_event = radio.RADIOEVENTREQ.read()?;
                    state.borrow_mut().radio_state = radio.RADIOSTATE.read()?;

                    if !radio.FIFOSTAT.read()?.contains(FIFOStat::EMPTY) {
                        let data = radio.FIFODATARX.read()?;
                        println!("{:?}", data);
                    }

                    let _ = term.borrow_mut().draw(|f| {
                        ui(f, &state.borrow());
                    });
                },
                //IRQ => service_irq(&mut radio_tx, &mut irqstate)?,
                STDIN => match crossterm::event::read()? {
                    Event::Key(KeyEvent{ code: KeyCode::Char('c'), ..}) => break 'outer,
                    _ => continue,
                }
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;

    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
    )?;
    terminal.show_cursor()?;

    Ok(())
}

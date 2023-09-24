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

use ax5043::*;
mod config_rpi;
use crate::config_rpi::configure_radio_rx;


fn pwr_mode<'a>(pwrmode: &PwrMode) -> Table<'a> {
    let on = Style::default().bg(Color::Gray).fg(Color::Black);
    let off = Style::default().fg(Color::DarkGray);
    Table::new(vec![
        Row::new(vec![
            Cell::from(format!("{:?}", pwrmode.mode)),
            Cell::from("REFEN").style(if pwrmode.flags.contains(PwrFlags::REFEN) { on } else { off }),
            Cell::from("XOEN").style(if pwrmode.flags.contains(PwrFlags::XOEN) { on } else { off }),
        ])
    ])
    .block(Block::default().borders(Borders::ALL).title("PWRMODE"))
    .widths(&[Constraint::Length(8), Constraint::Length(5), Constraint::Length(4)])
    .style(Style::default().fg(Color::White))
}

fn pow_stat<'a>(powstat: &PowStat) -> Table<'a> {
    let on = Style::default().bg(Color::Gray).fg(Color::Black);
    let off = Style::default().fg(Color::DarkGray);

    Table::new(vec![
        Row::new(vec![
            Cell::from("vIO").style(if powstat.contains(PowStat::VIO) { on } else { off }),
            Cell::from("BEvMODEM").style(if powstat.contains(PowStat::BEVMODEM) { on } else { off }),
            Cell::from("BEvANA").style(if powstat.contains(PowStat::BEVANA) { on } else { off }),
            Cell::from("vMODEM").style(if powstat.contains(PowStat::VMODEM) { on } else { off }),
            Cell::from("vANA").style(if powstat.contains(PowStat::VANA) { on } else { off }),
            Cell::from("vREF").style(if powstat.contains(PowStat::VREF) { on } else { off }),
            Cell::from("REF").style(if powstat.contains(PowStat::REF) { on } else { off }),
            Cell::from("SUM").style(if powstat.contains(PowStat::SUM) { on } else { off })
        ])
    ])
    .block(Block::default().borders(Borders::ALL).title("POWSTAT"))
    .widths(&[Constraint::Max(3), Constraint::Max(8), Constraint::Max(6), Constraint::Max(6), Constraint::Max(4), Constraint::Max(4), Constraint::Max(3), Constraint::Max(3)])
}

fn irq<'a>(irq: &IRQ) -> Table<'a> {
    let on = Style::default().bg(Color::Gray).fg(Color::Black);
    let off = Style::default().fg(Color::DarkGray);
    Table::new(vec![
        Row::new(vec![
            Cell::from("PLLUNLOCK").style(if irq.contains(IRQ::PLLUNLOCK) { on } else { off }),
            Cell::from("RADIOCTRL").style(if irq.contains(IRQ::RADIOCTRL) { on } else { off }),
            Cell::from("POWER").style(if irq.contains(IRQ::POWER) { on } else { off }),
            Cell::from("XTALREADY").style(if irq.contains(IRQ::XTALREADY) { on } else { off }),
            Cell::from("WAKEUPTIMER").style(if irq.contains(IRQ::WAKEUPTIMER) { on } else { off }),
            Cell::from("LPOSC").style(if irq.contains(IRQ::LPOSC) { on } else { off }),
            Cell::from("GPADC").style(if irq.contains(IRQ::GPADC) { on } else { off }),
            Cell::from("PLLRNGDONE").style(if irq.contains(IRQ::PLLRNGDONE) { on } else { off }),
        ]),
        Row::new(vec![
            Cell::from("FIFONOTEMPTY").style(if irq.contains(IRQ::FIFONOTEMPTY) { on } else { off }),
            Cell::from("FIFONOTFULL").style(if irq.contains(IRQ::FIFONOTFULL) { on } else { off }),
            Cell::from("FIFOTHRCNT").style(if irq.contains(IRQ::FIFOTHRCNT) { on } else { off }),
            Cell::from("FIFOTHRFREE").style(if irq.contains(IRQ::FIFOTHRFREE) { on } else { off }),
            Cell::from("FIFOERROR").style(if irq.contains(IRQ::FIFOERROR) { on } else { off }),
        ])
    ])
    .block(Block::default().borders(Borders::ALL).title("IRQ"))
    .widths(&[Constraint::Max(12), Constraint::Max(11), Constraint::Max(10), Constraint::Max(11), Constraint::Max(11), Constraint::Max(5), Constraint::Max(5), Constraint::Max(10)])
}

fn spi_status<'a>(status: &Status) -> Table<'a> {
    let on = Style::default().bg(Color::Gray).fg(Color::Black);
    let off = Style::default().fg(Color::DarkGray);

    Table::new(vec![
        Row::new(vec![
            Cell::from("READY"           ).style(if status.contains(Status::READY) { on } else { off }),
            Cell::from("PLL_LOCK"        ).style(if status.contains(Status::PLL_LOCK) { on } else { off }),
            Cell::from("FIFO_OVER"       ).style(if status.contains(Status::FIFO_OVER) { on } else { off }),
            Cell::from("FIFO_UNDER"      ).style(if status.contains(Status::FIFO_UNDER) { on } else { off }),
            Cell::from("THRESHOLD_FREE"  ).style(if status.contains(Status::THRESHOLD_FREE) { on } else { off }),
            Cell::from("THRESHOLD_COUNT" ).style(if status.contains(Status::THRESHOLD_COUNT) { on } else { off }),
            Cell::from("FIFO_FULL"       ).style(if status.contains(Status::FIFO_FULL) { on } else { off }),
            Cell::from("FIFO_EMPTY"      ).style(if status.contains(Status::FIFO_EMPTY) { on } else { off }),
            Cell::from("PWR_GOOD"        ).style(if status.contains(Status::PWR_GOOD) { on } else { off }),
            Cell::from("PWR_INTERRUPT"   ).style(if status.contains(Status::PWR_INTERRUPT) { on } else { off }),
            Cell::from("RADIO_EVENT"     ).style(if status.contains(Status::RADIO_EVENT) { on } else { off }),
            Cell::from("XTAL_OSC_RUNNING").style(if status.contains(Status::XTAL_OSC_RUNNING) { on } else { off }),
            Cell::from("WAKEUP_INTERRUPT").style(if status.contains(Status::WAKEUP_INTERRUPT) { on } else { off }),
            Cell::from("LPOSC_INTERRUPT" ).style(if status.contains(Status::LPOSC_INTERRUPT) { on } else { off }),
            Cell::from("GPADC_INTERRUPT" ).style(if status.contains(Status::GPADC_INTERRUPT) { on } else { off }),
        ])
    ])
    .block(Block::default().borders(Borders::ALL).title("STATUS"))
    .widths(&[Constraint::Max(5), Constraint::Max(8), Constraint::Max(9), Constraint::Max(10),
              Constraint::Max(14), Constraint::Max(15), Constraint::Max(9), Constraint::Max(10),
              Constraint::Max(8), Constraint::Max(13), Constraint::Max(11), Constraint::Max(16),
              Constraint::Max(16), Constraint::Max(15), Constraint::Max(15)])
}

fn spark<'a, T>(name: T, unit: T, data: &'a [u64]) -> Sparkline<'a> where T: AsRef<str> + std::fmt::Display {
    let min = data.iter().min().unwrap_or(&0);
    let max = data.iter().max().unwrap_or(&0);
    let name: &str = name.as_ref();
    Sparkline::default()
        .block(Block::default().title(format!("{name} ({min} {unit} - {max} {unit})")).borders(Borders::ALL))
        .data(data)
        .style(Style::default().fg(Color::Red).bg(Color::White))
}


fn rx_params<'a>(data: &'a [u64], last: RxParamCurSet) -> Sparkline<'a> {
    Sparkline::default()
    .block(Block::default().borders(Borders::ALL).title(format!("Current RX Parameters - stage {} ({:?}) special {}", last.index, last.number, last.special)))
    .data(data)
    .style(Style::default().fg(Color::Red).bg(Color::White))
}

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

fn receiver_parameter_set<'a>(set: &RXParameterSet, active: bool) -> Table<'a> {
    /* AGC Gain
     * AGC Target
     * AGC AHyst
     * AGC MinMax
     * Gain - Time
     * Gain - Data Rate
     * Gain - Phase
     * Gain - Freq A
     * Gain - Freq B
     * Gain - Freq C
     * Gain - Freq D
     * Gain - Ampl
     * Freq Deviaiton
     * FourFSK
     * BB Offset Resistor
     */

    let on = Style::default().bg(Color::Gray).fg(Color::Black);
    let off = Style::default().fg(Color::DarkGray);

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
    .style(if active { on } else { off })
}

#[derive(Default)]
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


fn receiver_parameters<'a>(param: &RXParams) -> Table<'a> {
/* IF Frequency
 * Decimation
 * Data Rate
 * Max DR Offset
 * Max RF Offset
 * FSK Dev Max
 * FSK Dev Min
 * AFSK Space
 * AFSK Mark
 * AFSK Ctrl
 * Ampl Filt
 * Freq Leak
 *
 * RX Param Set
 * RX Param Cur
 */
    Table::new(vec![
        Row::new(vec![
            "IF Freq", "Max Δ", "Baseband", "Bitrate", "Max Δ"
        ]),
        Row::new(vec![
            Cell::from(format!("{} Hz", param.iffreq)),
            Cell::from(format!("{} Hz", param.maxrfoffset)),
            Cell::from(format!("{} Hz", param.baseband)),
            Cell::from(format!("{} bit/s", param.bitrate)),
            Cell::from(format!("{} bit/s", param.maxdroffset)),
        ]),
        Row::new(vec![
            "FSK Dev Max", "Min", "AFSK Space", "Mark", "Ctrl"
        ]),
        Row::new(vec![
            Cell::from(format!("{} Hz", param.fskdevmax)),
            Cell::from(format!("{} Hz", param.fskdevmin)),
            Cell::from(param.afskspace.to_string()),
            Cell::from(param.afskmark.to_string()),
            Cell::from(param.afskctrl.to_string()),
        ]),
        Row::new(vec![
            "Ampl Filt", "Freq Leak", "RX 0", "RX 1", "RX 2", "RX 3"
        ]),
        Row::new(vec![
            Cell::from(param.amplfilt.to_string()),
            Cell::from(param.freqleak.to_string()),
            Cell::from(format!("{:?}", param.rxparamset.ps0)),
            Cell::from(format!("{:?}", param.rxparamset.ps1)),
            Cell::from(format!("{:?}", param.rxparamset.ps2)),
            Cell::from(format!("{:?}", param.rxparamset.ps3)),
        ]),

    ])
    .block(Block::default().borders(Borders::ALL).title("RX Parameters"))
    .widths(&[Constraint::Max(11), Constraint::Max(10), Constraint::Max(11), Constraint::Max(10),
           Constraint::Max(14), Constraint::Max(15)])

}


struct UIState {
    board: config::Board,
    rx: VecDeque<RXState>,
    status: Status,
    pwrmode: PwrMode,
    powstat: PowStat,
    irq: IRQ,
    rxparams: RXParams,
    set0 : RXParameterSet,
    set1 : RXParameterSet,
    set2 : RXParameterSet,
    set3 : RXParameterSet,
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
            Constraint::Min(50),
            Constraint::Min(50),
            Constraint::Min(0),
        ].as_ref())
        .split(chunks[0]);

    f.render_widget(pwr_mode(&state.pwrmode), power[0]);
    f.render_widget(pow_stat(&state.powstat), power[1]);
    f.render_widget(irq(&state.irq), power[2]);

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

    f.render_widget(receiver_parameters(&state.rxparams), parameters[0]);
    f.render_widget(receiver_parameter_set(&state.set0, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set0), parameters[1]);
    f.render_widget(receiver_parameter_set(&state.set1, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set1), parameters[2]);
    f.render_widget(receiver_parameter_set(&state.set2, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set2), parameters[3]);
    f.render_widget(receiver_parameter_set(&state.set3, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set3), parameters[4]);

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
    f.render_widget(spark("RSSI", "dB", &state.rx.iter().map(|r| u64::from(r.rssi)).collect::<Vec<u64>>()), sparks[0]);
    f.render_widget(spark("Background RSSI", "dB",  &state.rx.iter().map(|r| u64::from(r.bgndrssi)).collect::<Vec<u64>>()), sparks[1]);
    f.render_widget(spark("AGC Counter", "dB", &state.rx.iter().map(|r| u64::from(r.agccounter)).collect::<Vec<u64>>()), sparks[2]);
    f.render_widget(spark("Data Rate", "bits/s", &state.rx.iter().map(|r| u64::from(r.datarate)).collect::<Vec<u64>>()), sparks[3]);
    f.render_widget(spark("Amplitude", "", &state.rx.iter().map(|r| u64::from(r.ampl)).collect::<Vec<u64>>()), sparks[4]);
    f.render_widget(spark("Phase", "", &state.rx.iter().map(|r| u64::from(r.phase)).collect::<Vec<u64>>()), sparks[5]);
    //f.render_widget(spark("FSK Demodulation", "", &state.rx.iter().map(|r| r.fskdemod).collect::<Vec<u64>>()), sparks[6]);
    f.render_widget(spark("RF Frequency", "Hz", &state.rx.iter().map(|r| u64::from(r.rffreq)).collect::<Vec<u64>>()), sparks[7]);
    f.render_widget(spark("Frequency", "Hz", &state.rx.iter().map(|r| u64::from(r.freq)).collect::<Vec<u64>>()), sparks[8]);
    f.render_widget(rx_params(&state.rx.iter().map(|r| u64::from(r.paramcurset.index)).collect::<Vec<u64>>(), state.rx.back().unwrap_or(&RXState::default()).paramcurset), sparks[9]);
    f.render_widget(spi_status(&state.status), chunks[2]);
}

pub fn ax5043_listen(radio: &mut Registers) -> io::Result<()> {

    // pll not locked
    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::SYNTHRX,
    })?;
    let mut fifo = fifo::FIFO {
        threshold: 0,
        autocommit: false,
        radio,
    };
    fifo.reset()?;

    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    Ok(())
}

#[derive(Default)]
struct RXState {
    rssi: u8,
    bgndrssi: u8,
    agccounter: u32,
    datarate: u32,
    ampl: u16,
    phase: u16,
    fskdemod: i32,
    rffreq: u32,
    freq: u16,
    paramcurset: RxParamCurSet,
}

fn get_signal(radio: &mut Registers) -> io::Result<RXState> {
    Ok(RXState {
        rssi: radio.RSSI.read()?,
        bgndrssi: radio.BGNDRSSI.read()?,
        agccounter: (radio.AGCCOUNTER.read()? as u32 * 4) / 3,
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
        rffreq: radio.TRKRFFREQ.read()?,
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

    let mut uistate = UIState {
        board: config::Board::default(),
        rx: VecDeque::new(),
        status: Status::default(),
        pwrmode: PwrMode::default(),
        powstat: PowStat::default(),
        irq: IRQ::default(),
        rxparams: RXParams::default(),
        set0: RXParameterSet::default(),
        set1: RXParameterSet::default(),
        set2: RXParameterSet::default(),
        set3: RXParameterSet::default(),
    };

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
    let mut radio = ax5043::registers(&spi0, &callback);

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

    state.borrow_mut().rxparams = get_rx_params(&mut radio, &board)?;
    state.borrow_mut().set0 = get_rx_parameter_set0(&mut radio)?;
    state.borrow_mut().set1 = get_rx_parameter_set1(&mut radio)?;
    state.borrow_mut().set2 = get_rx_parameter_set2(&mut radio)?;
    state.borrow_mut().set3 = get_rx_parameter_set3(&mut radio)?;

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
                    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
                    state.borrow_mut().powstat = radio.POWSTAT.read()?;
                    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
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

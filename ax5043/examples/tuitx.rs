use std::{io, cell, time::Duration, os::fd::AsRawFd, panic, backtrace::Backtrace};
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

use ax5043::{*, registers::*, tui::*,};
mod config_rpi;
use crate::config_rpi::configure_radio_tx;

struct UIState {
    board: config::Board,
    status: Status,
    pwrmode: PwrMode,
    powstat: PowStat,
    irq: IRQ,
    radio_event: RadioEvent,
    radio_state: RadioState,
    synthesizer: Synthesizer,
    tx: TXParameters,
    chan: ChannelParameters,
}

impl Default for UIState {
    fn default() -> Self {
        Self {
            board: config::Board::default(),
            status: Status::empty(),
            pwrmode: PwrMode {
                mode: PwrModes::POWEROFF,
                flags: PwrFlags::empty(),
            },
            powstat: PowStat::empty(),
            irq: IRQ::empty(),
            radio_event: RadioEvent::empty(),
            radio_state: RadioState::IDLE,
            synthesizer: Synthesizer::default(),
            tx: TXParameters::default(),
            chan: ChannelParameters::default(),
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

    f.render_widget(state.pwrmode.widget(), power[0]);
    f.render_widget(state.powstat.widget(), power[1]);
    f.render_widget(state.irq.widget(), power[2]);
    f.render_widget(state.radio_event.widget(), power[3]);
    f.render_widget(state.radio_state.widget(), power[4]);
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
        .split(chunks[1]);

    f.render_widget(state.synthesizer.widget(), parameters[0]);
    f.render_widget(state.tx.widget(), parameters[1]);
    f.render_widget(state.chan.widget(), parameters[2]);
/*
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

    f.render_widget(state.receiver_parameters(), parameters[0]);
    f.render_widget(state.receiver_parameter_set(&state.set0, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set0), parameters[1]);
    f.render_widget(state.receiver_parameter_set(&state.set1, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set1), parameters[2]);
    f.render_widget(state.receiver_parameter_set(&state.set2, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set2), parameters[3]);
    f.render_widget(state.receiver_parameter_set(&state.set3, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set3), parameters[4]);

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
    f.render_widget(state.sparkline("RSSI", "dB", &state.rx.iter().map(|r| u64::from(r.rssi)).collect::<Vec<u64>>()), sparks[0]);
    f.render_widget(state.sparkline("Background RSSI", "dB",  &state.rx.iter().map(|r| u64::from(r.bgndrssi)).collect::<Vec<u64>>()), sparks[1]);
    f.render_widget(state.sparkline("AGC Counter", "dB", &state.rx.iter().map(|r| u64::from(r.agccounter)).collect::<Vec<u64>>()), sparks[2]);
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
*/

    f.render_widget(state.status.widget(), chunks[2]);
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

    let spi0 = ax5043::open("/dev/spidev0.0")?;
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

    let mut tfd = TimerFd::new_custom(timerfd::ClockId::Monotonic, true, false).unwrap();
    tfd.set_state(TimerState::Periodic{current: Duration::new(1, 0), interval: Duration::from_millis(500)}, SetTimeFlags::Default);
    const BEACON: Token = Token(2);
    registry.register(
        &mut SourceFd(&tfd.as_raw_fd()),
        BEACON,
        Interest::READABLE)?;

    let board = configure_radio_tx(&mut radio)?;
    radio.RADIOEVENTMASK.write(RadioEvent::all())?;

    state.borrow_mut().board = board.clone();
    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
    state.borrow_mut().powstat = radio.POWSTAT.read()?;
    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });


    radio.PWRMODE.write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
    state.borrow_mut().powstat = radio.POWSTAT.read()?;
    state.borrow_mut().irq = radio.IRQREQUEST.read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    state.borrow_mut().synthesizer = Synthesizer::new(&mut radio, &board)?;
    state.borrow_mut().tx = TXParameters::new(&mut radio, &board)?;
    state.borrow_mut().chan = ChannelParameters::new(&mut radio)?;

    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT.read()?; // clear sticky power flags for PWR_GOOD

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();

                    radio.PWRMODE.write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::TX,
                    })?;

                    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT.read()?; // clear sticky power flags for PWR_GOOD

                    radio.FIFOCMD.write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_ERROR,
                        auto_commit: false,
                    })?;
                    radio.FIFOCMD.write(FIFOCmd {
                        mode: FIFOCmds::CLEAR_DATA,
                        auto_commit: false,
                    })?;



                    // Preamble - see PM p16
                    let preamble = FIFOChunkTX::REPEATDATA {
                        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
                        count: 4,
                        data: 0x11,
                    };
                    let packet = FIFOChunkTX::REPEATDATA {
                        flags: FIFODataTXFlags::PKTSTART | FIFODataTXFlags::PKTEND,
                        count: 10,
                        data: 0x3,
                    };

                    radio.FIFODATATX.write(preamble)?;
                    radio.FIFODATATX.write(packet)?;

                    radio.FIFOCMD.write(FIFOCmd {
                        mode: FIFOCmds::COMMIT,
                        auto_commit: false,
                    })?;

                    // FIXME: FIFOSTAT CLEAR?

                    loop  { // TODO: Interrupt of some sort

                        _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                        state.borrow_mut().pwrmode = radio.PWRMODE.read()?;
                        state.borrow_mut().powstat = radio.POWSTAT.read()?;
                        state.borrow_mut().irq = radio.IRQREQUEST.read()?;
                        state.borrow_mut().radio_event = radio.RADIOEVENTREQ.read()?;
                        let radiostate = radio.RADIOSTATE.read()?;
                        state.borrow_mut().radio_state = radiostate;
                        let _ = term.borrow_mut().draw(|f| {
                            ui(f, &state.borrow());
                        });

                        if radiostate == RadioState::IDLE {
                            break
                        }
                    }
                    _ = radio.PLLRANGINGA.read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT.read()?; // clear sticky power flags for PWR_GOOD
                    radio.PWRMODE.write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::POWEROFF,
                    })?;

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

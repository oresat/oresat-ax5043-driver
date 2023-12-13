use anyhow::Result;
use crossterm::{
    event::{Event, KeyCode, KeyEvent},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use ratatui::{backend::CrosstermBackend, prelude::*, Terminal};
use std::{backtrace::Backtrace, cell, io, os::fd::AsRawFd, panic, time::Duration};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

use ax5043::config_rpi::configure_radio_tx;
use ax5043::{registers::*, tui::*, *};

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
            ]
            .as_ref(),
        )
        .split(f.size());

    let power = Layout::default()
        .direction(Direction::Horizontal)
        .constraints(
            [
                Constraint::Min(21),
                Constraint::Min(46),
                Constraint::Min(85),
                Constraint::Min(40),
                Constraint::Min(0),
            ]
            .as_ref(),
        )
        .split(chunks[0]);

    f.render_widget(state.pwrmode.widget(), power[0]);
    f.render_widget(state.powstat.widget(), power[1]);
    f.render_widget(state.irq.widget(), power[2]);
    f.render_widget(state.radio_event.widget(), power[3]);
    f.render_widget(state.radio_state.widget(), power[4]);
    let parameters = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints(
            [
                Constraint::Percentage(20),
                Constraint::Percentage(20),
                Constraint::Percentage(20),
                Constraint::Percentage(20),
                Constraint::Percentage(20),
            ]
            .as_ref(),
        )
        .split(chunks[1]);

    f.render_widget(state.synthesizer.widget(), parameters[0]);
    f.render_widget(state.tx.widget(), parameters[1]);
    f.render_widget(state.chan.widget(), parameters[2]);
    f.render_widget(state.status.widget(), chunks[2]);
}

fn main() -> Result<()> {
    panic::set_hook(Box::new(|panic| {
        disable_raw_mode().unwrap();
        execute!(io::stdout(), LeaveAlternateScreen,).unwrap();

        println!("{}", panic);
        println!("{}", Backtrace::capture());
    }));

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen,)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut uistate = UIState::default();
    terminal.draw(|f| {
        ui(f, &uistate);
    })?;

    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let state = cell::RefCell::new(&mut uistate);
    let term = cell::RefCell::new(&mut terminal);
    let mut callback = |_: &_, _, new, _: &_| {
        if new == state.borrow().status {
            return;
        }
        state.borrow_mut().status = new;
        let _ = term.borrow_mut().draw(|f| {
            ui(f, &state.borrow());
        });
    };
    let mut radio = ax5043::Registers::new(spi0, &mut callback);

    radio.reset()?;

    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
    state.borrow_mut().powstat = radio.POWSTAT().read()?;
    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
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
        Interest::READABLE,
    )?;

    let mut tfd = TimerFd::new_custom(timerfd::ClockId::Monotonic, true, false).unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(2000),
        },
        SetTimeFlags::Default,
    );
    const BEACON: Token = Token(2);
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), BEACON, Interest::READABLE)?;

    let board = configure_radio_tx(&mut radio)?;
    radio.RADIOEVENTMASK().write(RadioEvent::all())?;

    state.borrow_mut().board = board.clone();
    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
    state.borrow_mut().powstat = radio.POWSTAT().read()?;
    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::TX,
    })?;

    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
    state.borrow_mut().powstat = radio.POWSTAT().read()?;
    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    state.borrow_mut().synthesizer = Synthesizer::new(&mut radio, &board)?;
    state.borrow_mut().tx = TXParameters::new(&mut radio, &board)?;
    state.borrow_mut().chan = ChannelParameters::new(&mut radio)?;

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();

                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::TX,
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

                    radio.FIFOTHRESH().write(128)?;
                    // Preamble - see PM p16
                    let preamble = FIFOChunkTX::REPEATDATA {
                        flags: FIFODataTXFlags::RAW | FIFODataTXFlags::NOCRC,
                        count: 44,
                        data: 0x11,
                    };
                    let packets = [ FIFOChunkTX::REPEATDATA {
                            flags: FIFODataTXFlags::PKTSTART,
                            count: 128,
                            data: 0x3,
                        },
                        FIFOChunkTX::REPEATDATA {
                            flags: FIFODataTXFlags::empty(),
                            count: 128,
                            data: 0x3,
                        },
                        FIFOChunkTX::REPEATDATA {
                            flags: FIFODataTXFlags::empty(),
                            count: 128,
                            data: 0x3,
                        },
                        FIFOChunkTX::REPEATDATA {
                            flags: FIFODataTXFlags::PKTEND,
                            count: 128,
                            data: 0x3,
                        },
                    ];

                    radio.FIFODATATX().write(preamble)?;

                    for packet in packets {
                        radio.FIFODATATX().write(packet)?;
                        radio.FIFOCMD().write(FIFOCmd {
                            mode: FIFOCmds::COMMIT,
                            auto_commit: false,
                        })?;
                        while !radio.FIFOSTAT().read()?.contains(FIFOStat::FREE_THR) {}
                    }
                    // FIXME: FIFOSTAT CLEAR?

                    loop {
                        // TODO: Interrupt of some sort

                        _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                        state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
                        state.borrow_mut().powstat = radio.POWSTAT().read()?;
                        state.borrow_mut().irq = radio.IRQREQUEST().read()?;
                        state.borrow_mut().radio_event = radio.RADIOEVENTREQ().read()?;
                        let radiostate = radio.RADIOSTATE().read()?;
                        state.borrow_mut().radio_state = radiostate;
                        let _ = term.borrow_mut().draw(|f| {
                            ui(f, &state.borrow());
                        });

                        if radiostate == RadioState::IDLE {
                            break;
                        }
                    }
                    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD
                    radio.PWRMODE().write(PwrMode {
                        flags: PwrFlags::XOEN | PwrFlags::REFEN,
                        mode: PwrModes::POWEROFF,
                    })?;
                }
                //IRQ => service_irq(&mut radio_tx, &mut irqstate)?,
                STDIN => match crossterm::event::read()? {
                    Event::Key(KeyEvent {
                        code: KeyCode::Char('c'),
                        ..
                    }) => break 'outer,
                    _ => continue,
                },
                CTRLC => break 'outer,
                _ => unreachable!(),
            }
        }
    }

    radio.reset()?;

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen,)?;
    terminal.show_cursor()?;

    Ok(())
}

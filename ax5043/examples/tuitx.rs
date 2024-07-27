use anyhow::Result;
use crossterm::{
    event::{Event, KeyCode, KeyEvent},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use mio::{net::UdpSocket, unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use ratatui::{backend::CrosstermBackend, prelude::*, Terminal};
use std::{
    backtrace::Backtrace,
    collections::VecDeque,
    io,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    os::fd::AsRawFd,
    panic,
};

use ax5043::{registers::*, tui::*, *};

struct UIState {
    board: config::Board,
    packets: VecDeque<(usize, usize, FIFOChunkTX)>,
    status: Status,
    reg: StatusRegisters,
    config: Config,
    tx: TXParameters,
    chan: ChannelParameters,
}

impl Default for UIState {
    fn default() -> Self {
        Self {
            board: config::Board::default(),
            packets: VecDeque::<(usize, usize, FIFOChunkTX)>::default(),
            status: Status::empty(),
            reg: StatusRegisters {
                ranginga: PLLRanging {
                    vcor: 0,
                    flags: PLLRangingFlags::empty(),
                },
                pwrmode: PwrMode {
                    mode: PwrModes::POWEROFF,
                    flags: PwrFlags::empty(),
                },
                powstat: PowStat::empty(),
                irq: IRQ::empty(),
                radio_event: RadioEvent::empty(),
                radio_state: RadioState::IDLE,
            },
            config: Config {
                txparams: TXParameters::default(),
                rxparams: RXParams::default(),
                set0: RXParameterSet::default(),
                set1: RXParameterSet::default(),
                set2: RXParameterSet::default(),
                set3: RXParameterSet::default(),
                synthesizer: Synthesizer::default(),
                packet_controller: PacketController::default(),
                packet_format: PacketFormat::default(),
                channel: ChannelParameters::default(),
            },
            tx: TXParameters::default(),
            chan: ChannelParameters::default(),
        }
    }
}

fn ui(f: &mut Frame, state: &UIState) {
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

    f.render_widget(state.reg.pwrmode, power[0]);
    f.render_widget(state.reg.powstat, power[1]);
    f.render_widget(state.reg.irq, power[2]);
    f.render_widget(state.reg.radio_event, power[3]);
    f.render_widget(state.reg.radio_state, power[4]);
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

    f.render_widget(state.config.synthesizer, parameters[0]);
    f.render_widget(state.tx, parameters[1]);
    f.render_widget(state.chan, parameters[2]);
    f.render_widget(state.status, chunks[2]);
}

fn run_ui(terminal: &mut Terminal<CrosstermBackend<io::Stdout>>) -> Result<()> {
    let mut state = UIState::default();
    terminal.draw(|f| {
        ui(f, &state);
    })?;

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

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 10035);
    let mut uplink = UdpSocket::bind(src)?;
    const TELEMETRY: Token = Token(2);
    registry.register(&mut uplink, TELEMETRY, Interest::READABLE)?;

    let mut counter = 0;
    let mut events = Events::with_capacity(128);

    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                TELEMETRY => {
                    loop {
                        let mut buf = [0; 2048];
                        let Ok(amt) = uplink.recv(&mut buf) else {
                            break;
                        };
                        match ciborium::de::from_reader(&buf[..amt])? {
                            CommState::TX(chunk) => {
                                state.packets.push_front((counter, 0 /*len*/, chunk));
                                state.packets.truncate(10);
                                counter += 1;
                            }
                            //ERR(Result<()>) =>
                            CommState::STATUS(status) => {
                                state.status = status;
                            }
                            CommState::REGISTERS(reg) => state.reg = reg,
                            CommState::BOARD(board) => state.board = board,
                            CommState::CONFIG(conf) => state.config = conf,
                            CommState::RX(_) => (),
                            // TODO: does TX have any visible state?
                            //CommState::STATE(state) => {
                            //    state.rx.push_front(state);
                            //    state.rx.truncate(100);
                            //}
                            CommState::STATE(_) => (),
                        }
                        terminal.draw(|f| {
                            ui(f, &state);
                        })?;
                    }
                }
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

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen,)?;
    terminal.show_cursor()?;

    Ok(())
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

    let err = run_ui(&mut terminal);

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen,)?;
    terminal.show_cursor()?;

    err
}

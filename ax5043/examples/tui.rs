use anyhow::Result;
use ax5043::registers::*;
use ax5043::tui::*;
use ax5043::*;
use ciborium;
use crossterm::{
    event::{Event, KeyCode, KeyEvent},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use itertools::Itertools;
use mio::{net::UdpSocket, unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use ratatui::{backend::CrosstermBackend, prelude::*, widgets::*, Terminal};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::{backtrace::Backtrace, collections::VecDeque, io, os::fd::AsRawFd, panic};

// FIXME: Default isn't really the way to go, maybe ::new()?

struct UIState {
    board: config::Board,
    rx: VecDeque<RXState>,
    packets: VecDeque<(usize, usize, FIFOChunkRX)>,
    status: Status,
    reg: StatusRegisters,
    config: Config,
}

impl Default for UIState {
    fn default() -> Self {
        Self {
            board: config::Board::default(),
            rx: VecDeque::<RXState>::default(),
            packets: VecDeque::<(usize, usize, FIFOChunkRX)>::default(),
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
        }
    }
}

impl UIState {
    fn chart<'a, T>(&self, f: &mut Frame, area: Rect, name: T, unit: T, data: &'a [f64])
    where
        T: AsRef<str> + std::fmt::Display,
    {
        let min = data.iter().copied().fold(f64::NAN, f64::min).round() - 1.0;
        let max = data.iter().copied().fold(f64::NAN, f64::max).round() + 1.0;
        let values = data
            .iter()
            .enumerate()
            .map(|(i, v)| (i as f64, *v))
            .collect::<Vec<(f64, f64)>>();
        let set = vec![Dataset::default()
            .graph_type(GraphType::Line)
            .marker(symbols::Marker::Braille)
            .style(Style::default().fg(Color::Red))
            .data(&values)];

        let x_axis = Axis::default().bounds([0.0, data.len() as f64]);

        let y_axis = Axis::default()
            .bounds([min, max])
            .labels(vec![min.to_string().into(), max.to_string().into()]);

        let chart = Chart::new(set)
            .block(Block::default().title(format!("{} ({})", name, unit)))
            .style(Style::default().fg(Color::Black).bg(Color::White))
            .x_axis(x_axis)
            .y_axis(y_axis);

        f.render_widget(chart, area);
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

    f.render_widget(state.reg.pwrmode.widget(), power[0]);
    f.render_widget(state.reg.powstat.widget(), power[1]);
    f.render_widget(state.reg.irq.widget(), power[2]);
    f.render_widget(state.reg.radio_event.widget(), power[3]);
    f.render_widget(state.reg.radio_state.widget(), power[4]);

    let rx = Layout::default()
        .direction(Direction::Horizontal)
        .margin(1)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
        .split(chunks[1]);

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
        .split(rx[1]);

    f.render_widget(state.config.synthesizer, parameters[0]);
    state.config.packet_controller.widget(f, parameters[1]);
    let packets = Paragraph::new(
        state
            .packets
            .iter()
            .map(|x| format!("{}: {} {:02X?}", x.0, x.1, x.2))
            .join("\n"),
    )
    .style(Style::default().fg(Color::Yellow))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .title("Packets received")
            .border_type(BorderType::Rounded),
    );
    f.render_widget(packets, parameters[2]);
    f.render_widget(state.config.packet_format.widget(), parameters[3]);
    f.render_widget(state.config.rxparams.widget(), parameters[4]);
    /*
        f.render_widget(state.receiver_parameter_set(&state.set0, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set0), parameters[2]);
        f.render_widget(state.receiver_parameter_set(&state.set1, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set1), parameters[2]);
        f.render_widget(state.receiver_parameter_set(&state.set2, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set2), parameters[3]);
        f.render_widget(state.receiver_parameter_set(&state.set3, state.rx.back().unwrap_or(&RXState::default()).paramcurset.number == RxParamSet::Set3), parameters[3]);
    */
    let sparks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints(
            [
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Percentage(10),
                Constraint::Min(0),
            ]
            .as_ref(),
        )
        .split(rx[0]);

    state.chart(
        f,
        sparks[0],
        "RSSI",
        "dB",
        &state.rx.iter().map(|r| r.rssi).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[1],
        "AGC Counter",
        "dB",
        &state.rx.iter().map(|r| r.agccounter).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[2],
        "Amplitude",
        "",
        &state.rx.iter().map(|r| r.ampl).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[3],
        "RF Frequency (carrier?)",
        "Δ Hz",
        &state.rx.iter().map(|r| r.rffreq).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[4],
        "Phase",
        "",
        &state.rx.iter().map(|r| r.phase).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[5],
        "Data Rate",
        "Δ bits/s",
        &state.rx.iter().map(|r| r.datarate).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[6],
        "FSK Demodulation",
        "",
        &state.rx.iter().map(|r| r.fskdemod).collect::<Vec<f64>>(),
    );
    state.chart(
        f,
        sparks[7],
        "Frequency (intermediate?)",
        "Δ Hz",
        &state.rx.iter().map(|r| r.freq).collect::<Vec<f64>>(),
    );

    // TODO: Title like how it used to be
    //.block(Block::default().borders(Borders::ALL).title(format!(
    //    "Current RX Parameters - stage {} ({:?}) special {}",
    //    last.index, last.number, last.special
    //)))
    state.chart(
        f,
        sparks[8],
        "Current RX Parameters",
        "",
        &state
            .rx
            .iter()
            .map(|r| f64::from(r.paramcurset.index))
            .collect::<Vec<f64>>(),
    );
    f.render_widget(state.status.widget(), chunks[2]);
}

fn run_ui(terminal: &mut Terminal<CrosstermBackend<io::Stdout>>) -> Result<()> {
    let mut uistate = UIState::default();
    terminal.draw(|f| {
        ui(f, &uistate);
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

    let mut counter = 0;
    let mut events = Events::with_capacity(128);

    let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 10035);
    let mut uplink = UdpSocket::bind(src)?;
    const TELEMETRY: Token = Token(2);
    registry.register(&mut uplink, TELEMETRY, Interest::READABLE)?;

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
                            CommState::RX(chunk) => {
                                uistate.packets.push_front((counter, 0 /*len*/, chunk));
                                uistate.packets.truncate(10);
                                counter += 1;
                            }
                            CommState::TX(_) => (),
                            //ERR(Result<()>) =>
                            CommState::STATUS(status) => {
                                uistate.status = status;
                            }
                            CommState::STATE(state) => {
                                uistate.rx.push_front(state);
                                uistate.rx.truncate(100);
                            }
                            CommState::REGISTERS(reg) => uistate.reg = reg,
                            CommState::BOARD(board) => uistate.board = board,
                            CommState::CONFIG(conf) => uistate.config = conf,
                        }
                        terminal.draw(|f| {
                            ui(f, &uistate);
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

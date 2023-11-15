use anyhow::Result;
use crossterm::{
    event::{Event, KeyCode, KeyEvent},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use mio::{unix::SourceFd, Events, Interest, Poll, Token};
use mio_signals::{Signal, Signals};
use ratatui::{
    backend::CrosstermBackend,
    prelude::*,
    widgets::{Block, Borders, Sparkline},
    Terminal,
};
use std::{
    backtrace::Backtrace, cell, collections::VecDeque, io, os::fd::AsRawFd, panic, time::Duration,
};
use timerfd::{SetTimeFlags, TimerFd, TimerState};

use ax5043::config_rpi::configure_radio_rx;
use ax5043::registers::*;
use ax5043::tui::*;
use ax5043::*;

// FIXME: Default isn't really the way to go, maybe ::new()?

struct UIState {
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
    set0: RXParameterSet,
    set1: RXParameterSet,
    set2: RXParameterSet,
    set3: RXParameterSet,
    synthesizer: Synthesizer,
    packet_controller: PacketController,
    packet_format: PacketFormat,
}

impl Default for UIState {
    fn default() -> Self {
        Self {
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
            set0: RXParameterSet::default(),
            set1: RXParameterSet::default(),
            set2: RXParameterSet::default(),
            set3: RXParameterSet::default(),
            synthesizer: Synthesizer::default(),
            packet_controller: PacketController::default(),
            packet_format: PacketFormat::default(),
        }
    }
}

impl UIState {
    fn sparkline<'a, T>(&self, name: T, unit: T, data: &'a [u64]) -> Sparkline<'a>
    where
        T: AsRef<str> + std::fmt::Display,
    {
        let min = data.iter().min().unwrap_or(&0);
        let max = data.iter().max().unwrap_or(&0);
        let name: &str = name.as_ref();
        Sparkline::default()
            .block(
                Block::default()
                    .title(format!("{name} ({min} {unit} - {max} {unit})"))
                    .borders(Borders::ALL),
            )
            .data(data)
            .style(self.spark)
    }

    fn spark_signed<'a, T>(
        &self,
        name: T,
        unit: T,
        data: &'a [u64],
        min: i64,
        max: i64,
    ) -> Sparkline<'a>
    where
        T: AsRef<str> + std::fmt::Display,
    {
        Sparkline::default()
            .block(
                Block::default()
                    .title(format!("{name} ({min} {unit} - {max} {unit})"))
                    .borders(Borders::ALL),
            )
            .data(data)
            .style(self.spark)
    }

    fn rx_params<'a>(&self, data: &'a [u64], last: RxParamCurSet) -> Sparkline<'a> {
        Sparkline::default()
            .block(Block::default().borders(Borders::ALL).title(format!(
                "Current RX Parameters - stage {} ({:?}) special {}",
                last.index, last.number, last.special
            )))
            .data(data)
            .style(self.spark)
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

    f.render_widget(state.synthesizer.widget(), parameters[0]);
    f.render_widget(state.packet_controller.widget(), parameters[1]);
    f.render_widget(state.packet_controller.widget_flags(), parameters[2]);
    f.render_widget(state.packet_format.widget(), parameters[3]);
    f.render_widget(state.rxparams.widget(), parameters[4]);
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
                Constraint::Percentage(10),
                Constraint::Min(0),
            ]
            .as_ref(),
        )
        .split(rx[0]);
    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.rssi))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(state.spark_signed("RSSI", "dB", data, min, max), sparks[0]);

    f.render_widget(
        state.sparkline(
            "Background RSSI",
            "dB",
            &state
                .rx
                .iter()
                .map(|r| u64::from(r.bgndrssi))
                .collect::<Vec<u64>>(),
        ),
        sparks[1],
    );

    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.agccounter))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(
        state.spark_signed("AGC Counter", "dB", data, min, max),
        sparks[2],
    );

    f.render_widget(
        state.sparkline(
            "Amplitude",
            "",
            &state
                .rx
                .iter()
                .map(|r| u64::from(r.ampl))
                .collect::<Vec<u64>>(),
        ),
        sparks[3],
    );

    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.rffreq))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(
        state.spark_signed("RF Frequency", "Hz", data, min, max),
        sparks[4],
    );

    f.render_widget(
        state.sparkline(
            "Phase",
            "",
            &state
                .rx
                .iter()
                .map(|r| u64::from(r.phase))
                .collect::<Vec<u64>>(),
        ),
        sparks[5],
    );

    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.datarate))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(
        state.spark_signed("Δ Data Rate", "bits/s", data, min, max),
        sparks[6],
    );

    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.fskdemod))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(
        state.spark_signed("FSK Demodulation", "", data, min, max),
        sparks[7],
    );

    let data = &state
        .rx
        .iter()
        .map(|r| i64::from(r.freq))
        .collect::<Vec<i64>>();
    let min: i64 = *data.iter().min().unwrap_or(&0);
    let max: i64 = *data.iter().max().unwrap_or(&0);
    let data = &data
        .iter()
        .map(|x| u64::try_from(x - min).unwrap())
        .collect::<Vec<u64>>();
    f.render_widget(
        state.spark_signed("Frequency", "Hz", data, min, max),
        sparks[8],
    );
    //f.render_widget(state.sparkline("Frequency", "Hz", &state.rx.iter().map(|r| u64::from(r.freq)).collect::<Vec<u64>>()), sparks[8]);

    f.render_widget(
        state.rx_params(
            &state
                .rx
                .iter()
                .map(|r| u64::from(r.paramcurset.index))
                .collect::<Vec<u64>>(),
            state.rx.back().unwrap_or(&RXState::default()).paramcurset,
        ),
        sparks[9],
    );
    f.render_widget(state.status.widget(), chunks[2]);
}

pub fn ax5043_listen(radio: &mut Registers) -> Result<()> {
    // pll not locked
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;

    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_ERROR,
        auto_commit: false,
    })?;
    radio.FIFOCMD().write(FIFOCmd {
        mode: FIFOCmds::CLEAR_DATA,
        auto_commit: false,
    })?;
    /*
    radio.PWRMODE().write(PwrMode {
        flags: PwrFlags::XOEN | PwrFlags::REFEN,
        mode: PwrModes::RX,
    })?;
    */
    Ok(())
}

struct RXState {
    rssi: i64,
    bgndrssi: u8,
    agccounter: i32,
    datarate: i32,
    ampl: u16,
    phase: u16,
    fskdemod: i32,
    rffreq: i32,
    freq: i64,
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

// TODO: FRAMING::FRMRX

fn get_signal(radio: &mut Registers, channel: &config::ChannelParameters) -> Result<RXState> {
    Ok(RXState {
        rssi: i64::from(radio.RSSI().read()?),
        bgndrssi: radio.BGNDRSSI().read()?,
        agccounter: (i32::from(radio.AGCCOUNTER().read()?) * 4) / 3,
        datarate: radio.TRKDATARATE().read()?,
        ampl: radio.TRKAMPL().read()?,
        phase: radio.TRKPHASE().read()?,
        fskdemod: {
            let mut demod: i32 = radio.TRKFSKDEMOD().read()?.into();
            if demod > 2_i32.pow(13) {
                demod = demod - 2_i32.pow(14)
            }
            demod
        },
        rffreq: radio.TRKRFFREQ().read()?.0,
        freq: i64::from(radio.TRKFREQ().read()?) * 9600 / 2i64.pow(16), //channel.datarate
        paramcurset: radio.RXPARAMCURSET().read()?,
    })
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

    let spi0 = ax5043::open("/dev/spidev1.0")?;
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

    let mut tfd = TimerFd::new().unwrap();
    tfd.set_state(
        TimerState::Periodic {
            current: Duration::new(1, 0),
            interval: Duration::from_millis(50),
        },
        SetTimeFlags::Default,
    );
    const BEACON: Token = Token(2);
    registry.register(&mut SourceFd(&tfd.as_raw_fd()), BEACON, Interest::READABLE)?;

    let (board, channel) = configure_radio_rx(&mut radio)?;
    radio.RADIOEVENTMASK().write(RadioEvent::all())?;

    state.borrow_mut().board = board.clone();
    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
    state.borrow_mut().powstat = radio.POWSTAT().read()?;
    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    ax5043_listen(&mut radio)?;
    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
    state.borrow_mut().powstat = radio.POWSTAT().read()?;
    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
    let _ = term.borrow_mut().draw(|f| {
        ui(f, &state.borrow());
    });

    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
    _ = radio.POWSTICKYSTAT().read()?; // clear sticky power flags for PWR_GOOD

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
    state.borrow_mut().rxparams = RXParams::new(&mut radio, &board)?;
    state.borrow_mut().set0 = RXParameterSet::set0(&mut radio)?;
    state.borrow_mut().set1 = RXParameterSet::set1(&mut radio)?;
    state.borrow_mut().set2 = RXParameterSet::set2(&mut radio)?;
    state.borrow_mut().set3 = RXParameterSet::set3(&mut radio)?;
    state.borrow_mut().synthesizer = Synthesizer::new(&mut radio, &board)?;
    state.borrow_mut().packet_controller = PacketController::new(&mut radio)?;
    state.borrow_mut().packet_format = PacketFormat::new(&mut radio)?;

    // Expect 32 bit preamble
    //radio.MATCH0PAT().write(0x7E7E_7E7E)?;
    //radio.MATCH0LEN().write(MatchLen { len: 31, raw: true})?;
    //radio.TMGRXPREAMBLE1().write(TMG { m: 1, e: 6, })?;
    //radio.TMGRXPREAMBLE3().write(TMG { m: 1, e: 6, })?;
    //radio.TMGRXRSSI().write(TMG { m: 1, e: 6, })?;
    //radio.TMGRXAGC().write(TMG { m: 1, e: 6, })?;

    let mut events = Events::with_capacity(128);
    'outer: loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                BEACON => {
                    tfd.read();
                    let signal = get_signal(&mut radio, &channel)?;
                    state.borrow_mut().rx.push_front(signal);
                    if state.borrow().rx.len() > 100 {
                        state.borrow_mut().rx.pop_back();
                    }

                    _ = radio.PLLRANGINGA().read()?; // sticky lock bit ~ IRQPLLUNLIOCK, gate
                    state.borrow_mut().pwrmode = radio.PWRMODE().read()?;
                    state.borrow_mut().powstat = radio.POWSTAT().read()?;
                    state.borrow_mut().irq = radio.IRQREQUEST().read()?;
                    state.borrow_mut().radio_event = radio.RADIOEVENTREQ().read()?;
                    state.borrow_mut().radio_state = radio.RADIOSTATE().read()?;

                    if !radio.FIFOSTAT().read()?.contains(FIFOStat::EMPTY) {
                        let len = radio.FIFOCOUNT().read()?;
                        let data = radio.FIFODATARX().read(len.into())?;
                        println!("{:X?}", data);
                        break 'outer;
                    }

                    let _ = term.borrow_mut().draw(|f| {
                        ui(f, &state.borrow());
                    });
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

use anyhow::Result;
use ax5043::{registers::*, *};
use std::time;

struct State {
    stat: PowStat,
    sstat: PowStat,
    irq: IRQ,
}

fn print_powerstate(radio: &mut Registers, state: &mut State) -> Result<()> {
    let timeout = time::Instant::now() + time::Duration::from_millis(250);
    while time::Instant::now() <= timeout {
        let read = radio.POWSTAT().read()?;
        if state.stat != read {
            println!("POWSTAT:       {:?}", read);
            state.stat = read;
        }

        let sread = radio.POWSTICKYSTAT().read()?;
        if state.sstat != sread {
            println!("POWSTICKYSTAT: {:?}", sread);
            state.sstat = sread;
        }

        let iread = radio.IRQREQUEST().read()?;
        if state.irq != iread {
            println!("IRQREQUEST:    {:?}", iread);
            state.irq = iread;
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let spi0 = ax5043::open("/dev/spidev0.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, s, _: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
    };
    let mut radio = Registers::new(spi0, &mut callback);

    radio.reset()?;

    let mut state = State {
        stat: PowStat::empty(),
        sstat: PowStat::empty(),
        irq: IRQ::empty(),
    };

    // NOTE: POWSTICKYSTAT only keeps VIO through the reset.
    // reading it causes BEVMODEM amd BEVANA (the brownout detectors) to reset
    // and the SPI status to mark PWRGOOD

    println!("PWRMODE:        {:?}", radio.PWRMODE().read()?);
    print_powerstate(&mut radio, &mut state)?;
    println!("\nRef eneable");
    // NOTE: REFEN enables REF and VREF in POWSTAT
    radio.PWRMODE().write(PwrMode {
        mode: PwrModes::POWEROFF,
        flags: PwrFlags::REFEN,
    })?;
    print_powerstate(&mut radio, &mut state)?;

    radio.reset()?;
    Ok(())
}

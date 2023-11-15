extern crate ax5043;
use clap::Parser;
use std::path::PathBuf;
use anyhow::Result;

#[derive(Parser, Debug)]
struct Args {
    #[arg(default_values_os_t=vec![PathBuf::from("/dev/spidev0.0"), PathBuf::from("/dev/spidev1.1")])]
    path: Vec<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let callback = &mut |_: &_, _, status, _: &_| {
        println!("{:?}", status);
    };

    for path in args.path.iter() {
        println!("Resetting {}", path.display());
        let spi = ax5043::open(path)?;
        let mut radio = ax5043::Registers::new(spi, callback);
        radio.reset()?;
    }
    Ok(())
}

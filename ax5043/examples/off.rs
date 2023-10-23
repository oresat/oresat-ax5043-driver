extern crate ax5043;
use clap::Parser;
use std::{io, path::PathBuf};

#[derive(Parser, Debug)]
struct Args {
    #[arg(default_values_os_t=vec![PathBuf::from("/dev/spidev0.0"), PathBuf::from("/dev/spidev1.1")])]
    path: Vec<PathBuf>,
}

fn main() -> io::Result<()> {
    let args = Args::parse();

    for path in args.path.iter() {
        println!("Resetting {}", path.display());
        let spi = ax5043::open(path)?;
        let mut radio = ax5043::Registers::new(&spi, &|status| println!("{:?}", status));
        radio.reset()?;
    }
    Ok(())
}

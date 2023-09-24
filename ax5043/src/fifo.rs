use std::io;
use bitflags::bitflags;
use spidev::SpidevTransfer;

use crate::{ Registers, FIFOCmd, FIFOCmds, FIFOCmdFlags, RadioState };

pub enum FIFOChunkRX {
    RSSI       = 0b00110001, // RSSI
    FREQOFFS   = 0b01010010, // Frequency Offset
    ANTRSSI2   = 0b01010101, // Background Noise
    TIMER      = 0b01110000, // Timer
    RFFREQOFFS = 0b01110011, // RF Frequency Offset
    DATARATE   = 0b01110100, // Datarate
    ANTRSSI3   = 0b01110101, // Antenna Selection RSSI
    DATA       = 0b11100001, // Data
}

pub enum FIFOChunkTX {
    NOP        = 0b00000000, // No Operation
    TXCTRL     = 0b00111100, // Transmit Control
    REPEATDATA = 0b01100010, // Repeat Data
    DATA       = 0b11100001, // Data
    TXPWR      = 0b11111101, // Transmit Power
}

bitflags! {
    pub struct TXDataFlags: u8 {
        const UNENC    = 1 << 5;
        const RAW      = 1 << 4;
        const NOCRC    = 1 << 3;
        const RESIDUE  = 1 << 2;
        const PKTEND   = 1 << 1;
        const PKTSTART = 1 << 0;
    }
}

pub struct TXData {
    command: FIFOChunkTX,
    length: u8,
    flags: TXDataFlags,
}


pub struct FIFO<'a, 'b> {
    pub threshold: u16,
    pub autocommit: bool,
    // TODO: interrupt config?
    pub radio: &'a mut Registers<'b>,
}

pub struct State {
    pub free: u16,
    pub count: u16,
    // stat
}

impl FIFO<'_, '_> {
    const ADDR: u16 = 0x029;

    pub fn state(&mut self) -> io::Result<State> {
        Ok(State {
            free: self.radio.FIFOFREE.read()?,
            count: self.radio.FIFOCOUNT.read()?,
            //state: self.radio.FIFOSTAT.read()?,
        })
    }

    fn command(&mut self, cmd: FIFOCmds) -> io::Result<()> {
        // TODO: ensure that pwrmode is FULLTX/FULLRX/FIFO
        self.radio.FIFOCMD.write(FIFOCmd {
            mode: cmd,
            flags: if self.autocommit {
                FIFOCmdFlags::AUTO_COMMIT
            } else {
                FIFOCmdFlags::empty()
            },
        })

    }

    pub fn clear_error(&mut self) -> io::Result<()> {
        self.command(FIFOCmds::CLEAR_ERROR)
    }

    pub fn clear_data(&mut self) -> io::Result<()> {
        self.command(FIFOCmds::CLEAR_DATA)
    }

    pub fn reset(&mut self) -> io::Result<()> {
        self.clear_error()?;
        self.clear_data()
    }

    pub fn commit(&mut self) -> io::Result<()> {
        self.command(FIFOCmds::COMMIT)
    }

    pub fn rollback(&mut self) -> io::Result<()> {
        self.command(FIFOCmds::ROLLBACK)
    }

    pub fn write(&mut self, tx: &[u8], flags: TXDataFlags) -> io::Result<()> {

        // TODO: ensure that pwrmode is FIFOON/SYNTHTX/FULLTX
        let cmd = TXData {
            command: FIFOChunkTX::DATA,
            length:  (tx.len() + 1) as u8, // Includes flag byte (PM p.10)
            flags,
        };
        let addr = (Self::ADDR | 0xF000).to_be_bytes(); // FIXME: addr from radio
        let mut stat = [0; 2];

        let header = [cmd.command as u8, cmd.length, cmd.flags.bits()];
        let mut reader = [0; 3]; // TODO what gets read back during this phase?

        let mut rx = vec![0; tx.len()];

        self.radio.FIFODATA.spi.transfer_multiple(&mut [
                SpidevTransfer::read_write(&addr, &mut stat),
                SpidevTransfer::read_write(&header, &mut reader),
                SpidevTransfer::read_write(tx, rx.as_mut_slice()),
        ])?;

        Ok(())
    }

    // see PKTACCEPTFLAGS
    // PKTLENCFG, PKTLENOFFSET and PKTMAXLEN
    // PKTADDRCFG, PKTADDR and PKTADDRMASK
    pub fn read(&mut self) -> io::Result<()> {
        let count = self.radio.FIFOCOUNT.read()?;
        println!("fifo count:{:?}", count);

        if count == 0 {
            return Ok(());
        }

        let addr = (Self::ADDR | 0x7000).to_be_bytes(); // FIXME: addr from radio
        let mut stat = [0; 2];

        let tx = vec![0; (3 + count).into()];
        let mut rx = vec![0; (3 + count).into()];
        self.radio.FIFODATA.spi.transfer_multiple(&mut [
                SpidevTransfer::read_write(&addr, &mut stat),
                SpidevTransfer::read_write(&tx, rx.as_mut_slice()),
        ])?;

        if rx[0] != FIFOChunkRX::DATA as u8 {
            println!("Unexpected command from fifo: {:X}", rx[0]);
            return Err(io::Error::new(io::ErrorKind::Other, "Unexpected FIFO command"));
        }

        println!("rx flags: 0x{:x}", rx[2]);
        println!("data: {:?}", &rx[3..]);
        Ok(())
    }

    pub fn block_until_idle(&mut self) -> io::Result<()> {
        while self.radio.RADIOSTATE.read()? as u8 != RadioState::IDLE as u8 {} // TODO: Interrupt of some sort
        Ok(())
    }
}

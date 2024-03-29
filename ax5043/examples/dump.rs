extern crate ax5043;
use anyhow::Result;
use ax5043::*;

fn main() -> Result<()> {
    let spi = ax5043::open("/dev/spidev0.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, s, _: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
    };
    let mut radio = Registers::new(spi, &mut callback);
    radio.reset()?;
    println!("REVISION       {:?}", radio.REVISION().read()?);
    println!("SCRATCH        {:?}", radio.SCRATCH().read()?);
    println!("PWRMODE        {:?}", radio.PWRMODE().read()?);
    println!("POWSTAT        {:?}", radio.POWSTAT().read()?);
    println!("POWSTICKYSTAT  {:?}", radio.POWSTICKYSTAT().read()?);
    println!("POWIRQMASK     {:?}", radio.POWIRQMASK().read()?);
    println!("IRQMASK        {:?}", radio.IRQMASK().read()?);
    println!("RADIOEVENTMASK {:?}", radio.RADIOEVENTMASK().read()?);
    println!("IRQINVERSION   {:?}", radio.IRQINVERSION().read()?);
    println!("IRQREQUEST     {:?}", radio.IRQREQUEST().read()?);
    println!("RADIOEVENTREQ  {:?}", radio.RADIOEVENTREQ().read()?);
    println!("MODULATION     {:?}", radio.MODULATION().read()?);
    println!("ENCODING       {:?}", radio.ENCODING().read()?);
    println!("FRAMING        {:?}", radio.FRAMING().read()?);
    println!("CRCINIT        {:?}", radio.CRCINIT().read()?);
    println!("FEC            {:?}", radio.FEC().read()?);
    println!("FECSYNC        {:?}", radio.FECSYNC().read()?);
    println!("FECSTATUS      {:?}", radio.FECSTATUS().read()?);
    println!("RADIOSTATE     {:?}", radio.RADIOSTATE().read()?);
    println!("XTALSTATUS     {:?}", radio.XTALSTATUS().read()?);
    println!("PINSTATE       {:?}", radio.PINSTATE().read()?);
    println!("PINFUNCSYSCLK  {:?}", radio.PINFUNCSYSCLK().read()?);
    println!("PINFUNCDCLK    {:?}", radio.PINFUNCDCLK().read()?);
    println!("PINFUNCDATA    {:?}", radio.PINFUNCDATA().read()?);
    println!("PINFUNCIRQ     {:?}", radio.PINFUNCIRQ().read()?);
    println!("PINFUNCANTSEL  {:?}", radio.PINFUNCANTSEL().read()?);
    println!("PINFUNCPWRAMP  {:?}", radio.PINFUNCPWRAMP().read()?);
    println!("PWRAMP         {:?}", radio.PWRAMP().read()?);
    println!("FIFOSTAT       {:?}", radio.FIFOSTAT().read()?);
    println!("FIFODATA       {:?}", radio.FIFODATA().read()?);
    println!("FIFOCOUNT      {:?}", radio.FIFOCOUNT().read()?);
    println!("FIFOFREE       {:?}", radio.FIFOFREE().read()?);
    println!("FIFOTHRESH     {:?}", radio.FIFOTHRESH().read()?);
    println!("PLLLOOP        {:?}", radio.PLLLOOP().read()?);
    println!("PLLCPI         {:?}", radio.PLLCPI().read()?);
    println!("PLLVCODIV      {:?}", radio.PLLVCODIV().read()?);
    println!("PLLRANGINGA    {:?}", radio.PLLRANGINGA().read()?);
    println!("FREQA          {:?}", radio.FREQA().read()?);
    println!("PLLLOOPBOOST   {:?}", radio.PLLLOOPBOOST().read()?);
    println!("PLLCPIBOOST    {:?}", radio.PLLCPIBOOST().read()?);
    println!("PLLRANGINGB    {:?}", radio.PLLRANGINGB().read()?);
    println!("FREQB          {:?}", radio.FREQB().read()?);
    println!("RSSI           {:?}", radio.RSSI().read()?);
    println!("BGNDRSSI       {:?}", radio.BGNDRSSI().read()?);
    println!("DIVERSITY      {:?}", radio.DIVERSITY().read()?);
    println!("AGCCOUNTER     {:?}", radio.AGCCOUNTER().read()?);
    println!("TRKDATARATE    {:?}", radio.TRKDATARATE().read()?);
    println!("TRKAMPL        {:?}", radio.TRKAMPL().read()?);
    println!("TRKPHASE       {:?}", radio.TRKPHASE().read()?);
    println!("TRKRFFREQ      {:?}", radio.TRKRFFREQ().read()?);
    println!("TRKFREQ        {:?}", radio.TRKFREQ().read()?);
    println!("TRKFSKDEMOD    {:?}", radio.TRKFSKDEMOD().read()?);
    println!("TIMER2         {:?}", radio.TIMER2().read()?);
    println!("WAKEUPTIMER    {:?}", radio.WAKEUPTIMER().read()?);
    println!("WAKEUP         {:?}", radio.WAKEUP().read()?);
    println!("WAKEUPFREQ     {:?}", radio.WAKEUPFREQ().read()?);
    println!("WAKEUPXOEARLY  {:?}", radio.WAKEUPXOEARLY().read()?);
    println!("IFFREQ         {:?}", radio.IFFREQ().read()?);
    println!("DECIMATION     {:?}", radio.DECIMATION().read()?);
    println!("RXDATARATE     {:?}", radio.RXDATARATE().read()?);
    println!("MAXDROFFSET    {:?}", radio.MAXDROFFSET().read()?);
    println!("MAXRFOFFSET    {:?}", radio.MAXRFOFFSET().read()?);
    println!("FSKDMAX        {:?}", radio.FSKDMAX().read()?);
    println!("FSKDMIN        {:?}", radio.FSKDMIN().read()?);
    println!("AFSKSPACE      {:?}", radio.AFSKSPACE().read()?);
    println!("AFSKMARK       {:?}", radio.AFSKMARK().read()?);
    println!("AFSKCTRL       {:?}", radio.AFSKCTRL().read()?);
    println!("AMPLFILTER     {:?}", radio.AMPLFILTER().read()?);
    println!("FREQUENCYLEAK  {:?}", radio.FREQUENCYLEAK().read()?);
    println!("RXPARAMSETS    {:?}", radio.RXPARAMSETS().read()?);
    println!("RXPARAMCURSET  {:?}", radio.RXPARAMCURSET().read()?);
    println!("AGCGAIN0       {:?}", radio.AGCGAIN0().read()?);
    println!("AGCTARGET0     {:?}", radio.AGCTARGET0().read()?);
    println!("AGCAHYST0      {:?}", radio.AGCAHYST0().read()?);
    println!("AGCMINMAX0     {:?}", radio.AGCMINMAX0().read()?);
    println!("TIMEGAIN0      {:?}", radio.TIMEGAIN0().read()?);
    println!("DRGAIN0        {:?}", radio.DRGAIN0().read()?);
    println!("PHASEGAIN0     {:?}", radio.PHASEGAIN0().read()?);
    println!("FREQGAINA0     {:?}", radio.FREQGAINA0().read()?);
    println!("FREQGAINB0     {:?}", radio.FREQGAINB0().read()?);
    println!("FREQGAINC0     {:?}", radio.FREQGAINC0().read()?);
    println!("FREQGAIND0     {:?}", radio.FREQGAIND0().read()?);
    println!("AMPLGAIN0      {:?}", radio.AMPLGAIN0().read()?);
    println!("FREQDEV0       {:?}", radio.FREQDEV0().read()?);
    println!("FOURFSK0       {:?}", radio.FOURFSK0().read()?);
    println!("BBOFFSRES0     {:?}", radio.BBOFFSRES0().read()?);
    println!("AGCGAIN1       {:?}", radio.AGCGAIN1().read()?);
    println!("AGCTARGET1     {:?}", radio.AGCTARGET1().read()?);
    println!("AGCAHYST1      {:?}", radio.AGCAHYST1().read()?);
    println!("AGCMINMAX1     {:?}", radio.AGCMINMAX1().read()?);
    println!("TIMEGAIN1      {:?}", radio.TIMEGAIN1().read()?);
    println!("DRGAIN1        {:?}", radio.DRGAIN1().read()?);
    println!("PHASEGAIN1     {:?}", radio.PHASEGAIN1().read()?);
    println!("FREQGAINA1     {:?}", radio.FREQGAINA1().read()?);
    println!("FREQGAINB1     {:?}", radio.FREQGAINB1().read()?);
    println!("FREQGAINC1     {:?}", radio.FREQGAINC1().read()?);
    println!("FREQGAIND1     {:?}", radio.FREQGAIND1().read()?);
    println!("AMPLGAIN1      {:?}", radio.AMPLGAIN1().read()?);
    println!("FREQDEV1       {:?}", radio.FREQDEV1().read()?);
    println!("FOURFSK1       {:?}", radio.FOURFSK1().read()?);
    println!("BBOFFSRES1     {:?}", radio.BBOFFSRES1().read()?);
    println!("AGCGAIN2       {:?}", radio.AGCGAIN2().read()?);
    println!("AGCTARGET2     {:?}", radio.AGCTARGET2().read()?);
    println!("AGCAHYST2      {:?}", radio.AGCAHYST2().read()?);
    println!("AGCMINMAX2     {:?}", radio.AGCMINMAX2().read()?);
    println!("TIMEGAIN2      {:?}", radio.TIMEGAIN2().read()?);
    println!("DRGAIN2        {:?}", radio.DRGAIN2().read()?);
    println!("PHASEGAIN2     {:?}", radio.PHASEGAIN2().read()?);
    println!("FREQGAINA2     {:?}", radio.FREQGAINA2().read()?);
    println!("FREQGAINB2     {:?}", radio.FREQGAINB2().read()?);
    println!("FREQGAINC2     {:?}", radio.FREQGAINC2().read()?);
    println!("FREQGAIND2     {:?}", radio.FREQGAIND2().read()?);
    println!("AMPLGAIN2      {:?}", radio.AMPLGAIN2().read()?);
    println!("FREQDEV2       {:?}", radio.FREQDEV2().read()?);
    println!("FOURFSK2       {:?}", radio.FOURFSK2().read()?);
    println!("BBOFFSRES2     {:?}", radio.BBOFFSRES2().read()?);
    println!("AGCGAIN3       {:?}", radio.AGCGAIN3().read()?);
    println!("AGCTARGET3     {:?}", radio.AGCTARGET3().read()?);
    println!("AGCAHYST3      {:?}", radio.AGCAHYST3().read()?);
    println!("AGCMINMAX3     {:?}", radio.AGCMINMAX3().read()?);
    println!("TIMEGAIN3      {:?}", radio.TIMEGAIN3().read()?);
    println!("DRGAIN3        {:?}", radio.DRGAIN3().read()?);
    println!("PHASEGAIN3     {:?}", radio.PHASEGAIN3().read()?);
    println!("FREQGAINA3     {:?}", radio.FREQGAINA3().read()?);
    println!("FREQGAINB3     {:?}", radio.FREQGAINB3().read()?);
    println!("FREQGAINC3     {:?}", radio.FREQGAINC3().read()?);
    println!("FREQGAIND3     {:?}", radio.FREQGAIND3().read()?);
    println!("AMPLGAIN3      {:?}", radio.AMPLGAIN3().read()?);
    println!("FREQDEV3       {:?}", radio.FREQDEV3().read()?);
    println!("FOURFSK3       {:?}", radio.FOURFSK3().read()?);
    println!("BBOFFSRES3     {:?}", radio.BBOFFSRES3().read()?);
    println!("MODCFGF        {:?}", radio.MODCFGF().read()?);
    println!("FSKDEV         {:?}", radio.FSKDEV().read()?);
    println!("MODCFGA        {:?}", radio.MODCFGA().read()?);
    println!("TXRATE         {:?}", radio.TXRATE().read()?);
    println!("TXPWRCOEFFA    {:?}", radio.TXPWRCOEFFA().read()?);
    println!("TXPWRCOEFFB    {:?}", radio.TXPWRCOEFFB().read()?);
    println!("TXPWRCOEFFC    {:?}", radio.TXPWRCOEFFC().read()?);
    println!("TXPWRCOEFFD    {:?}", radio.TXPWRCOEFFD().read()?);
    println!("TXPWRCOEFFE    {:?}", radio.TXPWRCOEFFE().read()?);
    println!("PLLVCOI        {:?}", radio.PLLVCOI().read()?);
    println!("PLLVCOIR       {:?}", radio.PLLVCOIR().read()?);
    println!("PLLLOCKDET     {:?}", radio.PLLLOCKDET().read()?);
    println!("PLLRNGCLK      {:?}", radio.PLLRNGCLK().read()?);
    println!("XTALCAP        {:?}", radio.XTALCAP().read()?);
    println!("BBTUNE         {:?}", radio.BBTUNE().read()?);
    println!("BBOFFSCAP      {:?}", radio.BBOFFSCAP().read()?);
    println!("PKTADDRCFG     {:?}", radio.PKTADDRCFG().read()?);
    println!("PKTLENCFG      {:?}", radio.PKTLENCFG().read()?);
    println!("PKTLENOFFSET   {:?}", radio.PKTLENOFFSET().read()?);
    println!("PKTMAXLEN      {:?}", radio.PKTMAXLEN().read()?);
    println!("PKTADDR        {:?}", radio.PKTADDR().read()?);
    println!("PKTADDRMASK    {:?}", radio.PKTADDRMASK().read()?);
    println!("MATCH0PAT      {:?}", radio.MATCH0PAT().read()?);
    println!("MATCH0LEN      {:?}", radio.MATCH0LEN().read()?);
    println!("MATCH0MIN      {:?}", radio.MATCH0MIN().read()?);
    println!("MATCH0MAX      {:?}", radio.MATCH0MAX().read()?);
    println!("MATCH1PAT      {:?}", radio.MATCH1PAT().read()?);
    println!("MATCH1LEN      {:?}", radio.MATCH1LEN().read()?);
    println!("MATCH1MIN      {:?}", radio.MATCH1MIN().read()?);
    println!("MATCH1MAX      {:?}", radio.MATCH1MAX().read()?);
    println!("TMGTXBOOST     {:?}", radio.TMGTXBOOST().read()?);
    println!("TMGTXSETTLE    {:?}", radio.TMGTXSETTLE().read()?);
    println!("TMGRXBOOST     {:?}", radio.TMGRXBOOST().read()?);
    println!("TMGRXSETTLE    {:?}", radio.TMGRXSETTLE().read()?);
    println!("TMGRXOFFSACQ   {:?}", radio.TMGRXOFFSACQ().read()?);
    println!("TMGRXCOARSEAGC {:?}", radio.TMGRXCOARSEAGC().read()?);
    println!("TMGRXAGC       {:?}", radio.TMGRXAGC().read()?);
    println!("TMGRXRSSI      {:?}", radio.TMGRXRSSI().read()?);
    println!("TMGRXPREAMBLE1 {:?}", radio.TMGRXPREAMBLE1().read()?);
    println!("TMGRXPREAMBLE2 {:?}", radio.TMGRXPREAMBLE2().read()?);
    println!("TMGRXPREAMBLE3 {:?}", radio.TMGRXPREAMBLE3().read()?);
    println!("RSSIREFERENCE  {:?}", radio.RSSIREFERENCE().read()?);
    println!("RSSIABSTHR     {:?}", radio.RSSIABSTHR().read()?);
    println!("BGNDRSSIGAIN   {:?}", radio.BGNDRSSIGAIN().read()?);
    println!("BGNDRSSITHR    {:?}", radio.BGNDRSSITHR().read()?);
    println!("PKTCHUNKSIZE   {:?}", radio.PKTCHUNKSIZE().read()?);
    println!("PKTMISCFLAGS   {:?}", radio.PKTMISCFLAGS().read()?);
    println!("PKTSTOREFLAGS  {:?}", radio.PKTSTOREFLAGS().read()?);
    println!("PKTACCEPTFLAGS {:?}", radio.PKTACCEPTFLAGS().read()?);
    println!("GPADCCTRL      {:?}", radio.GPADCCTRL().read()?);
    println!("GPADCPERIOD    {:?}", radio.GPADCPERIOD().read()?);
    println!("GPADC13VALUE   {:?}", radio.GPADC13VALUE().read()?);
    println!("LPOSCCONFIG    {:?}", radio.LPOSCCONFIG().read()?);
    println!("LPOSCSTATUS    {:?}", radio.LPOSCSTATUS().read()?);
    println!("LPOSCKFILT     {:?}", radio.LPOSCKFILT().read()?);
    println!("LPOSCREF       {:?}", radio.LPOSCREF().read()?);
    println!("LPOSCFREQ      {:?}", radio.LPOSCFREQ().read()?);
    println!("LPOSCPER       {:?}", radio.LPOSCPER().read()?);
    println!("DACVALUE       {:?}", radio.DACVALUE().read()?);
    println!("DACCONFIG      {:?}", radio.DACCONFIG().read()?);
    println!("PERF_F00       {:?}", radio.PERF_F00().read()?);
    println!("PERF_F08       {:?}", radio.PERF_F08().read()?);
    println!("PERF_F0D       {:?}", radio.PERF_F0D().read()?);
    println!("PERF_F10       {:?}", radio.PERF_F10().read()?);
    println!("PERF_F11       {:?}", radio.PERF_F11().read()?);
    println!("PERF_F18       {:?}", radio.PERF_F18().read()?);
    println!("PERF_F1C       {:?}", radio.PERF_F1C().read()?);
    println!("PERF_F21       {:?}", radio.PERF_F21().read()?);
    println!("PERF_F22       {:?}", radio.PERF_F22().read()?);
    println!("PERF_F23       {:?}", radio.PERF_F23().read()?);
    println!("PERF_F26       {:?}", radio.PERF_F26().read()?);
    println!("PERF_F34       {:?}", radio.PERF_F34().read()?);
    println!("PERF_F35       {:?}", radio.PERF_F35().read()?);
    println!("PERF_F44       {:?}", radio.PERF_F44().read()?);

    Ok(())
}

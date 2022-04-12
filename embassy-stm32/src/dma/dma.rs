use core::sync::atomic::{fence, Ordering};
use core::task::Waker;

use embassy_util::waitqueue::AtomicWaker;

use super::{Burst, FlowControl, Request, TransferOptions, Word, WordSize};
use crate::_generated::DMA_CHANNEL_COUNT;
use crate::interrupt::{Interrupt, InterruptExt};
use crate::pac::dma::{regs, vals};
use crate::{interrupt, pac};

impl From<WordSize> for vals::Size {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BITS8,
            WordSize::TwoBytes => Self::BITS16,
            WordSize::FourBytes => Self::BITS32,
        }
    }
}

impl From<Burst> for vals::Burst {
    fn from(burst: Burst) -> Self {
        match burst {
            Burst::Single => vals::Burst::SINGLE,
            Burst::Incr4 => vals::Burst::INCR4,
            Burst::Incr8 => vals::Burst::INCR8,
            Burst::Incr16 => vals::Burst::INCR16,
        }
    }
}

impl From<FlowControl> for vals::Pfctrl {
    fn from(flow: FlowControl) -> Self {
        match flow {
            FlowControl::Dma => vals::Pfctrl::DMA,
            FlowControl::Peripheral => vals::Pfctrl::PERIPHERAL,
        }
    }
}

#[cfg(feature = "dcmi-giant-transfer")]
mod dcmi_giant_transfer {
    use atomic_polyfill::Ordering;
    use atomic_polyfill::{AtomicBool, AtomicU16, AtomicU32, AtomicU8};
    pub struct M0AR(pub u32);
    pub struct M1AR(pub u32);
    pub struct ChunkSize(pub u16);
    pub struct State {
        enabled: AtomicBool,
        data_address: AtomicU32,
        remaining_chunks: AtomicU32,
        chunk_size: AtomicU16,
        transfer_len_bytes: AtomicU8,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                enabled: AtomicBool::new(false),
                data_address: AtomicU32::new(0),
                remaining_chunks: AtomicU32::new(0),
                chunk_size: AtomicU16::new(0),
                transfer_len_bytes: AtomicU8::new(0),
            }
        }

        pub fn enable(
            &self,
            data_addr: u32,
            data_len: usize,
            transfer_len_bytes: u8,
        ) -> (M0AR, M1AR, ChunkSize) {
            assert!(data_len % 2 == 0);
            let chunk_estimate = data_len / 0xffff;

            let mut chunks = chunk_estimate + 1;
            while data_len % chunks != 0 {
                chunks += 1;
            }

            let chunk_size = data_len / chunks;

            let remaining_chunks = chunks - 2;

            trace!("chunks: {}, chunk_size {}", chunks, chunk_size);

            self.data_address.store(data_addr, Ordering::SeqCst);
            self.chunk_size.store(chunk_size as u16, Ordering::SeqCst);
            self.remaining_chunks
                .store(remaining_chunks as u32, Ordering::SeqCst);
            self.enabled.store(true, Ordering::SeqCst);
            self.transfer_len_bytes
                .store(transfer_len_bytes, Ordering::SeqCst);

            (
                M0AR(data_addr),
                M1AR(data_addr + chunk_size as u32 * transfer_len_bytes as u32),
                ChunkSize(chunk_size as u16),
            )
        }

        pub fn disable(&self) {
            self.enabled.store(false, Ordering::SeqCst);
        }

        pub fn is_enabled(&self) -> bool {
            self.enabled.load(Ordering::SeqCst)
        }

        pub fn remaining_chunks(&self) -> u32 {
            self.remaining_chunks.load(Ordering::SeqCst)
        }

        pub fn remaining_transfers(&self, ndtr: u32) -> u32 {
            ndtr + self.remaining_chunks.load(Ordering::SeqCst) * self.chunk_size() as u32
        }

        pub fn chunk_size(&self) -> u16 {
            self.chunk_size.load(Ordering::SeqCst)
        }

        pub fn transfer_len_bytes(&self) -> u8 {
            self.transfer_len_bytes.load(Ordering::SeqCst)
        }

        pub fn next_chunk_addres(&self, prev_mem_ptr: u32) -> u32 {
            prev_mem_ptr + 2 * self.chunk_size() as u32 * self.transfer_len_bytes() as u32
        }

        pub fn dequeue_next_chunk(&self, prev_mem_ptr: u32) -> u32 {
            self.remaining_chunks.fetch_sub(1, Ordering::SeqCst);
            self.next_chunk_addres(prev_mem_ptr)
        }

        pub fn data_address(&self) -> u32 {
            self.data_address.load(Ordering::SeqCst)
        }
    }
}
struct ChannelState {
    waker: AtomicWaker,
    #[cfg(feature = "dcmi-giant-transfer")]
    giant_transfer_state: dcmi_giant_transfer::State,
}

impl ChannelState {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            #[cfg(feature = "dcmi-giant-transfer")]
            giant_transfer_state: dcmi_giant_transfer::State::new(),
        }
    }
}

struct State {
    channels: [ChannelState; DMA_CHANNEL_COUNT],
}

impl State {
    const fn new() -> Self {
        const CH: ChannelState = ChannelState::new();
        Self {
            channels: [CH; DMA_CHANNEL_COUNT],
        }
    }
}

static STATE: State = State::new();

/// safety: must be called only once
pub(crate) unsafe fn init() {
    foreach_interrupt! {
        ($peri:ident, dma, $block:ident, $signal_name:ident, $irq:ident) => {
            interrupt::$irq::steal().enable();
        };
    }
    crate::_generated::init_dma();
}

foreach_dma_channel! {
    ($channel_peri:ident, $dma_peri:ident, dma, $channel_num:expr, $index:expr, $dmamux:tt) => {
        impl crate::dma::sealed::Channel for crate::peripherals::$channel_peri {
            unsafe fn start_write<W: Word>(&mut self, request: Request, buf: *const [W], reg_addr: *mut W, options: TransferOptions) {
                let (ptr, len) = super::slice_ptr_parts(buf);
                low_level_api::start_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    $index,
                    request,
                    vals::Dir::MEMORYTOPERIPHERAL,
                    reg_addr as *const u32,
                    ptr as *mut u32,
                    len,
                    true,
                    vals::Size::from(W::bits()),
                    options,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                )
            }

            unsafe fn start_write_repeated<W: Word>(&mut self, request: Request, repeated: W, count: usize, reg_addr: *mut W, options: TransferOptions) {
                let buf = [repeated];
                low_level_api::start_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    $index,
                    request,
                    vals::Dir::MEMORYTOPERIPHERAL,
                    reg_addr as *const u32,
                    buf.as_ptr() as *mut u32,
                    count,
                    false,
                    vals::Size::from(W::bits()),
                    options,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                )
            }

            unsafe fn start_read<W: Word>(&mut self, request: Request, reg_addr: *const W, buf: *mut [W], options: TransferOptions) {
                let (ptr, len) = super::slice_ptr_parts_mut(buf);
                    low_level_api::start_transfer(
                        pac::$dma_peri,
                        $channel_num,
                        $index,
                        request,
                        vals::Dir::PERIPHERALTOMEMORY,
                        reg_addr as *const u32,
                        ptr as *mut u32,
                        len,
                        true,
                        vals::Size::from(W::bits()),
                        options,
                        #[cfg(dmamux)]
                        <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                        #[cfg(dmamux)]
                        <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                    );
            }

            #[cfg(feature = "dcmi-giant-transfer")]
            unsafe fn start_giant_read<W: Word>(&mut self, request: Request, reg_addr: *const W, buf: *mut [W], options: TransferOptions) {
                let (ptr, len) = super::slice_ptr_parts_mut(buf);
                low_level_api::start_giant_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    $index,
                    request,
                    vals::Dir::PERIPHERALTOMEMORY,
                    reg_addr as *const u32,
                    ptr as *mut u32,
                    len,
                    true,
                    vals::Size::from(W::bits()),
                    options,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                );
            }

            // unsafe fn start_double_buffered_read<W: Word>(
            //     &mut self,
            //     request: Request,
            //     reg_addr: *const W,
            //     buffer0: *mut W,
            //     buffer1: *mut W,
            //     buffer_len: usize,
            //     options: TransferOptions,
            // ) {
            //     low_level_api::start_dbm_transfer(
            //         pac::$dma_peri,
            //         $channel_num,
            //         request,
            //         vals::Dir::PERIPHERALTOMEMORY,
            //         reg_addr as *const u32,
            //         buffer0 as *mut u32,
            //         buffer1 as *mut u32,
            //         buffer_len,
            //         true,
            //         vals::Size::from(W::bits()),
            //         options,
            //         #[cfg(dmamux)]
            //         <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
            //         #[cfg(dmamux)]
            //         <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
            //     );
            // }

            // unsafe fn set_buffer0<W: Word>(&mut self, buffer: *mut W) {
            //     low_level_api::set_dbm_buffer0(pac::$dma_peri, $channel_num, buffer as *mut u32);
            // }

            // unsafe fn set_buffer1<W: Word>(&mut self, buffer: *mut W) {
            //     low_level_api::set_dbm_buffer1(pac::$dma_peri, $channel_num, buffer as *mut u32);
            // }

            // unsafe fn is_buffer0_accessible(&mut self) -> bool {
            //     low_level_api::is_buffer0_accessible(pac::$dma_peri, $channel_num)
            // }

            fn request_stop(&mut self) {
                unsafe {low_level_api::request_stop(pac::$dma_peri, $channel_num, $index);}
            }

            fn is_running(&self) -> bool {
                unsafe {low_level_api::is_running(pac::$dma_peri, $channel_num)}
            }

            fn remaining_transfers(&mut self) -> u32 {
                unsafe {low_level_api::get_remaining_transfers(pac::$dma_peri, $channel_num, $index)}
            }

            fn set_waker(&mut self, waker: &Waker) {
                unsafe {low_level_api::set_waker($index, waker )}
            }

            fn on_irq() {
                unsafe {
                    low_level_api::on_irq_inner(pac::$dma_peri, $channel_num, $index);
                }
            }
        }
        impl crate::dma::Channel for crate::peripherals::$channel_peri { }
    };
}

mod low_level_api {
    use super::*;

    pub unsafe fn start_transfer(
        dma: pac::dma::Dma,
        channel_number: u8,
        #[allow(unused)] state_index: usize,
        request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: vals::Size,
        options: TransferOptions,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) {
        #[cfg(dmamux)]
        super::super::dmamux::configure_dmamux(dmamux_regs, dmamux_ch_num, request);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        reset_status(dma, channel_number);

        #[cfg(feature = "dcmi-giant-transfer")]
        STATE.channels[state_index].giant_transfer_state.disable();

        let ch = dma.st(channel_number as _);
        ch.par().write_value(peri_addr as u32);
        ch.m0ar().write_value(mem_addr as u32);
        ch.ndtr().write_value(regs::Ndtr(mem_len as _));
        ch.cr().write(|w| {
            w.set_dir(dir);
            w.set_msize(data_size);
            w.set_psize(data_size);
            w.set_pl(vals::Pl::VERYHIGH);
            if incr_mem {
                w.set_minc(vals::Inc::INCREMENTED);
            } else {
                w.set_minc(vals::Inc::FIXED);
            }
            w.set_pinc(vals::Inc::FIXED);
            w.set_teie(true);
            w.set_tcie(true);
            #[cfg(dma_v1)]
            w.set_trbuff(true);

            #[cfg(dma_v2)]
            w.set_chsel(request);

            w.set_pburst(options.pburst.into());
            w.set_mburst(options.mburst.into());
            w.set_pfctrl(options.flow_ctrl.into());

            w.set_en(true);
        });
    }

    #[cfg(feature = "dcmi-giant-transfer")]
    pub unsafe fn start_giant_transfer(
        dma: pac::dma::Dma,
        channel_number: u8,
        state_index: usize,
        request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: vals::Size,
        options: TransferOptions,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) {
        assert!(mem_len > 0xffff);
        assert!(mem_len % 2 == 0);

        #[cfg(dmamux)]
        super::super::dmamux::configure_dmamux(dmamux_regs, dmamux_ch_num, request);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        reset_status(dma, channel_number);

        let transfer_len_bytes = match data_size {
            vals::Size::BITS8 => 1,
            vals::Size::BITS16 => 2,
            vals::Size::BITS32 => 4,
            _ => panic!("Invalid data size."),
        };

        let (m0ar, m1ar, chunk_size) = STATE.channels[state_index].giant_transfer_state.enable(
            mem_addr as u32,
            mem_len,
            transfer_len_bytes,
        );

        let ch = dma.st(channel_number as _);
        ch.par().write_value(peri_addr as u32);
        ch.m0ar().write_value(m0ar.0);
        // configures the second buffer for DBM
        ch.m1ar().write_value(m1ar.0);
        ch.ndtr().write_value(regs::Ndtr(chunk_size.0 as _));
        ch.cr().write(|w| {
            w.set_dir(dir);
            w.set_msize(data_size);
            w.set_psize(data_size);
            w.set_pl(vals::Pl::VERYHIGH);
            if incr_mem {
                w.set_minc(vals::Inc::INCREMENTED);
            } else {
                w.set_minc(vals::Inc::FIXED);
            }
            w.set_pinc(vals::Inc::FIXED);
            w.set_teie(true);
            w.set_tcie(true);

            #[cfg(dma_v1)]
            w.set_trbuff(true);

            #[cfg(dma_v2)]
            w.set_chsel(request);

            // enable double buffered mode
            w.set_dbm(vals::Dbm::ENABLED);

            w.set_pburst(options.pburst.into());
            w.set_mburst(options.mburst.into());
            w.set_pfctrl(options.flow_ctrl.into());

            w.set_en(true);
        });
    }

    /// Stops the DMA channel.
    pub unsafe fn request_stop(
        dma: pac::dma::Dma,
        channel_number: u8,
        #[allow(unused)] state_index: usize,
    ) {
        // get a handle on the channel itself
        let ch = dma.st(channel_number as _);

        // Disable the channel. Keep the IEs enabled so the irqs still fire.
        ch.cr().write(|w| {
            w.set_teie(true);
            w.set_tcie(true);
        });

        #[cfg(feature = "dcmi-giant-transfer")]
        STATE.channels[state_index].giant_transfer_state.disable();

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }

    /// Gets the running status of the channel
    pub unsafe fn is_running(dma: pac::dma::Dma, ch: u8) -> bool {
        // get a handle on the channel itself
        let ch = dma.st(ch as _);
        // Get whether it's enabled (running)
        ch.cr().read().en()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub unsafe fn get_remaining_transfers(
        dma: pac::dma::Dma,
        ch: u8,
        #[allow(unused)] state_index: usize,
    ) -> u32 {
        // get a handle on the channel itself
        let ch = dma.st(ch as _);
        // read the remaining transfer count. If this is zero, the transfer completed fully.
        let ndtr = ch.ndtr().read().ndt() as u32;

        #[cfg(feature = "dcmi-giant-transfer")]
        {
            if STATE.channels[state_index]
                .giant_transfer_state
                .is_enabled()
            {
                STATE.channels[state_index]
                    .giant_transfer_state
                    .remaining_transfers(ndtr)
            } else {
                ndtr
            }
        }
        #[cfg(not(feature = "dcmi-giant-transfer"))]
        ndtr
    }

    /// Sets the waker for the specified DMA channel
    pub unsafe fn set_waker(state_number: usize, waker: &Waker) {
        STATE.channels[state_number].waker.register(waker);
    }

    pub unsafe fn reset_status(dma: pac::dma::Dma, channel_number: u8) {
        let isrn = channel_number as usize / 4;
        let isrbit = channel_number as usize % 4;

        dma.ifcr(isrn).write(|w| {
            w.set_tcif(isrbit, true);
            w.set_teif(isrbit, true);
        });
    }

    /// Safety: Must be called with a matching set of parameters for a valid dma channel
    pub unsafe fn on_irq_inner(dma: pac::dma::Dma, channel_num: u8, state_index: u8) {
        let channel_num = channel_num as usize;
        let state_index = state_index as usize;

        let cr = dma.st(channel_num).cr();
        let isr = dma.isr(channel_num / 4).read();

        if isr.teif(channel_num % 4) {
            panic!("DMA: error on DMA@{:08x} channel {}", dma.0 as u32, channel_num);
        }

        #[cfg(feature = "dcmi-giant-transfer")]
        if STATE.channels[state_index]
            .giant_transfer_state
            .is_enabled()
        {
            if isr.tcif(channel_num % 4) && cr.read().tcie() {
                dma.ifcr(channel_num / 4)
                    .write(|w| w.set_tcif(channel_num % 4, true));

                let remaining_transfers = STATE.channels[state_index]
                    .giant_transfer_state
                    .remaining_chunks();
                defmt::trace!(
                    "Giant DMA: {} remaining transfers {}",
                    state_index,
                    remaining_transfers
                );
                let current_target_memory1 = cr.read().ct() == vals::Ct::MEMORY1;
                if remaining_transfers != 0 {
                    if remaining_transfers % 2 == 0 && current_target_memory1 {
                        // update pointer to memory 0
                        let mem0_addr = dma.st(channel_num).m0ar().read();
                        let new_addr = STATE.channels[state_index]
                            .giant_transfer_state
                            .dequeue_next_chunk(mem0_addr);
                        dma.st(channel_num).m0ar().write_value(new_addr);
                    } else if !current_target_memory1 {
                        // update pointer to memory 1
                        let mem1_addr = dma.st(channel_num).m1ar().read();
                        let new_addr = STATE.channels[state_index]
                            .giant_transfer_state
                            .dequeue_next_chunk(mem1_addr);
                        dma.st(channel_num).m1ar().write_value(new_addr);
                    }
                } else {
                    if current_target_memory1 {
                        dma.st(channel_num).m0ar().write_value(
                            STATE.channels[state_index]
                                .giant_transfer_state
                                .data_address(),
                        )
                    } else {
                        dma.st(channel_num).m1ar().write_value(
                            STATE.channels[state_index]
                                .giant_transfer_state
                                .data_address(),
                        )
                    }
                    // TCIF here means that Chunk N - 1 was transferred, the giant transfer is marked as finished
                    // next TCIF will mean that all the data is transferred and will wake the future
                    STATE.channels[state_index].giant_transfer_state.disable();
                }
            }
        } else {
            if isr.tcif(channel_num % 4) && cr.read().tcie() {
                cr.write(|_| ()); // Disable channel with the default value.
                STATE.channels[state_index].waker.wake();
            }
        }

        #[cfg(not(feature = "dcmi-giant-transfer"))]
        if isr.tcif(channel_num % 4) && cr.read().tcie() {
            cr.write(|_| ()); // Disable channel with the default value.
            STATE.channels[state_index].waker.wake();
        }
    }
}

use core::intrinsics::{likely, unlikely};

#[inline(always)]
pub unsafe fn write_u32_fast(mut ptr: *mut u32, len: usize, x: u32, use_dma: bool) {
    if unlikely(len == 0) {
        return;
    } else if use_dma {
        use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0 as DMA};
        #[repr(align(16))]
        struct DmaData(u32);
        static mut DMA_SRC: DmaData= DmaData(0);
        unsafe { DMA_SRC.0 = x; }

        DMA::set_source(&DMA_SRC.0 as *const _);
        DMA::set_dest(ptr as *mut _);
        DMA::set_count(len as u16);
        DMA::set_control(DMAControlSetting::new()
            .with_source_address_control(DMASrcAddressControl::Fixed)
            .with_dest_address_control(DMADestAddressControl::Increment)
            .with_dma_repeat(false)
            .with_use_32bit(true)
            .with_start_time(DMAStartTiming::Immediate)
            .with_enabled(true));
    } else {
        for _ in 0..len / 4 {
            ptr.offset(0).write(x);
            ptr.offset(1).write(x);
            ptr.offset(2).write(x);
            ptr.offset(3).write(x);
            ptr = ptr.offset(4);
        }
        for i in 0..len % 4 {
            ptr.offset(i as isize).write(x);
        }
    }
}

#[inline(always)]
pub unsafe fn write_u16_fast(ptr: *mut u16, len: usize, x: u16, use_dma: bool) {
    if unlikely(len == 0) {
        return;
    } else if use_dma {
        #[repr(align(16))]
        struct DmaData(u16);
        static mut DMA_SRC: DmaData = DmaData(0);
        unsafe { DMA_SRC.0 = x; }

        use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0 as DMA};
        DMA::set_source(&DMA_SRC.0 as *const _ as *const _);
        DMA::set_dest(ptr as *mut _);
        DMA::set_count(len as u16);
        DMA::set_control(DMAControlSetting::new()
            .with_source_address_control(DMASrcAddressControl::Fixed)
            .with_dest_address_control(DMADestAddressControl::Increment)
            .with_dma_repeat(false)
            .with_use_32bit(false)
            .with_start_time(DMAStartTiming::Immediate)
            .with_enabled(true));
    } else {
        let ptr_mask = ptr as usize & 2;

        let ptr_inner = (ptr as usize + ptr_mask) as *mut _;
        // Give most of the work to the faster u32-specific function
        write_u32_fast(ptr_inner, (len - ptr_mask / 2) / 2, x as u32 | ((x as u32) << 16), use_dma);

        if ptr_mask != 0 {
            ptr.write(x);
        }

        if ptr.offset(len as isize) as usize & 2 != 0 {
            ptr.offset(len as isize - 1).write(x);
        }
    }
}

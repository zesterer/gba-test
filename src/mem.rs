use core::{
    intrinsics::{likely, unlikely},
    hint::black_box,
};

/// Delay for n cycles at minimum
#[inline(always)]
fn delay(n: usize) {
    for i in 0..n {
        black_box(i);
    }
}

#[link_section = ".text_fast"]
pub fn copy_fast<T: Copy>(src: &[T], dst: &mut [T]) {
    assert!(likely(src.len() == dst.len()));
    let len = src.len() * core::mem::size_of::<T>();

    let src_ptr = src.as_ptr() as usize;
    let dst_ptr = dst.as_ptr() as usize;

    if unlikely(len == 0) {
        return;
    } else if src_ptr & 1 == 0 && dst_ptr & 1 == 0 {
        unsafe {
            use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0 as DMA};
            DMA::set_source(src_ptr as *const _);
            DMA::set_dest(dst_ptr as *mut _);
            DMA::set_count((len / 2) as u16);
            DMA::set_control(DMAControlSetting::new()
                .with_source_address_control(DMASrcAddressControl::Increment)
                .with_dest_address_control(DMADestAddressControl::Increment)
                .with_dma_repeat(false)
                .with_use_32bit(false)
                .with_start_time(DMAStartTiming::Immediate)
                .with_enabled(true));
            delay(2);
        }
    } else {
        // TODO: Support unaligned copies in a cleverer way (copy a subset)
        for (i, x) in src.iter().enumerate() {
            unsafe { *dst.get_unchecked_mut(i) = *x; }
        }
    }
}

#[link_section = ".text_fast"]
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
        delay(2);
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

#[link_section = ".text_fast"]
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
        delay(2);
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

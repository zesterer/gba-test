use core::{
    alloc::{GlobalAlloc, Layout},
    ptr::NonNull,
};
use linked_list_allocator::Heap;

static mut HEAP_BYTES: [u8; 16384] = [0; 16384];

static mut HEAP: Heap = Heap::empty();

#[global_allocator]
static ALLOCATOR: Allocator = Allocator;

pub fn init() {
    unsafe {
        HEAP.init(
            (&HEAP_BYTES[0] as *const _) as usize,
            (&HEAP_BYTES[0] as *const _) as usize + core::mem::size_of_val(&HEAP_BYTES),
        );
    }
}

struct Allocator;

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        HEAP.allocate_first_fit(layout)
            .expect("Failed to allocate")
            .as_ptr()
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        HEAP.deallocate(NonNull::new(ptr).unwrap(), layout);
    }
}

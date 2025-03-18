use core::fmt::{Debug, Formatter};
use core::sync::atomic::AtomicBool;

pub trait BufferProducer<'a, T> {
    fn produce(&self, data: &T, data_size: usize);
    fn as_ref(&self) -> &T;
}

pub trait BufferConsumer<'a, T> {
    fn consume(&self) -> &T;
}
pub struct DoubleBuffer<T> {
    swapped: AtomicBool,
    buffers: [T; 2],
}

impl<T> DoubleBuffer<T> {
    #[inline]
    pub const fn new(current: T, next: T) -> Self {
        Self {
            swapped: AtomicBool::new(false),
            buffers: [current, next],
        }
    }

    #[inline]
    pub fn swap(&self) {
        self.swapped
            .fetch_not(core::sync::atomic::Ordering::Relaxed);
    }

    #[inline]
    fn current_offset(&self) -> usize {
        if self.swapped.load(core::sync::atomic::Ordering::Relaxed) {
            return 1;
        }
        return 0;
    }

    #[inline]
    fn next_offset(&self) -> usize {
        if self.swapped.load(core::sync::atomic::Ordering::Relaxed) {
            return 0;
        }
        return 1;
    }

    #[inline]
    pub fn current(&self) -> &T {
        &self.buffers[self.current_offset()]
    }

    #[inline]
    pub fn next(&self) -> &T {
        &self.buffers[self.next_offset()]
    }

    pub fn split(&mut self) -> (DoubleBufferProducer<'_, T>, DoubleBufferConsumer<'_, T>) {
        (
            DoubleBufferProducer { buf: self },
            DoubleBufferConsumer { buf: self },
        )
    }
}
impl<T: Debug> Debug for DoubleBuffer<T> {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DoubleBuffer")
            .field("current", self.current())
            .field("next", self.next())
            .finish()
    }
}
pub struct DoubleBufferConsumer<'a, T> {
    buf: &'a DoubleBuffer<T>,
}

pub struct DoubleBufferProducer<'a, T> {
    buf: &'a DoubleBuffer<T>,
}

unsafe impl<'a, T> Send for DoubleBufferConsumer<'a, T> where T: Send {}
unsafe impl<'a, T> Send for DoubleBufferProducer<'a, T> where T: Send {}

impl<'a, T> BufferConsumer<'a, T> for DoubleBufferConsumer<'a, T> {
    #[inline(always)]
    fn consume(&self) -> &T {
        self.buf.current()
    }
}

impl<'a, T> BufferProducer<'a, T> for DoubleBufferProducer<'a, T> {
    #[inline(always)]
    fn produce(&self, _data: &T, _data_size: usize) {}
    #[inline(always)]
    fn as_ref(&self) -> &T {
        self.buf.current()
    }
}

impl<'a, T> DoubleBufferProducer<'a, T> {
    #[inline(always)]
    pub fn swap(&self) {
        self.buf.swap();
    }
    #[inline(always)]
    pub fn buf(&self) -> &DoubleBuffer<T> {
        self.buf
    }
}

//! リングバッファ。設計 docs/20-integration/04-world-api.md §2.1
//! `pub struct Probe { pub target: ProbeTarget, pub history: RingBuffer<f64> }`
//! (Probeの観測履歴、UIのグラフ・CSVエクスポート用)。

use std::collections::VecDeque;

/// 固定容量のFIFOリングバッファ。容量を超えて`push`すると最も古い要素が捨てられる。
#[derive(Clone)]
pub struct RingBuffer<T> {
    capacity: usize,
    buffer: VecDeque<T>,
}

impl<T> RingBuffer<T> {
    pub fn new(capacity: usize) -> RingBuffer<T> {
        assert!(capacity > 0, "RingBuffer capacity must be positive");
        RingBuffer {
            capacity,
            buffer: VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, value: T) {
        if self.buffer.len() == self.capacity {
            self.buffer.pop_front();
        }
        self.buffer.push_back(value);
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// 古い順(挿入順)のイテレータ。
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.buffer.iter()
    }

    /// 全要素を古い順に取り出しつつ空にする(`sim-world::World::drain_events`が使う)。
    pub fn drain(&mut self) -> impl Iterator<Item = T> + '_ {
        self.buffer.drain(..)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_within_capacity_preserves_insertion_order() {
        let mut buf = RingBuffer::new(3);
        buf.push(1);
        buf.push(2);
        assert_eq!(buf.len(), 2);
        assert_eq!(buf.iter().copied().collect::<Vec<_>>(), vec![1, 2]);
    }

    #[test]
    fn push_beyond_capacity_discards_oldest() {
        let mut buf = RingBuffer::new(3);
        for v in 1..=5 {
            buf.push(v);
        }
        assert_eq!(buf.len(), 3);
        assert_eq!(buf.capacity(), 3);
        assert_eq!(buf.iter().copied().collect::<Vec<_>>(), vec![3, 4, 5]);
    }

    #[test]
    fn empty_buffer_reports_correctly() {
        let buf: RingBuffer<f64> = RingBuffer::new(5);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
    }

    #[test]
    fn drain_yields_all_elements_in_order_and_empties_the_buffer() {
        let mut buf = RingBuffer::new(3);
        buf.push(1);
        buf.push(2);
        buf.push(3);
        assert_eq!(buf.drain().collect::<Vec<_>>(), vec![1, 2, 3]);
        assert!(buf.is_empty());
    }
}

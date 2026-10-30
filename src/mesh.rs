//! Склейка полигонов в куски (chunk) для отрисовки.
//!
//! У macroquad ограничен размер одного draw-call, поэтому геометрия режется на
//! небольшие куски: не больше MAX_VERTS вершин и MAX_IDX индексов.
use crate::carmodels::Poly;
use crate::light::Mat;
use crate::math::*;

pub const MAX_VERTS: usize = 8000;
pub const MAX_IDX: usize = 4500;

/// Вершина. Для мира `c` - уже запечённый цвет; для машин `c` - альбедо, а `n` и `m`
/// нужны для динамического освещения каждый кадр.
#[derive(Clone)]
pub struct Vert {
    pub p: [f32; 3],
    pub uv: [f32; 2],
    pub c: [f32; 4],
    pub n: [f32; 3],
    pub m: Mat,
}

#[derive(Clone)]
pub struct Chunk {
    pub verts: Vec<Vert>,
    pub idx: Vec<u16>,
    /// ограничивающая сфера - для отсечения далёких кусков
    pub center: V3,
    pub radius: f32,
}

pub struct ChunkBuilder {
    chunks: Vec<Chunk>,
    v: Vec<Vert>,
    i: Vec<u16>,
}

impl ChunkBuilder {
    pub fn new() -> ChunkBuilder {
        ChunkBuilder { chunks: Vec::new(), v: Vec::new(), i: Vec::new() }
    }

    fn flush(&mut self) {
        if self.v.is_empty() {
            return;
        }
        let verts = std::mem::take(&mut self.v);
        let idx = std::mem::take(&mut self.i);
        let mut c = V3::default();
        for v in verts.iter() {
            c += v3(v.p[0], v.p[1], v.p[2]);
        }
        c = c * (1.0 / verts.len() as f32);
        let mut radius = 0.0f32;
        for v in verts.iter() {
            let d = v3(v.p[0], v.p[1], v.p[2]) - c;
            radius = radius.max(d.len());
        }
        self.chunks.push(Chunk { verts, idx, center: c, radius });
    }

    /// Выпуклый полигон из готовых вершин (веером от первой).
    pub fn push(&mut self, verts: &[Vert]) {
        let n = verts.len();
        if n < 3 {
            return;
        }
        if self.v.len() + n > MAX_VERTS || self.i.len() + 3 * (n - 2) > MAX_IDX {
            self.flush();
        }
        let base = self.v.len() as u16;
        for v in verts.iter() {
            self.v.push(v.clone());
        }
        for k in 1..(n - 1) {
            self.i.push(base);
            self.i.push(base + k as u16);
            self.i.push(base + k as u16 + 1);
        }
    }

    /// Полигоны моделей машин: цвет = альбедо, нормали и материал сохраняются.
    pub fn polys(&mut self, polys: &[Poly]) {
        for p in polys.iter() {
            let verts: Vec<Vert> = p
                .v
                .iter()
                .map(|x| Vert {
                    p: [x.p.x, x.p.y, x.p.z],
                    uv: [0.0, 0.0],
                    c: [x.c[0], x.c[1], x.c[2], 1.0],
                    n: [x.n.x, x.n.y, x.n.z],
                    m: x.m,
                })
                .collect();
            self.push(&verts);
        }
    }

    pub fn finish(mut self) -> Vec<Chunk> {
        self.flush();
        self.chunks
    }
}

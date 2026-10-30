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
    /// Обход приводится к согласованному с нормалью: macroquad по умолчанию
    /// режет грани, у которых экранная площадь отрицательная (back-cull), а
    /// зеркало камеры (`-x` в draw_model) переворачивает обход. Поэтому
    /// разворачиваем контур так, чтобы геометрическая нормаль Newell совпадала
    /// со средним `n` вершин — тогда веер всегда смотрит «лицом» к камере и
    /// машина не выглядит прозрачной/дырявой.
    pub fn push(&mut self, verts: &[Vert]) {
        let n = verts.len();
        if n < 3 {
            return;
        }
        // нормалем по Newell из порядка обхода
        let (mut nx, mut ny, mut nz) = (0.0f32, 0.0f32, 0.0f32);
        for i in 0..n {
            let a = verts[i].p;
            let b = verts[(i + 1) % n].p;
            nx += (a[1] - b[1]) * (a[2] + b[2]);
            ny += (a[2] - b[2]) * (a[0] + b[0]);
            nz += (a[0] - b[0]) * (a[1] + b[1]);
        }
        // средняя вершинная нормаль
        let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
        for v in verts.iter() {
            sx += v.n[0];
            sy += v.n[1];
            sz += v.n[2];
        }
        let mut order: Box<[Vert]> = verts.to_vec().into_boxed_slice();
        if nx * sx + ny * sy + nz * sz < 0.0 {
            // обход против нормали - разворачиваем, сохраняя первую вершину
            let first = order[0].clone();
            let mut r: Vec<Vert> = order[1..].to_vec();
            r.reverse();
            r.insert(0, first);
            order = r.into_boxed_slice();
        }
        if self.v.len() + n > MAX_VERTS || self.i.len() + 3 * (n - 2) > MAX_IDX {
            self.flush();
        }
        let base = self.v.len() as u16;
        for v in order.iter() {
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

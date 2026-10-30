//! Процедурные модели реальных автомобилей: плавные кузова с нормалями.
//!
//! Модуль не зависит от движка. Возвращает полигоны с позицией, нормалью,
//! цветом и материалом в каждой вершине (метры; x - вправо, y - вверх, z - вперёд).
//! Освещение считается позже (`light.rs`). Профиль кузова задаётся станциями,
//! между ними форма сглаживается кубической интерполяцией (PCHIP), поэтому
//! капот, крыша и нос идут плавно, а не ломаной линией.
use crate::light::Mat;
use crate::math::*;
use std::f32::consts::PI;

pub use crate::math::Rgb;

pub const DARK: Rgb = [0.05, 0.05, 0.07];
pub const GLASS: Rgb = [0.025, 0.035, 0.07];
pub const TIRE: Rgb = [0.05, 0.05, 0.06];
/// "нет значения" для высоты крыши: берётся линия капота/багажника
const NA: f32 = f32::NAN;

/// Вершина: позиция, нормаль, цвет (альбедо), материал.
#[derive(Clone, Copy)]
pub struct PV {
    pub p: V3,
    pub n: V3,
    pub c: Rgb,
    pub m: Mat,
}

/// Выпуклый полигон (рисуется веером от первой вершины).
#[derive(Clone)]
pub struct Poly {
    pub v: Vec<PV>,
}

fn newell(pts: &[V3]) -> V3 {
    let (mut nx, mut ny, mut nz) = (0.0f32, 0.0f32, 0.0f32);
    let n = pts.len();
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
    }
    let l = (nx * nx + ny * ny + nz * nz).sqrt();
    if l < 1e-12 {
        v3(0.0, 1.0, 0.0)
    } else {
        v3(nx / l, ny / l, nz / l)
    }
}

fn centroid(pts: &[V3]) -> V3 {
    let mut c = V3::default();
    for p in pts.iter() {
        c += *p;
    }
    c * (1.0 / pts.len() as f32)
}

pub struct Builder {
    pub polys: Vec<Poly>,
}

impl Builder {
    pub fn new() -> Builder {
        Builder { polys: Vec::new() }
    }

    /// Плоский полигон. `hint` - точка внутри тела, чтобы нормаль смотрела наружу.
    pub fn flat(&mut self, pts: &[V3], rgb: Rgb, mat: Mat, hint: Option<V3>) {
        let mut n = newell(pts);
        if let Some(h) = hint {
            if n.dot(centroid(pts) - h) < 0.0 {
                n = -n;
            }
        }
        self.polys.push(Poly {
            v: pts
                .iter()
                .map(|p| PV {
                    p: *p,
                    n,
                    c: rgb,
                    m: mat,
                })
                .collect(),
        });
    }

    pub fn boxed(&mut self, c: V3, s: V3, rgb: Rgb, mat: Mat) {
        let (hx, hy, hz) = (s.x / 2.0, s.y / 2.0, s.z / 2.0);
        let p = |x: f32, y: f32, z: f32| v3(c.x + x, c.y + y, c.z + z);
        let faces: [[V3; 4]; 6] = [
            [
                p(hx, -hy, -hz),
                p(hx, hy, -hz),
                p(hx, hy, hz),
                p(hx, -hy, hz),
            ],
            [
                p(-hx, -hy, hz),
                p(-hx, hy, hz),
                p(-hx, hy, -hz),
                p(-hx, -hy, -hz),
            ],
            [
                p(-hx, hy, -hz),
                p(-hx, hy, hz),
                p(hx, hy, hz),
                p(hx, hy, -hz),
            ],
            [
                p(-hx, -hy, hz),
                p(-hx, -hy, -hz),
                p(hx, -hy, -hz),
                p(hx, -hy, hz),
            ],
            [
                p(hx, -hy, hz),
                p(hx, hy, hz),
                p(-hx, hy, hz),
                p(-hx, -hy, hz),
            ],
            [
                p(-hx, -hy, -hz),
                p(-hx, hy, -hz),
                p(hx, hy, -hz),
                p(hx, -hy, -hz),
            ],
        ];
        for f in faces.iter() {
            self.flat(f, rgb, mat, Some(c));
        }
    }

    /// Гладкая поверхность: `rings[i][j]` - точка j кольца на станции i.
    /// Нормали считаются по сетке (поэтому освещение плавное), а цвет и материал
    /// берутся у функции `f(сегмент, ребро)`.
    fn grid(&mut self, rings: &[Vec<V3>], hints: &[V3], f: &dyn Fn(usize, usize) -> (Rgb, Mat)) {
        let ns = rings.len();
        let nr = rings[0].len();
        let mut nrm: Vec<Vec<V3>> = Vec::with_capacity(ns);
        for i in 0..ns {
            let mut row: Vec<V3> = Vec::with_capacity(nr);
            for j in 0..nr {
                let tz = rings[(i + 1).min(ns - 1)][j] - rings[if i > 0 { i - 1 } else { 0 }][j];
                let tr = rings[i][(j + 1).min(nr - 1)] - rings[i][if j > 0 { j - 1 } else { 0 }];
                let mut n = tr.cross(tz).norm();
                if n.dot(rings[i][j] - hints[i]) < 0.0 {
                    n = -n;
                }
                row.push(n);
            }
            nrm.push(row);
        }
        for i in 0..(ns - 1) {
            for j in 0..(nr - 1) {
                let (c, m) = f(i, j);
                let idx = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)];
                let v: Vec<PV> = idx
                    .iter()
                    .map(|&(a, b)| PV {
                        p: rings[a][b],
                        n: nrm[a][b],
                        c,
                        m,
                    })
                    .collect();
                self.polys.push(Poly { v });
            }
        }
    }
}

pub fn translate(polys: &[Poly], d: V3) -> Vec<Poly> {
    polys
        .iter()
        .map(|q| Poly {
            v: q.v.iter().map(|x| PV { p: x.p + d, ..*x }).collect(),
        })
        .collect()
}

// ----------------------------- спецификации -----------------------------

#[derive(Clone, Copy, PartialEq)]
pub enum Role {
    /// верх = стекло (лобовое / заднее), стойки кузова
    GlassTop,
    /// верх = крыша, борта = боковые стёкла
    GlassSide,
    /// сплошной кузов (центральная стойка)
    Pillar,
}

#[derive(Clone)]
pub struct CarSpec {
    pub name: &'static str,
    pub l: f32,
    pub w: f32,
    pub h: f32,
    pub wb: f32,
    pub track: f32,
    pub r: f32,
    /// станции кузова спереди назад: [z, y_низ, y_плечо, y_верх, полуширина_низ, полуширина_верх]
    pub body: Vec<[f32; 6]>,
    /// станции салона: [z, y_крыши (NaN = по линии кузова), полуширина_крыши]
    pub cabin: Vec<[f32; 3]>,
    pub roles: Vec<Role>,
    pub head: [f32; 4],
    pub grille: [f32; 3],
    pub intake: [f32; 3],
    pub plates: [f32; 2],
    pub tail: [f32; 4],
    pub exh: [f32; 2],
    pub mirror: [f32; 2],
    pub lip: [f32; 2],
    pub wing: [f32; 4],
    /// дополнительные тёмные детали (воздухозаборники): (центр, размер)
    pub extras: Vec<([f32; 3], [f32; 3])>,
}

#[allow(clippy::too_many_arguments)]
fn spec(
    name: &'static str,
    l: f32,
    w: f32,
    h: f32,
    wb: f32,
    track: f32,
    r: f32,
    body: &[[f32; 6]],
    cabin: &[[f32; 3]],
    head: [f32; 4],
    grille: [f32; 3],
    intake: [f32; 3],
    plates: [f32; 2],
    tail: [f32; 4],
    exh: [f32; 2],
    mirror: [f32; 2],
    lip: [f32; 2],
    wing: [f32; 4],
    extras: Vec<([f32; 3], [f32; 3])>,
) -> CarSpec {
    CarSpec {
        name,
        l,
        w,
        h,
        wb,
        track,
        r,
        body: body.to_vec(),
        cabin: cabin.to_vec(),
        roles: vec![
            Role::GlassTop,
            Role::GlassSide,
            Role::Pillar,
            Role::GlassSide,
            Role::GlassTop,
        ],
        head,
        grille,
        intake,
        plates,
        tail,
        exh,
        mirror,
        lip,
        wing,
        extras,
    }
}

/// Фургон/грузовик: без профиля, собираются из коробок.
fn box_spec(name: &'static str, l: f32, w: f32, h: f32, wb: f32, track: f32, r: f32) -> CarSpec {
    CarSpec {
        name,
        l,
        w,
        h,
        wb,
        track,
        r,
        body: Vec::new(),
        cabin: Vec::new(),
        roles: Vec::new(),
        head: [0.0; 4],
        grille: [0.0; 3],
        intake: [0.0; 3],
        plates: [0.0; 2],
        tail: [0.0; 4],
        exh: [0.0; 2],
        mirror: [0.0; 2],
        lip: [0.0; 2],
        wing: [0.0; 4],
        extras: Vec::new(),
    }
}

/// Порядок совпадает с `car::STATS`:
/// focus, cobalt, gt3000, evo9, z350, rx7, supra, r34
pub fn all_car_specs() -> Vec<CarSpec> {
    vec![
        // Ford Focus II hatchback (2005-2011)
        spec(
            "Ford Focus II Hatchback",
            4.34,
            1.84,
            1.50,
            2.64,
            1.58,
            0.32,
            &[
                [2.17, 0.32, 0.52, 0.60, 0.86, 0.80],
                [2.08, 0.24, 0.58, 0.72, 0.91, 0.85],
                [1.75, 0.22, 0.62, 0.86, 0.92, 0.86],
                [1.00, 0.22, 0.66, 0.97, 0.92, 0.85],
                [-1.35, 0.22, 0.68, 0.99, 0.92, 0.85],
                [-2.00, 0.22, 0.68, 0.97, 0.92, 0.85],
                [-2.17, 0.28, 0.60, 0.90, 0.88, 0.83],
            ],
            &[
                [1.00, NA, 0.85],
                [0.15, 1.48, 0.71],
                [-0.28, 1.49, 0.71],
                [-0.50, 1.49, 0.71],
                [-1.62, 1.47, 0.70],
                [-2.04, NA, 0.80],
            ],
            [0.60, 0.68, 0.52, 0.13],
            [0.56, 0.80, 0.13],
            [0.36, 1.15, 0.13],
            [0.40, 0.66],
            [0.70, 0.86, 0.46, 0.20],
            [0.55, 0.34],
            [0.85, 1.06],
            [1.47, -1.66],
            [1.00, -2.12, 1.45, 0.42],
            vec![],
        ),
        // Chevrolet Cobalt SS Coupe (2008)
        spec(
            "Chevrolet Cobalt SS Coupe 2008",
            4.51,
            1.72,
            1.42,
            2.63,
            1.50,
            0.33,
            &[
                [2.25, 0.30, 0.52, 0.62, 0.80, 0.75],
                [2.15, 0.22, 0.58, 0.76, 0.85, 0.79],
                [1.70, 0.20, 0.62, 0.90, 0.86, 0.80],
                [0.95, 0.20, 0.66, 0.96, 0.86, 0.79],
                [-1.00, 0.20, 0.68, 0.98, 0.86, 0.79],
                [-1.80, 0.20, 0.70, 1.02, 0.86, 0.79],
                [-2.25, 0.26, 0.64, 0.96, 0.82, 0.76],
            ],
            &[
                [0.95, NA, 0.78],
                [-0.05, 1.38, 0.66],
                [-0.50, 1.40, 0.66],
                [-0.58, 1.40, 0.66],
                [-1.05, 1.37, 0.65],
                [-1.60, NA, 0.74],
            ],
            [0.58, 0.68, 0.50, 0.12],
            [0.56, 0.70, 0.13],
            [0.34, 1.10, 0.13],
            [0.40, 0.70],
            [0.62, 0.88, 0.46, 0.16],
            [0.50, 0.34],
            [0.50, 1.06],
            [1.02, -2.05],
            [1.02, -2.02, 1.46, 0.30],
            vec![],
        ),
        // Mitsubishi 3000GT VR-4 (1991-1999)
        spec(
            "Mitsubishi 3000GT VR-4",
            4.59,
            1.84,
            1.28,
            2.47,
            1.58,
            0.33,
            &[
                [2.30, 0.28, 0.46, 0.54, 0.86, 0.80],
                [2.20, 0.20, 0.52, 0.64, 0.90, 0.84],
                [1.60, 0.20, 0.56, 0.76, 0.92, 0.84],
                [0.70, 0.20, 0.62, 0.92, 0.92, 0.84],
                [-0.80, 0.20, 0.68, 0.98, 0.92, 0.85],
                [-1.90, 0.20, 0.70, 1.00, 0.92, 0.85],
                [-2.30, 0.26, 0.62, 0.94, 0.88, 0.82],
            ],
            &[
                [0.70, NA, 0.82],
                [-0.15, 1.27, 0.68],
                [-0.65, 1.26, 0.67],
                [-0.75, 1.26, 0.67],
                [-1.30, 1.18, 0.68],
                [-1.95, NA, 0.78],
            ],
            [0.60, 0.62, 0.56, 0.10],
            [0.48, 0.70, 0.10],
            [0.32, 1.20, 0.12],
            [0.38, 0.70],
            [0.62, 0.90, 0.56, 0.12],
            [0.55, 0.30],
            [0.45, 1.00],
            [1.00, -2.20],
            [1.00, -2.15, 1.60, 0.30],
            vec![([0.0, 0.985, 1.15], [0.9, 0.02, 0.34])],
        ),
        // Mitsubishi Lancer Evolution IX
        spec(
            "Mitsubishi Lancer Evolution IX",
            4.49,
            1.77,
            1.45,
            2.62,
            1.54,
            0.34,
            &[
                [2.245, 0.30, 0.54, 0.64, 0.83, 0.78],
                [2.15, 0.22, 0.60, 0.80, 0.87, 0.81],
                [1.70, 0.22, 0.64, 0.92, 0.885, 0.82],
                [1.00, 0.22, 0.68, 0.98, 0.885, 0.82],
                [-1.10, 0.22, 0.70, 1.00, 0.885, 0.82],
                [-1.90, 0.22, 0.72, 1.04, 0.885, 0.82],
                [-2.245, 0.28, 0.66, 0.98, 0.85, 0.80],
            ],
            &[
                [1.00, NA, 0.80],
                [0.20, 1.43, 0.68],
                [-0.30, 1.45, 0.68],
                [-0.45, 1.45, 0.68],
                [-1.05, 1.43, 0.67],
                [-1.50, NA, 0.76],
            ],
            [0.60, 0.70, 0.52, 0.12],
            [0.56, 0.62, 0.15],
            [0.36, 1.10, 0.15],
            [0.40, 0.74],
            [0.64, 0.90, 0.44, 0.16],
            [0.52, 0.34],
            [0.85, 1.08],
            [1.04, -2.10],
            [1.04, -2.00, 1.62, 0.52],
            vec![
                ([-0.36, 0.995, 1.45], [0.22, 0.02, 0.55]),
                ([0.36, 0.995, 1.45], [0.22, 0.02, 0.55]),
            ],
        ),
        // Nissan 350Z
        spec(
            "Nissan 350Z",
            4.31,
            1.82,
            1.32,
            2.65,
            1.56,
            0.34,
            &[
                [2.155, 0.30, 0.52, 0.60, 0.88, 0.82],
                [2.05, 0.22, 0.58, 0.76, 0.91, 0.85],
                [1.60, 0.20, 0.62, 0.88, 0.91, 0.85],
                [0.80, 0.20, 0.66, 0.96, 0.91, 0.85],
                [-0.90, 0.20, 0.70, 1.00, 0.91, 0.85],
                [-1.80, 0.20, 0.72, 1.03, 0.91, 0.85],
                [-2.155, 0.26, 0.66, 0.96, 0.87, 0.82],
            ],
            &[
                [0.80, NA, 0.82],
                [-0.05, 1.30, 0.68],
                [-0.45, 1.31, 0.68],
                [-0.55, 1.31, 0.68],
                [-1.15, 1.24, 0.68],
                [-1.75, NA, 0.78],
            ],
            [0.62, 0.68, 0.52, 0.14],
            [0.52, 0.70, 0.12],
            [0.34, 1.10, 0.13],
            [0.38, 0.70],
            [0.66, 0.90, 0.46, 0.14],
            [0.50, 0.32],
            [0.45, 1.02],
            [1.03, -2.10],
            [1.03, -2.05, 1.50, 0.40],
            vec![],
        ),
        // Mazda RX-7 FD3S
        spec(
            "Mazda RX-7 (FD)",
            4.30,
            1.75,
            1.23,
            2.43,
            1.50,
            0.33,
            &[
                [2.15, 0.28, 0.46, 0.54, 0.84, 0.78],
                [2.05, 0.20, 0.52, 0.64, 0.875, 0.80],
                [1.55, 0.20, 0.56, 0.76, 0.875, 0.80],
                [0.65, 0.20, 0.60, 0.90, 0.875, 0.80],
                [-0.85, 0.20, 0.66, 0.96, 0.875, 0.81],
                [-1.80, 0.20, 0.68, 0.96, 0.875, 0.81],
                [-2.15, 0.26, 0.60, 0.88, 0.83, 0.78],
            ],
            &[
                [0.65, NA, 0.78],
                [-0.05, 1.22, 0.66],
                [-0.55, 1.23, 0.66],
                [-0.65, 1.23, 0.66],
                [-1.20, 1.14, 0.66],
                [-1.80, NA, 0.74],
            ],
            [0.60, 0.60, 0.50, 0.10],
            [0.46, 0.62, 0.10],
            [0.32, 1.05, 0.12],
            [0.38, 0.66],
            [0.60, 0.84, 0.46, 0.12],
            [0.50, 0.30],
            [0.40, 0.98],
            [0.96, -2.10],
            [0.96, -2.10, 1.60, 0.38],
            vec![],
        ),
        // Toyota Supra (A80)
        spec(
            "Toyota Supra (A80)",
            4.51,
            1.81,
            1.27,
            2.55,
            1.55,
            0.34,
            &[
                [2.255, 0.30, 0.50, 0.58, 0.86, 0.80],
                [2.15, 0.22, 0.56, 0.72, 0.90, 0.84],
                [1.70, 0.20, 0.60, 0.84, 0.905, 0.84],
                [0.80, 0.20, 0.64, 0.92, 0.905, 0.84],
                [-0.90, 0.20, 0.70, 0.98, 0.905, 0.84],
                [-1.95, 0.20, 0.72, 1.02, 0.905, 0.84],
                [-2.255, 0.26, 0.66, 0.96, 0.87, 0.82],
            ],
            &[
                [0.80, NA, 0.80],
                [0.00, 1.26, 0.67],
                [-0.50, 1.27, 0.67],
                [-0.60, 1.27, 0.67],
                [-1.15, 1.20, 0.67],
                [-1.80, NA, 0.78],
            ],
            [0.62, 0.64, 0.52, 0.11],
            [0.50, 0.70, 0.11],
            [0.32, 1.15, 0.12],
            [0.38, 0.72],
            [0.62, 0.90, 0.56, 0.14],
            [0.50, 0.32],
            [0.40, 1.00],
            [1.02, -2.15],
            [1.02, -2.10, 1.72, 0.62],
            vec![([0.0, 0.975, 1.50], [0.8, 0.02, 0.30])],
        ),
        // Nissan Skyline GT-R (R34)
        spec(
            "Nissan Skyline GT-R (R34)",
            4.60,
            1.79,
            1.36,
            2.67,
            1.53,
            0.34,
            &[
                [2.30, 0.30, 0.54, 0.62, 0.85, 0.80],
                [2.20, 0.22, 0.60, 0.78, 0.89, 0.83],
                [1.65, 0.22, 0.64, 0.90, 0.895, 0.835],
                [0.95, 0.22, 0.68, 0.98, 0.895, 0.835],
                [-1.00, 0.22, 0.72, 1.00, 0.895, 0.835],
                [-1.95, 0.22, 0.74, 1.04, 0.895, 0.835],
                [-2.30, 0.28, 0.68, 0.98, 0.86, 0.81],
            ],
            &[
                [0.95, NA, 0.81],
                [0.10, 1.34, 0.68],
                [-0.50, 1.36, 0.68],
                [-0.60, 1.36, 0.68],
                [-1.20, 1.34, 0.68],
                [-1.65, NA, 0.77],
            ],
            [0.60, 0.70, 0.52, 0.14],
            [0.52, 0.64, 0.15],
            [0.36, 1.15, 0.15],
            [0.42, 0.76],
            [0.62, 0.88, 0.60, 0.13],
            [0.56, 0.34],
            [0.50, 1.08],
            [1.04, -2.12],
            [1.04, -2.10, 1.60, 0.55],
            vec![
                ([-0.38, 0.995, 1.50], [0.28, 0.02, 0.50]),
                ([0.38, 0.995, 1.50], [0.28, 0.02, 0.50]),
            ],
        ),
    ]
}

/// Гражданский трафик: 0 седан, 1 хэтчбек, 2 фургон, 3 грузовик.
pub fn all_traffic_specs() -> Vec<CarSpec> {
    vec![
        spec(
            "Sedan",
            4.60,
            1.78,
            1.45,
            2.70,
            1.52,
            0.31,
            &[
                [2.30, 0.32, 0.52, 0.62, 0.84, 0.79],
                [2.20, 0.24, 0.58, 0.76, 0.88, 0.82],
                [1.75, 0.22, 0.62, 0.88, 0.89, 0.83],
                [1.00, 0.22, 0.66, 0.94, 0.89, 0.83],
                [-1.10, 0.22, 0.68, 0.96, 0.89, 0.83],
                [-1.85, 0.22, 0.70, 1.00, 0.89, 0.83],
                [-2.30, 0.28, 0.64, 0.94, 0.85, 0.80],
            ],
            &[
                [1.00, NA, 0.80],
                [0.25, 1.40, 0.68],
                [-0.35, 1.42, 0.68],
                [-0.50, 1.42, 0.68],
                [-1.00, 1.40, 0.67],
                [-1.55, NA, 0.76],
            ],
            [0.60, 0.68, 0.46, 0.12],
            [0.56, 0.62, 0.12],
            [0.36, 1.00, 0.12],
            [0.40, 0.70],
            [0.64, 0.88, 0.44, 0.14],
            [0.55, 0.34],
            [0.60, 1.04],
            [0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            vec![],
        ),
        spec(
            "Hatchback",
            4.20,
            1.76,
            1.46,
            2.58,
            1.52,
            0.31,
            &[
                [2.10, 0.30, 0.50, 0.60, 0.84, 0.79],
                [2.00, 0.24, 0.56, 0.74, 0.87, 0.82],
                [1.60, 0.22, 0.60, 0.86, 0.88, 0.82],
                [0.95, 0.22, 0.64, 0.92, 0.88, 0.82],
                [-1.20, 0.22, 0.66, 0.94, 0.88, 0.82],
                [-1.95, 0.22, 0.66, 0.92, 0.88, 0.82],
                [-2.10, 0.28, 0.60, 0.86, 0.85, 0.80],
            ],
            &[
                [0.95, NA, 0.80],
                [0.15, 1.42, 0.69],
                [-0.30, 1.44, 0.69],
                [-0.45, 1.44, 0.69],
                [-1.55, 1.42, 0.68],
                [-1.98, NA, 0.78],
            ],
            [0.58, 0.66, 0.46, 0.12],
            [0.54, 0.60, 0.12],
            [0.34, 1.00, 0.12],
            [0.40, 0.66],
            [0.64, 0.84, 0.42, 0.18],
            [0.55, 0.34],
            [0.55, 1.04],
            [0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            vec![],
        ),
        box_spec("Van", 5.00, 1.95, 2.00, 3.20, 1.66, 0.34),
        box_spec("Truck", 7.20, 2.40, 3.00, 4.50, 2.00, 0.45),
    ]
}

// ----------------------------- сглаживание станций -----------------------------

/// Монотонная кубическая интерполяция (PCHIP): гладкая и без "перелётов".
struct Pchip {
    xs: Vec<f32>,
    ys: Vec<f32>,
    m: Vec<f32>,
}

impl Pchip {
    fn new(xs: Vec<f32>, ys: Vec<f32>) -> Pchip {
        let n = xs.len();
        let mut h = vec![0.0f32; n - 1];
        let mut d = vec![0.0f32; n - 1];
        for i in 0..(n - 1) {
            h[i] = xs[i + 1] - xs[i];
            d[i] = (ys[i + 1] - ys[i]) / h[i];
        }
        let mut m = vec![0.0f32; n];
        if n == 2 {
            m[0] = d[0];
            m[1] = d[0];
        } else {
            for i in 1..(n - 1) {
                if d[i - 1] * d[i] > 0.0 {
                    let w1 = 2.0 * h[i] + h[i - 1];
                    let w2 = h[i] + 2.0 * h[i - 1];
                    m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
                }
            }
            m[0] = d[0];
            m[n - 1] = d[n - 2];
        }
        Pchip { xs, ys, m }
    }

    fn eval(&self, x: f32) -> f32 {
        let n = self.xs.len();
        let mut i = n - 2;
        for k in 0..(n - 1) {
            if self.xs[k] <= x && x <= self.xs[k + 1] {
                i = k;
                break;
            }
        }
        let h = self.xs[i + 1] - self.xs[i];
        let t = (x - self.xs[i]) / h;
        let (t2, t3) = (t * t, t * t * t);
        (2.0 * t3 - 3.0 * t2 + 1.0) * self.ys[i]
            + (t3 - 2.0 * t2 + t) * h * self.m[i]
            + (-2.0 * t3 + 3.0 * t2) * self.ys[i + 1]
            + (t3 - t2) * h * self.m[i + 1]
    }
}

/// Станции (по убыванию z) -> плотные станции с шагом ~`spacing` метров.
fn densify(st: &[[f32; 6]], spacing: f32) -> Vec<[f32; 6]> {
    let mut rows: Vec<[f32; 6]> = st.to_vec();
    rows.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
    let zs: Vec<f32> = rows.iter().map(|r| r[0]).collect();
    let funcs: Vec<Pchip> = (1..6)
        .map(|k| Pchip::new(zs.clone(), rows.iter().map(|r| r[k]).collect()))
        .collect();
    let mut out: Vec<[f32; 6]> = Vec::new();
    for i in 0..(zs.len() - 1) {
        let k = (((zs[i + 1] - zs[i]) / spacing).ceil() as usize).max(1);
        for s in 0..k {
            let z = zs[i] + (zs[i + 1] - zs[i]) * (s as f32) / (k as f32);
            let mut r = [z, 0.0, 0.0, 0.0, 0.0, 0.0];
            for q in 0..5 {
                r[q + 1] = funcs[q].eval(z);
            }
            out.push(r);
        }
    }
    let zl = zs[zs.len() - 1];
    let mut r = [zl, 0.0, 0.0, 0.0, 0.0, 0.0];
    for q in 0..5 {
        r[q + 1] = funcs[q].eval(zl);
    }
    out.push(r);
    out.reverse();
    out
}

/// Значение поля `idx` станции в точке z (станции идут по убыванию z).
pub fn interp(st: &[[f32; 6]], z: f32, idx: usize) -> f32 {
    if z >= st[0][0] {
        return st[0][idx];
    }
    for i in 0..(st.len() - 1) {
        let z0 = st[i][0];
        let z1 = st[i + 1][0];
        if z0 >= z && z >= z1 {
            let t = if z0 != z1 { (z0 - z) / (z0 - z1) } else { 0.0 };
            return st[i][idx] + (st[i + 1][idx] - st[i][idx]) * t;
        }
    }
    st[st.len() - 1][idx]
}

/// Плечо не ниже верха колеса; спереди и сзади добавлены сужающиеся станции
/// (закруглённые бампера).
fn body_stations(s: &CarSpec) -> Vec<[f32; 6]> {
    let st: Vec<[f32; 6]> = s
        .body
        .iter()
        .map(|r| {
            let ym = r[2].max(2.0 * s.r + 0.04).min(r[3] - 0.05);
            [r[0], r[1], ym, r[3], r[4], r[5]]
        })
        .collect();
    let tip = |r: [f32; 6], dz: f32| -> [f32; 6] {
        let yb2 = r[1] + (r[3] - r[1]) * 0.45;
        let yt2 = r[3] - 0.05;
        let ym2 = yb2 + (yt2 - yb2) * 0.55;
        [r[0] + dz, yb2, ym2, yt2, r[4] * 0.80, r[5] * 0.76]
    };
    let first = st[0];
    let last = st[st.len() - 1];
    let mut out: Vec<[f32; 6]> = vec![tip(first, 0.13)];
    out.extend(st.iter().cloned());
    out.push(tip(last, -0.13));
    out
}

/// Сечение кузова (14 точек): низ, порог, борт, скруглённое плечо, верх.
fn body_ring(r: &[f32; 6]) -> Vec<V3> {
    let (z, yb, ym, yt, w, wt) = (r[0], r[1], r[2], r[3], r[4], r[5]);
    let mut right: Vec<(f32, f32)> = vec![(w, yb), (w, yb + 0.10), (w, ym)];
    for th in [22.5f32, 45.0, 67.5].iter() {
        let a = th.to_radians();
        right.push((wt + (w - wt) * a.cos(), ym + (yt - ym) * a.sin()));
    }
    right.push((wt, yt));
    let mut ring: Vec<V3> = right.iter().map(|&(x, y)| v3(x, y, z)).collect();
    for &(x, y) in right.iter().rev() {
        ring.push(v3(-x, y, z));
    }
    ring
}

/// Станции салона: если высота крыши не задана, берётся линия кузова; плотность x3 (линейно,
/// чтобы лобовое и заднее стекло оставались плоскими).
fn cabin_stations(s: &CarSpec, st: &[[f32; 6]]) -> Vec<[f32; 3]> {
    let mut cab: Vec<[f32; 3]> = Vec::new();
    for c in s.cabin.iter() {
        let yb = interp(st, c[0], 3) + 0.002;
        cab.push([c[0], if c[1].is_nan() { yb } else { c[1] }, c[2]]);
    }
    let mut out: Vec<[f32; 3]> = Vec::new();
    for i in 0..(cab.len() - 1) {
        for k in 0..3 {
            let t = k as f32 / 3.0;
            out.push([
                lerpf(cab[i][0], cab[i + 1][0], t),
                lerpf(cab[i][1], cab[i + 1][1], t),
                lerpf(cab[i][2], cab[i + 1][2], t),
            ]);
        }
    }
    out.push(cab[cab.len() - 1]);
    out
}

fn arches(b: &mut Builder, s: &CarSpec, st: &[[f32; 6]]) {
    let r = s.r;
    for &sx in [-1.0f32, 1.0].iter() {
        for &zc in [s.wb / 2.0, -s.wb / 2.0].iter() {
            let w = interp(st, zc, 4);
            let yb = interp(st, zc, 1);
            let ym = interp(st, zc, 2);
            let mut pts: Vec<V3> = Vec::new();
            for k in 0..20 {
                let a = 2.0 * PI * (k as f32) / 20.0;
                let y = r + (r + 0.07) * a.sin();
                pts.push(v3(
                    sx * (w + 0.004),
                    y.min(ym - 0.03).max(yb),
                    zc + (r + 0.07) * a.cos(),
                ));
            }
            b.flat(
                &pts,
                [0.01, 0.01, 0.015],
                Mat::Plastic,
                Some(v3(0.0, r, zc)),
            );
        }
    }
}

fn details(b: &mut Builder, s: &CarSpec, st: &[[f32; 6]], rgb: Rgb, glow: Option<Rgb>) {
    let zf = st[0][0];
    let zr = st[st.len() - 1][0];
    let front_white: Rgb = [1.0, 0.97, 0.84];
    let front_lens: Rgb = [0.72, 0.82, 0.92];
    let tail_red: Rgb = [0.82, 0.025, 0.045];
    let tail_dark: Rgb = [0.20, 0.025, 0.035];
    let chrome: Rgb = [0.48, 0.50, 0.53];
    let black: Rgb = [0.018, 0.020, 0.026];

    // Front fascia: dark grille, lower intake, and a narrow painted bumper.
    b.boxed(
        v3(0.0, s.grille[0], zf - 0.025),
        v3(s.grille[1], s.grille[2], 0.075),
        black,
        Mat::Plastic,
    );
    // Thin horizontal grille vanes give the intake depth instead of a flat black patch.
    let grille_rows = 4usize;
    for k in 0..grille_rows {
        let t = (k as f32 + 1.0) / (grille_rows as f32 + 1.0);
        let y = s.grille[0] - s.grille[2] * 0.5 + s.grille[2] * t;
        b.boxed(
            v3(0.0, y, zf - 0.071),
            v3(s.grille[1] * 0.90, 0.012, 0.012),
            chrome,
            Mat::Rim,
        );
    }
    // Small central badge, lower intake and painted bumper lip.
    b.boxed(
        v3(0.0, s.grille[0], zf - 0.082),
        v3(0.095, 0.075, 0.018),
        chrome,
        Mat::Rim,
    );
    b.boxed(
        v3(0.0, s.intake[0], zf - 0.030),
        v3(s.intake[1], s.intake[2], 0.075),
        black,
        Mat::Plastic,
    );
    b.boxed(
        v3(0.0, s.intake[0] - s.intake[2] * 0.56, zf - 0.012),
        v3(s.intake[1] * 0.88, 0.025, 0.09),
        chrome,
        Mat::Rim,
    );
    b.boxed(
        v3(0.0, s.head[1] - s.head[3] * 0.90, zf - 0.018),
        v3(s.w * 0.82, 0.055, 0.11),
        rgb,
        Mat::Paint,
    );

    // Separate headlamp lenses and inner projector details on both corners.
    for &sx in &[-1.0_f32, 1.0] {
        let hx = sx * s.head[0];
        b.boxed(
            v3(hx, s.head[1], zf - 0.045),
            v3(s.head[2], s.head[3], 0.065),
            front_lens,
            Mat::Glass,
        );
        b.boxed(
            v3(hx, s.head[1], zf - 0.082),
            v3(s.head[2] * 0.72, s.head[3] * 0.48, 0.018),
            front_white,
            Mat::Emissive,
        );
        b.boxed(
            v3(sx * s.intake[1] * 0.38, s.intake[0] + 0.015, zf - 0.075),
            v3(0.12, 0.045, 0.018),
            front_white,
            Mat::Emissive,
        );

        // Side repeater and a subtle fender highlight above each wheel arch.
        let repeater_z = s.wb * 0.36;
        let repeater_x = sx * (interp(st, repeater_z, 4) + 0.018);
        let repeater_y = interp(st, repeater_z, 2) + 0.015;
        b.boxed(
            v3(repeater_x, repeater_y, repeater_z),
            v3(0.018, 0.035, 0.105),
            [0.95, 0.48, 0.08],
            Mat::Emissive,
        );

        // Tail lamps use a dark housing, a red lens, and a brighter brake strip.
        let tx = sx * s.tail[0];
        b.boxed(
            v3(tx, s.tail[1], zr - 0.035),
            v3(s.tail[2] * 1.10, s.tail[3] * 1.15, 0.065),
            tail_dark,
            Mat::Plastic,
        );
        b.boxed(
            v3(tx, s.tail[1], zr - 0.075),
            v3(s.tail[2], s.tail[3], 0.022),
            tail_red,
            Mat::Emissive,
        );
        b.boxed(
            v3(tx, s.tail[1] + s.tail[3] * 0.18, zr - 0.089),
            v3(s.tail[2] * 0.78, s.tail[3] * 0.18, 0.012),
            [1.0, 0.12, 0.08],
            Mat::Emissive,
        );

        // Mirrors, mirror glass, exhaust tips, and small parking sensors.
        let (mz, my) = (s.mirror[0], s.mirror[1]);
        let mx = interp(st, mz, 5) + 0.085;
        b.boxed(v3(sx * mx, my, mz), v3(0.16, 0.10, 0.20), rgb, Mat::Paint);
        b.boxed(
            v3(sx * (mx + 0.012), my + 0.004, mz - 0.012),
            v3(0.012, 0.062, 0.12),
            front_lens,
            Mat::Glass,
        );
        b.boxed(
            v3(sx * s.exh[0], s.exh[1], zr - 0.015),
            v3(0.12, 0.105, 0.13),
            chrome,
            Mat::Rim,
        );
        b.boxed(
            v3(sx * s.exh[0], s.exh[1], zr - 0.088),
            v3(0.075, 0.065, 0.018),
            black,
            Mat::Plastic,
        );
        for k in 0..2 {
            let z = zf - 0.10 - k as f32 * 0.12;
            b.boxed(
                v3(sx * (s.w * 0.28), s.intake[0] - 0.02, z),
                v3(0.035, 0.035, 0.018),
                [0.12, 0.13, 0.14],
                Mat::Plastic,
            );
        }
    }

    // Front and rear registration plates with a dark frame.
    for &(z, y) in &[(zf - 0.075, s.plates[0]), (zr - 0.075, s.plates[1])] {
        b.boxed(v3(0.0, y, z), v3(0.56, 0.13, 0.025), black, Mat::Plastic);
        b.boxed(
            v3(0.0, y, z + if z > 0.0 { -0.016 } else { 0.016 }),
            v3(0.50, 0.095, 0.012),
            [0.88, 0.90, 0.86],
            Mat::Plastic,
        );
    }

    // Door shut lines, handles, side skirts, and restrained trim on both sides.
    let front_door_z = s.wb * 0.16;
    let rear_door_z = -s.wb * 0.48;
    for &sx in &[-1.0_f32, 1.0] {
        for &z in &[front_door_z, rear_door_z] {
            let x = sx * (interp(st, z, 4) + 0.012);
            let y0 = interp(st, z, 1) + 0.12;
            let y1 = interp(st, z, 3) - 0.08;
            b.boxed(
                v3(x, (y0 + y1) * 0.5, z),
                v3(0.012, (y1 - y0).max(0.24), 0.014),
                black,
                Mat::Plastic,
            );
        }
        for &z in &[s.wb * 0.12, -s.wb * 0.52] {
            let x = sx * (interp(st, z, 4) + 0.02);
            let y = interp(st, z, 2) + 0.04;
            b.boxed(v3(x, y, z), v3(0.018, 0.028, 0.14), chrome, Mat::Rim);
            b.boxed(
                v3(x + sx * 0.009, y, z),
                v3(0.012, 0.014, 0.09),
                black,
                Mat::Plastic,
            );
        }
        let skirt_x = sx * (s.w * 0.465);
        b.boxed(
            v3(skirt_x, 0.285, -0.02),
            v3(0.055, 0.085, s.wb * 0.80),
            [rgb[0] * 0.72, rgb[1] * 0.72, rgb[2] * 0.72],
            Mat::Paint,
        );
        b.boxed(
            v3(sx * (s.w * 0.468), 0.43, 0.0),
            v3(0.018, 0.018, s.wb * 0.74),
            chrome,
            Mat::Rim,
        );

        // Fuel flap on the rear quarter, aligned to the body surface.
        let fuel_z = -s.wb * 0.72;
        let fuel_x = sx * (interp(st, fuel_z, 4) + 0.014);
        let fuel_y = interp(st, fuel_z, 2) + 0.10;
        b.boxed(
            v3(fuel_x, fuel_y, fuel_z),
            v3(0.012, 0.15, 0.19),
            [rgb[0] * 0.88, rgb[1] * 0.88, rgb[2] * 0.88],
            Mat::Paint,
        );
    }

    // Windscreen wipers, rear diffuser, and model-specific vents/spoilers.
    for &sx in &[-1.0_f32, 1.0] {
        b.boxed(
            v3(sx * 0.20, interp(st, 0.78, 3) + 0.015, 0.77),
            v3(0.025, 0.018, 0.42),
            black,
            Mat::Plastic,
        );
        b.boxed(
            v3(sx * 0.22, 0.31, zr + 0.012),
            v3(0.16, 0.035, 0.05),
            black,
            Mat::Plastic,
        );
    }
    b.boxed(
        v3(0.0, 0.30, zr - 0.015),
        v3(s.w * 0.68, 0.08, 0.07),
        black,
        Mat::Plastic,
    );
    for (k, (c, size)) in s.extras.iter().enumerate() {
        let _ = k;
        b.boxed(
            v3(c[0], c[1], c[2]),
            v3(size[0], size[1], size[2]),
            black,
            Mat::Plastic,
        );
    }
    if let Some(g) = glow {
        for &sx in &[-1.0_f32, 1.0] {
            b.boxed(
                v3(sx * (interp(st, 0.0, 4) + 0.015), st[0][1] + 0.18, 0.0),
                v3(0.045, 0.045, s.wb * 0.78),
                g,
                Mat::Emissive,
            );
        }
    }
}

fn spoiler(b: &mut Builder, s: &CarSpec, rgb: Rgb, kind: u8) {
    if kind == 1 {
        let (y, z) = (s.lip[0], s.lip[1]);
        b.boxed(
            v3(0.0, y + 0.04, z),
            v3(s.w * 0.78, 0.05, 0.26),
            rgb,
            Mat::Paint,
        );
    } else if kind == 2 {
        let (y, z, span, h) = (s.wing[0], s.wing[1], s.wing[2], s.wing[3]);
        b.boxed(v3(0.0, y + h, z), v3(span, 0.06, 0.42), DARK, Mat::Plastic);
        for &sx in [-1.0f32, 1.0].iter() {
            b.boxed(
                v3(sx * span * 0.32, y + h / 2.0, z),
                v3(0.06, h, 0.10),
                DARK,
                Mat::Plastic,
            );
            b.boxed(
                v3(sx * span / 2.0, y + h, z),
                v3(0.04, 0.20, 0.46),
                rgb,
                Mat::Paint,
            );
        }
    }
}

/// Полигоны кузова реальной модели (без колёс).
/// `spoiler_kind`: 0 нет, 1 лёгкий, 2 GT-крыло.
pub fn build_body(s: &CarSpec, rgb: Rgb, glow: Option<Rgb>, spoiler_kind: u8) -> Vec<Poly> {
    let mut b = Builder::new();
    let st = densify(&body_stations(s), 0.28);
    let rings: Vec<Vec<V3>> = st.iter().map(|r| body_ring(r)).collect();
    let hints: Vec<V3> = st
        .iter()
        .map(|r| v3(0.0, (r[1] + r[3]) / 2.0, r[0]))
        .collect();
    let body_f = |_i: usize, j: usize| -> (Rgb, Mat) {
        if j == 0 || j == 12 {
            (DARK, Mat::Plastic) // пороги
        } else {
            (rgb, Mat::Paint)
        }
    };
    b.grid(&rings, &hints, &body_f);
    // днище
    for i in 0..(rings.len() - 1) {
        let a = &rings[i];
        let c = &rings[i + 1];
        b.flat(
            &[a[13], a[0], c[0], c[13]],
            DARK,
            Mat::Plastic,
            Some(hints[i]),
        );
    }
    // нос и корма
    let last = rings.len() - 1;
    b.flat(
        &rings[0],
        rgb,
        Mat::Paint,
        Some(v3(hints[0].x, hints[0].y, hints[0].z - 1.0)),
    );
    b.flat(
        &rings[last],
        rgb,
        Mat::Paint,
        Some(v3(hints[last].x, hints[last].y, hints[last].z + 1.0)),
    );

    // салон: борта (стекло/рейка), крыша, лобовое и заднее стекло
    let cab = cabin_stations(s, &st);
    let mut crings: Vec<Vec<V3>> = Vec::new();
    let mut chints: Vec<V3> = Vec::new();
    for c in cab.iter() {
        let (z, yr, wr) = (c[0], c[1], c[2]);
        let yb = interp(&st, z, 3) + 0.002;
        let wb = interp(&st, z, 5) - 0.02;
        let (slx, sly) = (lerpf(-wb, -wr, 0.86), lerpf(yb, yr, 0.86));
        let (srx, sry) = (lerpf(wb, wr, 0.86), lerpf(yb, yr, 0.86));
        let crown = if yr > yb + 0.05 { 0.02 } else { 0.0 };
        crings.push(vec![
            v3(-wb, yb, z),
            v3(slx, sly, z),
            v3(-wr, yr, z),
            v3(0.0, yr + crown, z),
            v3(wr, yr, z),
            v3(srx, sry, z),
            v3(wb, yb, z),
        ]);
        chints.push(v3(0.0, (yb + yr) / 2.0 - 0.08, z));
    }
    let roles = &s.roles;
    let cab_f = |i: usize, e: usize| -> (Rgb, Mat) {
        let role = roles[(i / 3).min(roles.len() - 1)];
        let glass = (role == Role::GlassSide && (e == 0 || e == 5))
            || (role == Role::GlassTop && (e == 2 || e == 3));
        if glass {
            (GLASS, Mat::Glass)
        } else {
            (rgb, Mat::Paint)
        }
    };
    b.grid(&crings, &chints, &cab_f);

    // Slim window surrounds and pillars sit on the side glazing and make the
    // windscreen/cabin read as separate glass panels rather than one dark blob.
    for &sx in &[-1.0_f32, 1.0] {
        for c in s.cabin.iter().skip(1).take(s.cabin.len().saturating_sub(2)) {
            let z = c[0];
            let roof_y = if c[1].is_nan() {
                interp(&st, z, 3)
            } else {
                c[1]
            };
            let shoulder_y = interp(&st, z, 3) + 0.012;
            let x = sx * (c[2] - 0.006);
            let pillar_h = (roof_y - shoulder_y).max(0.10);
            b.boxed(
                v3(x, shoulder_y + pillar_h * 0.5, z),
                v3(0.025, pillar_h, 0.035),
                rgb,
                Mat::Paint,
            );
        }
    }

    arches(&mut b, s, &st);
    details(&mut b, s, &st, rgb, glow);
    if spoiler_kind > 0 {
        spoiler(&mut b, s, rgb, spoiler_kind);
    }
    b.polys
}

// ----------------------------- колёса -----------------------------

fn pv(p: V3, n: V3, c: Rgb, m: Mat) -> PV {
    PV { p, n, c, m }
}

/// Колесо в локальных координатах (начало - центр колеса, x - поперёк машины).
/// `style`: 0 диск, 1 спицы, 2 турбина.
pub fn wheel_polys(s: &CarSpec, sx: f32, rim: Rgb, style: u8) -> Vec<Poly> {
    let r = s.r;
    let half_width = 0.115_f32;
    let segments = 48usize;
    let mut polys = Vec::new();

    // Rounded tire carcass with a subtle tread surface.
    for k in 0..segments {
        let a0 = 2.0 * PI * k as f32 / segments as f32;
        let a1 = 2.0 * PI * (k + 1) as f32 / segments as f32;
        let n0 = v3(0.0, a0.cos(), a0.sin());
        let n1 = v3(0.0, a1.cos(), a1.sin());
        let point = |a: f32, x: f32, radius: f32| v3(x, a.cos() * radius, a.sin() * radius);
        polys.push(Poly {
            v: vec![
                pv(point(a0, -half_width, r), n0, TIRE, Mat::Tire),
                pv(point(a0, half_width, r), n0, TIRE, Mat::Tire),
                pv(point(a1, half_width, r), n1, TIRE, Mat::Tire),
                pv(point(a1, -half_width, r), n1, TIRE, Mat::Tire),
            ],
        });
        // Fine tread grooves around the shoulder, represented as narrow dark strips.
        if k % 2 == 0 {
            let mid = (a0 + a1) * 0.5;
            let p0 = point(mid, -half_width * 0.98, r * 0.985);
            let p1 = point(mid, half_width * 0.98, r * 0.985);
            let p2 = point(mid + 0.012, half_width * 0.98, r * 0.985);
            let p3 = point(mid + 0.012, -half_width * 0.98, r * 0.985);
            polys.push(Poly {
                v: vec![
                    pv(p0, n0, [0.025, 0.025, 0.03], Mat::Tire),
                    pv(p1, n0, [0.025, 0.025, 0.03], Mat::Tire),
                    pv(p2, n1, [0.025, 0.025, 0.03], Mat::Tire),
                    pv(p3, n1, [0.025, 0.025, 0.03], Mat::Tire),
                ],
            });
        }
    }

    // Sidewall faces and a subtle raised sidewall ring.
    for &side in &[-1.0_f32, 1.0] {
        let face: Vec<PV> = (0..segments)
            .map(|k| {
                let a = 2.0 * PI * k as f32 / segments as f32;
                pv(
                    v3(side * half_width, a.cos() * r, a.sin() * r),
                    v3(side, 0.0, 0.0),
                    [0.045, 0.045, 0.052],
                    Mat::Tire,
                )
            })
            .collect();
        polys.push(Poly { v: face });
        let side_ring: Vec<PV> = (0..segments)
            .map(|k| {
                let a = 2.0 * PI * k as f32 / segments as f32;
                pv(
                    v3(
                        side * (half_width + 0.001),
                        a.cos() * r * 0.84,
                        a.sin() * r * 0.84,
                    ),
                    v3(side, 0.0, 0.0),
                    [0.075, 0.075, 0.08],
                    Mat::Tire,
                )
            })
            .collect();
        polys.push(Poly { v: side_ring });
    }

    // Brake rotor behind the alloy wheel, followed by the rim barrel and spokes.
    let face_x = sx * (half_width + 0.006);
    let circle = |offset: f32, radius: f32, colour: Rgb, material: Mat| -> Poly {
        Poly {
            v: (0..segments)
                .map(|k| {
                    let a = 2.0 * PI * k as f32 / segments as f32;
                    pv(
                        v3(
                            sx * (half_width + offset),
                            a.cos() * radius,
                            a.sin() * radius,
                        ),
                        v3(sx, 0.0, 0.0),
                        colour,
                        material,
                    )
                })
                .collect(),
        }
    };
    let outer_r = r * 0.79;
    polys.push(circle(0.004, outer_r, [0.025, 0.027, 0.032], Mat::Plastic));
    polys.push(circle(0.008, outer_r * 0.90, [0.30, 0.31, 0.33], Mat::Rim));
    polys.push(circle(
        0.010,
        outer_r * 0.83,
        [0.055, 0.058, 0.065],
        Mat::Plastic,
    ));

    // A visible brake caliper near the upper rear quadrant of the rotor.
    let caliper_x = sx * (half_width + 0.013);
    let caliper = [
        v3(caliper_x, -outer_r * 0.40, outer_r * 0.18),
        v3(caliper_x, -outer_r * 0.12, outer_r * 0.18),
        v3(caliper_x, -outer_r * 0.08, outer_r * 0.52),
        v3(caliper_x, -outer_r * 0.38, outer_r * 0.52),
    ];
    polys.push(Poly {
        v: caliper
            .iter()
            .map(|&p| pv(p, v3(sx, 0.0, 0.0), [0.68, 0.07, 0.035], Mat::Paint))
            .collect(),
    });

    if style == 0 {
        polys.push(circle(0.014, outer_r * 0.74, rim, Mat::Rim));
        polys.push(circle(
            0.018,
            outer_r * 0.38,
            [rim[0] * 0.68, rim[1] * 0.68, rim[2] * 0.68],
            Mat::Rim,
        ));
    } else {
        polys.push(circle(0.014, outer_r * 0.94, rim, Mat::Rim));
        polys.push(circle(
            0.018,
            outer_r * 0.84,
            [0.025, 0.027, 0.032],
            Mat::Plastic,
        ));
        let spoke_count = if style == 1 { 5 } else { 10 };
        let thickness = if style == 1 { 0.047_f32 } else { 0.024_f32 };
        let x = sx * (half_width + 0.023);
        for k in 0..spoke_count {
            let a = 2.0 * PI * k as f32 / spoke_count as f32;
            let (ca, sa) = (a.cos(), a.sin());
            let inner = outer_r * 0.16;
            let outer = outer_r * 0.82;
            let (nx, ny) = (-sa * thickness, ca * thickness);
            let pts = [
                v3(x, ca * inner + nx, sa * inner + ny),
                v3(x, ca * inner - nx, sa * inner - ny),
                v3(x, ca * outer - nx * 0.45, sa * outer - ny * 0.45),
                v3(x, ca * outer + nx * 0.45, sa * outer + ny * 0.45),
            ];
            polys.push(Poly {
                v: pts
                    .iter()
                    .map(|&p| pv(p, v3(sx, 0.0, 0.0), rim, Mat::Rim))
                    .collect(),
            });
        }
    }

    // Hub and five lug nuts, placed around the centre rather than stacked on it.
    polys.push(circle(0.027, outer_r * 0.16, [0.12, 0.13, 0.15], Mat::Rim));
    let nut_x = face_x + sx * 0.025;
    for k in 0..5 {
        let a = 2.0 * PI * k as f32 / 5.0;
        let cy = a.cos() * outer_r * 0.24;
        let cz = a.sin() * outer_r * 0.24;
        let rr = outer_r * 0.035;
        let pts = [
            v3(nut_x, cy + rr, cz),
            v3(nut_x, cy - rr * 0.5, cz + rr * 0.87),
            v3(nut_x, cy - rr * 0.5, cz - rr * 0.87),
        ];
        polys.push(Poly {
            v: pts
                .iter()
                .map(|&p| pv(p, v3(sx, 0.0, 0.0), [0.72, 0.73, 0.75], Mat::Rim))
                .collect(),
        });
    }
    polys
}

/// Центры колёс. Внешняя грань диска выступает за кузов на ~2 см,
/// чтобы колесо было видно в арке.
pub fn wheel_positions(s: &CarSpec) -> Vec<V3> {
    let dense: Option<Vec<[f32; 6]>> = if !s.body.is_empty() {
        Some(densify(&body_stations(s), 0.28))
    } else {
        None
    };
    let mut out = Vec::new();
    for &sz in [1.0f32, -1.0].iter() {
        let z = sz * s.wb / 2.0;
        let half = match &dense {
            Some(d) => interp(d, z, 4) - 0.103,
            None => s.w / 2.0 - 0.103,
        };
        for &sx in [-1.0f32, 1.0].iter() {
            out.push(v3(sx * half, s.r, z));
        }
    }
    out
}

fn simple_wheel(r: f32, sx: f32) -> Vec<Poly> {
    let n = 12usize;
    let hw = 0.115f32;
    let mut polys: Vec<Poly> = Vec::new();
    for k in 0..n {
        let a0 = 2.0 * PI * (k as f32) / (n as f32);
        let a1 = 2.0 * PI * ((k + 1) as f32) / (n as f32);
        let n0 = v3(0.0, a0.cos(), a0.sin());
        let n1 = v3(0.0, a1.cos(), a1.sin());
        let p = |a: f32, x: f32| v3(x, a.cos() * r, a.sin() * r);
        polys.push(Poly {
            v: vec![
                pv(p(a0, -hw), n0, TIRE, Mat::Tire),
                pv(p(a0, hw), n0, TIRE, Mat::Tire),
                pv(p(a1, hw), n1, TIRE, Mat::Tire),
                pv(p(a1, -hw), n1, TIRE, Mat::Tire),
            ],
        });
    }
    let v: Vec<PV> = (0..n)
        .map(|k| {
            let a = 2.0 * PI * (k as f32) / (n as f32);
            pv(
                v3(sx * (hw + 0.004), a.cos() * r * 0.62, a.sin() * r * 0.62),
                v3(sx, 0.0, 0.0),
                [0.55, 0.55, 0.58],
                Mat::Rim,
            )
        })
        .collect();
    polys.push(Poly { v });
    polys
}

/// Гражданская машина целиком (кузов + неподвижные колёса), без тюнинга.
/// `kind`: 0 седан, 1 хэтчбек, 2 фургон, 3 грузовик.
pub fn build_traffic(s: &CarSpec, kind: usize, rgb: Rgb) -> Vec<Poly> {
    let mut polys: Vec<Poly>;
    if !s.body.is_empty() {
        polys = build_body(s, rgb, None, 0);
    } else {
        let mut b = Builder::new();
        let (l, w) = (s.l, s.w);
        let hl = l / 2.0;
        if kind == 2 {
            b.boxed(v3(0.0, 1.15, 0.0), v3(w, 1.75, l), rgb, Mat::Paint);
            b.boxed(
                v3(0.0, 1.55, hl - 0.02),
                v3(w - 0.2, 0.62, 0.10),
                GLASS,
                Mat::Glass,
            );
            for &sx in [-1.0f32, 1.0].iter() {
                b.boxed(
                    v3(sx * (w / 2.0), 1.55, hl - 0.85),
                    v3(0.04, 0.55, 1.0),
                    GLASS,
                    Mat::Glass,
                );
            }
        } else {
            b.boxed(v3(0.0, 1.05, hl - 1.2), v3(w, 1.70, 2.4), rgb, Mat::Paint);
            b.boxed(
                v3(0.0, 1.50, hl - 0.02),
                v3(w - 0.2, 0.65, 0.10),
                GLASS,
                Mat::Glass,
            );
            b.boxed(
                v3(0.0, 1.80, -1.25),
                v3(w, 2.50, l - 2.5),
                [0.84, 0.84, 0.86],
                Mat::Paint,
            );
        }
        b.boxed(
            v3(0.0, 0.50, 0.0),
            v3(w - 0.4, 0.30, l - 0.6),
            DARK,
            Mat::Plastic,
        );
        for &sx in [-1.0f32, 1.0].iter() {
            b.boxed(
                v3(sx * w * 0.32, 0.72, hl),
                v3(0.36, 0.14, 0.08),
                [1.0, 0.96, 0.80],
                Mat::Emissive,
            );
        }
        b.boxed(
            v3(0.0, 0.85, -hl),
            v3(w * 0.7, 0.14, 0.08),
            [0.6, 0.08, 0.10],
            Mat::Emissive,
        );
        polys = b.polys;
    }
    let spots: Vec<V3> = if !s.body.is_empty() {
        wheel_positions(s)
    } else {
        let zs: Vec<f32> = if kind == 2 {
            vec![1.6, -1.6]
        } else {
            vec![2.4, -1.6, -2.6]
        };
        let half = s.w / 2.0 - 0.103;
        let mut v = Vec::new();
        for &sx in [-1.0f32, 1.0].iter() {
            for &z in zs.iter() {
                v.push(v3(sx * half, s.r, z));
            }
        }
        v
    };
    for p in spots.iter() {
        let sx = if p.x > 0.0 { 1.0 } else { -1.0 };
        // Use the same detailed wheel assembly for traffic vehicles too.
        let wheel_style = if kind == 3 { 0 } else { 1 };
        let wheel_rim: Rgb = if kind == 3 {
            [0.42, 0.43, 0.45]
        } else {
            [0.62, 0.63, 0.65]
        };
        polys.extend(translate(&wheel_polys(s, sx, wheel_rim, wheel_style), *p));
    }
    polys
}

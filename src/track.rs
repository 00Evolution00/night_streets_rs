//! Трассы: сплайн, дорога, срезки и "какая поверхность под машиной".
use crate::math::*;

pub const ROAD_HALF: f32 = 13.0; // половина ширины основной дороги
pub const SC_HALF: f32 = 8.0; // половина ширины срезки

#[derive(Clone)]
pub struct TrackSpec {
    pub name: &'static str,
    pub laps: i32,
    pub reward: f32,
    pub ai: f32,
    pub traffic: usize,
    pub shortcuts: usize,
    pub seed: u64,
    pub sky: [f32; 3],
    pub ground: [f32; 3],
    pub asphalt: [u8; 3],
    pub buildings: usize,
    pub tints: Vec<[f32; 3]>,
    pub ctrl: Vec<[f32; 2]>,
}

pub fn track_specs() -> Vec<TrackSpec> {
    vec![
        TrackSpec {
            name: "Downtown Loop",
            laps: 3,
            reward: 1.0,
            ai: 1.00,
            traffic: 12,
            shortcuts: 2,
            seed: 1,
            sky: rgb!(6, 6, 18),
            ground: rgb!(8, 8, 16),
            asphalt: [30, 30, 42],
            buildings: 200,
            tints: vec![rgb!(150, 160, 230), rgb!(200, 150, 230), rgb!(140, 200, 230), rgb!(230, 160, 190)],
            ctrl: vec![
                [0.0, 0.0], [150.0, -10.0], [280.0, 40.0], [350.0, 150.0], [330.0, 260.0],
                [260.0, 320.0], [250.0, 420.0], [160.0, 475.0], [60.0, 430.0], [70.0, 330.0],
                [-20.0, 300.0], [-130.0, 320.0], [-210.0, 230.0], [-190.0, 110.0], [-110.0, 50.0],
            ],
        },
        TrackSpec {
            name: "Harbor Run",
            laps: 3,
            reward: 1.3,
            ai: 1.06,
            traffic: 16,
            shortcuts: 2,
            seed: 2,
            sky: rgb!(5, 14, 20),
            ground: rgb!(6, 12, 16),
            asphalt: [34, 36, 42],
            buildings: 170,
            tints: vec![rgb!(130, 190, 200), rgb!(150, 170, 190), rgb!(110, 160, 170), rgb!(190, 200, 210)],
            ctrl: vec![
                [0.0, 0.0], [200.0, 0.0], [380.0, 30.0], [460.0, 120.0], [430.0, 230.0],
                [330.0, 270.0], [245.0, 235.0], [240.0, 335.0], [330.0, 405.0], [300.0, 500.0],
                [180.0, 520.0], [60.0, 470.0], [0.0, 380.0], [-40.0, 260.0], [-30.0, 130.0],
            ],
        },
        TrackSpec {
            name: "Industrial Zone",
            laps: 2,
            reward: 1.6,
            ai: 1.12,
            traffic: 16,
            shortcuts: 2,
            seed: 3,
            sky: rgb!(20, 12, 10),
            ground: rgb!(14, 10, 8),
            asphalt: [40, 36, 34],
            buildings: 150,
            tints: vec![rgb!(200, 170, 140), rgb!(170, 150, 130), rgb!(150, 140, 135), rgb!(210, 150, 110)],
            ctrl: vec![
                [0.0, 0.0], [250.0, 0.0], [420.0, 0.0], [480.0, 100.0], [420.0, 190.0],
                [280.0, 200.0], [200.0, 260.0], [240.0, 350.0], [380.0, 380.0], [440.0, 470.0],
                [360.0, 560.0], [180.0, 560.0], [80.0, 500.0], [60.0, 400.0], [-60.0, 340.0],
                [-120.0, 230.0], [-80.0, 100.0],
            ],
        },
    ]
}

// ----------------------------- дорога -----------------------------

pub struct Road {
    pub pts: Vec<V3>,
    pub closed: bool,
    pub half: f32,
    pub xz: Vec<(f32, f32)>,
    pub tan: Vec<V3>,
    pub right: Vec<V3>,
    pub ang: Vec<f32>,
}

impl Road {
    pub fn new(pts: Vec<V3>, closed: bool, half: f32) -> Road {
        let n = pts.len();
        let mut xz = Vec::with_capacity(n);
        let mut tan = Vec::with_capacity(n);
        let mut right = Vec::with_capacity(n);
        let mut ang = Vec::with_capacity(n);
        for i in 0..n {
            let (a, b) = if closed {
                (pts[(i + n - 1) % n], pts[(i + 1) % n])
            } else {
                (pts[if i > 0 { i - 1 } else { 0 }], pts[if i + 1 < n { i + 1 } else { n - 1 }])
            };
            let t = (b - a).norm();
            xz.push((pts[i].x, pts[i].z));
            tan.push(t);
            right.push(v3(t.z, 0.0, -t.x));
            ang.push(t.x.atan2(t.z).to_degrees());
        }
        Road { pts, closed, half, xz, tan, right, ang }
    }

    pub fn n(&self) -> usize {
        self.pts.len()
    }

    /// Ближайшая точка осевой линии: (индекс, квадрат расстояния).
    /// С `hint` ищем только рядом с прошлым индексом (быстро).
    pub fn nearest(&self, p: V3, hint: Option<usize>, win: i32) -> (usize, f32) {
        let n = self.n();
        let mut best = 0usize;
        let mut bd = f32::MAX;
        let mut check = |i: usize| {
            let (x, z) = self.xz[i];
            let d = (x - p.x) * (x - p.x) + (z - p.z) * (z - p.z);
            if d < bd {
                bd = d;
                best = i;
            }
        };
        match hint {
            None => {
                for i in 0..n {
                    check(i);
                }
            }
            Some(h) => {
                if self.closed {
                    for k in -win..=win {
                        let i = (h as i32 + k).rem_euclid(n as i32) as usize;
                        check(i);
                    }
                } else {
                    let lo = (h as i32 - win).max(0) as usize;
                    let hi = ((h as i32 + win) as usize).min(n - 1);
                    for i in lo..=hi {
                        check(i);
                    }
                }
            }
        }
        (best, bd)
    }
}

fn catmull(p0: [f32; 2], p1: [f32; 2], p2: [f32; 2], p3: [f32; 2], t: f32) -> (f32, f32) {
    let t2 = t * t;
    let t3 = t2 * t;
    let f = |k: usize| -> f32 {
        0.5 * ((2.0 * p1[k])
            + (-p0[k] + p2[k]) * t
            + (2.0 * p0[k] - 5.0 * p1[k] + 4.0 * p2[k] - p3[k]) * t2
            + (-p0[k] + 3.0 * p1[k] - 3.0 * p2[k] + p3[k]) * t3)
    };
    (f(0), f(1))
}

/// Равномерная передискретизация ломаной (шаг `step` метров).
fn resample(pts: &[(f32, f32)], step: f32, closed: bool) -> (Vec<(f32, f32)>, f32) {
    let mut seq: Vec<(f32, f32)> = pts.to_vec();
    if closed {
        seq.push(pts[0]);
    }
    let mut cum = vec![0.0f32];
    for i in 1..seq.len() {
        let dx = seq[i].0 - seq[i - 1].0;
        let dz = seq[i].1 - seq[i - 1].1;
        cum.push(cum[i - 1] + (dx * dx + dz * dz).sqrt());
    }
    let total = cum[cum.len() - 1];
    let (count, denom) = if closed {
        let m = ((total / step) as usize).max(8);
        (m, m)
    } else {
        let m = ((total / step) as usize).max(2);
        (m + 1, m)
    };
    let mut out = Vec::with_capacity(count);
    let mut j = 0usize;
    for q in 0..count {
        let target = q as f32 * total / denom as f32;
        while j < cum.len() - 2 && cum[j + 1] < target {
            j += 1;
        }
        let seg = cum[j + 1] - cum[j];
        let f = if seg > 1e-9 { (target - cum[j]) / seg } else { 0.0 };
        out.push((
            seq[j].0 + (seq[j + 1].0 - seq[j].0) * f,
            seq[j].1 + (seq[j + 1].1 - seq[j].1) * f,
        ));
    }
    (out, total)
}

// ----------------------------- трасса и срезки -----------------------------

pub struct Shortcut {
    pub road: Road,
    pub a: usize,
    pub b: usize,
    pub id: usize,
    pub entry_side: i32,
    pub exit_side: i32,
}

pub struct Track {
    pub road: Road,
    pub m: usize,
    pub length: f32,
    pub ds: f32,
    pub spec: TrackSpec,
    pub shortcuts: Vec<Shortcut>,
    /// сторона (-1/+1), где у основной дороги нет стены из-за срезки; 0 - стена есть
    pub gap: Vec<i8>,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Surf {
    Main,
    Short(usize),
}

pub struct SurfInfo {
    pub surf: Surf,
    pub k: usize,
    pub i: usize,
    pub lat: f32,
}

impl Track {
    pub fn new(spec: &TrackSpec) -> Track {
        let ctrl = &spec.ctrl;
        let n = ctrl.len();
        let mut dense: Vec<(f32, f32)> = Vec::new();
        for i in 0..n {
            let p0 = ctrl[(i + n - 1) % n];
            let p1 = ctrl[i];
            let p2 = ctrl[(i + 1) % n];
            let p3 = ctrl[(i + 2) % n];
            for k in 0..60 {
                dense.push(catmull(p0, p1, p2, p3, k as f32 / 60.0));
            }
        }
        let (pts2, total) = resample(&dense, 4.0, true);
        let pts: Vec<V3> = pts2.iter().map(|p| v3(p.0, 0.0, p.1)).collect();
        let road = Road::new(pts, true, ROAD_HALF);
        let m = road.n();
        let mut t = Track {
            road,
            m,
            length: total,
            ds: total / m as f32,
            spec: spec.clone(),
            shortcuts: Vec::new(),
            gap: vec![0i8; m],
        };
        t.find_shortcuts(spec.shortcuts);
        t
    }

    /// Кривая Безье от точки `a` к точке `b` основной дороги.
    fn curve(&self, a: usize, b: usize) -> (Vec<V3>, f32) {
        let p0 = self.road.pts[a];
        let p3 = self.road.pts[b];
        let l = (p3 - p0).len();
        let dirc = (p3 - p0).norm();
        let d0 = (self.road.tan[a] * 0.35 + dirc * 0.65).norm();
        let d1 = (self.road.tan[b] * 0.35 + dirc * 0.65).norm();
        let p1 = p0 + d0 * (l * 0.4);
        let p2 = p3 - d1 * (l * 0.4);
        let mut dense: Vec<(f32, f32)> = Vec::new();
        for k in 0..=40 {
            let t = k as f32 / 40.0;
            let u = 1.0 - t;
            let q = p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t);
            dense.push((q.x, q.z));
        }
        let (pts2, total) = resample(&dense, 4.0, false);
        (pts2.iter().map(|p| v3(p.0, 0.0, p.1)).collect(), total)
    }

    /// Срезка не должна задевать остальные участки трассы.
    fn valid(&self, a: usize, b: usize, pts: &[V3], length: f32, arc: f32) -> bool {
        if pts.len() < 22 || length > arc * 0.78 {
            return false;
        }
        let need = (ROAD_HALF + SC_HALF + 6.0) * (ROAD_HALF + SC_HALF + 6.0);
        let need_sc = (2.0 * SC_HALF + 10.0) * (2.0 * SC_HALF + 10.0);
        let n = pts.len();
        let mut k = 10usize;
        while k + 10 < n {
            let (px, pz) = (pts[k].x, pts[k].z);
            let mut i = 0usize;
            while i < self.m {
                let da = (i as i32 - a as i32).abs();
                let db = (i as i32 - b as i32).abs();
                if da >= 22 && db >= 22 {
                    let (x, z) = self.road.xz[i];
                    if (x - px) * (x - px) + (z - pz) * (z - pz) < need {
                        return false;
                    }
                }
                i += 2;
            }
            for sc in self.shortcuts.iter() {
                let mut q = 0usize;
                while q < sc.road.n() {
                    let (x, z) = sc.road.xz[q];
                    if (x - px) * (x - px) + (z - pz) * (z - pz) < need_sc {
                        return false;
                    }
                    q += 3;
                }
            }
            k += 2;
        }
        true
    }

    fn find_shortcuts(&mut self, count: usize) {
        let m = self.m;
        let ds = self.ds;
        let mut cands: Vec<(f32, usize, usize)> = Vec::new();
        let a_lo = (m as f32 * 0.08) as usize;
        let a_hi = (m as f32 * 0.82) as usize;
        let b_max = (m as f32 * 0.92) as usize;
        let mut a = a_lo;
        while a < a_hi {
            let mut span = 36usize;
            while span < 116 {
                let b = a + span;
                if b > b_max {
                    break;
                }
                let chord = (self.road.pts[b] - self.road.pts[a]).len();
                let arc = span as f32 * ds;
                if chord >= 60.0 && chord <= arc * 0.62 && arc - chord * 1.15 >= 45.0 {
                    let dirc = (self.road.pts[b] - self.road.pts[a]).norm();
                    if self.road.tan[a].dot(dirc) >= -0.35 && self.road.tan[b].dot(dirc) >= -0.35 {
                        cands.push((arc - chord * 1.15, a, b));
                    }
                }
                span += 4;
            }
            a += 2;
        }
        cands.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut tries = 0;
        for &(_, a, b) in cands.iter() {
            if self.shortcuts.len() >= count || tries > 80 {
                break;
            }
            let overlaps = self
                .shortcuts
                .iter()
                .any(|sc| !((b as i32 + 25) < sc.a as i32 || (a as i32 - 25) > sc.b as i32));
            if overlaps {
                continue;
            }
            tries += 1;
            let (pts, length) = self.curve(a, b);
            if !self.valid(a, b, &pts, length, (b - a) as f32 * ds) {
                continue;
            }
            let road = Road::new(pts, false, SC_HALF);
            let n = road.n();
            let v0 = road.pts[8usize.min(n - 1)] - self.road.pts[a];
            let v1 = road.pts[n.saturating_sub(9)] - self.road.pts[b];
            let entry_side = if v0.dot(self.road.right[a]) > 0.0 { 1 } else { -1 };
            let exit_side = if v1.dot(self.road.right[b]) > 0.0 { 1 } else { -1 };
            for i in a..(a + 14) {
                self.gap[i] = entry_side as i8;
            }
            for i in (b - 13)..=b {
                self.gap[i] = exit_side as i8;
            }
            let id = self.shortcuts.len();
            self.shortcuts.push(Shortcut { road, a, b, id, entry_side, exit_side });
        }
    }

    /// Какая дорога под машиной: основная или срезка.
    pub fn surface(&self, pos: V3, hint: usize, win: i32) -> SurfInfo {
        let (i, mut d2) = self.road.nearest(pos, Some(hint), win);
        let mut surf = Surf::Main;
        let mut k = i;
        for (si, sc) in self.shortcuts.iter().enumerate() {
            if sc.a as i32 - 30 <= i as i32 && i as i32 <= sc.b as i32 + 30 {
                let (ks, d2s) = sc.road.nearest(pos, None, 0);
                if d2s < d2 {
                    surf = Surf::Short(si);
                    k = ks;
                    d2 = d2s;
                }
            }
        }
        let road = self.road_of(surf);
        let lat = (pos - road.pts[k]).dot(road.right[k]);
        SurfInfo { surf, k, i, lat }
    }

    pub fn road_of(&self, s: Surf) -> &Road {
        match s {
            Surf::Main => &self.road,
            Surf::Short(i) => &self.shortcuts[i].road,
        }
    }
}

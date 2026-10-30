//! Освещение. Те же формулы, что и в прототипе (gfx_proto/light.py).
//!
//! * `bake` - диффузный свет для статичных поверхностей (запекается при загрузке);
//! * `shade_car` - динамическое освещение машин каждый кадр: диффуз, блики,
//!   отражение неба с эффектом Френеля (лак и стёкла выглядят глянцевыми).
use crate::math::*;

#[derive(Clone, Copy, PartialEq)]
pub enum Mat {
    Paint,
    Glass,
    Plastic,
    Tire,
    Rim,
    /// светится сам (фары, стоп-сигналы): освещением не меняется
    Emissive,
}

pub const MOON_COL: Rgb = [0.38, 0.45, 0.70];
const SKY_AMB: Rgb = [0.10, 0.13, 0.24];
const GND_AMB: Rgb = [0.05, 0.04, 0.05];
const LAMP_COL: Rgb = [1.0, 0.78, 0.52];
pub const LAMP_I: f32 = 3.0;
pub const LAMP_R: f32 = 36.0;

fn moon() -> V3 {
    v3(-0.35, 0.75, -0.55).norm()
}

fn mix(a: Rgb, b: Rgb, t: f32) -> Rgb {
    [lerpf(a[0], b[0], t), lerpf(a[1], b[1], t), lerpf(a[2], b[2], t)]
}

/// (сила блика, резкость, сила отражения среды, отражение "в лоб")
fn params(m: Mat) -> (f32, f32, f32, f32) {
    match m {
        Mat::Paint => (0.9, 70.0, 0.40, 0.05),
        Mat::Glass => (1.0, 160.0, 0.95, 0.12),
        Mat::Plastic => (0.15, 12.0, 0.06, 0.04),
        Mat::Tire => (0.05, 8.0, 0.0, 0.0),
        Mat::Rim => (1.0, 90.0, 0.55, 0.45),
        Mat::Emissive => (0.0, 1.0, 0.0, 0.0),
    }
}

/// Мягкий свет неба/земли: поверхности, смотрящие вверх, светлее.
pub fn hemi(n: V3) -> Rgb {
    mix(GND_AMB, SKY_AMB, n.y * 0.5 + 0.5)
}

/// Свет фонарей (с "обёрнутым" ламбертом, чтобы не было совсем чёрных граней).
pub fn lamp_light(p: V3, n: V3, lamps: &[V3]) -> Rgb {
    let mut r = [0.0f32; 3];
    for lp in lamps.iter() {
        let d = *lp - p;
        let dist = d.len();
        if dist >= LAMP_R || dist < 1e-6 {
            continue;
        }
        let a = 1.0 - dist / LAMP_R;
        let att = a * a;
        let l = d * (1.0 / dist);
        let w = n.dot(l) * 0.5 + 0.5;
        let k = LAMP_I * att * w * w;
        r[0] += LAMP_COL[0] * k;
        r[1] += LAMP_COL[1] * k;
        r[2] += LAMP_COL[2] * k;
    }
    r
}

/// Освещение статичных поверхностей: небо + луна + фонари.
pub fn bake(p: V3, n: V3, albedo: Rgb, lamps: &[V3]) -> Rgb {
    let h = hemi(n);
    let m = n.dot(moon()).max(0.0);
    let ll = lamp_light(p, n, lamps);
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        out[i] = (albedo[i] * (h[i] + MOON_COL[i] * m + ll[i])).min(1.0);
    }
    out
}

/// Дешёвое освещение для зеркальных отражений и далёких машин: только небо и луна.
pub fn shade_cheap(n: V3, albedo: Rgb) -> Rgb {
    let h = hemi(n);
    let m = n.dot(moon()).max(0.0);
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        out[i] = (albedo[i] * (h[i] + MOON_COL[i] * m)).min(1.0);
    }
    out
}

/// Что видно в отражении: ночное небо с городским заревом и луной.
fn env(r: V3) -> Rgb {
    let c = if r.y > 0.0 {
        mix([0.22, 0.15, 0.16], [0.015, 0.025, 0.07], r.y.sqrt())
    } else {
        [0.05, 0.045, 0.06]
    };
    let m = r.dot(moon()).max(0.0).powf(220.0) * 3.0;
    [c[0] + MOON_COL[0] * m, c[1] + MOON_COL[1] * m, c[2] + MOON_COL[2] * m]
}

/// Динамическое освещение вершины машины.
/// `detail = false` - без бликов и отражений (для далёких машин).
pub fn shade_car(p: V3, n: V3, albedo: Rgb, mat: Mat, cam: V3, lamps: &[V3], detail: bool) -> Rgb {
    if mat == Mat::Emissive {
        return [albedo[0].min(1.0), albedo[1].min(1.0), albedo[2].min(1.0)];
    }
    let h = hemi(n);
    let m = n.dot(moon()).max(0.0);
    let ll = lamp_light(p, n, lamps);
    let mut diff = [0.0f32; 3];
    for i in 0..3 {
        diff[i] = albedo[i] * (h[i] + MOON_COL[i] * m + ll[i]);
    }
    if !detail {
        return [diff[0].min(1.0), diff[1].min(1.0), diff[2].min(1.0)];
    }
    let (spec_k, pw, refl, f0) = params(mat);
    let v = (cam - p).norm();
    let nv = n.dot(v).max(0.0);

    // блики от луны и фонарей
    let mut spec = [0.0f32; 3];
    if spec_k > 0.0 {
        let mut add_spec = |l: V3, col: Rgb, k: f32| {
            let hv = (l + v).norm();
            let s = n.dot(hv).max(0.0).powf(pw) * spec_k * k;
            spec[0] += col[0] * s;
            spec[1] += col[1] * s;
            spec[2] += col[2] * s;
        };
        add_spec(moon(), MOON_COL, 2.0);
        for lp in lamps.iter() {
            let d = *lp - p;
            let dist = d.len();
            if dist < LAMP_R && dist > 1e-6 {
                let a = 1.0 - dist / LAMP_R;
                add_spec(d * (1.0 / dist), LAMP_COL, LAMP_I * a * a * 1.5);
            }
        }
    }

    // отражение среды (Френель)
    let mut f = 0.0f32;
    let mut e = [0.0f32; 3];
    if refl > 0.0 {
        let r = n * (2.0 * n.dot(v)) - v;
        f = (f0 + (1.0 - f0) * (1.0 - nv).powi(5)) * refl;
        e = env(r);
        if mat == Mat::Rim {
            e = [e[0] * albedo[0], e[1] * albedo[1], e[2] * albedo[2]];
        }
    }
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        out[i] = (diff[i] * (1.0 - f) + e[i] * f + spec[i]).min(1.0);
    }
    out
}

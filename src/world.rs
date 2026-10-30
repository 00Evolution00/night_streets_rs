//! Геометрия мира с запечённым светом: дорога (мокрая, чуть прозрачная),
//! отбойники, срезки, арка, фонари, город.
use crate::carmodels::{Builder, Poly};
use crate::light::*;
use crate::math::*;
use crate::mesh::{Chunk, ChunkBuilder, Vert};
use crate::track::*;

/// true - неоновые отбойники, false - бетонные с оранжево-белыми полосами
pub const NEON_WALLS: bool = false;
/// Прозрачность асфальта: сквозь него видно отражение машин (эффект мокрой дороги).
pub const ROAD_ALPHA: f32 = 0.74;

pub struct Lamp {
    pub pos: V3,
    /// индекс точки трассы, у которой стоит фонарь
    pub idx: usize,
}

pub struct WorldMesh {
    /// асфальт (прозрачный, рисуется после отражений)
    pub road: Vec<Chunk>,
    /// отбойники, арка, фонари, знаки
    pub plain: Vec<Chunk>,
    /// здания (с текстурой окон)
    pub city: Vec<Chunk>,
    pub lamps: Vec<Lamp>,
}

fn c8(r: u8, g: u8, b: u8) -> Rgb {
    [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0]
}

fn vert(p: V3, c: [f32; 4], n: V3) -> Vert {
    Vert { p: [p.x, p.y, p.z], uv: [0.0, 0.0], c, n: [n.x, n.y, n.z], m: Mat::Plastic }
}

/// Непрозрачная вершина с запечённым светом.
fn bv(p: V3, n: V3, albedo: Rgb, lamps: &[V3]) -> Vert {
    let c = bake(p, n, albedo, lamps);
    vert(p, [c[0], c[1], c[2], 1.0], n)
}

/// Вершина прозрачной дороги: цвет подобран так, чтобы вместе с фоном дать нужный.
fn rv(p: V3, albedo: Rgb, lamps: &[V3], bg: Rgb) -> Vert {
    let up = v3(0.0, 1.0, 0.0);
    let b = bake(p, up, albedo, lamps);
    let mut c = [0.0f32; 4];
    for i in 0..3 {
        c[i] = ((b[i] - bg[i] * (1.0 - ROAD_ALPHA)) / ROAD_ALPHA).max(0.0).min(1.0);
    }
    c[3] = ROAD_ALPHA;
    vert(p, c, up)
}

fn quad_opaque(cb: &mut ChunkBuilder, pts: [V3; 4], n: V3, albedo: Rgb, lamps: &[V3]) {
    let v: Vec<Vert> = pts.iter().map(|p| bv(*p, n, albedo, lamps)).collect();
    cb.push(&v);
}

fn quad_road(cb: &mut ChunkBuilder, pts: [V3; 4], albedo: Rgb, lamps: &[V3], bg: Rgb) {
    let v: Vec<Vert> = pts.iter().map(|p| rv(*p, albedo, lamps, bg)).collect();
    cb.push(&v);
}

/// Полигоны из `Builder` (коробки): свет запекается, светящиеся детали остаются как есть.
fn add_baked(cb: &mut ChunkBuilder, polys: &[Poly], lamps: &[V3]) {
    for poly in polys.iter() {
        let v: Vec<Vert> = poly
            .v
            .iter()
            .map(|x| {
                if x.m == Mat::Emissive {
                    vert(x.p, [x.c[0], x.c[1], x.c[2], 1.0], x.n)
                } else {
                    bv(x.p, x.n, x.c, lamps)
                }
            })
            .collect();
        cb.push(&v);
    }
}

/// Здание: боковые грани режутся на плитки 9x9 м, каждая плитка берёт случайный
/// квадрант текстуры окон (так не нужен повтор текстуры).
#[allow(clippy::too_many_arguments)]
fn add_building(cb: &mut ChunkBuilder, rng: &mut Rng, lamps: &[V3], cx: f32, cz: f32, w: f32, d: f32, h: f32, tint: Rgb) {
    let (x0, x1, z0, z1) = (cx - w / 2.0, cx + w / 2.0, cz - d / 2.0, cz + d / 2.0);
    let faces: [(V3, V3, V3); 4] = [
        (v3(x1, 0.0, z0), v3(x1, 0.0, z1), v3(1.0, 0.0, 0.0)),
        (v3(x0, 0.0, z1), v3(x0, 0.0, z0), v3(-1.0, 0.0, 0.0)),
        (v3(x1, 0.0, z1), v3(x0, 0.0, z1), v3(0.0, 0.0, 1.0)),
        (v3(x0, 0.0, z0), v3(x1, 0.0, z0), v3(0.0, 0.0, -1.0)),
    ];
    for (a, b, n) in faces.iter() {
        let len = (*b - *a).len();
        let nu = ((len / 9.0).ceil() as usize).max(1);
        let nv = ((h / 9.0).ceil() as usize).max(1);
        for iu in 0..nu {
            for iv in 0..nv {
                let u0 = iu as f32 / nu as f32;
                let u1 = (iu + 1) as f32 / nu as f32;
                let v0 = iv as f32 / nv as f32 * h;
                let v1 = (iv + 1) as f32 / nv as f32 * h;
                let pa = *a + (*b - *a) * u0;
                let pb = *a + (*b - *a) * u1;
                let q = [
                    pa + v3(0.0, v0, 0.0),
                    pa + v3(0.0, v1, 0.0),
                    pb + v3(0.0, v1, 0.0),
                    pb + v3(0.0, v0, 0.0),
                ];
                let qx = if rng.chance(0.5) { 0.0 } else { 0.5 };
                let qy = if rng.chance(0.5) { 0.0 } else { 0.5 };
                let uv = [[qx, qy + 0.5], [qx, qy], [qx + 0.5, qy], [qx + 0.5, qy + 0.5]];
                let mut verts: Vec<Vert> = Vec::with_capacity(4);
                for k in 0..4 {
                    // окна светятся сами, поэтому фасад лишь слегка подсвечивается фонарями
                    let ll = lamp_light(q[k], *n, lamps);
                    let ls = (ll[0] + ll[1] + ll[2]) / 3.0;
                    let kk = (0.78 + 0.45 * ls).min(1.15);
                    let c = [(tint[0] * kk).min(1.0), (tint[1] * kk).min(1.0), (tint[2] * kk).min(1.0), 1.0];
                    let mut vx = vert(q[k], c, *n);
                    vx.uv = uv[k];
                    verts.push(vx);
                }
                cb.push(&verts);
            }
        }
    }
    // крыша: берём тёмный пиксель текстуры
    let top = [v3(x0, h, z0), v3(x0, h, z1), v3(x1, h, z1), v3(x1, h, z0)];
    let mut verts: Vec<Vert> = Vec::with_capacity(4);
    for p in top.iter() {
        let mut vx = vert(*p, [tint[0] * 0.35, tint[1] * 0.35, tint[2] * 0.35, 1.0], v3(0.0, 1.0, 0.0));
        vx.uv = [0.002, 0.002];
        verts.push(vx);
    }
    cb.push(&verts);
}

pub fn build_world(tr: &Track) -> WorldMesh {
    let spec = &tr.spec;
    let h = ROAD_HALF;
    let m = tr.m;
    let bg = spec.sky;

    // фонари: каждые 14 точек трассы с обеих сторон
    let mut lamps: Vec<Lamp> = Vec::new();
    let mut i = 0usize;
    while i < m {
        for &sd in [-1.0f32, 1.0].iter() {
            let base = tr.road.pts[i] + tr.road.right[i] * (sd * (h + 2.2));
            let head = base - tr.road.right[i] * sd;
            lamps.push(Lamp { pos: v3(head.x, 7.0, head.z), idx: i });
        }
        i += 14;
    }
    let lamp_pos: Vec<V3> = lamps.iter().map(|l| l.pos).collect();
    let lp = &lamp_pos[..];

    let mut road_cb = ChunkBuilder::new();
    let mut cb = ChunkBuilder::new();
    let mut deco = Builder::new();
    let (ar, ag, ab) = (spec.asphalt[0] as f32 / 255.0, spec.asphalt[1] as f32 / 255.0, spec.asphalt[2] as f32 / 255.0);
    let white = [0.75, 0.75, 0.80];
    let yellow = [0.70, 0.65, 0.30];

    // поперечные границы полос дороги (асфальт нарезан мельче, чтобы свет фонарей давал плавные пятна)
    let mut lat: Vec<f32> = vec![-h, -(h - 0.9), -(h - 1.4)];
    for t in 1..=3 {
        lat.push(-(h - 1.4) + ((h - 1.4) - 0.25) * t as f32 / 3.0);
    }
    lat.push(0.25);
    for t in 1..=3 {
        lat.push(0.25 + ((h - 1.4) - 0.25) * t as f32 / 3.0);
    }
    lat.push(h - 0.9);
    lat.push(h);

    let up8 = v3(0.0, 0.8, 0.0);
    let up12 = v3(0.0, 1.2, 0.0);
    for i in 0..m {
        let j = (i + 1) % m;
        let p = tr.road.pts[i];
        let q = tr.road.pts[j];
        let r = tr.road.right[i];
        let s = tr.road.right[j];
        let tint = if (i / 10) % 2 == 1 { 0.012 } else { 0.0 };
        let asph = [ar + tint, ag + tint, ab + tint];
        for k in 0..(lat.len() - 1) {
            let (o0, o1) = (lat[k], lat[k + 1]);
            let mut alb = asph;
            if k == 1 && tr.gap[i] != -1 {
                alb = white;
            }
            if k == 9 && tr.gap[i] != 1 {
                alb = white;
            }
            if k == 5 && i % 8 < 4 {
                alb = yellow;
            }
            quad_road(&mut road_cb, [p + r * o0, p + r * o1, q + s * o1, q + s * o0], alb, lp, bg);
        }
        for &sd in [-1.0f32, 1.0].iter() {
            if tr.gap[i] as f32 == sd {
                continue; // здесь стены нет: вход/выход срезки
            }
            let off = h + 0.6;
            let lo0 = p + r * (sd * off);
            let lo1 = q + s * (sd * off);
            let nin = r * (-sd); // стена смотрит на дорогу
            quad_opaque(&mut cb, [lo0, lo1, lo1 + up8, lo0 + up8], nin, c8(95, 95, 105), lp);
            let stripe = if NEON_WALLS {
                if (i / 6) % 2 == 1 { c8(255, 40, 160) } else { c8(30, 220, 255) }
            } else if (i / 6) % 2 == 1 {
                c8(255, 150, 30)
            } else {
                c8(225, 225, 230)
            };
            quad_opaque(&mut cb, [lo0 + up8, lo1 + up8, lo1 + up12, lo0 + up12], nin, stripe, lp);
        }
    }

    // срезки: грунтовка (тоже чуть прозрачная), бетонные блоки, жёлто-чёрные столбы
    let up7 = v3(0.0, 0.7, 0.0);
    let up10 = v3(0.0, 1.0, 0.0);
    for sc in tr.shortcuts.iter() {
        let hs = SC_HALF;
        let n = sc.road.n();
        for k in 0..(n - 1) {
            let p = sc.road.pts[k];
            let q = sc.road.pts[k + 1];
            let r = sc.road.right[k];
            let s = sc.road.right[k + 1];
            let up = v3(0.0, 0.06, 0.0);
            let tint = if (k / 6) % 2 == 1 { 0.015 } else { 0.0 };
            quad_road(
                &mut road_cb,
                [p - r * hs + up, p + r * hs + up, q + s * hs + up, q - s * hs + up],
                [0.21 + tint, 0.19 + tint, 0.17 + tint],
                lp,
                bg,
            );
            if k >= 9 && k + 11 <= n {
                for &sd in [-1.0f32, 1.0].iter() {
                    let off = hs + 0.5;
                    let lo0 = p + r * (sd * off);
                    let lo1 = q + s * (sd * off);
                    let nin = r * (-sd);
                    quad_opaque(&mut cb, [lo0, lo1, lo1 + up7, lo0 + up7], nin, c8(85, 85, 92), lp);
                    let col = if (k / 4) % 2 == 1 { c8(240, 200, 40) } else { c8(30, 30, 30) };
                    quad_opaque(&mut cb, [lo0 + up7, lo1 + up7, lo1 + up10, lo0 + up10], nin, col, lp);
                }
            }
        }
        let marks: [(usize, i32); 2] = [(sc.a - 4, sc.entry_side), (sc.b - 2, sc.exit_side)];
        for &(idx, side) in marks.iter() {
            let bp = tr.road.pts[idx] + tr.road.right[idx] * (side as f32 * (h + 3.0));
            deco.boxed(v3(bp.x, 2.0, bp.z), v3(1.5, 4.0, 1.5), rgb!(250, 200, 30), Mat::Paint);
            deco.boxed(v3(bp.x, 3.3, bp.z), v3(1.6, 0.5, 1.6), rgb!(25, 25, 25), Mat::Paint);
            deco.boxed(v3(bp.x, 1.0, bp.z), v3(1.6, 0.5, 1.6), rgb!(25, 25, 25), Mat::Paint);
        }
    }

    // стартовая арка
    let p0 = tr.road.pts[0];
    let r0 = tr.road.right[0];
    for &sd in [-1.0f32, 1.0].iter() {
        let c = p0 + r0 * (sd * (h + 1.5));
        deco.boxed(v3(c.x, 4.0, c.z), v3(1.0, 8.0, 1.0), rgb!(55, 55, 70), Mat::Paint);
    }
    let n_chk = 18usize;
    let span = 2.0 * (h + 1.5);
    let tan0 = tr.road.tan[0];
    for k in 0..n_chk {
        let o0 = -(h + 1.5) + k as f32 * span / n_chk as f32;
        let o1 = -(h + 1.5) + (k + 1) as f32 * span / n_chk as f32;
        let pa = p0 + r0 * o0 + v3(0.0, 7.2, 0.0);
        let pb = p0 + r0 * o1 + v3(0.0, 7.2, 0.0);
        let alb = if k % 2 == 1 { [0.9, 0.9, 0.9] } else { [0.06, 0.06, 0.06] };
        quad_opaque(&mut cb, [pa, pb, pb + v3(0.0, 1.6, 0.0), pa + v3(0.0, 1.6, 0.0)], tan0 * -1.0, alb, lp);
    }

    // фонари: столбы и светящиеся головки
    for l in lamps.iter() {
        let base_x = l.pos.x;
        let base_z = l.pos.z;
        // столб стоит чуть дальше от дороги, чем головка
        let r = tr.road.right[l.idx];
        let sd = if (l.pos - tr.road.pts[l.idx]).dot(r) > 0.0 { 1.0 } else { -1.0 };
        let pole = v3(base_x + r.x * sd, 3.5, base_z + r.z * sd);
        deco.boxed(pole, v3(0.25, 7.0, 0.25), rgb!(50, 50, 62), Mat::Paint);
        deco.boxed(l.pos, v3(2.2, 0.25, 0.8), rgb!(255, 215, 150), Mat::Emissive);
    }
    add_baked(&mut cb, &deco.polys, lp);

    // город
    let mut city = ChunkBuilder::new();
    let mut rng = Rng::new(7 + spec.seed);
    let mut obstacles: Vec<(f32, f32)> = tr.road.xz.clone();
    for sc in tr.shortcuts.iter() {
        obstacles.extend(sc.road.xz.iter().cloned());
    }
    let mut placed = 0usize;
    for _ in 0..800 {
        if placed >= spec.buildings {
            break;
        }
        let i = rng.int(0, m as i32) as usize;
        let sd = if rng.chance(0.5) { 1.0f32 } else { -1.0f32 };
        let dist = h + rng.range(22.0, 100.0);
        let pos = tr.road.pts[i] + tr.road.right[i] * (sd * dist);
        let w = rng.range(14.0, 34.0);
        let d = rng.range(14.0, 34.0);
        let bh = rng.range(25.0, 120.0);
        let need = h + 14.0 + w.max(d) * 0.6;
        let mut ok = true;
        for &(x, z) in obstacles.iter() {
            if (x - pos.x) * (x - pos.x) + (z - pos.z) * (z - pos.z) < need * need {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }
        let tint = spec.tints[rng.int(0, spec.tints.len() as i32) as usize];
        add_building(&mut city, &mut rng, lp, pos.x, pos.z, w, d, bh, tint);
        placed += 1;
    }

    WorldMesh { road: road_cb.finish(), plain: cb.finish(), city: city.finish(), lamps }
}

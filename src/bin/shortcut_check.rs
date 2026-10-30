//! Оффлайн-проверка срезок: строит трассы, прогоняет машину через срезку и отдельно
//! сканирует зону стыка (въезд/выезд) по сетке позиций+углов. Печатает худшие
//! одномоментные скачки позиции (телепорты) и удары о стены у стыков.
include!("../../tools/libembed.rs");

fn fwd_of(deg: f32) -> V3 {
    let r = deg.to_radians();
    v3(r.sin(), 0.0, r.cos())
}

fn main() {
    let specs = track_specs();
    for spec in specs.iter() {
        let tr = Track::new(spec);
        println!("== {} == shortcuts: {}", spec.name, tr.shortcuts.len());
        for sc in tr.shortcuts.iter() {
            println!(
                "  SC{}: a={} b={} n={} k_entry={} k_exit={} sides={}/{}",
                sc.id, sc.a, sc.b, sc.road.n(), sc.k_entry, sc.k_exit, sc.entry_side, sc.exit_side
            );
            // 1) динамический прогон с физикой как в car.rs
            let (jump, wall) = drive(&tr, sc);
            println!("  SC{} drive: max_step={:.2} m  max_wall_corr={:.2} m", sc.id, jump, wall);
            // 2) статический скан зоны стыков: все позиции на сетке 1 м и углы каждые 30°
            let (jump2, wall2) = scan_joints(&tr, sc);
            println!("  SC{} joint scan: worst_step={:.2} m  worst_wall={:.2} m", sc.id, jump2, wall2);
        }
    }
}

struct Sim<'a> {
    tr: &'a Track,
    pos: V3,
    heading: f32,
    speed: f32,
    vel: V3,
    idx: usize,
}

impl<'a> Sim<'a> {
    fn step(&mut self, steer_deg: f32, dt: f32) -> (f32, f32) {
        self.heading += steer_deg;
        self.vel = fwd_of(self.heading) * self.speed;
        let old = self.pos;
        self.pos = v3(old.x + self.vel.x * dt, 0.0, old.z + self.vel.z * dt);
        let jump = (self.pos - old).len();
        let info = self.tr.surface(self.pos, self.idx, 40);
        self.idx = info.i;
        let road = self.tr.road_of(info.surf);
        let mut limit = road.half - 1.2;
        if let Surf::Short(si) = info.surf {
            let sc = &self.tr.shortcuts[si];
            let pad = ROAD_HALF - SC_HALF;
            if info.k < sc.k_entry {
                let t = (info.k as f32 / (sc.k_entry.max(1) as f32)).clamp(0.0, 1.0);
                limit += pad * (1.0 - t);
            } else if info.k > sc.k_exit {
                let rem = (sc.road.n() - 1 - sc.k_exit).max(1) as f32;
                let t = ((sc.road.n() - 1 - info.k) as f32 / rem).clamp(0.0, 1.0);
                limit += pad * (1.0 - t);
            }
        }
        let mut corr = 0.0f32;
        if info.lat.abs() > limit {
            let s: f32 = if info.lat > 0.0 { 1.0 } else { -1.0 };
            let skip = match info.surf {
                Surf::Main => self.tr.gap[info.k] as f32 == s,
                Surf::Short(si) => {
                    let sc = &self.tr.shortcuts[si];
                    info.k < sc.k_entry || info.k > sc.k_exit
                }
            };
            if !skip {
                corr = info.lat.abs() - limit;
                self.pos -= road.right[info.k] * (s * corr);
            }
        }
        (jump, corr)
    }
}

fn drive(tr: &Track, sc: &Shortcut) -> (f32, f32) {
    let mut sim = Sim {
        tr,
        pos: tr.road.pts[sc.a - 6] + tr.road.right[sc.a - 6] * (sc.entry_side as f32 * 9.0),
        heading: tr.road.ang[sc.a - 6],
        speed: 28.0,
        vel: V3::default(),
        idx: sc.a - 6,
    };
    let dt = 1.0 / 60.0;
    let (mut mj, mut mw) = (0.0f32, 0.0f32);
    for _ in 0..1500 {
        let target = if sim.idx <= sc.a + 8 {
            sc.road.pts[sc.k_entry.min(sc.road.n() - 1)]
        } else {
            let (k, _) = sc.road.nearest(sim.pos, None, 0);
            sc.road.pts[(k + 8).min(sc.road.n() - 1)]
        };
        let want = (target.x - sim.pos.x).atan2(target.z - sim.pos.z).to_degrees();
        let diff = wrap180(want - sim.heading);
        let (j, w) = sim.step(diff.clamp(-3.0, 3.0), dt);
        mj = mj.max(j);
        mw = mw.max(w);
        if sim.idx > sc.b + 20 {
            break;
        }
    }
    (mj, mw)
}

fn scan_joints(tr: &Track, sc: &Shortcut) -> (f32, f32) {
    let mut worst_jump = 0.0f32;
    let mut worst_wall = 0.0f32;
    let zones = [
        (sc.a.saturating_sub(10), sc.a + 20),
        (sc.b.saturating_sub(20), sc.b + 10),
    ];
    for (lo, hi) in zones {
        let mut i = lo;
        while i <= hi && i < tr.m {
            let p = tr.road.pts[i];
            let r = tr.road.right[i];
            for &lat in [-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0].iter() {
                for adeg in (0..360).step_by(30) {
                    let pos = p + r * lat;
                    let mut sim = Sim {
                        tr,
                        pos,
                        heading: tr.road.ang[i] + (adeg as f32 - 180.0) * 0.25,
                        speed: 25.0,
                        vel: V3::default(),
                        idx: i,
                    };
                    let (j, w) = sim.step(0.0, 1.0 / 60.0);
                    worst_jump = worst_jump.max(j);
                    worst_wall = worst_wall.max(w);
                }
            }
            i += 1;
        }
    }
    (worst_jump, worst_wall)
}

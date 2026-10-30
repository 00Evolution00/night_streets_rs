//! Машины игрока и соперников, физика, ИИ, гражданский трафик, столкновения.
use crate::carmodels::Rgb;
use crate::math::*;
use crate::track::*;

pub const PLAYER_MAX_SPEED: f32 = 62.0;
pub const KIND_COUNT: usize = 8;

pub struct KindStats {
    pub name: &'static str,
    pub price: i32,
    pub accel: f32,
    pub top: f32,
    pub grip: f32,
}

/// Порядок совпадает с `carmodels::all_car_specs()`.
pub static STATS: [KindStats; KIND_COUNT] = [
    KindStats { name: "Ford Focus II", price: 0, accel: 1.00, top: 0.92, grip: 0.4 },
    KindStats { name: "Chevrolet Cobalt SS", price: 6000, accel: 1.06, top: 0.96, grip: 0.0 },
    KindStats { name: "Mitsubishi 3000GT", price: 12000, accel: 1.15, top: 1.06, grip: 0.2 },
    KindStats { name: "Mitsubishi Evo IX", price: 20000, accel: 1.22, top: 1.04, grip: 1.0 },
    KindStats { name: "Nissan 350Z", price: 26000, accel: 1.20, top: 1.08, grip: 0.3 },
    KindStats { name: "Mazda RX-7 FD", price: 32000, accel: 1.22, top: 1.07, grip: 0.5 },
    KindStats { name: "Toyota Supra A80", price: 40000, accel: 1.32, top: 1.14, grip: 0.2 },
    KindStats { name: "Nissan Skyline R34", price: 50000, accel: 1.36, top: 1.15, grip: 0.8 },
];

pub const BODY_COLORS: [(&str, Rgb); 12] = [
    ("Racing Blue", rgb!(30, 120, 255)),
    ("Hot Pink", rgb!(255, 50, 160)),
    ("Lava Red", rgb!(230, 40, 50)),
    ("Sun Yellow", rgb!(250, 200, 30)),
    ("Toxic Green", rgb!(60, 220, 110)),
    ("Orange", rgb!(255, 120, 20)),
    ("Violet", rgb!(150, 70, 255)),
    ("Cyan", rgb!(30, 220, 255)),
    ("Pearl White", rgb!(235, 235, 245)),
    ("Matte Black", rgb!(25, 25, 32)),
    ("Gunmetal", rgb!(90, 95, 110)),
    ("Mint", rgb!(120, 255, 200)),
];
pub const RIM_COLORS: [(&str, Rgb); 8] = [
    ("Silver", rgb!(200, 205, 215)),
    ("Gold", rgb!(255, 200, 60)),
    ("Black", rgb!(20, 20, 26)),
    ("Neon Green", rgb!(60, 255, 120)),
    ("Red", rgb!(240, 40, 50)),
    ("White", rgb!(250, 250, 255)),
    ("Pink", rgb!(255, 70, 170)),
    ("Chrome Blue", rgb!(80, 160, 255)),
];
pub const GLOW_COLORS: [(&str, Option<Rgb>); 9] = [
    ("Pink", Some(rgb!(255, 40, 160))),
    ("Cyan", Some(rgb!(30, 220, 255))),
    ("Green", Some(rgb!(60, 255, 120))),
    ("Purple", Some(rgb!(160, 70, 255))),
    ("Orange", Some(rgb!(255, 130, 30))),
    ("White", Some(rgb!(240, 240, 255))),
    ("Red", Some(rgb!(255, 40, 50))),
    ("Blue", Some(rgb!(50, 90, 255))),
    ("Off", None),
];
pub const RIM_STYLES: [&str; 3] = ["Disc", "Spoke", "Turbine"];
pub const SPOILERS: [&str; 3] = ["None", "Lip", "GT Wing"];

#[derive(Clone, Copy, Default)]
pub struct Tune {
    pub engine: u8,
    pub turbo: u8,
    pub tires: u8,
    pub nitro: u8,
}

#[derive(Clone, Copy)]
pub struct Look {
    pub body: Rgb,
    pub glow: Option<Rgb>,
    pub rim: Rgb,
    pub rimstyle: u8,
    pub spoiler: u8,
}

#[derive(Clone, Copy, Default)]
pub struct DriveInput {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
    pub handbrake: bool,
    pub nitro: bool,
}

pub struct Particle {
    pub pos: V3,
    pub life: f32,
}

pub struct Car {
    pub kind: usize,
    pub slot: usize,
    pub label: String,
    pub is_player: bool,
    pub skill0: f32,
    pub skill: f32,
    pub lane: f32,
    pub shortcut_pref: f32,
    pub look: Look,
    pub tune: Tune,

    pub pos: V3,
    pub heading: f32,
    pub speed: f32,
    pub vel: V3,
    pub steer: f32,
    pub idx: usize,
    pub lap: i32,
    pub progress: i32,
    pub nitro: f32,
    pub boosting: bool,
    pub drifting: bool,
    pub on_short: bool,
    pub done: bool,
    pub finish_time: f32,
    pub autopilot: bool,
    pub lap_times: Vec<f32>,
    pub route: Option<usize>,
    pub route_k: usize,
    pub sc_lap: Vec<i32>,
    pub stuck: f32,
    pub smoke_t: f32,
    pub wheel_spin: f32,
    pub roll: f32,

    pub accel: f32,
    pub base_max: f32,
    pub max_speed: f32,
    pub grip_base: f32,
    pub steer_mul: f32,
    pub nitro_use: f32,
    pub boost_mul: f32,
    pub boost_accel: f32,
}

impl Car {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tr: &Track,
        slot: usize,
        label: &str,
        kind: usize,
        look: Look,
        is_player: bool,
        skill: f32,
        lane: f32,
        pref: f32,
    ) -> Car {
        let mut c = Car {
            kind,
            slot,
            label: label.to_string(),
            is_player,
            skill0: skill,
            skill,
            lane,
            shortcut_pref: pref,
            look,
            tune: Tune::default(),
            pos: V3::default(),
            heading: 0.0,
            speed: 0.0,
            vel: V3::default(),
            steer: 0.0,
            idx: 0,
            lap: 0,
            progress: 0,
            nitro: 60.0,
            boosting: false,
            drifting: false,
            on_short: false,
            done: false,
            finish_time: 0.0,
            autopilot: !is_player,
            lap_times: Vec::new(),
            route: None,
            route_k: 0,
            sc_lap: Vec::new(),
            stuck: 0.0,
            smoke_t: 0.0,
            wheel_spin: 0.0,
            roll: 0.0,
            accel: 34.0,
            base_max: PLAYER_MAX_SPEED,
            max_speed: PLAYER_MAX_SPEED,
            grip_base: 8.5,
            steer_mul: 1.0,
            nitro_use: 30.0,
            boost_mul: 1.3,
            boost_accel: 1.9,
        };
        c.apply_tune(Tune::default());
        c.reset_to_grid(tr);
        c
    }

    pub fn apply_tune(&mut self, t: Tune) {
        let k = &STATS[self.kind];
        self.tune = t;
        self.accel = 34.0 * k.accel * (1.0 + 0.12 * (t.engine as f32));
        self.base_max = PLAYER_MAX_SPEED * self.skill * k.top * (1.0 + 0.04 * (t.turbo as f32));
        self.max_speed = self.base_max;
        self.grip_base = 8.5 + k.grip + 0.7 * (t.tires as f32);
        self.steer_mul = 1.0 + 0.04 * (t.tires as f32);
        self.nitro_use = 30.0 * (1.0 - 0.12 * (t.nitro as f32));
        self.boost_mul = 1.3 + 0.04 * (t.nitro as f32);
        self.boost_accel = 1.9 + 0.2 * (t.nitro as f32);
    }

    pub fn reset_to_grid(&mut self, tr: &Track) {
        let row = self.slot / 2;
        let col = self.slot % 2;
        let idx = (tr.m + tr.m - 4 - row * 3) % tr.m;
        let lat = if col == 0 { -4.5 } else { 4.5 };
        self.pos = tr.road.pts[idx] + tr.road.right[idx] * lat;
        self.heading = tr.road.ang[idx];
        self.speed = 0.0;
        self.vel = V3::default();
        self.steer = 0.0;
        self.idx = idx;
        self.lap = 0;
        self.progress = idx as i32;
        self.nitro = 60.0;
        self.boosting = false;
        self.drifting = false;
        self.on_short = false;
        self.done = false;
        self.finish_time = 0.0;
        self.autopilot = !self.is_player;
        self.lap_times.clear();
        self.route = None;
        self.route_k = 0;
        self.sc_lap = vec![-999; tr.shortcuts.len()];
        self.stuck = 0.0;
        self.wheel_spin = 0.0;
        self.roll = 0.0;
    }

    /// Вернуть на ближайшую точку трассы (клавиша R).
    pub fn recover(&mut self, tr: &Track) {
        let (i, _) = tr.road.nearest(self.pos, None, 0);
        self.idx = i;
        self.pos = tr.road.pts[i];
        self.heading = tr.road.ang[i];
        self.speed = 0.0;
        self.vel = V3::default();
    }

    pub fn fwd(&self) -> V3 {
        let r = self.heading.to_radians();
        v3(r.sin(), 0.0, r.cos())
    }

    /// Управление ИИ: ведёт по трассе, иногда выбирает срезку, объезжает трафик.
    pub fn ai_inputs(&mut self, tr: &Track, traffic: &[Traffic], rng: &mut Rng) -> DriveInput {
        let f = self.fwd();
        let r = v3(f.z, 0.0, -f.x);

        // решаем, ехать ли по срезке (один раз за круг для каждой срезки)
        if self.route.is_none() && self.shortcut_pref > 0.0 {
            for sc in tr.shortcuts.iter() {
                let a = sc.a as i32;
                let idx = self.idx as i32;
                if a - 14 <= idx && idx <= a - 3 && self.sc_lap[sc.id] != self.lap {
                    self.sc_lap[sc.id] = self.lap;
                    if rng.chance(self.shortcut_pref) {
                        self.route = Some(sc.id);
                        self.route_k = 0;
                    }
                    break;
                }
            }
        }
        if let Some(rid) = self.route {
            let sc = &tr.shortcuts[rid];
            let (k, _) = sc.road.nearest(self.pos, Some(self.route_k), 14);
            self.route_k = k;
            if k + 4 >= sc.road.n() {
                self.route = None;
            }
        }

        let target: V3;
        let turn: f32;
        let cap: f32;
        if let Some(rid) = self.route {
            let sc = &tr.shortcuts[rid];
            let k = self.route_k;
            let n = sc.road.n();
            let ti = (k + (5.0 + self.speed / 8.0) as usize).min(n - 1);
            target = sc.road.pts[ti];
            turn = wrap180(sc.road.ang[(k + 14).min(n - 1)] - sc.road.ang[k]).abs();
            cap = 0.75;
        } else {
            let look = (7.0 + self.speed / 5.0) as usize;
            let ti = (self.idx + look) % tr.m;
            target = tr.road.pts[ti] + tr.road.right[ti] * self.lane;
            let a2 = tr.road.ang[(self.idx + 18 + (self.speed / 4.0) as usize) % tr.m];
            turn = wrap180(a2 - tr.road.ang[self.idx]).abs();
            cap = 1.0;
        }
        let d = target - self.pos;
        let mut steer = clampf(wrap180(d.x.atan2(d.z).to_degrees() - self.heading) / 20.0, -1.0, 1.0);
        let target_speed = self.max_speed * cap * (1.0 - clampf(turn / 110.0, 0.0, 0.5));

        // объезд трафика
        let mut slow = false;
        for tc in traffic.iter() {
            let dd = tc.pos - self.pos;
            let lz = dd.dot(f) - tc.length / 2.0;
            let lx = dd.dot(r);
            if lz > 0.0 && lz < 38.0 && lx.abs() < tc.width / 2.0 + 2.2 {
                let w = 1.0 - lz / 38.0;
                let sgn = if lx.abs() > 0.3 { lx } else { 1.0 };
                steer -= (1.4 * w).copysign(sgn);
                if lz < 16.0 && lx.abs() < tc.width / 2.0 + 1.2 {
                    slow = true;
                }
            }
        }
        steer = clampf(steer, -1.0, 1.0);
        let throttle = if self.speed < target_speed && !slow { 1.0 } else { 0.0 };
        let brake = if self.speed > target_speed * 1.12 || (slow && self.speed > 14.0) { 0.6 } else { 0.0 };
        let nitro = turn < 8.0 && self.nitro > 50.0 && !slow && self.route.is_none();
        DriveInput { throttle, brake, steer, handbrake: false, nitro }
    }

    /// Один шаг физики. Возвращает силу удара о стену (для тряски камеры).
    pub fn step(&mut self, dt: f32, inp: &DriveInput, tr: &Track, particles: &mut Vec<Particle>) -> f32 {
        let boosting = inp.nitro && self.nitro > 0.0 && inp.throttle > 0.0;
        let mut max_speed = self.max_speed;
        let mut accel = self.accel;
        if boosting {
            self.nitro = (self.nitro - self.nitro_use * dt).max(0.0);
            max_speed *= self.boost_mul;
            accel *= self.boost_accel;
        } else {
            self.nitro = (self.nitro + 3.0 * dt).min(100.0);
        }
        self.boosting = boosting;

        if inp.throttle > 0.0 {
            if self.speed < 0.0 {
                self.speed += 60.0 * dt;
            } else {
                self.speed += inp.throttle * accel * (1.0 - self.speed / (max_speed * 1.04)).max(0.0) * dt;
            }
        } else if inp.brake == 0.0 {
            let d = self.speed.abs().min(5.0 * dt);
            self.speed -= d.copysign(self.speed);
        }
        if inp.brake > 0.0 {
            if self.speed > 0.5 {
                self.speed -= 60.0 * inp.brake * dt;
            } else {
                self.speed = (self.speed - 18.0 * dt).max(-14.0);
            }
        }
        if self.speed > max_speed {
            self.speed -= 22.0 * dt;
        }
        if inp.handbrake {
            let d = self.speed.abs().min(10.0 * dt);
            self.speed -= d.copysign(self.speed);
        }

        self.steer += (inp.steer - self.steer) * (9.0 * dt).min(1.0);
        let sf = clampf(self.speed.abs() / 9.0, 0.0, 1.0) / (1.0 + self.speed.abs() / 55.0);
        let rate = 115.0 * self.steer_mul * if inp.handbrake { 1.35 } else { 1.0 };
        let dir = if self.speed >= 0.0 { 1.0 } else { -1.0 };
        self.heading += self.steer * rate * sf * dt * dir;

        let f = self.fwd();
        let r = v3(f.z, 0.0, -f.x);
        let mut grip = (self.grip_base - self.steer.abs() * self.speed.abs() / 20.0).max(3.0);
        if inp.handbrake {
            grip = 1.7;
        }
        let desired = f * self.speed;
        self.vel += (desired - self.vel) * (grip * dt).min(1.0);
        self.drifting = self.vel.dot(r).abs() > 7.0 && self.vel.len() > 15.0;
        if self.drifting {
            self.speed -= 6.0 * dt;
            self.nitro = (self.nitro + 9.0 * dt).min(100.0);
            self.smoke_t -= dt;
            if self.smoke_t <= 0.0 && self.is_player {
                self.smoke_t = 0.06;
                for &sx in [-0.9f32, 0.9].iter() {
                    let p = self.pos - f * 1.5 + r * sx;
                    particles.push(Particle { pos: v3(p.x, 0.4, p.z), life: 0.7 });
                }
            }
        }

        self.pos += self.vel * dt;
        self.pos.y = 0.0;

        // поверхность: основная дорога или срезка
        let info = tr.surface(self.pos, self.idx, 40);
        let on_main = info.surf == Surf::Main;
        let road = tr.road_of(info.surf);
        let mut lat_limit = road.half - 1.2;
        if let Surf::Short(si) = info.surf {
            let sc = &tr.shortcuts[si];
            self.on_short = info.k >= sc.k_entry && info.k + (sc.road.n() - sc.k_exit) <= sc.road.n();
            // У стыков с основной дорогой ширина "коридора" расширяется до полосы
            // основной дороги: иначе на въезде/выезде машина ловила невидимый удар
            // об край узкой срезки, а при выезде её выплёвывало телепортом на трассу.
            let pad = ROAD_HALF - SC_HALF;
            if info.k < sc.k_entry {
                let t = (info.k as f32 / (sc.k_entry.max(1) as f32)).clamp(0.0, 1.0);
                lat_limit += pad * (1.0 - t);
            } else if info.k > sc.k_exit {
                let rem = (sc.road.n() - 1 - sc.k_exit).max(1) as f32;
                let t = ((sc.road.n() - 1 - info.k) as f32 / rem).clamp(0.0, 1.0);
                lat_limit += pad * (1.0 - t);
            }
        } else {
            self.on_short = false;
        }
        let limit = lat_limit;
        let mut impact = 0.0f32;
        if info.lat.abs() > limit {
            let s: f32 = if info.lat > 0.0 { 1.0 } else { -1.0 };
            let skip = match info.surf {
                Surf::Main => tr.gap[info.k] as f32 == s,
                Surf::Short(si) => {
                    // у стыков стена срезки отсутствует (переход в полосу основной дороги)
                    let sc = &tr.shortcuts[si];
                    info.k < sc.k_entry || info.k > sc.k_exit
                }
            };
            if !skip {
                let rt = road.right[info.k];
                let corr = s * (info.lat.abs() - limit);
                if corr > 3.5 {
                    // Позиция сильно рассинхронизирована с поверхностью (например, после
                    // удара/скольжения у стыка). Мягко возвращаем на дорогу вместо рывка,
                    // который выглядел как телепорт.
                    self.pos -= rt * (s * corr * (6.0 * dt).min(0.5));
                } else {
                    self.pos -= rt * (s * corr);
                }
                let out = self.vel.dot(rt) * s;
                if out > 0.0 {
                    self.vel -= rt * (s * out * 1.5);
                    impact = out;
                    self.speed *= (1.0 - out / 60.0).max(0.55);
                    let diff = wrap180(road.ang[info.k] - self.heading);
                    if diff.abs() < 90.0 {
                        self.heading += diff * (3.0 * dt).min(1.0);
                    }
                }
            }
        }

        // круги считаем по основной дороге
        let prev = self.idx as i32;
        let i = info.i as i32;
        let m = tr.m as i32;
        self.idx = info.i;
        if prev > m * 3 / 4 && i < m / 4 {
            self.lap += 1;
        } else if prev < m / 4 && i > m * 3 / 4 {
            self.lap -= 1;
        }
        self.progress = self.lap * m + i;

        self.roll = -self.steer * clampf(self.speed.abs() / 40.0, 0.0, 1.0) * 3.0;
        self.wheel_spin += self.speed * dt / 0.33;
        impact
    }
}

// ----------------------------- трафик -----------------------------

/// Гражданская машина: едет по своей полосе с постоянной скоростью.
pub struct Traffic {
    pub kind: usize,
    pub col: Rgb,
    pub length: f32,
    pub width: f32,
    pub t0: f32,
    pub t: f32,
    pub lane: f32,
    pub lane_speed: f32,
    pub v: f32,
    pub kick: f32,
    pub kick_v: f32,
    pub pos: V3,
    pub heading: f32,
}

impl Traffic {
    #[allow(clippy::too_many_arguments)]
    pub fn new(tr: &Track, kind: usize, col: Rgb, length: f32, width: f32, t0: f32, lane: f32, speed: f32) -> Traffic {
        let mut t = Traffic {
            kind,
            col,
            length,
            width,
            t0,
            t: t0,
            lane,
            lane_speed: speed,
            v: speed,
            kick: 0.0,
            kick_v: 0.0,
            pos: V3::default(),
            heading: 0.0,
        };
        t.place(tr);
        t
    }

    pub fn reset(&mut self, tr: &Track) {
        self.t = self.t0;
        self.v = self.lane_speed;
        self.kick = 0.0;
        self.kick_v = 0.0;
        self.place(tr);
    }

    pub fn place(&mut self, tr: &Track) {
        let m = tr.m;
        let i = (self.t as usize) % m;
        let f = self.t - i as f32;
        let j = (i + 1) % m;
        let p = tr.road.pts[i] * (1.0 - f) + tr.road.pts[j] * f;
        let r = tr.road.right[i] * (1.0 - f) + tr.road.right[j] * f;
        self.pos = p + r * (self.lane + self.kick);
        self.heading = tr.road.ang[i] + wrap180(tr.road.ang[j] - tr.road.ang[i]) * f;
    }

    pub fn drive(&mut self, dt: f32, tr: &Track) {
        self.v += clampf(self.lane_speed - self.v, -8.0 * dt, 3.0 * dt);
        self.t = (self.t + self.v * dt / tr.ds).rem_euclid(tr.m as f32);
        self.kick += self.kick_v * dt;
        self.kick_v *= (1.0 - 2.5 * dt).max(0.0);
        self.kick -= self.kick * (0.7 * dt).min(1.0);
        self.place(tr);
    }
}

// ----------------------------- столкновения -----------------------------

pub fn resolve_car_collisions(cars: &mut [Car]) {
    let n = cars.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let (left, right) = cars.split_at_mut(j);
            let a = &mut left[i];
            let b = &mut right[0];
            let mut d = a.pos - b.pos;
            d.y = 0.0;
            let dist = d.len();
            if dist > 0.01 && dist < 3.0 {
                let nrm = d * (1.0 / dist);
                let push = (3.0 - dist) / 2.0;
                a.pos += nrm * push;
                b.pos -= nrm * push;
                let rel = (a.vel - b.vel).dot(nrm);
                if rel < 0.0 {
                    a.vel -= nrm * (rel * 0.6);
                    b.vel += nrm * (rel * 0.6);
                    a.speed = a.vel.dot(a.fwd());
                    b.speed = b.vel.dot(b.fwd());
                }
            }
        }
    }
}

/// Столкновения с трафиком. Возвращает силу удара по игроку.
pub fn resolve_traffic(cars: &mut [Car], traffic: &mut [Traffic]) -> f32 {
    let mut hit = 0.0f32;
    for tc in traffic.iter_mut() {
        let h = tc.heading.to_radians();
        let tf = v3(h.sin(), 0.0, h.cos());
        let trr = v3(tf.z, 0.0, -tf.x);
        let hx = tc.width / 2.0 + 1.0;
        let hz = tc.length / 2.0 + 1.7;
        let reach = (hz + 1.5) * (hz + 1.5);
        for c in cars.iter_mut() {
            let d = c.pos - tc.pos;
            if d.x * d.x + d.z * d.z > reach {
                continue;
            }
            let lz = d.dot(tf);
            let lx = d.dot(trr);
            if lx.abs() < hx && lz.abs() < hz {
                let px = hx - lx.abs();
                let pz = hz - lz.abs();
                let (n, pen) = if px < pz {
                    (trr * if lx >= 0.0 { 1.0 } else { -1.0 }, px)
                } else {
                    (tf * if lz >= 0.0 { 1.0 } else { -1.0 }, pz)
                };
                c.pos += n * pen;
                let rel = c.vel.dot(n);
                if rel < 0.0 {
                    c.vel -= n * (rel * 1.5);
                    c.speed = c.vel.dot(c.fwd()) * 0.85;
                    tc.kick_v = clampf(tc.kick_v + n.dot(trr) * rel * 0.5, -8.0, 8.0);
                    tc.v = (tc.v * 0.9).max(6.0);
                    if c.is_player {
                        hit = hit.max(-rel);
                    }
                }
            }
        }
    }
    hit
}

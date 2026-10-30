//! Состояние игры, гонка, камера, отрисовка, HUD и гараж.
use macroquad::prelude::*;

use crate::car::*;
use crate::carmodels::{self, build_body, build_traffic, wheel_polys, wheel_positions, CarSpec, Rgb};
use crate::light::*;
use crate::math::*;
use crate::mesh::{Chunk, ChunkBuilder};
use crate::save::Save;
use crate::track::*;
use crate::world::{build_world, Lamp};
use std::f32::consts::PI;

const PAYOUT: [i32; 4] = [3000, 2000, 1200, 600];
const UPGRADE_COST: [i32; 3] = [1500, 3000, 5000];
const TRAFFIC_COLORS: [Rgb; 8] = [
    rgb!(225, 225, 230),
    rgb!(150, 155, 165),
    rgb!(70, 75, 85),
    rgb!(30, 40, 70),
    rgb!(160, 150, 130),
    rgb!(110, 25, 30),
    rgb!(40, 80, 60),
    rgb!(200, 200, 190),
];

#[derive(Clone, Copy, PartialEq)]
enum State {
    Garage,
    Countdown,
    Racing,
    Finished,
}

struct StaticMesh {
    mesh: Mesh,
    center: V3,
    radius: f32,
}

struct CarModel {
    body: Vec<Chunk>,
    wheel_l: Vec<Chunk>,
    wheel_r: Vec<Chunk>,
    wheels: Vec<V3>,
}

// ----------------------------- вспомогательное -----------------------------

/// Игровой мир "левый" (x вправо, z вперёд), а macroquad - "правый",
/// поэтому при выводе зеркалим ось x: картинка получается правильной.
fn mqv(p: V3) -> Vec3 {
    vec3(-p.x, p.y, p.z)
}

fn col4(c: [f32; 4]) -> Color {
    Color::new(c[0], c[1], c[2], c[3])
}

fn fmt_time(t: f32) -> String {
    let m = (t / 60.0) as i32;
    format!("{}:{:05.2}", m, t - (m as f32) * 60.0)
}

fn make_static(chunks: &[Chunk], tex: Option<&Texture2D>) -> Vec<StaticMesh> {
    chunks
        .iter()
        .map(|c| {
            let vertices: Vec<Vertex> = c
                .verts
                .iter()
                .map(|v| Vertex::new(-v.p[0], v.p[1], v.p[2], v.uv[0], v.uv[1], col4(v.c)))
                .collect();
            StaticMesh {
                mesh: Mesh { vertices, indices: c.idx.clone(), texture: tex.cloned() },
                center: c.center,
                radius: c.radius,
            }
        })
        .collect()
}

/// Текстура окон 128x128: четыре квадранта по 64x64, в каждом сетка окон 8x8.
fn make_window_texture() -> Texture2D {
    let (w, h) = (128usize, 128usize);
    let mut bytes = vec![0u8; w * h * 4];
    for px in bytes.chunks_mut(4) {
        px[0] = 10;
        px[1] = 12;
        px[2] = 26;
        px[3] = 255;
    }
    let mut rng = Rng::new(3);
    let palette: [[u8; 3]; 4] = [[255, 220, 140], [140, 200, 255], [255, 220, 170], [220, 240, 220]];
    for qy in 0..2usize {
        for qx in 0..2usize {
            for gy in 0..8usize {
                for gx in 0..8usize {
                    if !rng.chance(0.4) {
                        continue;
                    }
                    let col = palette[rng.int(0, 4) as usize];
                    for dy in 2..6usize {
                        for dx in 2..6usize {
                            let x = qx * 64 + gx * 8 + dx;
                            let y = qy * 64 + gy * 8 + dy;
                            let i = (y * w + x) * 4;
                            bytes[i] = col[0];
                            bytes[i + 1] = col[1];
                            bytes[i + 2] = col[2];
                        }
                    }
                }
            }
        }
    }
    let tex = Texture2D::from_rgba8(w as u16, h as u16, &bytes);
    tex.set_filter(FilterMode::Nearest);
    tex
}

fn build_car_model(spec: &CarSpec, look: &Look) -> CarModel {
    let mut cb = ChunkBuilder::new();
    cb.polys(&build_body(spec, look.body, look.glow, look.spoiler));
    let mut wl = ChunkBuilder::new();
    wl.polys(&wheel_polys(spec, -1.0, look.rim, look.rimstyle));
    let mut wr = ChunkBuilder::new();
    wr.polys(&wheel_polys(spec, 1.0, look.rim, look.rimstyle));
    CarModel { body: cb.finish(), wheel_l: wl.finish(), wheel_r: wr.finish(), wheels: wheel_positions(spec) }
}

/// Преобразование модели машины в мир: вращение колеса, крен, поворот, перенос.
struct Xf {
    pos: V3,
    cy: f32,
    sy: f32,
    cr: f32,
    sr: f32,
    spin_c: f32,
    spin_s: f32,
    off: V3,
}

impl Xf {
    fn new(pos: V3, heading_deg: f32, roll_deg: f32, spin_rad: f32, off: V3) -> Xf {
        let h = heading_deg.to_radians();
        let r = roll_deg.to_radians();
        Xf { pos, cy: h.cos(), sy: h.sin(), cr: r.cos(), sr: r.sin(), spin_c: spin_rad.cos(), spin_s: spin_rad.sin(), off }
    }

    /// Точка модели -> точка в игровом мире.
    fn apply(&self, p: [f32; 3]) -> V3 {
        let y1 = p[1] * self.spin_c - p[2] * self.spin_s;
        let z1 = p[1] * self.spin_s + p[2] * self.spin_c;
        let x = p[0] + self.off.x;
        let y = y1 + self.off.y;
        let z = z1 + self.off.z;
        let x2 = x * self.cr - y * self.sr;
        let y2 = x * self.sr + y * self.cr;
        let wx = x2 * self.cy + z * self.sy;
        let wz = -x2 * self.sy + z * self.cy;
        v3(self.pos.x + wx, self.pos.y + y2, self.pos.z + wz)
    }

    /// Только повороты (для нормалей).
    fn rot(&self, n: [f32; 3]) -> V3 {
        let y1 = n[1] * self.spin_c - n[2] * self.spin_s;
        let z1 = n[1] * self.spin_s + n[2] * self.spin_c;
        let x2 = n[0] * self.cr - y1 * self.sr;
        let y2 = n[0] * self.sr + y1 * self.cr;
        v3(x2 * self.cy + z1 * self.sy, y2, -x2 * self.sy + z1 * self.cy)
    }
}

/// Рисует модель машины. Освещение считается для каждой вершины каждый кадр.
/// `mirror` - отражение в мокром асфальте (перевёрнутая по y, тёмная модель).
fn draw_model(chunks: &[Chunk], xf: &Xf, cam: V3, lamps: &[V3], detail: bool, mirror: bool) {
    for ch in chunks.iter() {
        let mut vertices: Vec<Vertex> = Vec::with_capacity(ch.verts.len());
        for v in ch.verts.iter() {
            let mut wp = xf.apply(v.p);
            let mut wn = xf.rot(v.n);
            let albedo = [v.c[0], v.c[1], v.c[2]];
            let c: Rgb = if mirror {
                wp.y = -wp.y;
                wn.y = -wn.y;
                let base = if v.m == Mat::Emissive { albedo } else { shade_cheap(wn, albedo) };
                [base[0] * 0.5, base[1] * 0.5, base[2] * 0.5]
            } else {
                shade_car(wp, wn, albedo, v.m, cam, lamps, detail)
            };
            vertices.push(Vertex::new(-wp.x, wp.y, wp.z, 0.0, 0.0, Color::new(c[0], c[1], c[2], 1.0)));
        }
        draw_mesh(&Mesh { vertices, indices: ch.idx.clone(), texture: None });
    }
}

/// Полупрозрачный выпуклый полигон с цветом и прозрачностью в каждой вершине.
fn draw_alpha(pts: &[V3], cols: &[[f32; 4]]) {
    let n = pts.len();
    if n < 3 {
        return;
    }
    let vertices: Vec<Vertex> = (0..n)
        .map(|i| {
            let p = pts[i];
            let c = cols[i];
            Vertex::new(-p.x, p.y, p.z, 0.0, 0.0, Color::new(c[0], c[1], c[2], c[3]))
        })
        .collect();
    let mut indices: Vec<u16> = Vec::new();
    for k in 1..(n - 1) {
        indices.push(0);
        indices.push(k as u16);
        indices.push(k as u16 + 1);
    }
    draw_mesh(&Mesh { vertices, indices, texture: None });
}

fn draw_local_quad(xf: &Xf, pts: [[f32; 3]; 4], color: [f32; 4]) {
    let w: Vec<V3> = pts.iter().map(|p| xf.apply(*p)).collect();
    draw_alpha(&w, &[color, color, color, color]);
}

/// Мягкое свечение вокруг источника света (поддельный bloom): круг, повёрнутый к камере.
fn draw_halo(center: V3, radius: f32, col: Rgb, a_center: f32, rgt: V3, up: V3) {
    let n = 14usize;
    let mut pts: Vec<V3> = vec![center];
    let mut cols: Vec<[f32; 4]> = vec![[col[0], col[1], col[2], a_center]];
    for k in 0..n {
        let a = 2.0 * PI * (k as f32) / (n as f32);
        pts.push(center + rgt * (a.cos() * radius) + up * (a.sin() * radius));
        cols.push([col[0], col[1], col[2], 0.0]);
    }
    let first = pts[1];
    let first_c = cols[1];
    pts.push(first);
    cols.push(first_c);
    draw_alpha(&pts, &cols);
}

fn draw_shadow(pos: V3, heading: f32, l: f32, w: f32) {
    let xf = Xf::new(pos, heading, 0.0, 0.0, V3::default());
    let (hw, hl) = (w / 2.0 + 0.15, l / 2.0 + 0.2);
    let pts = [
        xf.apply([-hw, 0.03, -hl]),
        xf.apply([hw, 0.03, -hl]),
        xf.apply([hw, 0.03, hl]),
        xf.apply([-hw, 0.03, hl]),
    ];
    let c = [0.0f32, 0.0, 0.0, 0.55];
    draw_alpha(&pts, &[c, c, c, c]);
}

/// Лучи фар на асфальте: две полосы с мягкими краями и затуханием вдаль.
fn draw_beams(pos: V3, heading: f32, l: f32) {
    let xf = Xf::new(pos, heading, 0.0, 0.0, V3::default());
    let zf = l / 2.0;
    let col = [1.0f32, 0.9, 0.65];
    for &sx in [-1.0f32, 1.0].iter() {
        let x0 = sx * 0.6;
        let xs0 = [x0 - 0.45, x0, x0 + 0.45];
        let xs1 = [x0 + sx * 0.6 - 3.6, x0 + sx * 0.6, x0 + sx * 0.6 + 3.6];
        let al0 = [0.0f32, 0.26, 0.0];
        for q in 0..2 {
            let pts = [
                xf.apply([xs0[q], 0.05, zf]),
                xf.apply([xs0[q + 1], 0.05, zf]),
                xf.apply([xs1[q + 1], 0.05, zf + 26.0]),
                xf.apply([xs1[q], 0.05, zf + 26.0]),
            ];
            let cols = [
                [col[0], col[1], col[2], al0[q]],
                [col[0], col[1], col[2], al0[q + 1]],
                [col[0], col[1], col[2], 0.0],
                [col[0], col[1], col[2], 0.0],
            ];
            draw_alpha(&pts, &cols);
        }
    }
}

fn read_player() -> DriveInput {
    let down = |a: KeyCode, b: KeyCode| is_key_down(a) || is_key_down(b);
    let throttle = if down(KeyCode::W, KeyCode::Up) { 1.0 } else { 0.0 };
    let brake = if down(KeyCode::S, KeyCode::Down) { 1.0 } else { 0.0 };
    let mut steer = 0.0f32;
    if down(KeyCode::D, KeyCode::Right) {
        steer += 1.0;
    }
    if down(KeyCode::A, KeyCode::Left) {
        steer -= 1.0;
    }
    DriveInput {
        throttle,
        brake,
        steer,
        handbrake: is_key_down(KeyCode::Space),
        nitro: is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift),
    }
}

fn rank_key(c: &Car) -> (f32, f32) {
    if c.done {
        (0.0, c.finish_time)
    } else {
        (1.0, -(c.progress as f32))
    }
}

fn ui_button(x: f32, y: f32, w: f32, h: f32, label: &str, color: Color, enabled: bool) -> bool {
    let (mx, my) = mouse_position();
    let hover = mx >= x && mx <= x + w && my >= y && my <= y + h;
    let mut c = if enabled { color } else { Color::new(0.18, 0.18, 0.24, 1.0) };
    if enabled && hover {
        c = Color::new((c.r + 0.12).min(1.0), (c.g + 0.12).min(1.0), (c.b + 0.12).min(1.0), 1.0);
    }
    draw_rectangle(x, y, w, h, c);
    let size = (h * 0.5).max(10.0);
    let dims = measure_text(label, None, size as u16, 1.0);
    draw_text(label, x + (w - dims.width) / 2.0, y + (h + dims.height) / 2.0, size, WHITE);
    enabled && hover && is_mouse_button_pressed(MouseButton::Left)
}

fn draw_text_c(text: &str, cx: f32, y: f32, size: f32, color: Color) {
    let dims = measure_text(text, None, size as u16, 1.0);
    draw_text(text, cx - dims.width / 2.0, y, size, color);
}

// ----------------------------- игра -----------------------------

pub struct Game {
    save: Save,
    specs: Vec<CarSpec>,
    tspecs: Vec<CarSpec>,
    tracks: Vec<TrackSpec>,
    track: Track,
    loaded_track: usize,
    world_plain: Vec<StaticMesh>,
    world_city: Vec<StaticMesh>,
    world_road: Vec<StaticMesh>,
    lamps: Vec<Lamp>,
    window_tex: Texture2D,
    cars: Vec<Car>,
    car_models: Vec<CarModel>,
    traffic: Vec<Traffic>,
    traffic_models: Vec<Vec<Chunk>>,
    particles: Vec<Particle>,
    state: State,
    cd: f32,
    go_timer: f32,
    race_time: f32,
    lap_start: f32,
    finish_timer: f32,
    shake: f32,
    cam_heading: f32,
    cam_pos: V3,
    cam_target: V3,
    cam_fov: f32,
    best_lap: Option<f32>,
    garage_angle: f32,
    view_kind: usize,
    page: usize,
    msg: String,
    results: Vec<String>,
    show_results: bool,
    center_text: String,
    rng: Rng,
    sky: Color,
    race_laps: i32,
    mm_cx: f32,
    mm_cz: f32,
    mm_scale: f32,
}

impl Game {
    pub fn new() -> Game {
        let specs = carmodels::all_car_specs();
        let tspecs = carmodels::all_traffic_specs();
        let tracks = track_specs();
        let save = Save::load(tracks.len());
        let track = Track::new(&tracks[save.track]);
        let loaded_track = save.track;
        let view_kind = save.car;
        let mut g = Game {
            save,
            specs,
            tspecs,
            tracks,
            track,
            loaded_track,
            world_plain: Vec::new(),
            world_city: Vec::new(),
            world_road: Vec::new(),
            lamps: Vec::new(),
            window_tex: make_window_texture(),
            cars: Vec::new(),
            car_models: Vec::new(),
            traffic: Vec::new(),
            traffic_models: Vec::new(),
            particles: Vec::new(),
            state: State::Garage,
            cd: 3.99,
            go_timer: 0.0,
            race_time: 0.0,
            lap_start: 0.0,
            finish_timer: 0.0,
            shake: 0.0,
            cam_heading: 0.0,
            cam_pos: V3::default(),
            cam_target: V3::default(),
            cam_fov: 55.0,
            best_lap: None,
            garage_angle: 200.0,
            view_kind,
            page: 0,
            msg: String::new(),
            results: Vec::new(),
            show_results: false,
            center_text: String::new(),
            rng: Rng::new(12345),
            sky: BLACK,
            race_laps: 3,
            mm_cx: 0.0,
            mm_cz: 0.0,
            mm_scale: 0.1,
        };
        g.build_scene();
        g.create_cars();
        g.enter_garage();
        g
    }

    fn player_look(&self) -> Look {
        Look {
            body: BODY_COLORS[self.save.body].1,
            glow: GLOW_COLORS[self.save.glow].1,
            rim: RIM_COLORS[self.save.rim].1,
            rimstyle: self.save.rimstyle as u8,
            spoiler: self.save.spoiler as u8,
        }
    }

    /// Мир текущей трассы: меши, трафик, мини-карта.
    fn build_scene(&mut self) {
        let w = build_world(&self.track);
        self.world_road = make_static(&w.road, None);
        self.world_plain = make_static(&w.plain, None);
        self.world_city = make_static(&w.city, Some(&self.window_tex));
        self.lamps = w.lamps;
        let sky = self.track.spec.sky;
        self.sky = Color::new(sky[0], sky[1], sky[2], 1.0);
        self.race_laps = self.track.spec.laps;
        let traffic_n = self.track.spec.traffic;

        // границы для мини-карты
        let (mut minx, mut maxx, mut minz, mut maxz) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for p in self.track.road.pts.iter() {
            minx = minx.min(p.x);
            maxx = maxx.max(p.x);
            minz = minz.min(p.z);
            maxz = maxz.max(p.z);
        }
        self.mm_cx = (minx + maxx) / 2.0;
        self.mm_cz = (minz + maxz) / 2.0;
        self.mm_scale = 240.0 / (maxx - minx).max(maxz - minz);

        // трафик: четыре полосы, в каждой своя скорость
        self.traffic.clear();
        self.traffic_models.clear();
        let mut rng = Rng::new(self.save.track as u64 * 13 + 5);
        let lanes = [-8.5f32, -3.0, 3.0, 8.5];
        let speeds = [17.0f32, 22.0, 18.0, 24.0];
        let per_lane = (traffic_n / 4).max(1);
        let span = self.track.m as f32 - 60.0;
        let kinds = [0usize, 0, 1, 2, 3];
        for ln in 0..4usize {
            for j in 0..per_lane {
                let t0 = 25.0 + (j as f32 + 0.5 + ln as f32 * 0.25) * span / per_lane as f32 + rng.range(-6.0, 6.0);
                let kind = kinds[rng.int(0, 5) as usize];
                let col = TRAFFIC_COLORS[rng.int(0, 8) as usize];
                let (len, wid) = (self.tspecs[kind].l, self.tspecs[kind].w);
                self.traffic.push(Traffic::new(&self.track, kind, col, len, wid, t0, lanes[ln], speeds[ln]));
                let mut cb = ChunkBuilder::new();
                cb.polys(&build_traffic(&self.tspecs[kind], kind, col));
                self.traffic_models.push(cb.finish());
            }
        }
    }

    fn create_cars(&mut self) {
        self.cars.clear();
        self.car_models.clear();
        let look = self.player_look();
        let vk = self.view_kind;
        self.cars.push(Car::new(&self.track, 3, "YOU", vk, look, true, 1.0, 0.0, 0.0));
        let ai: [(&str, usize, Look, f32, f32, f32); 3] = [
            (
                "RIVAL 1",
                1,
                Look { body: rgb!(230, 40, 50), glow: Some(rgb!(255, 90, 40)), rim: rgb!(200, 205, 215), rimstyle: 1, spoiler: 2 },
                0.95,
                -3.0,
                0.7,
            ),
            (
                "RIVAL 2",
                3,
                Look { body: rgb!(235, 235, 240), glow: Some(rgb!(255, 230, 80)), rim: rgb!(20, 20, 26), rimstyle: 2, spoiler: 2 },
                0.90,
                3.0,
                0.5,
            ),
            (
                "RIVAL 3",
                0,
                Look { body: rgb!(60, 120, 220), glow: Some(rgb!(30, 255, 170)), rim: rgb!(255, 200, 60), rimstyle: 0, spoiler: 1 },
                0.97,
                0.0,
                0.3,
            ),
        ];
        for (i, a) in ai.iter().enumerate() {
            self.cars.push(Car::new(&self.track, i, a.0, a.1, a.2, false, a.3, a.4, a.5));
        }
        let tune = self.save.tune[vk];
        self.cars[0].apply_tune(tune);
        self.apply_track_to_cars();
        for i in 0..self.cars.len() {
            let m = build_car_model(&self.specs[self.cars[i].kind], &self.cars[i].look);
            self.car_models.push(m);
        }
    }

    fn apply_track_to_cars(&mut self) {
        let ai = self.track.spec.ai;
        for c in self.cars.iter_mut() {
            if !c.is_player {
                c.skill = c.skill0 * ai;
                c.apply_tune(Tune::default());
            }
            c.reset_to_grid(&self.track);
        }
    }

    fn load_track(&mut self) {
        self.track = Track::new(&self.tracks[self.save.track]);
        self.loaded_track = self.save.track;
        self.build_scene();
        self.apply_track_to_cars();
    }

    fn reset_race(&mut self) {
        for c in self.cars.iter_mut() {
            c.reset_to_grid(&self.track);
        }
        for t in self.traffic.iter_mut() {
            t.reset(&self.track);
        }
        self.cd = 3.99;
        self.go_timer = 0.0;
        self.race_time = 0.0;
        self.lap_start = 0.0;
        self.finish_timer = 0.0;
        self.best_lap = None;
        self.show_results = false;
        self.center_text.clear();
        self.results.clear();
        self.particles.clear();
        self.cam_heading = self.cars[0].heading;
    }

    fn restart(&mut self) {
        self.reset_race();
        self.state = State::Countdown;
    }

    fn enter_garage(&mut self) {
        self.reset_race();
        self.state = State::Garage;
        self.msg.clear();
    }

    fn start_race(&mut self) {
        if !self.save.owned[self.view_kind] {
            self.msg = "Buy this car first!".to_string();
            return;
        }
        self.save.car = self.view_kind;
        self.save.store();
        if self.save.track != self.loaded_track {
            self.load_track();
        }
        let tune = self.save.tune[self.view_kind];
        self.cars[0].apply_tune(tune);
        self.restart();
    }

    fn ranking(&self) -> Vec<usize> {
        let mut v: Vec<usize> = (0..self.cars.len()).collect();
        v.sort_by(|&a, &b| {
            let ka = rank_key(&self.cars[a]);
            let kb = rank_key(&self.cars[b]);
            ka.partial_cmp(&kb).unwrap_or(std::cmp::Ordering::Equal)
        });
        v
    }

    // ----------------------------- обновление -----------------------------

    /// Возвращает false, когда нужно выйти из игры.
    pub fn update(&mut self, dt: f32) -> bool {
        if is_key_pressed(KeyCode::Escape) {
            return false;
        }
        match self.state {
            State::Garage => {
                self.update_garage(dt);
                if is_key_pressed(KeyCode::Enter) {
                    self.start_race();
                }
            }
            _ => {
                self.update_race(dt);
                if is_key_pressed(KeyCode::R) && (self.state == State::Racing || self.state == State::Finished) {
                    self.cars[0].recover(&self.track);
                }
                if is_key_pressed(KeyCode::Enter) && self.state == State::Finished && self.show_results {
                    self.enter_garage();
                }
            }
        }
        true
    }

    fn update_garage(&mut self, dt: f32) {
        self.garage_angle += 22.0 * dt;
        let a = self.garage_angle.to_radians();
        let p = self.cars[0].pos;
        self.cam_pos = p + v3(a.sin() * 9.0, 2.6, a.cos() * 9.0);
        let yaw = (self.garage_angle + 180.0).to_radians();
        let right = v3(yaw.cos(), 0.0, -yaw.sin());
        // машина смещена вправо от панели меню
        self.cam_target = p + v3(0.0, 0.8, 0.0) - right * 2.6;
        self.cam_fov += (36.0 - self.cam_fov) * (4.0 * dt).min(1.0);
        self.cars[0].wheel_spin += 2.0 * dt;
    }

    fn update_race(&mut self, dt: f32) {
        let mut impact = 0.0f32;
        if self.state == State::Countdown {
            self.cd -= dt;
            if self.cd > 0.0 {
                self.center_text = format!("{}", self.cd as i32 + 1);
            } else {
                self.state = State::Racing;
                self.go_timer = 1.0;
                self.center_text = "GO!".to_string();
            }
        } else {
            self.race_time += dt;
            if self.go_timer > 0.0 {
                self.go_timer -= dt;
                if self.go_timer <= 0.0 {
                    self.center_text.clear();
                }
            }
            for tc in self.traffic.iter_mut() {
                tc.drive(dt, &self.track);
            }
            let pp = self.cars[0].progress;
            let m = self.track.m as f32;
            for i in 0..self.cars.len() {
                let manual = self.cars[i].is_player && !self.cars[i].autopilot;
                let inp: DriveInput;
                if manual {
                    inp = read_player();
                } else {
                    if !self.cars[i].is_player {
                        // лёгкая "резинка": отстающие соперники чуть быстрее
                        let gap = clampf((pp - self.cars[i].progress) as f32 / (m * 0.5) * 0.12, -0.06, 0.06);
                        self.cars[i].max_speed = self.cars[i].base_max * (1.0 + gap);
                    }
                    let mut x = self.cars[i].ai_inputs(&self.track, &self.traffic, &mut self.rng);
                    if self.cars[i].done {
                        x.throttle = if self.cars[i].speed < 35.0 { 1.0 } else { 0.0 };
                        x.nitro = false;
                    }
                    inp = x;
                }
                let imp = self.cars[i].step(dt, &inp, &self.track, &mut self.particles);
                if manual {
                    impact = impact.max(imp);
                }
                if !self.cars[i].is_player {
                    if self.cars[i].speed.abs() < 3.0 {
                        self.cars[i].stuck += dt;
                    } else {
                        self.cars[i].stuck = 0.0;
                    }
                    if self.cars[i].stuck > 2.5 {
                        self.cars[i].recover(&self.track);
                        self.cars[i].stuck = 0.0;
                    }
                }
                if !self.cars[i].done && self.cars[i].lap > self.race_laps {
                    self.cars[i].done = true;
                    self.cars[i].finish_time = self.race_time;
                    if self.cars[i].is_player {
                        self.cars[i].autopilot = true;
                        self.state = State::Finished;
                        self.finish_timer = 2.0;
                        self.center_text = "FINISH!".to_string();
                    }
                }
            }
            resolve_car_collisions(&mut self.cars);
            let hit = resolve_traffic(&mut self.cars, &mut self.traffic);
            impact = impact.max(hit);

            // время кругов игрока
            let done_laps = (self.cars[0].lap.min(self.race_laps + 1) - 1).max(0) as usize;
            if self.cars[0].lap_times.len() < done_laps {
                let lt = self.race_time - self.lap_start;
                self.lap_start = self.race_time;
                self.cars[0].lap_times.push(lt);
                self.best_lap = Some(match self.best_lap {
                    Some(b) => b.min(lt),
                    None => lt,
                });
            }

            if self.state == State::Finished {
                self.finish_timer -= dt;
                if self.finish_timer <= 0.0 && !self.show_results {
                    self.make_results();
                }
            }
        }
        for p in self.particles.iter_mut() {
            p.life -= dt;
        }
        self.particles.retain(|p| p.life > 0.0);
        self.update_camera(dt, impact);
    }

    fn make_results(&mut self) {
        let order = self.ranking();
        let place = order.iter().position(|&i| i == 0).unwrap_or(0);
        let mut payout = 0i32;
        if self.cars[0].done {
            payout = (PAYOUT[place.min(3)] as f32 * self.track.spec.reward) as i32;
        }
        self.save.money += payout;
        self.save.store();
        self.results.clear();
        self.results.push("RESULTS".to_string());
        self.results.push(String::new());
        for (n, &i) in order.iter().enumerate() {
            let c = &self.cars[i];
            let t = if c.done { fmt_time(c.finish_time) } else { "DNF".to_string() };
            self.results.push(format!("{}.  {:<10} {}", n + 1, c.label, t));
        }
        self.results.push(String::new());
        self.results.push(format!("PRIZE  +${}", payout));
        self.results.push(format!("MONEY  ${}", self.save.money));
        self.results.push(String::new());
        self.results.push("ENTER - garage".to_string());
        self.show_results = true;
        self.center_text.clear();
    }

    fn update_camera(&mut self, dt: f32, impact: f32) {
        let (ppos, pheading, pspeed, pboost) = {
            let p = &self.cars[0];
            (p.pos, p.heading, p.speed, p.boosting)
        };
        self.cam_heading += wrap180(pheading - self.cam_heading) * (6.0 * dt).min(1.0);
        let r = self.cam_heading.to_radians();
        let d = v3(r.sin(), 0.0, r.cos());
        let dist = 8.5 + pspeed.abs() * 0.04;
        let mut pos = ppos - d * dist + v3(0.0, 3.3, 0.0);
        if impact > 6.0 {
            self.shake = self.shake.max((impact / 40.0).min(0.35));
        }
        if self.shake > 0.0 {
            self.shake -= dt;
            let k = self.shake.max(0.0) * 4.0;
            pos += v3(self.rng.range(-0.25, 0.25), self.rng.range(-0.2, 0.2), self.rng.range(-0.25, 0.25)) * k;
        }
        self.cam_pos = pos;
        self.cam_target = ppos + d * 6.0 + v3(0.0, 1.3, 0.0);
        let target_fov = 62.0 + pspeed.abs() * 0.18 + if pboost { 10.0 } else { 0.0 };
        self.cam_fov += (target_fov - self.cam_fov) * (4.0 * dt).min(1.0);
    }

    // ----------------------------- отрисовка -----------------------------

    pub fn draw(&mut self) {
        clear_background(self.sky);
        set_camera(&Camera3D {
            position: mqv(self.cam_pos),
            target: mqv(self.cam_target),
            up: vec3(0.0, 1.0, 0.0),
            fovy: self.cam_fov.to_radians(),
            ..Default::default()
        });
        self.draw_scene();
        set_default_camera();
        if self.state == State::Garage {
            self.draw_garage();
        } else {
            self.draw_hud();
        }
    }

    /// Фонари рядом с точкой (для освещения машины): не больше шести ближайших.
    fn near_lamps(&self, p: V3) -> Vec<V3> {
        let mut v: Vec<(f32, V3)> = Vec::new();
        for l in self.lamps.iter() {
            let d = (l.pos - p).len();
            if d < LAMP_R + 6.0 {
                v.push((d, l.pos));
            }
        }
        v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        v.truncate(6);
        v.into_iter().map(|x| x.1).collect()
    }

    fn draw_car_parts(&self, i: usize, cam: V3, lamps: &[V3], detail: bool, mirror: bool) {
        let c = &self.cars[i];
        let m = &self.car_models[i];
        let xf = Xf::new(c.pos, c.heading, c.roll, 0.0, V3::default());
        draw_model(&m.body, &xf, cam, lamps, detail, mirror);
        for p in m.wheels.iter() {
            let xw = Xf::new(c.pos, c.heading, c.roll, c.wheel_spin, *p);
            if p.x < 0.0 {
                draw_model(&m.wheel_l, &xw, cam, lamps, detail, mirror);
            } else {
                draw_model(&m.wheel_r, &xw, cam, lamps, detail, mirror);
            }
        }
    }

    fn draw_scene(&self) {
        let cp = self.cam_pos;
        let garage = self.state == State::Garage;

        // 1. отражения машин: рисуются под дорогой, сверху ляжет полупрозрачный асфальт
        for i in 0..self.cars.len() {
            if garage && i > 0 {
                continue;
            }
            if (self.cars[i].pos - cp).len() < 130.0 {
                self.draw_car_parts(i, cp, &[], false, true);
            }
        }
        if !garage {
            for (i, tc) in self.traffic.iter().enumerate() {
                if (tc.pos - cp).len() > 110.0 {
                    continue;
                }
                let xf = Xf::new(tc.pos, tc.heading + tc.kick * 3.0, 0.0, 0.0, V3::default());
                draw_model(&self.traffic_models[i], &xf, cp, &[], false, true);
            }
        }

        // 2. мокрый асфальт (чуть прозрачный)
        for sm in self.world_road.iter() {
            if (sm.center - cp).len() - sm.radius < 1800.0 {
                draw_mesh(&sm.mesh);
            }
        }

        // 3. остальной мир
        for sm in self.world_plain.iter() {
            if (sm.center - cp).len() - sm.radius < 1800.0 {
                draw_mesh(&sm.mesh);
            }
        }
        for sm in self.world_city.iter() {
            if (sm.center - cp).len() - sm.radius < 1000.0 {
                draw_mesh(&sm.mesh);
            }
        }

        // 4. машины с динамическим освещением
        if !garage {
            for (i, tc) in self.traffic.iter().enumerate() {
                let dist = (tc.pos - cp).len();
                if dist > 600.0 {
                    continue;
                }
                let near = self.near_lamps(tc.pos);
                let xf = Xf::new(tc.pos, tc.heading + tc.kick * 3.0, 0.0, 0.0, V3::default());
                draw_model(&self.traffic_models[i], &xf, cp, &near, dist < 45.0, false);
            }
        }
        for i in 0..self.cars.len() {
            if garage && i > 0 {
                continue;
            }
            let near = self.near_lamps(self.cars[i].pos);
            self.draw_car_parts(i, cp, &near, true, false);
        }

        // 5. полупрозрачные эффекты
        self.draw_effects();
    }

    fn draw_effects(&self) {
        let cp = self.cam_pos;
        let garage = self.state == State::Garage;
        let fwd = (self.cam_target - cp).norm();
        let rgt = v3(0.0, 1.0, 0.0).cross(fwd).norm();
        let up = fwd.cross(rgt);
        let t = get_time() as f32;

        // тени под машинами и лучи фар
        for i in 0..self.cars.len() {
            if garage && i > 0 {
                continue;
            }
            let c = &self.cars[i];
            let sp = &self.specs[c.kind];
            draw_shadow(c.pos, c.heading, sp.l, sp.w);
            if !garage {
                draw_beams(c.pos, c.heading, sp.l);
            }
        }
        if !garage {
            for tc in self.traffic.iter() {
                let dist = (tc.pos - cp).len();
                if dist > 110.0 {
                    continue;
                }
                draw_shadow(tc.pos, tc.heading, tc.length, tc.width);
                if dist < 80.0 {
                    draw_beams(tc.pos, tc.heading, tc.length);
                }
            }
        }

        // отражения фонарей на мокром асфальте: размытые полосы вдоль взгляда
        for l in self.lamps.iter() {
            let d = l.pos - cp;
            let dist = d.len();
            if dist > 200.0 || d.dot(fwd) < 5.0 {
                continue;
            }
            let k = cp.y / (cp.y + l.pos.y);
            let pr = v3(cp.x + (l.pos.x - cp.x) * k, 0.04, cp.z + (l.pos.z - cp.z) * k);
            let (ni, _) = self.track.road.nearest(pr, None, 0);
            let lat = (pr - self.track.road.pts[ni]).dot(self.track.road.right[ni]);
            if lat.abs() > ROAD_HALF - 0.8 {
                continue; // отражение должно лежать на дороге
            }
            let dp = ((pr.x - cp.x) * (pr.x - cp.x) + (pr.z - cp.z) * (pr.z - cp.z)).sqrt();
            if dp < 7.0 {
                continue;
            }
            let dir = v3(cp.x - pr.x, 0.0, cp.z - pr.z).norm();
            let side = v3(-dir.z, 0.0, dir.x);
            let ls = 1.2 + 0.04 * dp;
            let ws = 0.28 + 0.006 * dp;
            let a = (1.0 - dist / 200.0).max(0.0) * 0.45 * ((dp - 7.0) / 8.0).min(1.0);
            let e0 = pr + dir * (ls * 0.5);
            let e1 = pr - dir * ls;
            let col = [1.0f32, 0.78, 0.52];
            let parts = [(e0, pr, 0.0f32, a), (pr, e1, a, 0.0f32)];
            for &(pa, pb, ca, cb) in parts.iter() {
                let pts = [pa + side * (ws * 0.4), pa - side * (ws * 0.4), pb - side * ws, pb + side * ws];
                let cols = [
                    [col[0], col[1], col[2], ca],
                    [col[0], col[1], col[2], ca],
                    [col[0], col[1], col[2], cb],
                    [col[0], col[1], col[2], cb],
                ];
                draw_alpha(&pts, &cols);
            }
        }

        // подсветка днища, огонь нитро
        for i in 0..self.cars.len() {
            if garage && i > 0 {
                continue;
            }
            let c = &self.cars[i];
            let spec = &self.specs[c.kind];
            let xf = Xf::new(c.pos, c.heading, 0.0, 0.0, V3::default());
            if let Some(g) = c.look.glow {
                let (hw, hl) = (spec.w / 2.0 + 0.45, spec.l / 2.0 + 0.6);
                draw_local_quad(
                    &xf,
                    [[-hw, 0.12, -hl], [hw, 0.12, -hl], [hw, 0.12, hl], [-hw, 0.12, hl]],
                    [g[0], g[1], g[2], 0.4],
                );
            }
            if c.boosting {
                let zr = -(spec.l / 2.0 + 0.4);
                let len = 1.3 + 0.5 * (t * 40.0 + i as f32).sin().abs();
                let fc = [0.35f32, 0.67, 1.0, 0.85];
                for &sx in [-0.55f32, 0.55].iter() {
                    draw_local_quad(
                        &xf,
                        [[sx - 0.12, 0.55, zr], [sx + 0.12, 0.55, zr], [sx + 0.04, 0.55, zr - len], [sx - 0.04, 0.55, zr - len]],
                        fc,
                    );
                    draw_local_quad(
                        &xf,
                        [[sx, 0.43, zr], [sx, 0.67, zr], [sx, 0.60, zr - len], [sx, 0.50, zr - len]],
                        fc,
                    );
                }
            }
        }
        for p in self.particles.iter() {
            let k = 1.0 - p.life / 0.7;
            draw_sphere(mqv(p.pos), 0.3 + 1.2 * k, None, Color::new(0.86, 0.86, 0.94, 0.5 * (1.0 - k)));
        }

        // ореолы фонарей, фар и стоп-сигналов
        for l in self.lamps.iter() {
            let d = l.pos - cp;
            let dist = d.len();
            if dist > 320.0 || d.dot(fwd) < 1.0 {
                continue;
            }
            let r = (0.9 + dist * 0.028).max(1.4).min(7.0);
            let c = l.pos - v3(0.0, 0.1, 0.0);
            draw_halo(c, r, [1.0, 0.78, 0.5], 0.55, rgt, up);
            draw_halo(c, r * 0.35, [1.0, 0.95, 0.85], 0.95, rgt, up);
        }
        for i in 0..self.cars.len() {
            if garage && i > 0 {
                continue;
            }
            let c = &self.cars[i];
            let sp = &self.specs[c.kind];
            let xf = Xf::new(c.pos, c.heading, 0.0, 0.0, V3::default());
            for &sx in [-1.0f32, 1.0].iter() {
                let tail = xf.apply([sx * sp.tail[0], sp.tail[1], -sp.l / 2.0 - 0.05]);
                draw_halo(tail, 0.9, [1.0, 0.1, 0.1], 0.55, rgt, up);
                let head = xf.apply([sx * sp.head[0], sp.head[1], sp.l / 2.0 + 0.05]);
                draw_halo(head, 0.8, [1.0, 0.95, 0.8], 0.40, rgt, up);
            }
        }
        if !garage {
            for tc in self.traffic.iter() {
                if (tc.pos - cp).len() > 90.0 {
                    continue;
                }
                let sp = &self.tspecs[tc.kind];
                if sp.body.is_empty() {
                    continue;
                }
                let xf = Xf::new(tc.pos, tc.heading + tc.kick * 3.0, 0.0, 0.0, V3::default());
                for &sx in [-1.0f32, 1.0].iter() {
                    let tail = xf.apply([sx * sp.tail[0], sp.tail[1], -sp.l / 2.0 - 0.05]);
                    draw_halo(tail, 0.8, [1.0, 0.1, 0.1], 0.45, rgt, up);
                }
            }
        }
    }

    fn draw_hud(&self) {
        let sc = screen_height() / 720.0;
        let (sw, sh) = (screen_width(), screen_height());
        let p = &self.cars[0];
        let order = self.ranking();
        let place = order.iter().position(|&i| i == 0).unwrap_or(0) + 1;

        // позиция, круг, время
        let orange = Color::new(1.0, 0.47, 0.16, 1.0);
        draw_text(&format!("{}/{}", place, self.cars.len()), 30.0 * sc, 70.0 * sc, 64.0 * sc, orange);
        let lap = p.lap.max(1).min(self.race_laps);
        draw_text(&format!("LAP {}/{}", lap, self.race_laps), 30.0 * sc, 108.0 * sc, 32.0 * sc, WHITE);
        draw_text(&fmt_time(self.race_time), 30.0 * sc, 140.0 * sc, 28.0 * sc, WHITE);
        if let Some(b) = self.best_lap {
            draw_text(&format!("BEST {}", fmt_time(b)), 30.0 * sc, 168.0 * sc, 22.0 * sc, SKYBLUE);
        }

        // скорость и нитро
        let kmh = (p.vel.len() * 3.6) as i32;
        let st = format!("{}", kmh);
        let dims = measure_text(&st, None, (84.0 * sc) as u16, 1.0);
        draw_text(&st, sw - 40.0 * sc - dims.width, sh - 80.0 * sc, 84.0 * sc, WHITE);
        draw_text("KM/H", sw - 110.0 * sc, sh - 50.0 * sc, 26.0 * sc, SKYBLUE);
        let (bx, by, bw, bh) = (sw - 340.0 * sc, sh - 38.0 * sc, 300.0 * sc, 14.0 * sc);
        draw_rectangle(bx, by, bw, bh, Color::new(0.16, 0.16, 0.27, 1.0));
        let fill = if p.boosting { orange } else { SKYBLUE };
        draw_rectangle(bx, by, bw * (p.nitro / 100.0).max(0.0).min(1.0), bh, fill);
        draw_text("NITRO", bx, by - 6.0 * sc, 20.0 * sc, SKYBLUE);

        if p.on_short {
            draw_text_c("SHORTCUT", sw / 2.0, 80.0 * sc, 40.0 * sc, YELLOW);
        }
        if !self.center_text.is_empty() {
            draw_text_c(&self.center_text, sw / 2.0, sh * 0.30, 130.0 * sc, Color::new(1.0, 0.86, 0.24, 1.0));
        }

        // мини-карта
        let mx = sw - 170.0 * sc;
        let my = 200.0 * sc;
        let map = |x: f32, z: f32| -> (f32, f32) {
            (mx + (x - self.mm_cx) * self.mm_scale * sc, my - (z - self.mm_cz) * self.mm_scale * sc)
        };
        let n = self.track.road.n();
        for i in 0..n {
            let a = self.track.road.pts[i];
            let b = self.track.road.pts[(i + 1) % n];
            let (x1, y1) = map(a.x, a.z);
            let (x2, y2) = map(b.x, b.z);
            draw_line(x1, y1, x2, y2, 2.0 * sc, Color::new(0.47, 0.47, 0.7, 1.0));
        }
        for sc_i in self.track.shortcuts.iter() {
            for k in 0..(sc_i.road.n() - 1) {
                let a = sc_i.road.pts[k];
                let b = sc_i.road.pts[k + 1];
                let (x1, y1) = map(a.x, a.z);
                let (x2, y2) = map(b.x, b.z);
                draw_line(x1, y1, x2, y2, 2.0 * sc, YELLOW);
            }
        }
        for (i, c) in self.cars.iter().enumerate() {
            let (x, y) = map(c.pos.x, c.pos.z);
            let b = c.look.body;
            draw_circle(x, y, if i == 0 { 6.0 * sc } else { 4.5 * sc }, Color::new(b[0], b[1], b[2], 1.0));
        }

        // итоги гонки
        if self.show_results {
            draw_rectangle(sw / 2.0 - 280.0 * sc, sh / 2.0 - 220.0 * sc, 560.0 * sc, 440.0 * sc, Color::new(0.0, 0.0, 0.05, 0.85));
            for (n, line) in self.results.iter().enumerate() {
                let color = if n == 0 { orange } else { WHITE };
                let size = if n == 0 { 44.0 * sc } else { 28.0 * sc };
                draw_text_c(line, sw / 2.0, sh / 2.0 - 170.0 * sc + n as f32 * 36.0 * sc, size, color);
            }
        }
    }

    // ----------------------------- гараж -----------------------------

    fn refresh_player(&mut self) {
        let look = self.player_look();
        let vk = self.view_kind;
        self.cars[0].kind = vk;
        self.cars[0].look = look;
        let tune = self.save.tune[vk];
        self.cars[0].apply_tune(tune);
        self.car_models[0] = build_car_model(&self.specs[vk], &look);
    }

    fn cycle_car(&mut self, d: i32) {
        self.view_kind = ((self.view_kind as i32 + d).rem_euclid(KIND_COUNT as i32)) as usize;
        self.msg.clear();
        self.refresh_player();
    }

    fn cycle_track(&mut self, d: i32) {
        let n = self.tracks.len() as i32;
        self.save.track = ((self.save.track as i32 + d).rem_euclid(n)) as usize;
        self.save.store();
    }

    fn cycle_paint(&mut self, which: usize, d: i32) {
        let lens = [BODY_COLORS.len(), RIM_COLORS.len(), RIM_STYLES.len(), GLOW_COLORS.len(), SPOILERS.len()];
        let cur = match which {
            0 => self.save.body,
            1 => self.save.rim,
            2 => self.save.rimstyle,
            3 => self.save.glow,
            _ => self.save.spoiler,
        };
        let nv = ((cur as i32 + d).rem_euclid(lens[which] as i32)) as usize;
        match which {
            0 => self.save.body = nv,
            1 => self.save.rim = nv,
            2 => self.save.rimstyle = nv,
            3 => self.save.glow = nv,
            _ => self.save.spoiler = nv,
        }
        self.refresh_player();
        self.save.store();
    }

    fn buy_or_select(&mut self) {
        let k = &STATS[self.view_kind];
        if self.save.owned[self.view_kind] {
            self.save.car = self.view_kind;
            self.msg = format!("{} selected", k.name);
        } else if self.save.money >= k.price {
            self.save.money -= k.price;
            self.save.owned[self.view_kind] = true;
            self.save.car = self.view_kind;
            self.msg = format!("{} purchased!", k.name);
        } else {
            self.msg = "Not enough money".to_string();
        }
        self.save.store();
    }

    fn buy_upgrade(&mut self, n: usize) {
        let vk = self.view_kind;
        if !self.save.owned[vk] {
            self.msg = "Buy this car first".to_string();
            return;
        }
        let mut t = self.save.tune[vk];
        let lv = match n {
            0 => t.engine,
            1 => t.turbo,
            2 => t.tires,
            _ => t.nitro,
        } as usize;
        if lv >= 3 {
            return;
        }
        let cost = UPGRADE_COST[lv];
        if self.save.money < cost {
            self.msg = "Not enough money".to_string();
            return;
        }
        self.save.money -= cost;
        match n {
            0 => t.engine += 1,
            1 => t.turbo += 1,
            2 => t.tires += 1,
            _ => t.nitro += 1,
        }
        self.save.tune[vk] = t;
        self.cars[0].apply_tune(t);
        self.save.store();
    }

    fn draw_garage(&mut self) {
        let sc = screen_height() / 720.0;
        let (px, py, pw, ph) = (20.0 * sc, 20.0 * sc, 490.0 * sc, 680.0 * sc);
        let orange = Color::new(1.0, 0.47, 0.16, 1.0);
        let blue = Color::new(0.2, 0.24, 0.43, 1.0);
        draw_rectangle(px, py, pw, ph, Color::new(0.03, 0.03, 0.09, 0.86));
        draw_text("GARAGE", px + 20.0 * sc, py + 44.0 * sc, 42.0 * sc, orange);
        draw_text(&format!("MONEY ${}", self.save.money), px + 20.0 * sc, py + 78.0 * sc, 26.0 * sc, YELLOW);
        let vk = self.view_kind;
        let tune = self.save.tune[vk];
        let k = &STATS[vk];
        let rating = 40.0 + (k.accel + k.top) * 40.0 + k.grip * 8.0
            + 25.0 * (tune.engine as f32 + tune.turbo as f32 + tune.tires as f32 + tune.nitro as f32);
        let rt = format!("PERFORMANCE {}", rating as i32);
        let rd = measure_text(&rt, None, (24.0 * sc) as u16, 1.0);
        draw_text(&rt, px + pw - 20.0 * sc - rd.width, py + 78.0 * sc, 24.0 * sc, SKYBLUE);

        let tabs = ["CAR", "PAINT", "TUNE"];
        for (n, name) in tabs.iter().enumerate() {
            let col = if self.page == n { Color::new(0.16, 0.47, 0.78, 1.0) } else { Color::new(0.16, 0.18, 0.31, 1.0) };
            if ui_button(px + (20.0 + n as f32 * 154.0) * sc, py + 96.0 * sc, 144.0 * sc, 36.0 * sc, name, col, true) {
                self.page = n;
                self.msg.clear();
            }
        }

        let owned = self.save.owned[vk];
        let row_x = |n: f32| px + n * sc; // удобный перевод координат
        match self.page {
            0 => {
                // выбор машины
                draw_text("CAR", row_x(20.0), py + 190.0 * sc, 26.0 * sc, WHITE);
                if ui_button(row_x(150.0), py + 164.0 * sc, 40.0 * sc, 36.0 * sc, "-", blue, true) {
                    self.cycle_car(-1);
                }
                draw_text_c(k.name, px + 300.0 * sc, py + 190.0 * sc, 26.0 * sc, WHITE);
                if ui_button(row_x(410.0), py + 164.0 * sc, 40.0 * sc, 36.0 * sc, "+", blue, true) {
                    self.cycle_car(1);
                }
                draw_text(
                    &format!("ACCEL x{:.2}   TOP x{:.2}   GRIP {:+.1}", k.accel, k.top, k.grip),
                    row_x(20.0),
                    py + 232.0 * sc,
                    22.0 * sc,
                    Color::new(0.55, 0.6, 0.78, 1.0),
                );
                let status = if owned {
                    if self.save.car == vk { "OWNED (selected)".to_string() } else { "OWNED".to_string() }
                } else {
                    format!("PRICE ${}", k.price)
                };
                draw_text(&status, row_x(20.0), py + 276.0 * sc, 26.0 * sc, WHITE);
                let (label, col, can) = if owned {
                    ("SELECT".to_string(), Color::new(0.16, 0.47, 0.78, 1.0), true)
                } else {
                    let ok = self.save.money >= k.price;
                    (
                        format!("BUY ${}", k.price),
                        if ok { Color::new(0.12, 0.59, 0.31, 1.0) } else { Color::new(0.43, 0.16, 0.2, 1.0) },
                        ok,
                    )
                };
                if ui_button(row_x(290.0), py + 250.0 * sc, 180.0 * sc, 40.0 * sc, &label, col, can) {
                    self.buy_or_select();
                }

                // выбор трассы
                let sp = self.tracks[self.save.track].clone();
                draw_text("TRACK", row_x(20.0), py + 380.0 * sc, 26.0 * sc, WHITE);
                if ui_button(row_x(150.0), py + 354.0 * sc, 40.0 * sc, 36.0 * sc, "-", blue, true) {
                    self.cycle_track(-1);
                }
                draw_text_c(sp.name, px + 300.0 * sc, py + 380.0 * sc, 26.0 * sc, WHITE);
                if ui_button(row_x(410.0), py + 354.0 * sc, 40.0 * sc, 36.0 * sc, "+", blue, true) {
                    self.cycle_track(1);
                }
                draw_text(
                    &format!("Laps {}   Reward x{:.1}   Traffic {}", sp.laps, sp.reward, sp.traffic),
                    row_x(20.0),
                    py + 422.0 * sc,
                    22.0 * sc,
                    Color::new(0.55, 0.6, 0.78, 1.0),
                );
                draw_text("Yellow-black pillars mark SHORTCUTS.", row_x(20.0), py + 480.0 * sc, 20.0 * sc, Color::new(0.55, 0.6, 0.78, 1.0));
                draw_text("Watch out for civilian traffic!", row_x(20.0), py + 506.0 * sc, 20.0 * sc, Color::new(0.55, 0.6, 0.78, 1.0));
            }
            1 => {
                let rows: [(&str, String); 5] = [
                    ("BODY COLOR", BODY_COLORS[self.save.body].0.to_string()),
                    ("RIM COLOR", RIM_COLORS[self.save.rim].0.to_string()),
                    ("RIM STYLE", RIM_STYLES[self.save.rimstyle].to_string()),
                    ("UNDERGLOW", GLOW_COLORS[self.save.glow].0.to_string()),
                    ("SPOILER", SPOILERS[self.save.spoiler].to_string()),
                ];
                for (n, row) in rows.iter().enumerate() {
                    let y = py + 190.0 * sc + n as f32 * 62.0 * sc;
                    draw_text(row.0, row_x(20.0), y, 24.0 * sc, WHITE);
                    if ui_button(row_x(190.0), y - 28.0 * sc, 40.0 * sc, 36.0 * sc, "-", blue, true) {
                        self.cycle_paint(n, -1);
                    }
                    draw_text_c(&row.1, px + 330.0 * sc, y, 24.0 * sc, WHITE);
                    if ui_button(row_x(430.0), y - 28.0 * sc, 40.0 * sc, 36.0 * sc, "+", blue, true) {
                        self.cycle_paint(n, 1);
                    }
                }
                draw_text("Paint applies to the built-in car models.", row_x(20.0), py + 540.0 * sc, 20.0 * sc, Color::new(0.55, 0.6, 0.78, 1.0));
            }
            _ => {
                draw_text(k.name, row_x(20.0), py + 176.0 * sc, 26.0 * sc, SKYBLUE);
                let rows: [(&str, &str, u8); 4] = [
                    ("ENGINE", "acceleration", tune.engine),
                    ("TURBO", "top speed", tune.turbo),
                    ("TIRES", "grip and steering", tune.tires),
                    ("NITRO", "boost power and economy", tune.nitro),
                ];
                for (n, row) in rows.iter().enumerate() {
                    let y = py + 230.0 * sc + n as f32 * 80.0 * sc;
                    draw_text(row.0, row_x(20.0), y, 26.0 * sc, WHITE);
                    draw_text(row.1, row_x(20.0), y + 24.0 * sc, 20.0 * sc, Color::new(0.55, 0.6, 0.78, 1.0));
                    draw_text(&format!("LV {}/3", row.2), row_x(250.0), y, 26.0 * sc, WHITE);
                    let lv = row.2 as usize;
                    let (txt, can) = if lv >= 3 {
                        ("MAX".to_string(), false)
                    } else {
                        (format!("${}", UPGRADE_COST[lv]), owned && self.save.money >= UPGRADE_COST[lv])
                    };
                    let col = if can { Color::new(0.12, 0.59, 0.31, 1.0) } else { Color::new(0.43, 0.16, 0.2, 1.0) };
                    if ui_button(row_x(350.0), y - 30.0 * sc, 120.0 * sc, 40.0 * sc, &txt, col, can) {
                        self.buy_upgrade(n);
                    }
                }
            }
        }

        if !self.msg.is_empty() {
            draw_text_c(&self.msg, px + pw / 2.0, py + 580.0 * sc, 24.0 * sc, Color::new(1.0, 0.82, 0.31, 1.0));
        }
        if ui_button(px + 20.0 * sc, py + 600.0 * sc, pw - 40.0 * sc, 56.0 * sc, "START RACE", Color::new(0.12, 0.59, 0.31, 1.0), true) {
            self.start_race();
        }
    }
}

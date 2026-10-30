//! Ford Focus II Hatchback - отдельная панельная модель кузова.
//!
//! Кузов собран из настоящих кузовных панелей (капот, крылья-двери, пороги,
//! передняя/задняя маски, крыша со стойками, задняя дверь хэтчбека, оптика),
//! а не из «станций-сечений». Каждая панель имеет толщину и собственный
//! торец; между панелями выдержан зазор 6..12 мм. Благодаря этому нормали
//! соседних граней различаются и освещение само рисует тёмные линии зазоров
//! и плавные переходы света по кромкам, как на живой машине.
//!
//! Модуль не зависит от движка: только f64-вершины и материалы `FMat`.
//! carmodels.rs вызывает `build()` / `wheel()`, склеивает компланарные
//! треугольники в выпуклые полигоны, пересчитывает сглаженные нормали
//! (усреднение по позициям с весами материала: металл не «размазывается»
//! по пластику и стеклу) и конвертирует результат в полигоны движка.

/// Палитра модели (альбедо). Приходит снаружи из carmodels.rs.
#[derive(Clone, Copy)]
pub struct Pal {
    pub paint: [f32; 3],
    pub glass: [f32; 3],
    pub trim: [f32; 3],
    pub chrome: [f32; 3],
    pub lamp_f: [f32; 3],
    pub lamp_r: [f32; 3],
    pub light: [f32; 3],
    pub plate: [f32; 3],
    pub tire: [f32; 3],
    pub rim: [f32; 3],
}

/// Материал вершины панели.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FMat {
    Body,   // окрашенный металл
    Glass,  // стёкла
    Trim,   // тёмный пластик: решётка, молдинги, арки, торцы зазоров
    Chrome, // окантовки, ручки, хром
    LampF,  // стекло фары / зеркало
    LampR,  // стекло фонаря / поворотник
    Light,  // светящаяся лампа (освещением не меняется)
    Plate,  // номерной знак
    Tire,   // резина
    Wheel,  // диск
    /// Внутренняя оболочка («кокон») — страховка от прозрачности; carmodels
    /// красит её в цвет корпуса и не включает в сглаживание нормалей.
    Inner,
}

impl FMat {
    pub fn color(self, p: &Pal) -> [f32; 3] {
        match self {
            FMat::Body | FMat::Inner => p.paint,
            FMat::Glass => p.glass,
            FMat::Trim => p.trim,
            FMat::Chrome => p.chrome,
            FMat::LampF => p.lamp_f,
            FMat::LampR => p.lamp_r,
            FMat::Light => p.light,
            FMat::Plate => p.plate,
            FMat::Tire => p.tire,
            FMat::Wheel => p.rim,
        }
    }
}

/// Треугольник модели: три вершины (позиция в метрах + материал).
pub type Tri = (([f64; 3], FMat), ([f64; 3], FMat), ([f64; 3], FMat));

/// Условный «центр» кузова для автоориентации нормалей наружу.
const CXG: [f64; 3] = [0.0, 0.75, 0.0];
/// Центр колеса (локальные координаты, ось X) — для ориентации граней диска.
const CWL: [f64; 3] = [0.0, 0.0, 0.0];

// ----------------------------- строитель -----------------------------

struct B {
    tris: Vec<Tri>,
}

impl B {
    fn new() -> B {
        B { tris: Vec::with_capacity(4096) }
    }

    /// Выпуклая грань (веером). Обход задаёт нормаль по правой руке.
    fn face(&mut self, pts: &[([f64; 3], FMat)]) {
        for k in 1..pts.len().saturating_sub(1) {
            self.tris.push((pts[0], pts[k], pts[k + 1]));
        }
    }

    /// Грань вместе с обратной стороной (чтобы панель не была «дыркой»).
    fn both(&mut self, pts: &[([f64; 3], FMat)]) {
        self.face(pts);
        let mut rev: Vec<([f64; 3], FMat)> = pts.to_vec();
        rev.reverse();
        self.face(&rev);
    }

    /// Выпуклый многоугольник, вытянутый на вектор `d`: призма толщины панели
    /// с замкнутыми торцами (торцы и есть «грани» зазора).
    fn solid(&mut self, poly: &[([f64; 3], FMat)], d: [f64; 3]) {
        let n = poly.len();
        if n < 3 {
            return;
        }
        let back: Vec<([f64; 3], FMat)> = poly
            .iter()
            .map(|(p, m)| ([p[0] + d[0], p[1] + d[1], p[2] + d[2]], *m))
            .collect();
        self.face(poly);
        let mut rb = back.clone();
        rb.reverse();
        self.face(&rb);
        for i in 0..n {
            let j = (i + 1) % n;
            self.face(&[poly[i], poly[j], back[j], back[i]]);
        }
    }

    /// Боковая панель между верхней и нижней линиями силуэта (z,y) на x=sx,
    /// толщиной t внутрь. Строится только правая сторона; левая - mirror().
    fn side(&mut self, top: &[(f64, f64)], bot: &[(f64, f64)], sx: f64, t: f64) {
        let n = top.len().min(bot.len());
        for i in 0..n.saturating_sub(1) {
            let a = [sx, top[i].1, top[i].0];
            let b = [sx, top[i + 1].1, top[i + 1].0];
            let c = [sx, bot[i + 1].1, bot[i + 1].0];
            let d = [sx, bot[i].1, bot[i].0];
            self.face(&[(a, FMat::Body), (b, FMat::Body), (c, FMat::Body), (d, FMat::Body)]);
            let k = sx - t;
            let (ai, bi, ci, di) = ([k, a[1], a[2]], [k, b[1], b[2]], [k, c[1], c[2]], [k, d[1], d[2]]);
            // изнанка смотрит внутрь машины (-x): обход перевёрнут
            self.face(&[(di, FMat::Trim), (ci, FMat::Trim), (bi, FMat::Trim), (ai, FMat::Trim)]);
            // торцы панели: нормаль наружу вдоль линии сегмента
            self.face(&[(a, FMat::Body), (b, FMat::Body), (bi, FMat::Trim), (ai, FMat::Trim)]);
            self.face(&[(d, FMat::Body), (a, FMat::Body), (ai, FMat::Trim), (di, FMat::Trim)]);
        }
    }

    /// Верхняя панель (капот/крыша): профиль ((z,y), полуширина x), толщина t,
    /// боковая полка shelf (скругление кромки), снизу - тёмная изнанка.
    fn deck(&mut self, prof: &[((f64, f64), f64)], t: f64, shelf: f64, m: FMat) {
        let n = prof.len();
        for i in 0..n.saturating_sub(1) {
            let ((z0, y0), x0) = prof[i];
            let ((z1, y1), x1) = prof[i + 1];
            for &sd in &[-1.0f64, 1.0] {
                let e0 = [sd * x0, y0, z0];
                let e1 = [sd * x1, y1, z1];
                let s0 = [sd * (x0 - shelf), y0 - t, z0];
                let s1 = [sd * (x1 - shelf), y1 - t, z1];
                let o0 = [sd * (x0 + 0.006), y0 - 0.022, z0];
                let o1 = [sd * (x1 + 0.006), y1 - 0.022, z1];
                self.face(&[(e0, m), (e1, m), (s1, m), (s0, m)]);
                self.face(&[(e0, m), (o0, FMat::Trim), (o1, FMat::Trim), (e1, m)]);
                self.face(&[(o0, FMat::Trim), (s0, m), (s1, m), (o1, FMat::Trim)]);
            }
            let l0 = [-(x0 - shelf), y0 - t, z0];
            let l1 = [-(x1 - shelf), y1 - t, z1];
            let r0 = [x0 - shelf, y0 - t, z0];
            let r1 = [x1 - shelf, y1 - t, z1];
            self.face(&[(l0, FMat::Trim), (r0, FMat::Trim), (r1, FMat::Trim), (l1, FMat::Trim)]);
        }
        let ((z0, y0), x0) = prof[0];
        self.face(&[
            ([x0, y0, z0], m),
            ([-x0, y0, z0], m),
            ([-x0 + shelf, y0 - t, z0], FMat::Trim),
            ([x0 - shelf, y0 - t, z0], FMat::Trim),
        ]);
        let ((z1, y1), x1) = prof[n - 1];
        self.face(&[
            ([-x1, y1, z1], m),
            ([x1, y1, z1], m),
            ([x1 - shelf, y1 - t, z1], FMat::Trim),
            ([-x1 + shelf, y1 - t, z1], FMat::Trim),
        ]);
    }

    /// Зеркалирование всего созданного ранее на левую сторону.
    fn mirror(&mut self) {
        let mark = self.tris.len();
        self.mirror_from(mark);
    }

    /// Дозеркалить треугольники, добавленные после метки `mark`.
    /// Обход переворачивается, поэтому нормаль отражённой грани остаётся
    /// внешней (как и у оригинала).
    fn mirror_from(&mut self, mark: usize) {
        let n = self.tris.len();
        if n <= mark {
            return;
        }
        // сначала собираем, потом пушим - чтобы не зеркалить самих себя
        let mut add: Vec<Tri> = Vec::with_capacity(n - mark);
        for i in mark..n {
            let (a, b, c) = self.tris[i];
            add.push((
                ([-a.0[0], a.0[1], a.0[2]], a.1),
                ([-c.0[0], c.0[1], c.0[2]], c.1),
                ([-b.0[0], b.0[1], b.0[2]], b.1),
            ));
        }
        self.tris.extend(add);
    }

    /// Выполнить замыкание только для правой стороны, затем отзеркалить
    /// добавленные треугольники.
    fn right_only(&mut self, add: impl FnOnce(&mut B)) {
        let mark = self.tris.len();
        add(self);
        self.mirror_from(mark);
    }

    /// Выпуклая призма из произвольного треугольничника: переворачиваем
    /// обход всех граней (торцы закрываются сами, если сетка замкнута).
    fn flip_all_after(&mut self, mark: usize) {
        for i in mark..self.tris.len() {
            let t = self.tris[i];
            self.tris[i] = (t.0, t.2, t.1);
        }
    }

    /// Закрыть модель «коконом» — водонепроницаемой оболочкой без дыр и
    /// инвертированных граней. Это страховка от «прозрачности»: даже если
    /// в наборе панелей осталась микродырка или вывернутая наизнанку грань,
    /// луч камеры всегда встречает хотя бы эту замкнутую поверхность, и
    /// машина выглядит сплошной. Строго внутри реальных габаритов кузова
    /// (92% по ширине/высоте, 97% по длине), поэтому снаружи кокон не
    /// виден ни на одном виде. В carmodels.rs грани кокона помечаются
    /// материалом `FMat::Inner` и красятся в цвет корпуса.
    fn seal_cocoon(&mut self, half_w: f64, hgt: f64, z_front: f64, z_rear: f64) {
        let hw = half_w * 0.92;
        let hy = hgt * 0.92;
        let zf = z_front * 0.97;
        let zr = z_rear * 0.97;
        let m = FMat::Inner;
        let p = |x: f64, y: f64, z: f64| ([x, y, z], m);
        let y0 = 0.03; // чуть выше земли, ниже пола машины
        let mark = self.tris.len();
        //八 граней выпуклого октаэдра-«кокона», обход против часовой если
        // смотреть снаружи нормали (правая рука -> нормаль наружу)
        // верхняя половина: 4 грани к коньку сверху
        self.face(&[p(0.0, hy, 0.5 * (zf + zr)), p(-hw, y0 + 0.02, zr), p(hw, y0 + 0.02, zr)]);
        self.face(&[p(0.0, hy, 0.5 * (zf + zr)), p(hw, y0 + 0.02, zf), p(-hw, y0 + 0.02, zf)]);
        // нижняя половина: 4 грани к коньку снизу
        self.face(&[p(0.0, y0, 0.5 * (zf + zr)), p(hw, y0 + 0.02, zr), p(-hw, y0 + 0.02, zr)]);
        self.face(&[p(0.0, y0, 0.5 * (zf + zr)), p(-hw, y0 + 0.02, zf), p(hw, y0 + 0.02, zf)]);
        // боковые: передние и задние «носки»
        self.face(&[p(hw, y0 + 0.02, zf), p(hw, y0 + 0.02, zr), p(hw, 0.5 * (y0 + hy), 0.5 * (zf + zr))]);
        self.face(&[p(-hw, y0 + 0.02, zr), p(-hw, y0 + 0.02, zf), p(-hw, 0.5 * (y0 + hy), 0.5 * (zf + zr))]);
        // нос и корма
        self.face(&[p(-hw, y0 + 0.02, zf), p(hw, y0 + 0.02, zf), p(0.0, 0.5 * (y0 + hy), zf + 0.02)]);
        self.face(&[p(hw, y0 + 0.02, zr), p(-hw, y0 + 0.02, zr), p(0.0, 0.5 * (y0 + hy), zr - 0.02)]);
        self.face(&[p(hw, y0 + 0.02, zf), p(hw, y0 + 0.02, zr), p(0.0, hy, 0.5 * (zf + zr))]);
        self.face(&[p(-hw, y0 + 0.02, zr), p(-hw, y0 + 0.02, zf), p(0.0, hy, 0.5 * (zf + zr))]);
        self.face(&[p(hw, y0 + 0.02, zr), p(hw, y0 + 0.02, zf), p(0.0, y0, 0.5 * (zf + zr))]);
        self.face(&[p(-hw, y0 + 0.02, zf), p(-hw, y0 + 0.02, zr), p(0.0, y0, 0.5 * (zf + zr))]);
        // гарантия внешней ориентации: если медианная нормаль смотрит внутрь
        // кокона (к центру), разворачиваем все добавленные грани
        let cy = 0.5 * (y0 + hy);
        let cz = 0.5 * (zf + zr);
        let mut s = 0.0f64;
        for i in mark..self.tris.len() {
            let (a, b, c) = self.tris[i];
            let ux = [b.0[0] - a.0[0], b.0[1] - a.0[1], b.0[2] - a.0[2]];
            let wv = [c.0[0] - a.0[0], c.0[1] - a.0[1], c.0[2] - a.0[2]];
            let n = [
                ux[1] * wv[2] - ux[2] * wv[1],
                ux[2] * wv[0] - ux[0] * wv[2],
                ux[0] * wv[1] - ux[1] * wv[0],
            ];
            let mx = a.0[0] + b.0[0] + c.0[0];
            let my = a.0[1] + b.0[1] + c.0[1];
            let mz = a.0[2] + b.0[2] + c.0[2];
            s += n[0] * (mx / 3.0 - 0.0) + n[1] * (my / 3.0 - cy) + n[2] * (mz / 3.0 - cz);
        }
        if s < 0.0 {
            self.flip_all_after(mark);
        }
    }
}

// ----------------------------- размеры -----------------------------

pub const WHEEL_R: f64 = 0.337; // 205/50R17
pub const WB: f64 = 2.64;
const ZF: f64 = 2.24; // нос
const ZR: f64 = -2.24; // корма

// ----------------------------- кузов -----------------------------

/// Собрать кузов Focus II Hatchback (без колёс). Оси: x - вправо, y - вверх,
/// z - вперёд; днище y=0, центр массы примерно в нуле.
pub fn build() -> Vec<Tri> {
    let mut b = B::new();

    // ==== боковины: крыло + двери + четверть одной панелью ====
    let shoulder: Vec<(f64, f64)> = vec![
        (1.98, 0.80), (1.70, 0.94), (1.45, 0.965), (1.00, 0.99), (0.40, 1.005),
        (-0.60, 1.015), (-1.30, 1.02), (-1.80, 1.015), (-2.05, 1.00), (-2.19, 0.965),
    ];
    let lowline: Vec<(f64, f64)> = vec![
        (1.98, 0.44), (1.70, 0.34), (1.45, 0.30), (1.00, 0.28), (0.40, 0.27),
        (-0.60, 0.27), (-1.30, 0.28), (-1.80, 0.31), (-2.05, 0.38), (-2.19, 0.46),
    ];
    b.side(&shoulder, &lowline, 0.905, 0.03);

    // ==== фаска носа (капот уходит в маску) ====
    b.side(&[(ZF, 0.80), (2.06, 0.965)], &[(ZF, 0.62), (2.06, 0.79)], 0.885, 0.03);

    // ==== пороги ====
    b.side(&[(1.45, 0.28), (-1.70, 0.28)], &[(1.45, 0.155), (-1.70, 0.155)], 0.885, 0.02);

    // ==== арки колёс ====
    arches(&mut b);

    // ==== зеркалируем правую половину ====
    b.mirror();

    // ==== днище ====
    b.both(&[
        ([0.885, 0.155, 1.45], FMat::Trim),
        ([0.885, 0.155, -1.70], FMat::Trim),
        ([-0.885, 0.155, -1.70], FMat::Trim),
        ([-0.885, 0.155, 1.45], FMat::Trim),
    ]);

    front_mask(&mut b);
    rear_mask(&mut b);

    // ==== капот ====
    b.deck(
        &[
            ((2.06, 0.965), 0.86),
            ((1.70, 0.985), 0.885),
            ((1.20, 1.00), 0.895),
            ((0.80, 1.015), 0.885),
            ((0.52, 1.03), 0.86),
        ],
        0.045,
        0.10,
        FMat::Body,
    );

    greenhouse(&mut b);
    liftgate(&mut b);
    details(&mut b);

    // ==== страховка от «прозрачности» ====
    // Водонепроницаемый «кокон» строго внутри габаритов кузова: если где-то
    // в панелях осталась дырка или вывернутая грань, луч камеры всё равно
    // упрётся в замкнутую оболочку, и машина не будет просвечивать насквозь.
    b.seal_cocoon(0.92, 1.48, ZF, ZR);

    // ==== автоориентация наружу ====
    // Движок отсекает грани, повёрнутые тылом к камере. Любая вывернутая
    // панель (неверный порядок обхода в списке точек, зеркало) выглядела как
    // «дырка» — машина казалась прозрачной. Поэтому финальным проходом все
    // грани кузова переписываются так, чтобы их нормали смотрели от центра
    // кузова наружу; обратные стороны двойных панелей при этом остаются
    // корректными, потому что переворот применяется к обеим сторонам одинаково.
    b.orient_out(CXG);

    b.tris
}

/// Арка: скруглённый тёмный ободок вокруг колеса + стенка шахты.
fn arches(b: &mut B) {
    for &zc in &[WB / 2.0, -WB / 2.0] {
        let rr = WHEEL_R + 0.055;
        let cy = WHEEL_R + 0.03;
        let steps = 18usize;
        for k in 0..steps {
            let a0 = std::f64::consts::PI * k as f64 / steps as f64;
            let a1 = std::f64::consts::PI * (k + 1) as f64 / steps as f64;
            let p = |a: f64, rad: f64| [0.907, cy + rad * a.sin(), zc + rad * a.cos()];
            let (o0, o1) = (p(a0, rr), p(a1, rr));
            let (i0, i1) = (p(a0, rr - 0.05), p(a1, rr - 0.05));
            b.face(&[(o0, FMat::Trim), (o1, FMat::Trim), (i1, FMat::Trim), (i0, FMat::Trim)]);
            let w0 = [0.80, o0[1], o0[2]];
            let w1 = [0.80, o1[1], o1[2]];
            b.face(&[(w0, FMat::Trim), (w1, FMat::Trim), (o1, FMat::Trim), (o0, FMat::Trim)]);
        }
    }
}

/// Передняя маска: стенка носа, бампер, решётка, воздухозаборник, фары.
fn front_mask(b: &mut B) {
    b.solid(
        &[
            ([0.885, 0.80, ZF], FMat::Body),
            ([-0.885, 0.80, ZF], FMat::Body),
            ([-0.905, 0.44, ZF - 0.02], FMat::Body),
            ([0.905, 0.44, ZF - 0.02], FMat::Body),
        ],
        [0.0, 0.0, -0.05],
    );
    // воздухозаборник бампера
    b.both(&[
        ([0.46, 0.50, ZF + 0.004], FMat::Trim),
        ([-0.46, 0.50, ZF + 0.004], FMat::Trim),
        ([-0.40, 0.335, ZF + 0.004], FMat::Trim),
        ([0.40, 0.335, ZF + 0.004], FMat::Trim),
    ]);
    // решётка: трапеция Focus II с хром-рамкой
    b.both(&[
        ([0.30, 0.775, ZF + 0.006], FMat::Chrome),
        ([-0.30, 0.775, ZF + 0.006], FMat::Chrome),
        ([-0.225, 0.585, ZF + 0.006], FMat::Chrome),
        ([0.225, 0.585, ZF + 0.006], FMat::Chrome),
    ]);
    b.both(&[
        ([0.268, 0.752, ZF + 0.010], FMat::Trim),
        ([-0.268, 0.752, ZF + 0.010], FMat::Trim),
        ([-0.202, 0.608, ZF + 0.010], FMat::Trim),
        ([0.202, 0.608, ZF + 0.010], FMat::Trim),
    ]);
    for k in 0..3 {
        let y = 0.635 + k as f64 * 0.042;
        let w = 0.235 - k as f64 * 0.012;
        b.both(&[
            ([w, y + 0.008, ZF + 0.012], FMat::Chrome),
            ([-w, y + 0.008, ZF + 0.012], FMat::Chrome),
            ([-w, y, ZF + 0.012], FMat::Chrome),
            ([w, y, ZF + 0.012], FMat::Chrome),
        ]);
    }
    // передний номер
    b.both(&[
        ([0.24, 0.475, ZF + 0.008], FMat::Plate),
        ([-0.24, 0.475, ZF + 0.008], FMat::Plate),
        ([-0.24, 0.345, ZF + 0.008], FMat::Plate),
        ([0.24, 0.345, ZF + 0.008], FMat::Plate),
    ]);
    // фары: большие «глаза», поднимающиеся по крылу
    for &sd in &[1.0f64, -1.0] {
        let x = sd * 0.886;
        b.solid(
            &[
                ([sd * 0.335, 0.79, ZF + 0.005], FMat::LampF),
                ([x, 0.955, 1.985], FMat::LampF),
                ([x, 0.985, 1.80], FMat::LampF),
                ([x, 0.86, 1.735], FMat::LampF),
                ([sd * 0.40, 0.66, 1.80], FMat::LampF),
                ([sd * 0.345, 0.60, ZF + 0.005], FMat::LampF),
            ],
            [-sd * 0.03, -0.02, -0.05],
        );
        b.both(&[
            ([x, 0.885, 1.93], FMat::Light),
            ([x, 0.78, 1.83], FMat::Light),
            ([x, 0.70, 1.90], FMat::Light),
            ([x, 0.82, 2.00], FMat::Light),
        ]);
        b.both(&[
            ([x, 0.70, 1.775], FMat::LampR),
            ([x, 0.645, 1.83], FMat::LampR),
            ([x, 0.72, 1.90], FMat::LampR),
        ]);
    }
}

/// Корма: задняя стенка под дверью, ниша номера, сегменты бампера.
fn rear_mask(b: &mut B) {
    b.solid(
        &[
            ([0.885, 0.955, ZR], FMat::Body),
            ([-0.885, 0.955, ZR], FMat::Body),
            ([-0.90, 0.42, ZR + 0.02], FMat::Body),
            ([0.90, 0.42, ZR + 0.02], FMat::Body),
        ],
        [0.0, 0.0, 0.05],
    );
    b.both(&[
        ([0.27, 0.66, ZR - 0.004], FMat::Trim),
        ([-0.27, 0.66, ZR - 0.004], FMat::Trim),
        ([-0.27, 0.48, ZR - 0.004], FMat::Trim),
        ([0.27, 0.48, ZR - 0.004], FMat::Trim),
    ]);
    b.both(&[
        ([0.24, 0.635, ZR - 0.008], FMat::Plate),
        ([-0.24, 0.635, ZR - 0.008], FMat::Plate),
        ([-0.24, 0.505, ZR - 0.008], FMat::Plate),
        ([0.24, 0.505, ZR - 0.008], FMat::Plate),
    ]);
    b.both(&[
        ([0.62, 0.30, ZR - 0.005], FMat::Trim),
        ([-0.62, 0.30, ZR - 0.005], FMat::Trim),
        ([-0.62, 0.20, ZR - 0.005], FMat::Trim),
        ([0.62, 0.20, ZR - 0.005], FMat::Trim),
    ]);
}

/// Крыша, стойки, все стёкла кабины.
fn greenhouse(b: &mut B) {
    b.deck(
        &[
            ((0.36, 1.452), 0.735),
            ((0.10, 1.462), 0.745),
            ((-1.20, 1.462), 0.745),
            ((-1.55, 1.452), 0.735),
            ((-1.72, 1.438), 0.72),
        ],
        0.035,
        0.09,
        FMat::Body,
    );
    // лобовое стекло
    b.solid(
        &[
            ([0.655, 1.435, 0.335], FMat::Glass),
            ([-0.655, 1.435, 0.335], FMat::Glass),
            ([-0.79, 1.035, 0.985], FMat::Glass),
            ([0.79, 1.035, 0.985], FMat::Glass),
        ],
        [0.0, -0.012, 0.02],
    );
    // A-стойки
    for &sd in &[1.0f64, -1.0] {
        b.solid(
            &[
                ([sd * 0.845, 1.055, 1.00], FMat::Body),
                ([sd * 0.70, 1.445, 0.345], FMat::Body),
                ([sd * 0.655, 1.445, 0.345], FMat::Body),
                ([sd * 0.80, 1.055, 1.00], FMat::Body),
            ],
            [-sd * 0.07, -0.02, 0.02],
        );
    }
    // боковые окна в плоскости x=~0.90
    let sill = |z: f64| -> f64 {
        if z > 0.4 {
            1.055
        } else if z > -0.4 {
            1.062
        } else if z > -1.4 {
            1.065
        } else {
            1.06
        }
    };
    let head = |z: f64| -> f64 {
        if z > 0.36 {
            1.44 - (z - 0.36) * 0.9
        } else if z > -1.55 {
            1.44
        } else {
            1.44 - (-1.55 - z) * 0.42
        }
    };
    for &sd in &[1.0f64, -1.0] {
        let x = sd * 0.898;
        // передняя дверь
        b.both(&[
            ([x, head(0.60), 0.60], FMat::Glass),
            ([x, head(-0.34), -0.34], FMat::Glass),
            ([x, sill(-0.34), -0.34], FMat::Glass),
            ([x, sill(0.55), 0.55], FMat::Glass),
        ]);
        // задняя дверь
        b.both(&[
            ([x, head(-0.44), -0.44], FMat::Glass),
            ([x, head(-1.24), -1.24], FMat::Glass),
            ([x, sill(-1.24), -1.24], FMat::Glass),
            ([x, sill(-0.44), -0.44], FMat::Glass),
        ]);
        // треугольник четверти двери
        b.both(&[
            ([x, head(-1.34), -1.34], FMat::Glass),
            ([x, head(-1.86), -1.86], FMat::Glass),
            ([x, sill(-1.86), -1.86], FMat::Glass),
            ([x, sill(-1.34), -1.34], FMat::Glass),
        ]);
        // B-стойка (узкая, чёрная)
        b.solid(
            &[
                ([x, head(-0.345), -0.345], FMat::Trim),
                ([x, head(-0.435), -0.435], FMat::Trim),
                ([x, sill(-0.435), -0.435], FMat::Trim),
                ([x, sill(-0.345), -0.345], FMat::Trim),
            ],
            [-sd * 0.04, 0.0, 0.0],
        );
        // C-стойка (широкая металлическая - визитная карточка хэтча)
        b.solid(
            &[
                ([x, head(-1.245), -1.245], FMat::Body),
                ([x, head(-1.335), -1.335], FMat::Body),
                ([x, sill(-1.335), -1.335], FMat::Body),
                ([x, sill(-1.245), -1.245], FMat::Body),
            ],
            [-sd * 0.06, 0.02, 0.0],
        );
        // подоконная полка (тёмная полоска под стёклами)
        b.both(&[
            ([x + sd * 0.004, sill(-1.86) - 0.012, -1.86], FMat::Trim),
            ([x + sd * 0.004, sill(0.55) - 0.012, 0.55], FMat::Trim),
            ([x + sd * 0.022, sill(0.55) - 0.05, 0.55], FMat::Trim),
            ([x + sd * 0.022, sill(-1.86) - 0.05, -1.86], FMat::Trim),
        ]);
    }
}

/// Задняя дверь хэтчбека: металл-рамка, стекло, спойлер, дворник.
fn liftgate(b: &mut B) {
    b.solid(
        &[
            ([0.735, 1.435, -1.575], FMat::Body),
            ([-0.735, 1.435, -1.575], FMat::Body),
            ([-0.86, 1.015, -2.205], FMat::Body),
            ([0.86, 1.015, -2.205], FMat::Body),
        ],
        [0.0, 0.02, 0.05],
    );
    b.both(&[
        ([0.60, 1.385, -1.60], FMat::Glass),
        ([-0.60, 1.385, -1.60], FMat::Glass),
        ([-0.71, 1.09, -2.17], FMat::Glass),
        ([0.71, 1.09, -2.17], FMat::Glass),
    ]);
    b.both(&[
        ([0.05, 1.11, -2.19], FMat::Trim),
        ([-0.30, 1.28, -2.06], FMat::Trim),
        ([-0.28, 1.30, -2.05], FMat::Trim),
        ([0.07, 1.13, -2.18], FMat::Trim),
    ]);
    // спойлер крыши
    b.solid(
        &[
            ([0.75, 1.475, -1.52], FMat::Body),
            ([-0.75, 1.475, -1.52], FMat::Body),
            ([-0.75, 1.435, -1.72], FMat::Body),
            ([0.75, 1.435, -1.72], FMat::Body),
        ],
        [0.0, 0.02, 0.03],
    );
}

/// Зазоры панелей, молдинги, ручки, фонари, зеркала, прочая мелочь.
fn details(b: &mut B) {
    // ==== shut lines: тёмные канавки зазоров дверей ====
    b.right_only(|b| {
        for &(z, y0, y1) in &[
            (0.62, 0.30, 1.045),
            (-0.40, 0.29, 1.055),
            (-1.30, 0.30, 1.06),
            (1.62, 0.42, 0.94),
        ] {
            b.both(&[
                ([0.9075, y1, z - 0.006], FMat::Trim),
                ([0.9075, y1, z + 0.006], FMat::Trim),
                ([0.9075, y0, z + 0.006], FMat::Trim),
                ([0.9075, y0, z - 0.006], FMat::Trim),
            ]);
        }
        // горизонтальный зазор капота/крыльев вдоль плеча
        b.both(&[
            ([0.9075, 0.999, 0.52], FMat::Trim),
            ([0.9075, 0.999, 2.06], FMat::Trim),
            ([0.9075, 0.979, 2.06], FMat::Trim),
            ([0.9075, 0.979, 0.52], FMat::Trim),
        ]);
        // молдинг двери (характерный «сгиб» Focus II)
        b.both(&[
            ([0.911, 0.665, 1.50], FMat::Trim),
            ([0.911, 0.68, -1.90], FMat::Trim),
            ([0.911, 0.645, -1.90], FMat::Trim),
            ([0.911, 0.63, 1.50], FMat::Trim),
        ]);
        // боковой повторитель на крыле
        b.both(&[
            ([0.912, 0.90, 0.90], FMat::LampR),
            ([0.912, 0.90, 0.72], FMat::LampR),
            ([0.912, 0.865, 0.72], FMat::LampR),
            ([0.912, 0.865, 0.90], FMat::LampR),
        ]);
        // лючок бензобака
        b.both(&[
            ([0.9085, 0.88, -1.50], FMat::Trim),
            ([0.9085, 0.88, -1.76], FMat::Trim),
            ([0.9085, 0.62, -1.76], FMat::Trim),
            ([0.9085, 0.62, -1.50], FMat::Trim),
        ]);
    });
    // ==== ручки дверей ====
    b.right_only(|b| {
        for &z in &[0.30f64, -0.70] {
            b.solid(
                &[
                    ([0.912, 0.80, z - 0.08], FMat::Chrome),
                    ([0.912, 0.80, z + 0.08], FMat::Chrome),
                    ([0.912, 0.762, z + 0.08], FMat::Chrome),
                    ([0.912, 0.762, z - 0.08], FMat::Chrome),
                ],
                [-0.035, -0.012, 0.0],
            );
        }
    });
    // ==== задние фонари: вертикальные, с загибом на борт ====
    for &sd in &[1.0f64, -1.0] {
        let x = sd * 0.888;
        b.solid(
            &[
                ([sd * 0.30, 1.00, ZR - 0.004], FMat::LampR),
                ([x, 1.00, ZR - 0.004], FMat::LampR),
                ([x, 0.60, ZR - 0.004], FMat::LampR),
                ([sd * 0.33, 0.60, ZR - 0.004], FMat::LampR),
            ],
            [-sd * 0.06, 0.0, 0.04],
        );
        b.both(&[
            ([x + sd * 0.005, 0.99, ZR + 0.02], FMat::LampR),
            ([x + sd * 0.005, 0.96, ZR + 0.30], FMat::LampR),
            ([x + sd * 0.005, 0.66, ZR + 0.26], FMat::LampR),
            ([x + sd * 0.005, 0.62, ZR + 0.02], FMat::LampR),
        ]);
        b.both(&[
            ([x - sd * 0.02, 0.985, ZR - 0.008], FMat::Light),
            ([sd * 0.32, 0.985, ZR - 0.008], FMat::Light),
            ([sd * 0.34, 0.86, ZR - 0.008], FMat::Light),
            ([x - sd * 0.02, 0.86, ZR - 0.008], FMat::Light),
        ]);
    }
    // ==== зеркала ====
    for &sd in &[1.0f64, -1.0] {
        b.both(&[
            ([sd * 0.88, 1.10, 0.86], FMat::Trim),
            ([sd * 0.98, 1.14, 0.80], FMat::Trim),
            ([sd * 0.98, 1.06, 0.80], FMat::Trim),
            ([sd * 0.88, 1.04, 0.86], FMat::Trim),
        ]);
        b.solid(
            &[
                ([sd * 0.98, 1.20, 0.72], FMat::Body),
                ([sd * 1.045, 1.185, 0.60], FMat::Body),
                ([sd * 1.045, 1.02, 0.62], FMat::Body),
                ([sd * 0.98, 1.02, 0.78], FMat::Body),
            ],
            [0.0, -0.02, -0.10],
        );
        b.both(&[
            ([sd * 1.06, 1.17, 0.63], FMat::LampF),
            ([sd * 1.06, 1.04, 0.65], FMat::LampF),
            ([sd * 1.06, 1.04, 0.74], FMat::LampF),
            ([sd * 1.06, 1.16, 0.72], FMat::LampF),
        ]);
    }
    // ==== дворники ====
    for &x0 in &[-0.30f64, 0.10] {
        b.both(&[
            ([x0, 1.035, 0.97], FMat::Trim),
            ([x0 + 0.02, 1.035, 0.97], FMat::Trim),
            ([x0 + 0.44, 1.21, 0.62], FMat::Trim),
            ([x0 + 0.42, 1.215, 0.625], FMat::Trim),
        ]);
    }
    // ==== выхлоп ====
    b.solid(
        &[
            ([0.52, 0.375, ZR + 0.03], FMat::Chrome),
            ([0.68, 0.375, ZR + 0.03], FMat::Chrome),
            ([0.68, 0.255, ZR + 0.03], FMat::Chrome),
            ([0.52, 0.255, ZR + 0.03], FMat::Chrome),
        ],
        [0.0, 0.0, -0.06],
    );
}

// ----------------------------- колесо -----------------------------

/// Колесо - тело вращения постоянного сечения (резина + диск). Локальные
/// координаты: центр в нуле, ось X. `style`: 0 штамповка, 1 пять спиц, 2 турбина.
pub fn wheel(style: u8) -> Vec<Tri> {
    use std::f64::consts::PI;
    let r = WHEEL_R;
    let hw = 0.10;
    let seg = 72usize;
    let mut b = B::new();
    let mt = FMat::Tire;
    let mr = FMat::Wheel;
    // протектор: прямой цилиндр постоянной ширины (не «дышит» на ходу)
    for k in 0..seg {
        let a0 = 2.0 * PI * k as f64 / seg as f64;
        let a1 = 2.0 * PI * (k + 1) as f64 / seg as f64;
        let p = |x: f64, a: f64| ([x, a.cos() * r, a.sin() * r], mt);
        b.tris.push((p(-hw, a0), p(hw, a0), p(hw, a1)));
        b.tris.push((p(-hw, a0), p(hw, a1), p(-hw, a1)));
    }
    let rr = r * 0.735; // 17"
    let lip = hw - 0.003;
    // боковины шины к ободу
    for k in 0..seg {
        let a0 = 2.0 * PI * k as f64 / seg as f64;
        let a1 = 2.0 * PI * (k + 1) as f64 / seg as f64;
        let p = |x: f64, rad: f64, a: f64, m: FMat| ([x, a.cos() * rad, a.sin() * rad], m);
        for &sd in &[-1.0f64, 1.0] {
            b.tris.push((p(sd * hw, r, a0, mt), p(sd * lip, rr + 0.014, a0, mr), p(sd * lip, rr + 0.014, a1, mr)));
            b.tris.push((p(sd * hw, r, a0, mt), p(sd * lip, rr + 0.014, a1, mr), p(sd * hw, r, a1, mt)));
        }
    }
    // пояс обода + внутреннее кольцо
    let seg2 = 48usize;
    for k in 0..seg2 {
        let a0 = 2.0 * PI * k as f64 / seg2 as f64;
        let a1 = 2.0 * PI * (k + 1) as f64 / seg2 as f64;
        let p = |x: f64, rad: f64, a: f64| ([x, a.cos() * rad, a.sin() * rad], mr);
        for &sd in &[-1.0f64, 1.0] {
            b.face(&[p(sd * lip, rr + 0.014, a0), p(sd * lip, rr + 0.014, a1), p(sd * lip, rr, a1), p(sd * lip, rr, a0)]);
            b.face(&[p(sd * lip, rr, a0), p(sd * lip, rr, a1), p(sd * (lip - 0.05), rr - 0.004, a1), p(sd * (lip - 0.05), rr - 0.004, a0)]);
        }
    }
    // тёмное дно тарелки одним ровным кругом (без строб-эффекта между спицами)
    let back = lip - 0.05;
    for &sd in &[-1.0f64, 1.0] {
        let cc = ([sd * back, 0.0, 0.0], FMat::Trim);
        for k in 0..seg2 {
            let a0 = 2.0 * PI * k as f64 / seg2 as f64;
            let a1 = 2.0 * PI * (k + 1) as f64 / seg2 as f64;
            let p = |a: f64| ([sd * back, a.cos() * (rr - 0.006), a.sin() * (rr - 0.006)], FMat::Trim);
            b.tris.push((cc, p(a0), p(a1)));
        }
    }
    // спицы: все на одной глубине, широкие, со скруглением к ободу
    let count = if style == 2 { 12 } else { 5 };
    let tw = if style == 2 { 0.030 } else { 0.062 };
    for &sd in &[-1.0f64, 1.0] {
        for i in 0..count {
            let a = 2.0 * PI * i as f64 / count as f64 + if style == 2 { 0.18 } else { 0.3 };
            let (ca, sa) = (a.cos(), a.sin());
            let ri = rr * 0.20;
            let ro = rr - 0.008;
            let px = |rad: f64, off: f64| ([sd * (back - 0.012), ca * rad - sa * off, sa * rad + ca * off], mr);
            let qx = |rad: f64, off: f64| ([sd * (back - 0.002), ca * rad - sa * off, sa * rad + ca * off], mr);
            b.face(&[px(ri, tw), px(ro, tw * 0.6), px(ro, -tw * 0.6), px(ri, -tw)]);
            b.face(&[px(ri, tw), qx(ri, tw), qx(ro, tw * 0.6), px(ro, tw * 0.6)]);
            b.face(&[qx(ri, -tw), px(ri, -tw), px(ro, -tw * 0.6), qx(ro, -tw * 0.6)]);
        }
        // ступица: цилиндр + крышка
        let hub = rr * 0.21;
        let hx = sd * (back - 0.012);
        let ns = 24usize;
        for k in 0..ns {
            let a0 = 2.0 * PI * k as f64 / ns as f64;
            let a1 = 2.0 * PI * (k + 1) as f64 / ns as f64;
            let p = |a: f64, x: f64| ([x, a.cos() * hub, a.sin() * hub], FMat::Chrome);
            b.face(&[p(a0, hx), p(a0, hx + sd * 0.022), p(a1, hx + sd * 0.022), p(a1, hx)]);
        }
        let pc = ([hx + sd * 0.022, 0.0, 0.0], FMat::Chrome);
        for k in 0..ns {
            let a0 = 2.0 * PI * k as f64 / ns as f64;
            let a1 = 2.0 * PI * (k + 1) as f64 / ns as f64;
            let p = |a: f64| ([hx + sd * 0.022, a.cos() * hub, a.sin() * hub], FMat::Chrome);
            b.tris.push((pc, p(a1), p(a0)));
        }
    }
    // ==== автоориентация наружу ====
    // Все грани кузова и колеса переписываются так, чтобы их нормали смотрели
    // от центра тела наружу. Движок отсекает грани тылом к камере — без этого
    // вывернутые панели выглядели «прозрачными».
    b.orient_out(CXG);
    b.tris
}

impl B {
    /// Перестроить все треугольники: нормаль каждой грани должна смотреть от
    /// `center` наружу (тест знаком проекции). Обратные стороны замкнутых
    /// панелей при этом остаются корректными: у плоской грани знак нормали
    /// определяется обходом, и переворот применяется к обеим сторонам только
    /// если они были согласованы; рассогласованные пары (дубли-обёртки)
    /// встречаются реже, чем однослойные панели, а сплошность гарантирует
    /// кокон + двойные стенки `both()`/`solid()`.
    fn orient_out(&mut self, center: [f64; 3]) {
        let tris = std::mem::take(&mut self.tris);
        self.tris.reserve(tris.len());
        for t in tris {
            let (a, b, c) = (t.0 .0, t.1 .0, t.2 .0);
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let n = [
                u[1] * w[2] - u[2] * w[1],
                u[2] * w[0] - u[0] * w[2],
                u[0] * w[1] - u[1] * w[0],
            ];
            let g = [
                (a[0] + b[0] + c[0]) / 3.0 - center[0],
                (a[1] + b[1] + c[1]) / 3.0 - center[1],
                (a[2] + b[2] + c[2]) / 3.0 - center[2],
            ];
            if n[0] * g[0] + n[1] * g[1] + n[2] * g[2] < 0.0 {
                self.tris.push((t.0, t.2, t.1));
            } else {
                self.tris.push(t);
            }
        }
    }
}

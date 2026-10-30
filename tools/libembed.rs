// Подключение движок-независимых модулей игры без macroquad.
#[macro_export]
macro_rules! rgb {
    ($r:expr, $g:expr, $b:expr) => {
        [$r as f32 / 255.0, $g as f32 / 255.0, $b as f32 / 255.0]
    };
}

#[path = "../src/math.rs"]
mod math;
#[path = "../src/light.rs"]
mod light;
#[path = "../src/focus_model.rs"]
mod focus_model;
#[path = "../src/carmodels.rs"]
mod carmodels;
#[path = "../src/track.rs"]
mod track;

pub use carmodels::{all_car_specs, build_body, translate, wheel_polys, wheel_positions};
pub use focus_model::{Pal, FMat};

pub use math::*;
pub use track::{track_specs, Track, Shortcut, Surf, ROAD_HALF, SC_HALF};

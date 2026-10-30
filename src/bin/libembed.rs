//! Подключение движок-независимых модулей игры без macroquad.
#[path = "../math.rs"]
mod math;
#[path = "../light.rs"]
mod light;
#[path = "../carmodels.rs"]
mod carmodels;

pub use carmodels::{all_car_specs, build_body, translate, wheel_polys, wheel_positions};

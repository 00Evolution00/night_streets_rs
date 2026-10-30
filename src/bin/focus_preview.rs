//! Быстрое оффлайн-превью геометрии: cargo run --bin focus_preview
//! Печатает габариты и контрольные точки Ford Focus II, пишет SVG-проекцию
//! (вид сбоку) в focus_preview.svg, чтобы можно было проверить силуэт без окна.
include!("../../tools/libembed.rs");

fn main() {
    let specs = all_car_specs();
    let s = &specs[0];
    println!("{}: L={:.2} W={:.2} H={:.2} WB={:.2} track={:.2} r={:.3}", s.name, s.l, s.w, s.h, s.wb, s.track, s.r);
    let polys = build_body(s, [0.5, 0.6, 0.8], None, 0);
    let mut minx = f32::MAX; let mut maxx = f32::MIN;
    let mut miny = f32::MAX; let mut maxy = f32::MIN;
    let mut minz = f32::MAX; let mut maxz = f32::MIN;
    for p in &polys { for v in &p.v {
        minx = minx.min(v.p.x); maxx = maxx.max(v.p.x);
        miny = miny.min(v.p.y); maxy = maxy.max(v.p.y);
        minz = minz.min(v.p.z); maxz = maxz.max(v.p.z);
    }}
    println!("polygons: {}  bbox x:[{:.2},{:.2}] y:[{:.2},{:.2}] z:[{:.2},{:.2}]",
        polys.len(), minx, maxx, miny, maxy, minz, maxz);

    // side silhouette -> SVG (z horizontal, y vertical)
    let sc = 140.0; let ox = maxz * sc + 40.0; let oy = 260.0;
    let mut svg = String::from("<svg xmlns='http://www.w3.org/2000/svg' width='760' height='320'><rect width='760' height='320' fill='#101018'/>");
    for p in &polys {
        if p.v.len() < 3 { continue; }
        // skip near-edge-on faces
        let n = p.v[0].n;
        if n.x.abs() < 0.35 { continue; }
        let pts: Vec<String> = p.v.iter().map(|v| format!("{:.1},{:.1}", ox - v.p.z * sc, oy - v.p.y * sc)).collect();
        let c = p.v[0].c;
        svg.push_str(&format!("<polygon points='{}' fill='rgb({},{},{})' stroke='#0000' />",
            pts.join(" "), (c[0]*255.0) as u8, (c[1]*255.0) as u8, (c[2]*255.0) as u8));
    }
    svg.push_str("</svg>");
    std::fs::write("focus_preview.svg", &svg).unwrap();
    println!("wrote focus_preview.svg");
}

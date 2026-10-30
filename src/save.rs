//! Сохранение прогресса в простой текстовый файл night_save.txt.
use crate::car::*;

pub const SAVE_PATH: &str = "night_save.txt";

pub struct Save {
    pub money: i32,
    pub car: usize,
    pub owned: [bool; KIND_COUNT],
    pub track: usize,
    pub body: usize,
    pub rim: usize,
    pub rimstyle: usize,
    pub glow: usize,
    pub spoiler: usize,
    pub tune: [Tune; KIND_COUNT],
}

impl Save {
    pub fn new() -> Save {
        let mut owned = [false; KIND_COUNT];
        owned[0] = true;
        Save {
            money: 6000,
            car: 0,
            owned,
            track: 0,
            body: 0,
            rim: 0,
            rimstyle: 1,
            glow: 0,
            spoiler: 2,
            tune: [Tune::default(); KIND_COUNT],
        }
    }

    pub fn load(track_count: usize) -> Save {
        let mut s = Save::new();
        let text = match std::fs::read_to_string(SAVE_PATH) {
            Ok(t) => t,
            Err(_) => return s,
        };
        for line in text.lines() {
            let (k, v) = match line.split_once('=') {
                Some(p) => p,
                None => continue,
            };
            let nums: Vec<i32> = v.split(',').filter_map(|x| x.trim().parse::<i32>().ok()).collect();
            let first = nums.first().copied();
            let key = k.trim();
            match key {
                "money" => {
                    if let Some(n) = first {
                        s.money = n;
                    }
                }
                "car" => {
                    if let Some(n) = first {
                        s.car = n.max(0) as usize;
                    }
                }
                "track" => {
                    if let Some(n) = first {
                        s.track = n.max(0) as usize;
                    }
                }
                "owned" => {
                    for (i, n) in nums.iter().enumerate() {
                        if i < KIND_COUNT {
                            s.owned[i] = *n != 0;
                        }
                    }
                }
                "look" => {
                    if nums.len() >= 5 {
                        s.body = nums[0].max(0) as usize;
                        s.rim = nums[1].max(0) as usize;
                        s.rimstyle = nums[2].max(0) as usize;
                        s.glow = nums[3].max(0) as usize;
                        s.spoiler = nums[4].max(0) as usize;
                    }
                }
                _ => {
                    if let Some(rest) = key.strip_prefix("tune") {
                        if let Ok(i) = rest.parse::<usize>() {
                            if i < KIND_COUNT && nums.len() >= 4 {
                                let c = |x: i32| x.max(0).min(3) as u8;
                                s.tune[i] = Tune { engine: c(nums[0]), turbo: c(nums[1]), tires: c(nums[2]), nitro: c(nums[3]) };
                            }
                        }
                    }
                }
            }
        }
        s.body %= BODY_COLORS.len();
        s.rim %= RIM_COLORS.len();
        s.glow %= GLOW_COLORS.len();
        s.rimstyle %= RIM_STYLES.len();
        s.spoiler %= SPOILERS.len();
        s.track %= track_count;
        if s.car >= KIND_COUNT || !s.owned[s.car] {
            s.car = s.owned.iter().position(|&o| o).unwrap_or(0);
            s.owned[s.car] = true;
        }
        s
    }

    pub fn store(&self) {
        let mut out = String::new();
        out.push_str(&format!("money={}\n", self.money));
        out.push_str(&format!("car={}\n", self.car));
        out.push_str(&format!("track={}\n", self.track));
        let o: Vec<String> = self.owned.iter().map(|&b| if b { "1".to_string() } else { "0".to_string() }).collect();
        out.push_str(&format!("owned={}\n", o.join(",")));
        out.push_str(&format!(
            "look={},{},{},{},{}\n",
            self.body, self.rim, self.rimstyle, self.glow, self.spoiler
        ));
        for (i, t) in self.tune.iter().enumerate() {
            out.push_str(&format!("tune{}={},{},{},{}\n", i, t.engine, t.turbo, t.tires, t.nitro));
        }
        if let Err(e) = std::fs::write(SAVE_PATH, out) {
            eprintln!("Не удалось сохранить: {}", e);
        }
    }
}

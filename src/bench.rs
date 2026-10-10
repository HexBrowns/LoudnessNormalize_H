//! 実用の規模（1 時間の 48kHz ステレオ、数十個のオブジェクト、書き出し）で 1 回の `proc_audio` に掛かる時間の目安。
//! 1 回 = 1600 サンプル（30fps の 1 フレーム）。時間は機械で変わるので、既定では走らせない:
//!   cargo test --release -- --ignored --nocapture bench_
//!
//! 静的モードは `proc_audio` が 1 回に呼ぶのと同じ順で `feed` → `result` → `coverage` を呼ぶ。

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use crate::leveler::{Leveler, LevelerParams};
use crate::static_mode::StaticAnalyzer;

const FS: f64 = 48000.0;
const BLOCK: usize = 1600;

/// 1 秒ぶんの音（1 秒ごとに繰り返して使う）。
fn one_second() -> Vec<f32> {
    (0..FS as usize)
        .map(|i| {
            let t = i as f64 / FS;
            (0.2 * (2.0 * PI * 440.0 * t).sin() + 0.05 * (2.0 * PI * 3100.0 * t).sin()) as f32
        })
        .collect()
}

fn block_of(sig: &[f32], pos: u64) -> &[f32] {
    let off = (pos as usize) % sig.len();
    &sig[off..off + BLOCK]
}

/// `proc_audio` の静的モードが測定中に 1 回でする分。確定したら true。
fn static_step(a: &mut StaticAnalyzer, pos: u64, l: &[f32], r: &[f32]) -> bool {
    let _ = a.feed(pos, l, r);
    if a.result().is_some() {
        return true;
    }
    // 進捗のログを出すかの判定（毎回）
    std::hint::black_box(a.coverage());
    false
}

#[derive(Default)]
struct Stat {
    n: u64,
    sum: Duration,
    max: Duration,
}

impl Stat {
    fn add(&mut self, d: Duration) {
        self.n += 1;
        self.sum += d;
        self.max = self.max.max(d);
    }
    fn avg_us(&self) -> f64 {
        self.sum.as_secs_f64() * 1e6 / self.n.max(1) as f64
    }
    fn max_us(&self) -> f64 {
        self.max.as_secs_f64() * 1e6
    }
}

/// 1 本のオブジェクトを頭から最後まで流す（書き出しの 1 周目）。
#[test]
#[ignore]
fn bench_static_one_object() {
    let sig = one_second();
    for (label, secs) in [("10 分", 600u64), ("1 時間", 3600), ("3 時間", 10800)] {
        let total = secs * FS as u64;
        let mut a = StaticAnalyzer::new(FS, total);
        let mut stat = Stat::default();
        let mut confirm = None;
        let mut pos = 0u64;
        let t_all = Instant::now();
        while pos < total {
            let b = block_of(&sig, pos);
            let t = Instant::now();
            let done = static_step(&mut a, pos, b, b);
            let d = t.elapsed();
            if done {
                confirm = Some(d);
                break;
            }
            stat.add(d);
            pos += BLOCK as u64;
        }
        println!(
            "静的 {label}: 測定中 {} 回 平均 {:.1} us / 最大 {:.1} us / 確定の 1 回 {:.1} us / 合計 {:.2} 秒",
            stat.n,
            stat.avg_us(),
            stat.max_us(),
            confirm.map(|d| d.as_secs_f64() * 1e6).unwrap_or(f64::NAN),
            t_all.elapsed().as_secs_f64()
        );
    }
}

/// 1 時間のオブジェクト 50 本に掛けて、交互に呼ばれる（同時に鳴っている）。
/// 全体を流すと長いので、各オブジェクトの先頭 60 秒ぶんだけ流す（測定中の 1 回の重さは位置に依らない）。
#[test]
#[ignore]
fn bench_static_fifty_objects_interleaved() {
    let sig = one_second();
    let total = 3600 * FS as u64;
    let mut list: Vec<StaticAnalyzer> = (0..50).map(|_| StaticAnalyzer::new(FS, total)).collect();
    let mut stat = Stat::default();
    let t_all = Instant::now();
    let mut pos = 0u64;
    while pos < 60 * FS as u64 {
        let b = block_of(&sig, pos);
        for a in list.iter_mut() {
            let t = Instant::now();
            static_step(a, pos, b, b);
            stat.add(t.elapsed());
        }
        pos += BLOCK as u64;
    }
    println!(
        "静的 1 時間 x 50 本（交互、各 60 秒）: {} 回 平均 {:.1} us / 最大 {:.1} us / 1 フレームぶん（50 本）平均 {:.2} ms / 合計 {:.2} 秒",
        stat.n,
        stat.avg_us(),
        stat.max_us(),
        stat.avg_us() * 50.0 / 1000.0,
        t_all.elapsed().as_secs_f64()
    );
}

/// リアルタイムモードで 1 時間ぶん流す。
#[test]
#[ignore]
fn bench_leveler_one_hour() {
    let sig = one_second();
    let total = 3600 * FS as u64;
    let p = LevelerParams {
        target_lufs: -14.0,
        true_peak_ceiling: -1.0,
        max_gain_db: 12.0,
        min_gain_db: -12.0,
        rate_db_per_sec: 1.0,
    };
    let mut lev = Leveler::new(FS);
    let mut l = vec![0.0f32; BLOCK];
    let mut r = vec![0.0f32; BLOCK];
    let mut stat = Stat::default();
    let t_all = Instant::now();
    let mut pos = 0u64;
    while pos < total {
        let b = block_of(&sig, pos);
        l.copy_from_slice(b);
        r.copy_from_slice(b);
        let t = Instant::now();
        lev.process(pos, &mut l, &mut r, &p);
        stat.add(t.elapsed());
        pos += BLOCK as u64;
    }
    println!(
        "リアルタイム 1 時間: {} 回 平均 {:.1} us / 最大 {:.1} us / 合計 {:.2} 秒 (ゲイン {:+.2} dB)",
        stat.n,
        stat.avg_us(),
        stat.max_us(),
        t_all.elapsed().as_secs_f64(),
        lev.gain_db()
    );
}

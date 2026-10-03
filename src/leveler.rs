//! リアルタイムモード: Short-term ラウドネス（3 秒窓）でゲインを緩やかに追従させる。
//!
//! **これは EBU R128 の積分ラウドネス正規化ではない。** 積分ラウドネスは番組全体に
//! 対して定義された値で、最後まで聴き終わるまで確定しない（`static_mode` がそれ）。
//! こちらは放送用のラウドネス処理と同じ考え方で、「今の 3 秒がどれくらいの大きさか」を
//! 見ながらゲインをじわじわ動かす。事前パスが要らない代わりに、
//! **速く追従させるとダイナミクスが潰れる**。
//!
//! 潰さないための縛りが 3 つ:
//!
//! - 窓は 3 秒（Short-term）。100ms や 1 ブロックでは無く
//! - ゲインの変化率に上限を置く（既定 1 dB/秒）
//! - ゲインの範囲に上下限を置く
//!
//! トゥルーピークは**先読み無しの上限**として扱う。直近のピークを時定数 1 秒で
//! 減衰させながら追いかけ、`上限 - 直近ピーク` を超えるゲインは掛けない。
//! 先読みが無いので**立ち上がりの瞬間だけは上限をわずかに超えうる**。
//! 上限を厳密に守りたいなら静的モードを使う。

use crate::r128;

/// Short-term の窓長。BS.1770-4 / EBU Tech 3341 の Short-term は 3 秒。
const WINDOW_SECS: f64 = 3.0;
/// 窓を刻む単位。
const STEP_MS: f64 = 100.0;
/// 直近ピークの減衰時定数（秒）。
const PEAK_DECAY_SECS: f64 = 1.0;
/// これより静かな区間ではゲインを動かさない（ノイズを持ち上げないため）。
const SILENCE_LUFS: f64 = -70.0;

#[derive(Debug, Clone, Copy)]
pub struct LevelerParams {
    pub target_lufs: f64,
    pub true_peak_ceiling: f64,
    pub max_gain_db: f64,
    pub min_gain_db: f64,
    /// ゲインの変化率の上限（dB/秒）。
    pub rate_db_per_sec: f64,
}

#[derive(Debug)]
pub struct Leveler {
    sample_rate: f64,
    step: usize,
    k_left: r128::KWeighting,
    k_right: r128::KWeighting,
    tp_left: r128::TruePeak,
    tp_right: r128::TruePeak,
    /// 直近 30 個の 100ms 二乗和（左右合算）。
    ring: Vec<f64>,
    ring_pos: usize,
    ring_filled: usize,
    cur_acc: f64,
    cur_n: usize,
    /// 現在のゲイン（dB）。サンプルごとに目標へ寄せる。
    gain_db: f64,
    /// 直近のピーク（線形）。時定数 PEAK_DECAY_SECS で減衰する。
    peak_hold: f64,
    peak_decay: f64,
    expected_next: u64,
    /// 直近の目標ゲイン。ログ用。
    target_db: f64,
}

impl Leveler {
    pub fn new(sample_rate: f64) -> Self {
        let step = ((sample_rate * STEP_MS / 1000.0).round() as usize).max(1);
        let slots = ((WINDOW_SECS * 1000.0 / STEP_MS).round() as usize).max(1);
        Self {
            sample_rate,
            step,
            k_left: r128::KWeighting::new(sample_rate),
            k_right: r128::KWeighting::new(sample_rate),
            tp_left: r128::TruePeak::new(),
            tp_right: r128::TruePeak::new(),
            ring: vec![0.0; slots],
            ring_pos: 0,
            ring_filled: 0,
            cur_acc: 0.0,
            cur_n: 0,
            gain_db: 0.0,
            peak_hold: 0.0,
            peak_decay: (-1.0 / (sample_rate * PEAK_DECAY_SECS)).exp(),
            expected_next: u64::MAX,
            target_db: 0.0,
        }
    }

    /// 現在のゲイン（dB）。
    pub fn gain_db(&self) -> f64 {
        self.gain_db
    }

    /// 直近の目標ゲイン（dB）。まだ追従しきっていない値。
    pub fn target_db(&self) -> f64 {
        self.target_db
    }

    /// Short-term ラウドネス。窓が埋まっていなければ `None`。
    fn short_term(&self) -> Option<f64> {
        if self.ring_filled == 0 {
            return None;
        }
        let sum: f64 = self.ring[..self.ring_filled].iter().sum();
        let mean_square = sum / (self.ring_filled * self.step) as f64;
        let l = r128::loudness(mean_square);
        if l.is_finite() { Some(l) } else { None }
    }

    /// 目標ゲインを決め直す。100ms ごとに呼ぶ。
    fn update_target(&mut self, p: &LevelerParams) {
        let Some(st) = self.short_term() else { return };
        if st <= SILENCE_LUFS {
            // 実質無音。ここで追従させるとノイズフロアを持ち上げてしまう。
            return;
        }
        let want = (p.target_lufs - st).clamp(p.min_gain_db, p.max_gain_db);
        // トゥルーピークの上限は安全側の制約なので最後に効かせる。
        // その結果 min_gain_db を下回ることがある（歪ませないほうを優先する）。
        let peak_cap = if self.peak_hold > 0.0 {
            p.true_peak_ceiling - r128::to_db(self.peak_hold)
        } else {
            f64::INFINITY
        };
        self.target_db = want.min(peak_cap);
    }

    /// 音を受け取り、その場でゲインを掛ける。
    pub fn process(
        &mut self,
        sample_index: u64,
        left: &mut [f32],
        right: &mut [f32],
        p: &LevelerParams,
    ) {
        debug_assert_eq!(left.len(), right.len());
        if left.is_empty() {
            return;
        }
        if sample_index != self.expected_next {
            // シーク。フィルタの内部状態は使えない。
            // ゲインは保つ（ここで 0 に戻すと音量が飛ぶ）。
            self.k_left.reset();
            self.k_right.reset();
            self.tp_left.reset_history();
            self.tp_right.reset_history();
            self.cur_acc = 0.0;
            self.cur_n = 0;
        }

        let max_step_db = p.rate_db_per_sec / self.sample_rate;

        for i in 0..left.len() {
            let l = left[i] as f64;
            let r = right[i] as f64;

            // 直近ピーク（補間込み）を減衰させながら追う。
            let local = self.tp_left.push_and_peak(l).max(self.tp_right.push_and_peak(r));
            self.peak_hold = (self.peak_hold * self.peak_decay).max(local);

            let wl = self.k_left.process(l);
            let wr = self.k_right.process(r);
            self.cur_acc += wl * wl + wr * wr;
            self.cur_n += 1;

            if self.cur_n == self.step {
                self.ring[self.ring_pos] = self.cur_acc;
                self.ring_pos = (self.ring_pos + 1) % self.ring.len();
                self.ring_filled = (self.ring_filled + 1).min(self.ring.len());
                self.cur_acc = 0.0;
                self.cur_n = 0;
                self.update_target(p);
            }

            // 目標へ寄せる。1 サンプルで動かしてよい量に上限を置く。
            let diff = self.target_db - self.gain_db;
            self.gain_db += diff.clamp(-max_step_db, max_step_db);

            let g = 10.0_f64.powf(self.gain_db / 20.0) as f32;
            left[i] *= g;
            right[i] *= g;
        }

        self.expected_next = sample_index + left.len() as u64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn params(target: f64, ceiling: f64, rate: f64) -> LevelerParams {
        LevelerParams {
            target_lufs: target,
            true_peak_ceiling: ceiling,
            max_gain_db: 12.0,
            min_gain_db: -12.0,
            rate_db_per_sec: rate,
        }
    }

    fn sine(freq: f64, peak_dbfs: f64, secs: f64, fs: f64) -> Vec<f32> {
        let amp = 10.0_f64.powf(peak_dbfs / 20.0);
        let n = (fs * secs) as usize;
        (0..n)
            .map(|i| (amp * (2.0 * PI * freq * i as f64 / fs).sin()) as f32)
            .collect()
    }

    /// ブロックに分けて流す。
    fn run(lev: &mut Leveler, sig: &[f32], chunk: usize, p: &LevelerParams) {
        let mut pos = 0usize;
        while pos < sig.len() {
            let end = (pos + chunk).min(sig.len());
            let mut l: Vec<f32> = sig[pos..end].to_vec();
            let mut r = l.clone();
            lev.process(pos as u64, &mut l, &mut r, p);
            pos = end;
        }
    }

    /// 一定レベルの入力なら、ゲインは「ターゲット − 実測」に収束する。
    #[test]
    fn converges_to_target() {
        let fs = 48000.0;
        // ピーク -23 dBFS の正弦波は Short-term も約 -23 LUFS。
        // ターゲット -14 なので +9 dB へ寄るはず。
        let sig = sine(1000.0, -23.0, 12.0, fs);
        let p = params(-14.0, -1.0, 6.0);
        let mut lev = Leveler::new(fs);
        run(&mut lev, &sig, 1600, &p);
        assert!(
            (lev.gain_db() - 9.0).abs() < 0.3,
            "ゲインが {:.2} dB（期待 +9.0）",
            lev.gain_db()
        );
    }

    /// 変化率の上限を守る。1 dB/秒 なら 2 秒で 2 dB までしか動かない。
    #[test]
    fn respects_slew_rate() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 2.0, fs);
        let p = params(-14.0, -1.0, 1.0);
        let mut lev = Leveler::new(fs);
        run(&mut lev, &sig, 1600, &p);
        assert!(
            lev.gain_db() <= 2.0 + 1e-6,
            "2 秒で {:.3} dB 動いた（上限 2.0）",
            lev.gain_db()
        );
        // 窓が埋まるまで動かないので、それなりに小さい値にはなる
        assert!(lev.gain_db() > 0.0, "まったく動いていない");
    }

    /// トゥルーピークの上限がゲインを頭打ちにする。
    #[test]
    fn true_peak_ceiling_caps_gain() {
        let fs = 48000.0;
        // ピーク -23 dBFS、上限 -20 dBTP なら、ピーク的に許されるのは +3 dB まで。
        // ラウドネス的には +9 dB 欲しいが +3 で頭打ちになるはず。
        let sig = sine(1000.0, -23.0, 12.0, fs);
        let p = params(-14.0, -20.0, 6.0);
        let mut lev = Leveler::new(fs);
        run(&mut lev, &sig, 1600, &p);
        assert!(
            (lev.gain_db() - 3.0).abs() < 0.4,
            "ゲインが {:.2} dB（期待 +3.0）",
            lev.gain_db()
        );
    }

    /// 無音ではゲインを動かさない（ノイズフロアを持ち上げない）。
    #[test]
    fn silence_does_not_move_gain() {
        let fs = 48000.0;
        let sig = vec![0.0f32; (fs * 8.0) as usize];
        let p = params(-14.0, -1.0, 6.0);
        let mut lev = Leveler::new(fs);
        run(&mut lev, &sig, 1600, &p);
        assert!(
            lev.gain_db().abs() < 1e-6,
            "無音でゲインが {:.3} dB 動いた",
            lev.gain_db()
        );
    }

    /// 出力が実際に掛かっている（素通しではない）。
    #[test]
    fn output_is_actually_scaled() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 12.0, fs);
        let p = params(-14.0, -1.0, 6.0);
        let mut lev = Leveler::new(fs);

        let mut last_l = Vec::new();
        let mut pos = 0usize;
        while pos < sig.len() {
            let end = (pos + 1600).min(sig.len());
            let mut l: Vec<f32> = sig[pos..end].to_vec();
            let mut r = l.clone();
            lev.process(pos as u64, &mut l, &mut r, &p);
            last_l = l;
            pos = end;
        }

        let in_peak = sig.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        let out_peak = last_l.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        let ratio_db = 20.0 * (out_peak / in_peak).log10();
        assert!(
            (ratio_db as f64 - 9.0).abs() < 0.4,
            "末尾ブロックの比が {ratio_db:.2} dB（期待 +9.0）"
        );
    }
}

//! ITU-R BS.1770-4 / EBU R128 のラウドネス測定とトゥルーピーク測定。
//!
//! 係数と手順は Python の参照実装で数値検証してから移植した。
//! 検証内容（48kHz）:
//!
//! - K-weighting の係数が BS.1770-4 の 48kHz 参照値と 1e-15 以内で一致
//!   - Stage1: b=[1.53512485958697, -2.69169618940638, 1.19839281085285],
//!             a=[1, -1.69065929318241, 0.73248077421585]
//!   - Stage2: b=[1, -2, 1], a=[1, -1.99004745483398, 0.99007225036621]
//! - 1kHz 正弦波（ピーク -23 / -33 / -14 / -40 dBFS、L=R）の積分ラウドネスが
//!   それぞれ誤差 +0.007 LU 以内
//! - 片チャンネルのみの場合に -3.01 LU 下がる
//! - -23dBFS 10 秒 + -50dBFS 10 秒 を -23.06 LUFS と測る（ゲート無しなら約 -26）
//!
//! トゥルーピークは BS.1770-4 Annex 2 相当の 4 倍オーバーサンプリングだが、
//! 係数は規格の Table 3 そのものではなく Blackman 窓付き sinc から生成している。
//! 最悪ケース（fs/4 の正弦を 45 度ずらしたもの）で -0.078 dB、
//! 検証した 5 パターンの最大誤差は 0.095 dB。過小評価側に出る。

use std::f64::consts::PI;

/// 100ms ブロックのゲート単位。BS.1770-4 は 400ms ブロック / 75% オーバーラップ。
/// 本番の測定は `static_mode` / `leveler` が行うので、ここの定数は
/// 下の `Analyzer`（テスト用の逐次リファレンス）だけが使う。
#[allow(dead_code)]
const STEP_MS: f64 = 100.0;
#[allow(dead_code)]
const BLOCK_PARTS: usize = 4;

/// 絶対ゲート。
const ABSOLUTE_GATE_LUFS: f64 = -70.0;
/// 相対ゲートのオフセット。
const RELATIVE_GATE_LU: f64 = -10.0;

/// 2 次 IIR フィルタ（Direct Form I）。
#[derive(Debug, Clone, Copy)]
struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
}

impl Biquad {
    fn new(b0: f64, b1: f64, b2: f64, a1: f64, a2: f64) -> Self {
        Self {
            b0,
            b1,
            b2,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    #[inline]
    fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// K-weighting（Stage1 プリフィルタ + Stage2 RLB ハイパス）。
#[derive(Debug, Clone, Copy)]
pub struct KWeighting {
    stage1: Biquad,
    stage2: Biquad,
}

impl KWeighting {
    pub fn new(sample_rate: f64) -> Self {
        // Stage1: ハイシェルフ。BS.1770-4 Annex 1 の f0 / G / Q からプリワープ付き
        // bilinear 変換で算出する。48kHz で規格の参照係数と一致することを確認済み。
        let stage1 = {
            let f0 = 1681.974450955533_f64;
            let g = 3.999843853973347_f64;
            let q = 0.7071752369554196_f64;
            let k = (PI * f0 / sample_rate).tan();
            let k2 = k * k;
            let vh = 10.0_f64.powf(g / 20.0);
            let vb = vh.powf(0.4996667741545416);
            let a0 = 1.0 + k / q + k2;
            Biquad::new(
                (vh + vb * k / q + k2) / a0,
                2.0 * (k2 - vh) / a0,
                (vh - vb * k / q + k2) / a0,
                2.0 * (k2 - 1.0) / a0,
                (1.0 - k / q + k2) / a0,
            )
        };

        // Stage2: RLB ハイパス。分子は [1, -2, 1] で正規化しない（規格の表と同じ形）。
        let stage2 = {
            let f0 = 38.13547087602444_f64;
            let q = 0.5003270373238773_f64;
            let k = (PI * f0 / sample_rate).tan();
            let k2 = k * k;
            let a0 = 1.0 + k / q + k2;
            Biquad::new(
                1.0,
                -2.0,
                1.0,
                2.0 * (k2 - 1.0) / a0,
                (1.0 - k / q + k2) / a0,
            )
        };

        Self { stage1, stage2 }
    }

    /// フィルタの内部状態だけを消す。係数は保つ。
    /// シークで音が不連続になったときに呼ぶ。
    pub fn reset(&mut self) {
        self.stage1.reset();
        self.stage2.reset();
    }

    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        self.stage2.process(self.stage1.process(x))
    }
}

/// 4 倍オーバーサンプリングによるトゥルーピーク検出。
const TP_PHASES: usize = 4;
const TP_TAPS: usize = 12;

#[derive(Debug, Clone)]
pub struct TruePeak {
    /// 位相ごとの FIR 係数。`phases[p][k]` は k サンプル前に掛かる。
    phases: [[f64; TP_TAPS]; TP_PHASES],
    history: [f64; TP_TAPS],
    peak: f64,
}

impl TruePeak {
    pub fn new() -> Self {
        let total = TP_PHASES * TP_TAPS;
        let center = (total - 1) as f64 / 2.0;
        let mut h = vec![0.0_f64; total];
        for (i, slot) in h.iter_mut().enumerate() {
            let t = i as f64 - center;
            let x = t / TP_PHASES as f64;
            let sinc = if x.abs() < 1e-12 {
                1.0
            } else {
                (PI * x).sin() / (PI * x)
            };
            // Blackman 窓
            let n = (total - 1) as f64;
            let w = 0.42 - 0.5 * (2.0 * PI * i as f64 / n).cos()
                + 0.08 * (4.0 * PI * i as f64 / n).cos();
            *slot = sinc * w;
        }

        let mut phases = [[0.0_f64; TP_TAPS]; TP_PHASES];
        for (p, phase) in phases.iter_mut().enumerate() {
            for (k, coef) in phase.iter_mut().enumerate() {
                *coef = h[p + TP_PHASES * k];
            }
            // 各位相の DC ゲインを 1 に揃える
            let gain: f64 = phase.iter().sum();
            if gain.abs() > 1e-12 {
                for coef in phase.iter_mut() {
                    *coef /= gain;
                }
            }
        }

        Self {
            phases,
            history: [0.0; TP_TAPS],
            peak: 0.0,
        }
    }

    /// 検出した最大値（線形）。
    pub fn peak(&self) -> f64 {
        self.peak
    }

    /// 履歴だけ消す。検出済みの最大値は保つ。
    pub fn reset_history(&mut self) {
        self.history = [0.0; TP_TAPS];
    }

    #[inline]
    pub fn push(&mut self, x: f64) {
        self.push_and_peak(x);
    }

    /// 1 サンプル入れて、**そのサンプル周辺の**ピーク（補間値込み）を返す。
    /// 累積の最大値も同時に更新する。
    /// リアルタイム側は「直近のピーク」が要るので戻り値を使う。
    #[inline]
    pub fn push_and_peak(&mut self, x: f64) -> f64 {
        for i in (1..TP_TAPS).rev() {
            self.history[i] = self.history[i - 1];
        }
        self.history[0] = x;

        let mut local = x.abs();
        for phase in &self.phases {
            let mut v = 0.0;
            for k in 0..TP_TAPS {
                v += phase[k] * self.history[k];
            }
            let a = v.abs();
            if a > local {
                local = a;
            }
        }
        if local > self.peak {
            self.peak = local;
        }
        local
    }
}

/// 平均二乗からラウドネス（LKFS）へ。
pub fn loudness(mean_square: f64) -> f64 {
    if mean_square > 0.0 {
        -0.691 + 10.0 * mean_square.log10()
    } else {
        f64::NEG_INFINITY
    }
}

/// リニア値を dB へ。0 以下は -inf。
pub fn to_db(linear: f64) -> f64 {
    if linear > 0.0 {
        20.0 * linear.log10()
    } else {
        f64::NEG_INFINITY
    }
}

/// 400ms ブロックの平均二乗の並びから、2 段ゲートを掛けて積分ラウドネスを出す。
///
/// - 絶対ゲート -70 LUFS
/// - 相対ゲート = 絶対ゲート通過分の平均 - 10 LU
///
/// 通過するブロックが無ければ `None`（実質無音）。
pub fn gated_integrated(blocks: &[f64]) -> Option<f64> {
    let passed: Vec<f64> = blocks
        .iter()
        .copied()
        .filter(|&z| loudness(z) > ABSOLUTE_GATE_LUFS)
        .collect();
    if passed.is_empty() {
        return None;
    }
    let mean: f64 = passed.iter().sum::<f64>() / passed.len() as f64;
    let gamma = loudness(mean) + RELATIVE_GATE_LU;
    let passed2: Vec<f64> = passed
        .iter()
        .copied()
        .filter(|&z| loudness(z) > gamma)
        .collect();
    if passed2.is_empty() {
        return None;
    }
    let mean2: f64 = passed2.iter().sum::<f64>() / passed2.len() as f64;
    let l = loudness(mean2);
    if l.is_finite() { Some(l) } else { None }
}

/// 測定結果。
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// 積分ラウドネス。全ブロックがゲートで落ちた（実質無音）場合は `None`。
    pub integrated_lufs: Option<f64>,
    /// トゥルーピーク（dBTP）。完全無音なら `f64::NEG_INFINITY`。
    pub true_peak_dbtp: f64,
    /// 測定に使ったサンプル数（1 チャンネルあたり）。
    pub samples: u64,
}

/// 頭から順に食わせるだけの逐次リファレンス。
///
/// 本番の経路は `static_mode`（位置つき累積）と `leveler`（Short-term 追従）で、
/// これはそちらと同じ部品（`KWeighting` / `TruePeak` / `gated_integrated`）を
/// 素直に並べたもの。**BS.1770-4 の参照値と突き合わせる単体テストが使う。**
/// 測定チェーンが壊れていないことを固定するために残してある。
#[allow(dead_code)]
#[derive(Debug)]
pub struct Analyzer {
    k_left: KWeighting,
    k_right: KWeighting,
    tp_left: TruePeak,
    tp_right: TruePeak,

    step: usize,
    /// 100ms ぶんの二乗和（左右合計する前に個別で持つ必要はないので合算しておく）
    acc: f64,
    acc_n: usize,
    /// 直近 3 個の 100ms パート。400ms ブロックを作るのに使う。
    tail: Vec<f64>,
    /// 各 400ms ブロックの平均二乗（左右合計）。
    blocks: Vec<f64>,
    samples: u64,
}

#[allow(dead_code)]
impl Analyzer {
    pub fn new(sample_rate: f64) -> Self {
        let step = ((sample_rate * STEP_MS / 1000.0).round() as usize).max(1);
        Self {
            k_left: KWeighting::new(sample_rate),
            k_right: KWeighting::new(sample_rate),
            tp_left: TruePeak::new(),
            tp_right: TruePeak::new(),
            step,
            acc: 0.0,
            acc_n: 0,
            tail: Vec::with_capacity(BLOCK_PARTS),
            blocks: Vec::new(),
            samples: 0,
        }
    }

    /// 左右のサンプルを追加する。2 つのスライスは同じ長さである必要がある。
    pub fn feed(&mut self, left: &[f32], right: &[f32]) {
        debug_assert_eq!(left.len(), right.len());
        for (&l, &r) in left.iter().zip(right.iter()) {
            let l = l as f64;
            let r = r as f64;

            self.tp_left.push(l);
            self.tp_right.push(r);

            let wl = self.k_left.process(l);
            let wr = self.k_right.process(r);
            self.acc += wl * wl + wr * wr;
            self.acc_n += 1;
            self.samples += 1;

            if self.acc_n == self.step {
                self.push_part(self.acc);
                self.acc = 0.0;
                self.acc_n = 0;
            }
        }
    }

    fn push_part(&mut self, part_sum: f64) {
        self.tail.push(part_sum);
        if self.tail.len() > BLOCK_PARTS {
            self.tail.remove(0);
        }
        if self.tail.len() == BLOCK_PARTS {
            let sum: f64 = self.tail.iter().sum();
            self.blocks.push(sum / (self.step * BLOCK_PARTS) as f64);
        }
    }

    /// 2 段ゲートを掛けて積分ラウドネスを確定する。
    pub fn finish(self) -> Measurement {
        Measurement {
            integrated_lufs: gated_integrated(&self.blocks),
            true_peak_dbtp: to_db(self.tp_left.peak().max(self.tp_right.peak())),
            samples: self.samples,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(freq: f64, peak_dbfs: f64, secs: f64, fs: f64) -> Vec<f32> {
        let amp = 10.0_f64.powf(peak_dbfs / 20.0);
        let n = (fs * secs) as usize;
        (0..n)
            .map(|i| (amp * (2.0 * PI * freq * i as f64 / fs).sin()) as f32)
            .collect()
    }

    fn measure(left: &[f32], right: &[f32], fs: f64) -> Measurement {
        let mut a = Analyzer::new(fs);
        a.feed(left, right);
        a.finish()
    }

    /// 実機と同じ規模（約 205 秒 / 48kHz ステレオ）を食わせて所要時間を測る。
    /// `cargo test --release -- --ignored --nocapture bench` で見る。
    #[test]
    #[ignore]
    fn bench_feed_throughput() {
        use std::time::Instant;
        let fs = 48000.0;
        let total = 9_825_600usize;
        let chunk = 1usize << 16;
        let buf: Vec<f32> = (0..chunk)
            .map(|i| (0.5 * (2.0 * PI * 440.0 * i as f64 / fs).sin()) as f32)
            .collect();
        let mut a = Analyzer::new(fs);
        let t0 = Instant::now();
        let mut fed = 0usize;
        while fed < total {
            let n = chunk.min(total - fed);
            a.feed(&buf[..n], &buf[..n]);
            fed += n;
        }
        let feed_took = t0.elapsed();
        let t1 = Instant::now();
        let m = a.finish();
        let finish_took = t1.elapsed();
        println!(
            "feed {} サンプル: {:.3} 秒 / finish: {:.3} 秒 / 合計 {:.3} 秒 (LUFS {:?})",
            total,
            feed_took.as_secs_f64(),
            finish_took.as_secs_f64(),
            (feed_took + finish_took).as_secs_f64(),
            m.integrated_lufs
        );
    }

    #[test]
    fn kweighting_matches_bs1770_reference_at_48k() {
        let k = KWeighting::new(48000.0);
        let s1 = k.stage1;
        for (got, want) in [
            (s1.b0, 1.53512485958697),
            (s1.b1, -2.69169618940638),
            (s1.b2, 1.19839281085285),
            (s1.a1, -1.69065929318241),
            (s1.a2, 0.73248077421585),
        ] {
            assert!((got - want).abs() < 1e-12, "stage1: {got} != {want}");
        }
        let s2 = k.stage2;
        for (got, want) in [
            (s2.b0, 1.0),
            (s2.b1, -2.0),
            (s2.b2, 1.0),
            (s2.a1, -1.99004745483398),
            (s2.a2, 0.99007225036621),
        ] {
            assert!((got - want).abs() < 1e-12, "stage2: {got} != {want}");
        }
    }

    #[test]
    fn sine_reads_its_own_level() {
        let fs = 48000.0;
        for target in [-23.0, -33.0, -14.0, -40.0] {
            let s = sine(1000.0, target, 20.0, fs);
            let m = measure(&s, &s, fs);
            let got = m.integrated_lufs.expect("測定できるはず");
            assert!(
                (got - target).abs() < 0.1,
                "{target} dBFS の正弦波が {got} LUFS になった"
            );
        }
    }

    #[test]
    fn single_channel_is_three_lu_quieter() {
        let fs = 48000.0;
        let s = sine(1000.0, -23.0, 20.0, fs);
        let zero = vec![0.0f32; s.len()];
        let got = measure(&s, &zero, fs).integrated_lufs.unwrap();
        assert!((got - (-26.01)).abs() < 0.1, "片チャンネルで {got} LUFS");
    }

    #[test]
    fn relative_gate_ignores_quiet_part() {
        let fs = 48000.0;
        let mut s = sine(1000.0, -23.0, 10.0, fs);
        s.extend(sine(1000.0, -50.0, 10.0, fs));
        let got = measure(&s, &s, fs).integrated_lufs.unwrap();
        // ゲートが効いていなければ約 -26 になる
        assert!((got - (-23.0)).abs() < 0.2, "ゲート後が {got} LUFS");
    }

    #[test]
    fn silence_has_no_integrated_loudness() {
        let fs = 48000.0;
        let zero = vec![0.0f32; 48000 * 2];
        let m = measure(&zero, &zero, fs);
        assert!(m.integrated_lufs.is_none());
        assert!(m.true_peak_dbtp.is_infinite());
    }

    #[test]
    fn true_peak_sees_intersample_peak() {
        let fs = 48000.0;
        // fs/4 の正弦を 45 度ずらすとサンプル点が山を外し、サンプルピークは -3.01dB になる。
        let n = 4800;
        let s: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * (fs / 4.0) * i as f64 / fs + PI / 4.0).sin() as f32)
            .collect();
        let sample_peak = s.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        assert!(
            (20.0 * (sample_peak as f64).log10() - (-3.01)).abs() < 0.05,
            "サンプルピークが想定と違う"
        );
        let tp = measure(&s, &s, fs).true_peak_dbtp;
        assert!(tp > -0.15 && tp <= 0.05, "トゥルーピークが {tp} dBTP");
    }
}

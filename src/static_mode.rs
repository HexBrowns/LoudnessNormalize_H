//! 静的モード: **実際に鳴っている音**を位置つきで貯めて、全体をカバーしたら測る。
//!
//! v1.x はソースファイルを直接読んで測っていたが、それは「ファイルの音」であって
//! 「オブジェクトの音」ではない。分割・再生範囲・再生速度・上流のフィルタ効果・音量が
//! 何も反映されない。ここでは `proc_audio` に渡ってくるバッファそのものを測る。
//!
//! 積分ラウドネスは全区間が揃わないと確定しないので、
//! **1 周ぶん再生（または書き出し）されるまでは結果が出ない**。
//! これは実装の都合ではなく EBU R128 の定義がそうなっている。
//!
//! シークで音が飛んだらフィルタの内部状態を捨て、落ち着くまでのパートは
//! 「取れなかった」扱いにする。そのパートは後で通ったときに埋まる。

use crate::r128;

/// ゲートの最小単位。BS.1770-4 は 400ms ブロック / 75% オーバーラップ = 100ms 刻み。
const STEP_MS: f64 = 100.0;
const BLOCK_PARTS: usize = 4;

/// シーク直後に捨てる時間。K-weighting の Stage2 は 38.13Hz / Q≒0.5 で
/// 時定数が長く、切り替え直後の出力は信用できない。
const SETTLE_MS: f64 = 200.0;

/// 「揃った」とみなすカバー率。
///
/// 全パートを必須にすると、**1 個でも欠けたら永久に完了しない**。
/// ホストが末尾のブロックを配らない、シーク直後の捨て区間が最後まで埋まらない、
/// といった理由で 1〜2 個残るのは普通に起きる（v2.0.0 で実際に踏んだ）。
/// R128 のゲート自体がブロックを取捨するものなので、
/// 数パーセント欠けても積分値はほとんど動かない。欠けた窓は計算から外す。
const COMPLETE_COVERAGE: f64 = 0.98;

/// 測定の確定値。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measured {
    pub integrated_lufs: Option<f64>,
    pub true_peak_dbtp: f64,
}

#[derive(Debug)]
pub struct StaticAnalyzer {
    step: usize,
    settle_samples: usize,
    /// 100ms パートごとの二乗和（左右合算）。未取得は `None`。
    parts: Vec<Option<f64>>,
    k_left: r128::KWeighting,
    k_right: r128::KWeighting,
    tp_left: r128::TruePeak,
    tp_right: r128::TruePeak,
    /// 次に来るはずのサンプル位置。ここと違えば不連続。
    expected_next: u64,
    settle_remaining: usize,
    /// 書き込み中のパート
    cur_part: usize,
    cur_acc: f64,
    cur_n: usize,
    /// 書き込み中のパートが信用できるか（シーク直後は false）
    cur_valid: bool,
    /// 一度でも音を受け取ったか
    touched: bool,
    /// 直前に食わせた範囲。ホストが同じブロックを再描画したときに
    /// 「シーク」と誤認してフィルタ状態を捨てないようにするための記録。
    last_fed: Option<(u64, usize)>,
}

impl StaticAnalyzer {
    pub fn new(sample_rate: f64, sample_total: u64) -> Self {
        let step = ((sample_rate * STEP_MS / 1000.0).round() as usize).max(1);
        let part_count = (sample_total / step as u64) as usize;
        Self {
            step,
            settle_samples: (sample_rate * SETTLE_MS / 1000.0).round() as usize,
            parts: vec![None; part_count],
            k_left: r128::KWeighting::new(sample_rate),
            k_right: r128::KWeighting::new(sample_rate),
            tp_left: r128::TruePeak::new(),
            tp_right: r128::TruePeak::new(),
            expected_next: u64::MAX,
            settle_remaining: 0,
            cur_part: usize::MAX,
            cur_acc: 0.0,
            cur_n: 0,
            cur_valid: false,
            touched: false,
            last_fed: None,
        }
    }

    /// 100ms パートの総数。400ms ブロックを 1 つも作れないほど短ければ 0 に近い。
    pub fn part_count(&self) -> usize {
        self.parts.len()
    }

    /// 埋まったパートの割合（0.0〜1.0）。進捗表示用。
    pub fn coverage(&self) -> f64 {
        if self.parts.is_empty() {
            return 0.0;
        }
        let filled = self.parts.iter().filter(|p| p.is_some()).count();
        filled as f64 / self.parts.len() as f64
    }

    /// 埋まったパート数。
    pub fn filled_parts(&self) -> usize {
        self.parts.iter().filter(|p| p.is_some()).count()
    }

    pub fn is_complete(&self) -> bool {
        self.parts.len() >= BLOCK_PARTS && self.coverage() >= COMPLETE_COVERAGE
    }

    /// 実際に鳴っている音を食わせる。`sample_index` はオブジェクト先頭からの位置。
    pub fn feed(&mut self, sample_index: u64, left: &[f32], right: &[f32]) {
        debug_assert_eq!(left.len(), right.len());
        if left.is_empty() {
            return;
        }
        // 同じ範囲がもう一度来たら、それは再描画であってシークではない。
        // ここでフィルタ状態を捨てると、再描画のたびに前後のパートが
        // 「落ち着き待ち」で捨てられ、いつまでも埋まらなくなる。
        if self.last_fed == Some((sample_index, left.len())) {
            return;
        }
        self.touched = true;

        if sample_index != self.expected_next {
            // 不連続。フィルタの内部状態は使えない。
            self.k_left.reset();
            self.k_right.reset();
            self.tp_left.reset_history();
            self.tp_right.reset_history();
            // 先頭からならフィルタは元々初期状態なので落ち着かせる必要が無い。
            self.settle_remaining = if sample_index == 0 {
                0
            } else {
                self.settle_samples
            };
            self.cur_part = usize::MAX;
            self.cur_acc = 0.0;
            self.cur_n = 0;
            self.cur_valid = false;
        }

        let step = self.step as u64;
        for (i, (&l, &r)) in left.iter().zip(right.iter()).enumerate() {
            let pos = sample_index + i as u64;
            let part = (pos / step) as usize;

            if part != self.cur_part {
                // パートの切り替わり。前のパートが最後まで揃っていれば確定させる。
                self.commit_part();
                self.cur_part = part;
                self.cur_acc = 0.0;
                self.cur_n = 0;
                // 途中から始まったパートは頭が欠けているので使わない。
                self.cur_valid = self.settle_remaining == 0 && (pos % step == 0);
            }

            let l = l as f64;
            let r = r as f64;
            self.tp_left.push(l);
            self.tp_right.push(r);

            let wl = self.k_left.process(l);
            let wr = self.k_right.process(r);

            if self.settle_remaining > 0 {
                self.settle_remaining -= 1;
                self.cur_valid = false;
            } else {
                self.cur_acc += wl * wl + wr * wr;
                self.cur_n += 1;
            }
        }

        // 最後のパートは「次のパートへ移る」瞬間が来ないので、
        // ここで揃っていれば確定させる。これが無いと末尾 1 個が埋まらない。
        self.commit_part();

        self.expected_next = sample_index + left.len() as u64;
        self.last_fed = Some((sample_index, left.len()));
    }

    /// 書き込み中のパートが最後まで揃っていれば確定させる。
    fn commit_part(&mut self) {
        if self.cur_valid && self.cur_n == self.step {
            if let Some(slot) = self.parts.get_mut(self.cur_part) {
                *slot = Some(self.cur_acc);
            }
        }
    }

    /// 全パートが揃っていれば測定値を返す。揃っていなければ `None`。
    pub fn result(&self) -> Option<Measured> {
        if !self.is_complete() || self.parts.len() < BLOCK_PARTS {
            return None;
        }
        // 100ms パート 4 個で 400ms ブロック 1 個（75% オーバーラップ）。
        // 欠けたパートを含む窓は作れないので飛ばす。
        let denom = (self.step * BLOCK_PARTS) as f64;
        let blocks: Vec<f64> = (0..=self.parts.len() - BLOCK_PARTS)
            .filter_map(|j| {
                let mut sum = 0.0;
                for slot in &self.parts[j..j + BLOCK_PARTS] {
                    sum += (*slot)?;
                }
                Some(sum / denom)
            })
            .collect();
        if blocks.is_empty() {
            return None;
        }

        Some(Measured {
            integrated_lufs: r128::gated_integrated(&blocks),
            true_peak_dbtp: r128::to_db(self.tp_left.peak().max(self.tp_right.peak())),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn sine(freq: f64, peak_dbfs: f64, secs: f64, fs: f64) -> Vec<f32> {
        let amp = 10.0_f64.powf(peak_dbfs / 20.0);
        let n = (fs * secs) as usize;
        (0..n)
            .map(|i| (amp * (2.0 * PI * freq * i as f64 / fs).sin()) as f32)
            .collect()
    }

    /// 頭から順に食わせれば、ファイルを直接読んだときと同じ値になる。
    #[test]
    fn sequential_feed_matches_expected_level() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 20.0, fs);
        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);

        assert!(a.result().is_none(), "食わせる前に結果が出ている");

        let chunk = 1600; // 30fps 相当
        let mut pos = 0u64;
        while (pos as usize) < sig.len() {
            let end = (pos as usize + chunk).min(sig.len());
            a.feed(pos, &sig[pos as usize..end], &sig[pos as usize..end]);
            pos = end as u64;
        }

        assert!(a.is_complete(), "カバー率 {:.3}", a.coverage());
        let m = a.result().expect("揃ったのに結果が出ない");
        let lufs = m.integrated_lufs.expect("測定できるはず");
        assert!((lufs - (-23.0)).abs() < 0.1, "{lufs} LUFS");
        assert!((m.true_peak_dbtp - (-23.0)).abs() < 0.15, "{} dBTP", m.true_peak_dbtp);
    }

    /// 途中までしか食わせなければ結果は出ない。
    #[test]
    fn partial_coverage_has_no_result() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 20.0, fs);
        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);
        let half = sig.len() / 2;
        a.feed(0, &sig[..half], &sig[..half]);
        assert!(!a.is_complete());
        assert!(a.result().is_none());
        assert!((a.coverage() - 0.5).abs() < 0.02, "{}", a.coverage());
    }

    /// 飛ばし飛ばしに再生しても、最終的に全部通れば正しい値になる。
    #[test]
    fn seeking_still_converges() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 20.0, fs);
        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);
        let chunk = 4800usize;

        // まず後半 → 前半、という順で食わせる（シークを再現）
        for half in [1usize, 0] {
            let mut pos = half * (sig.len() / 2);
            let end_of_half = (half + 1) * (sig.len() / 2);
            while pos < end_of_half {
                let end = (pos + chunk).min(end_of_half);
                a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
                pos = end;
            }
        }

        // シーク直後のパートは捨てられているので、まだ埋まりきらない
        let after_seek = a.coverage();
        assert!(after_seek < 1.0, "捨てるはずのパートが埋まっている");

        // 頭から通しで流し直せば全部埋まる
        let mut pos = 0usize;
        while pos < sig.len() {
            let end = (pos + chunk).min(sig.len());
            a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
            pos = end;
        }
        assert!(a.is_complete(), "カバー率 {:.3}", a.coverage());
        let lufs = a.result().unwrap().integrated_lufs.unwrap();
        assert!((lufs - (-23.0)).abs() < 0.1, "{lufs} LUFS");
    }

    /// 同じ区間を二度食わせても値が壊れない（再描画で同じブロックが来る想定）。
    #[test]
    fn refeeding_same_range_is_harmless() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 20.0, fs);
        let chunk = 4800usize;

        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);
        let mut pos = 0usize;
        while pos < sig.len() {
            let end = (pos + chunk).min(sig.len());
            a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
            // 同じ区間をもう一度
            a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
            pos = end;
        }
        assert!(a.is_complete(), "カバー率 {:.3}", a.coverage());
        let lufs = a.result().unwrap().integrated_lufs.unwrap();
        assert!((lufs - (-23.0)).abs() < 0.2, "{lufs} LUFS");
    }

    /// パートが少し欠けていても完了し、値もほとんど変わらない。
    ///
    /// 全パート必須にしていた v2.0.0 は、末尾 1 個が埋まらないだけで
    /// 永久に完了しなかった。実機で「静的モードが効かない」として出た症状。
    #[test]
    fn completes_with_a_few_missing_parts() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 20.0, fs);
        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);

        // 末尾 1 秒（10 パート、全体の 5%）を配らない
        let stop = sig.len() - (fs as usize);
        let chunk = 1600usize;
        let mut pos = 0usize;
        while pos < stop {
            let end = (pos + chunk).min(stop);
            a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
            pos = end;
        }
        assert!(!a.is_complete(), "5% 欠けで完了してはいけない: {:.3}", a.coverage());

        // 末尾 0.2 秒（2 パート、全体の 1%）だけ欠ける状態にする
        let stop = sig.len() - (fs as usize) / 5;
        while pos < stop {
            let end = (pos + chunk).min(stop);
            a.feed(pos as u64, &sig[pos..end], &sig[pos..end]);
            pos = end;
        }
        assert!(a.is_complete(), "1% 欠けで完了しない: {:.3}", a.coverage());
        let lufs = a.result().expect("結果が出ない").integrated_lufs.unwrap();
        assert!((lufs - (-23.0)).abs() < 0.1, "{lufs} LUFS");
    }

    /// 400ms に満たないオブジェクトでは結果を出さない。
    #[test]
    fn too_short_object_has_no_result() {
        let fs = 48000.0;
        let sig = sine(1000.0, -23.0, 0.25, fs);
        let mut a = StaticAnalyzer::new(fs, sig.len() as u64);
        a.feed(0, &sig, &sig);
        assert!(a.result().is_none());
    }
}

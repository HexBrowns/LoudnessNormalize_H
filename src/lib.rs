//! ラウドネスノーマライズ_H — EBU R128 / ITU-R BS.1770-4 に沿った音量そろえ。
//!
//! **測るのは「実際に鳴っている音」**。`proc_audio` に渡ってくるバッファそのものを見るので、
//! 分割・再生範囲・再生速度・上流のフィルタ効果が全部反映される。
//! v1.x はソースファイルを直接読んでいたが、それは「ファイルの音」であって
//! 「オブジェクトの音」ではなかった。
//!
//! モードが 2 つある。
//!
//! - **静的（EBU R128 準拠）**: 全区間を貯めてから測り、確定した 1 つのゲインを掛ける。
//!   規格どおりの積分ラウドネスで、ダイナミクスを一切潰さない。
//!   ただし**積分ラウドネスは番組全体に対して定義された値**なので、
//!   1 周ぶん再生（または書き出し）されるまで結果が出ない。これは定義上の制約で、
//!   どんな実装でも回避できない。測定値はプロジェクトに保存するので、次回からは即座に効く。
//!   測っている通しの途中で確定しても、その通しが途切れるまでは素通しのまま（途中で音量を段にしない）。
//! - **リアルタイム（レベラー）**: Short-term ラウドネス（3 秒窓）でゲインを緩やかに追従。
//!   事前パス不要で載せた瞬間から効く。R128 の積分ラウドネス正規化ではない。
//!
//! トゥルーピークは**リミッターではなくゲインの上限**として使う。
//! `clamp` によるハードクリップは歪みそのものなので使わない。
//! 上限が効いてターゲットに届かない場合は、届かないままにしてログに残す。
//!
//! やらないこと:
//!
//! - **`proc_audio` の中から編集セクション（`read_section` 等）を呼ばない**。
//!   `aviutl2-rs` の作例 18 本のうち、フィルタ処理中にこれを触るものは 0 件。
//!   そのためシーンの長さや出力範囲は分からない（フィルタに渡る情報に無い）。
//! - **レンダースレッドで無制限の I/O をしない**。v2 はファイルを読まないので
//!   そもそも I/O が無い。

#[cfg(test)]
mod bench;
mod leveler;
mod r128;
mod static_mode;

use std::time::{Duration, Instant};

use aviutl2::{
    filter::{
        AudioChannel, FilterConfigDataHandle, FilterConfigItemSliceExt, FilterConfigItems,
        FilterPlugin, FilterPluginTable, FilterProcAudio,
    },
    tracing,
};

/// 保存データの版。構造を変えたら上げる。合わなければ捨てて測り直す。
const PERSIST_MAGIC: u32 = 0x4C4E_4832; // "LNH2"

/// 素性（長さ・レート）の変化がこの間隔より短く続いたら「連打」とみなす。
const RENEW_BURST_WINDOW: Duration = Duration::from_secs(2);
/// 連打が何回続いたら警告を 1 回出してログを止めるか（最初の 1 回の後に続いた回数）。
const RENEW_BURST_LIMIT: u32 = 3;
/// リアルタイムのゲインをログに出す行数の上限（エフェクトごと）。
const REALTIME_LOG_LINES: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq, aviutl2::filter::FilterConfigSelectItems)]
pub enum Mode {
    #[item(name = "静的（EBU R128 準拠）")]
    Static,
    #[item(name = "リアルタイム（レベラー）")]
    Realtime,
}

/// プロジェクトに保存する測定値。`FILTER_ITEM_DATA` に生で載るので POD にする。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Persisted {
    magic: u32,
    /// 積分ラウドネスが取れているか（0 = 実質無音）
    has_loudness: u32,
    /// 測ったときのサンプリングレート。素材が変わったかの判定に使う。
    sample_rate: u32,
    /// 最後に見た「解析をやり直す」（1 = ON）。開き直したときに ON のままの設定を
    /// 「OFF から ON にした」と取り違えないために持つ。v2.0.2 までは使っておらず常に 0。
    force_flag: u32,
    /// 測ったときのオブジェクト総サンプル数。同上。
    sample_total: u64,
    integrated_lufs: f64,
    true_peak_dbtp: f64,
}

impl Persisted {
    fn is_valid(&self) -> bool {
        self.magic == PERSIST_MAGIC
    }

    /// この素性（レート・長さ）で測った値か。
    fn matches(&self, sample_rate: f64, sample_total: u64) -> bool {
        self.is_valid() && self.sample_rate == sample_rate as u32 && self.sample_total == sample_total
    }
}

#[aviutl2::filter::filter_config_items]
// FilterConfigDataHandle は PartialEq を持たないので derive しない。
#[derive(Debug, Clone)]
pub struct FilterConfig {
    #[check(name = "バイパス", default = false)]
    bypass: bool,

    #[select(name = "モード", items = Mode, default = Mode::Static)]
    mode: Mode,

    #[track(name = "ターゲット LUFS", range = -40.0..=0.0, step = 0.1, default = -14.0)]
    target_lufs: f64,
    #[track(name = "トゥルーピーク上限 dBTP", range = -9.0..=0.0, step = 0.1, default = -1.0)]
    true_peak_ceiling: f64,
    #[track(name = "最大ゲイン dB", range = 0.0..=30.0, step = 0.1, default = 12.0)]
    max_gain_db: f64,

    #[group(name = "リアルタイム")]
    realtime: group! {
        #[track(name = "追従の速さ dB/秒", range = 0.1..=12.0, step = 0.1, default = 1.0)]
        rate_db_per_sec: f64,
        #[track(name = "最小ゲイン dB", range = -30.0..=0.0, step = 0.1, default = -12.0)]
        min_gain_db: f64,
    },

    #[group(name = "静的")]
    static_group: group! {
        #[check(name = "解析をやり直す", default = false)]
        force_reanalyze: bool,
    },

    #[data(name = "測定値")]
    measurement: FilterConfigDataHandle<Persisted>,
}

/// ゲイン計算に使う測定値。
#[derive(Debug, Clone, Copy)]
struct Analysis {
    integrated_lufs: Option<f64>,
    true_peak_dbtp: f64,
}

/// エフェクトのインスタンスごとの状態。
struct EffectState {
    mode: Mode,
    sample_rate: f64,
    sample_total: u64,
    /// 静的モードで測っている間だけ持つ。確定したら放す（1 時間のオブジェクトで約 576KB）。
    /// リアルタイムモードと、保存済みの測定値を使うときは作らない。
    analyzer: Option<static_mode::StaticAnalyzer>,
    leveler: leveler::Leveler,
    /// 静的モードで確定した測定値。
    measured: Option<static_mode::Measured>,
    /// 「解析をやり直す」の直近の状態。立ち上がりを拾うために持つ。
    last_force: bool,
    /// 測っている通しの途中で確定したとき、その通しが途切れるまで素通しを続けるための記録
    /// （直前のブロックの先頭, 次に来るはずの位置）。確定した瞬間からゲインを掛けると、
    /// そこで音量が段になる（最大ゲインの既定で +12 dB）。
    hold: Option<(u64, u64)>,
    /// 直近にログへ出したゲイン。毎ブロック出さないための記録。
    logged_gain_db: Option<f64>,
    /// リアルタイムのゲインをログに出した行数。
    realtime_log_lines: u32,
    /// 直近にログへ出したカバー率。進捗ログを 10% 刻みに間引くための記録。
    logged_coverage: f64,
    /// 静的モードの開始ログを出したか（保存済みの値を使うとき・素性が変わったときは出さない）。
    logged_start: bool,
    /// 「末尾が届いていない」を知らせたか。
    warned_tail: bool,
    /// 「途中が欠けている」を知らせたか。
    warned_gaps: bool,
}

impl EffectState {
    fn new(mode: Mode, sample_rate: f64, sample_total: u64, last_force: bool) -> Self {
        Self {
            mode,
            sample_rate,
            sample_total,
            analyzer: None,
            leveler: leveler::Leveler::new(sample_rate),
            measured: None,
            last_force,
            hold: None,
            logged_gain_db: None,
            realtime_log_lines: 0,
            logged_coverage: 0.0,
            logged_start: false,
            warned_tail: false,
            warned_gaps: false,
        }
    }

    fn reset_analysis(&mut self) {
        self.analyzer = None;
        self.measured = None;
        self.hold = None;
        self.logged_gain_db = None;
        self.logged_coverage = 0.0;
        self.logged_start = false;
        self.warned_tail = false;
        self.warned_gaps = false;
    }
}

/// 状態を取る。無ければ作り、素性（レート・長さ）が変わっていたら作り直す。
/// 作り直したときは前の (レート, 総サンプル数) を返す。
///
/// `initial_force` は新しく作るときの「解析をやり直す」の前回値。今の値を渡せば立ち上がりにならない。
/// 作り直すときは前の状態の値を引き継ぐ。
fn state_for(
    state: &mut Option<EffectState>,
    mode: Mode,
    sample_rate: f64,
    sample_total: u64,
    initial_force: bool,
) -> (&mut EffectState, Option<(f64, u64)>) {
    let (keep, old) = match state {
        Some(s) => (
            s.sample_rate == sample_rate && s.sample_total == sample_total,
            Some((s.sample_rate, s.sample_total, s.last_force)),
        ),
        None => (false, None),
    };
    if !keep {
        let last_force = old.map_or(initial_force, |o| o.2);
        *state = Some(EffectState::new(mode, sample_rate, sample_total, last_force));
    }
    let s = state.as_mut().expect("直前で作った");
    s.mode = mode;
    (s, if keep { None } else { old.map(|o| (o.0, o.1)) })
}

/// 素性の変化をログに出すかの判断。短い間に繰り返す（グループ制御などで 1 つの効果に
/// 複数の音が来ている疑い）なら、警告を 1 回だけ出して以後は黙る。
#[derive(Debug, Default)]
struct RenewLog {
    last: Option<Instant>,
    burst: u32,
    quiet: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RenewNotice {
    Info,
    Warn,
    Silent,
}

impl RenewLog {
    fn on_renew(&mut self, now: Instant) -> RenewNotice {
        let quick = self
            .last
            .is_some_and(|t| now.saturating_duration_since(t) < RENEW_BURST_WINDOW);
        self.last = Some(now);
        if self.quiet {
            return RenewNotice::Silent;
        }
        self.burst = if quick { self.burst + 1 } else { 0 };
        if self.burst == 0 {
            RenewNotice::Info
        } else if self.burst >= RENEW_BURST_LIMIT {
            self.quiet = true;
            RenewNotice::Warn
        } else {
            RenewNotice::Silent
        }
    }
}

/// エフェクトのインスタンスごとのユーザーデータ。
///
/// 本体がエフェクトのインスタンスごとに作り（`func_create`）、編集データと Undo の中から
/// そのインスタンスが全部消えたら片付ける（`func_destroy`。`aviutl2-sys` 0.47 の `filter2.rs`）。
/// v2.0.1 までは `effect_id` をキーにした `DashMap` に持ち、エフェクトを消しても取り除いていなかった。
///
/// 作られる時点ではサンプリングレートも総サンプル数も分からないので、中身は最初の `proc_audio` で作る。
#[derive(Default)]
struct EffectSlot {
    state: Option<EffectState>,
    /// 音の受け渡しに使うバッファ。呼ばれるたびに確保しないよう使い回す。
    left: Vec<f32>,
    right: Vec<f32>,
    renew_log: RenewLog,
}

impl aviutl2::filter::FilterUserdata for EffectSlot {
    fn new(_effect_id: i64) -> Self {
        Self::default()
    }
}

/// 1 回の `proc_audio` が受け取ったブロックの素性。
#[derive(Debug, Clone, Copy)]
struct Block {
    effect_id: i64,
    sample_rate: f64,
    sample_total: u64,
    sample_index: u64,
}

/// 1 回の処理の結果。
#[derive(Debug, Default, PartialEq)]
struct Outcome {
    /// 音を書き換えた（false なら素通しで、書き戻さない）
    modified: bool,
    /// プロジェクトに保存する測定値（変えるときだけ）
    persist: Option<Persisted>,
}

impl EffectSlot {
    /// 本体を呼ばない部分の 1 回ぶん。音は `left` / `right` の先頭 `n` 個に入っている。
    fn process(
        &mut self,
        config: &FilterConfig,
        stored: Persisted,
        b: Block,
        n: usize,
        now: Instant,
    ) -> Outcome {
        let EffectSlot {
            state,
            left,
            right,
            renew_log,
        } = self;
        let left = &mut left[..n];
        let right = &mut right[..n];
        let effect_id = b.effect_id;
        let mut out = Outcome::default();
        let mut stored = stored;

        // 新しく作る状態の「解析をやり直す」の前回値。保存済みの値があればそれが最後に見た値。
        // 無ければ今の値（立ち上がりにしない。捨てる測定値も無い）。
        let initial_force = if stored.is_valid() {
            stored.force_flag != 0
        } else {
            config.force_reanalyze
        };
        let (state, renewed) = state_for(
            state,
            config.mode,
            b.sample_rate,
            b.sample_total,
            initial_force,
        );

        if let Some((old_rate, old_total)) = renewed {
            // 素性が変わったときは 1 行だけ出す（「解析開始」は重ねない）
            state.logged_start = true;
            match renew_log.on_renew(now) {
                RenewNotice::Info => tracing::info!(
                    "オブジェクトの素性が変わったので測り直す: {}Hz {} サンプル → {}Hz {} サンプル (effect_id {effect_id})",
                    old_rate,
                    old_total,
                    b.sample_rate,
                    b.sample_total
                ),
                RenewNotice::Warn => tracing::warn!(
                    "オブジェクトの素性（長さ・サンプリングレート）が短い間に何度も変わっている。\
                     グループ制御やシーンオブジェクトに掛けて、1 つの効果に長さの違う複数の音が来ているときに起きうる。\
                     このままでは静的モードの測定が確定しない。以後このエフェクトでは素性の変化をログに出さない (effect_id {effect_id})"
                ),
                RenewNotice::Silent => {}
            }
        }

        // 「解析をやり直す」の立ち上がりで、保存済みも含めて捨てる。
        if config.force_reanalyze && !state.last_force {
            state.reset_analysis();
            stored = Persisted::default();
            out.persist = Some(stored);
            tracing::info!("解析をやり直す (effect_id {effect_id})");
        }
        state.last_force = config.force_reanalyze;
        // 保存済みの値に、最後に見た状態を写す（変わったときだけ書く）
        if stored.is_valid() && (stored.force_flag != 0) != config.force_reanalyze {
            stored.force_flag = u32::from(config.force_reanalyze);
            out.persist = Some(stored);
        }

        match config.mode {
            Mode::Realtime => {
                let params = leveler::LevelerParams {
                    target_lufs: config.target_lufs,
                    true_peak_ceiling: config.true_peak_ceiling,
                    max_gain_db: config.max_gain_db,
                    min_gain_db: config.min_gain_db,
                    rate_db_per_sec: config.rate_db_per_sec,
                };
                state.leveler.process(b.sample_index, left, right, &params);
                out.modified = true;

                // 連続的に動くので、1 dB 以上変わったときだけ、エフェクトごとに数行まで出す。
                if state.realtime_log_lines < REALTIME_LOG_LINES {
                    let g = state.leveler.gain_db();
                    let changed = match state.logged_gain_db {
                        Some(prev) => (prev - g).abs() >= 1.0,
                        None => true,
                    };
                    if changed {
                        state.logged_gain_db = Some(g);
                        state.realtime_log_lines += 1;
                        let last = state.realtime_log_lines == REALTIME_LOG_LINES;
                        tracing::info!(
                            "リアルタイム: ゲイン {:+.2} dB（目標 {:+.2} dB）{} (effect_id {effect_id})",
                            g,
                            state.leveler.target_db(),
                            if last {
                                "。以後このエフェクトのゲインはログに出さない"
                            } else {
                                ""
                            }
                        );
                    }
                }
                out
            }
            Mode::Static => {
                // 保存済みの測定値があれば、素性が一致する限りそれを使う。
                if state.measured.is_none() && stored.matches(b.sample_rate, b.sample_total) {
                    state.measured = Some(static_mode::Measured {
                        integrated_lufs: (stored.has_loudness != 0).then_some(stored.integrated_lufs),
                        true_peak_dbtp: stored.true_peak_dbtp,
                    });
                    state.analyzer = None;
                    state.logged_start = true;
                    tracing::info!(
                        "保存済みの測定値を使う: {:.2} LUFS / {:.2} dBTP (effect_id {effect_id})",
                        stored.integrated_lufs,
                        stored.true_peak_dbtp
                    );
                }

                if state.measured.is_none() {
                    measure_static(state, config, b, left, right, &mut out);
                    return out;
                }

                // 測っていた通しの途中で確定した。通しが途切れるまで素通しを続ける
                // （同じブロックの描き直しは通しの続きとみなす）。
                if let Some((last_start, next)) = state.hold {
                    if b.sample_index == next || b.sample_index == last_start {
                        state.hold = Some((b.sample_index, b.sample_index + n as u64));
                        return out;
                    }
                    state.hold = None;
                }

                let m = state.measured.expect("直前で埋めた");
                let analysis = Analysis {
                    integrated_lufs: m.integrated_lufs,
                    true_peak_dbtp: m.true_peak_dbtp,
                };
                let Some(gain_db) = decide_gain_db(&analysis, config) else {
                    return out;
                };

                let changed = match state.logged_gain_db {
                    Some(prev) => (prev - gain_db).abs() > 0.01,
                    None => true,
                };
                if changed {
                    state.logged_gain_db = Some(gain_db);
                    let limited_by_peak = analysis.true_peak_dbtp.is_finite()
                        && (config.true_peak_ceiling - analysis.true_peak_dbtp) <= gain_db + 1e-9;
                    tracing::info!(
                        "適用: {:+.2} dB (実測 {:.2} LUFS / {:.2} dBTP → ターゲット {:.1} LUFS{}) (effect_id {effect_id})",
                        gain_db,
                        analysis.integrated_lufs.unwrap_or(f64::NAN),
                        analysis.true_peak_dbtp,
                        config.target_lufs,
                        if limited_by_peak {
                            "、トゥルーピーク上限で頭打ち"
                        } else {
                            ""
                        }
                    );
                }

                if gain_db.abs() < 1e-3 {
                    return out;
                }
                let gain = 10.0_f64.powf(gain_db / 20.0) as f32;
                for s in left.iter_mut() {
                    *s *= gain;
                }
                for s in right.iter_mut() {
                    *s *= gain;
                }
                out.modified = true;
                out
            }
        }
    }

    /// テスト用: 状態を取る（`state_for` と同じ）。
    #[cfg(test)]
    fn get(&mut self, mode: Mode, sample_rate: f64, sample_total: u64) -> (&mut EffectState, Option<(f64, u64)>) {
        state_for(&mut self.state, mode, sample_rate, sample_total, false)
    }
}

/// 静的モードで測っている間の 1 回。音は変えない（素通し）。
fn measure_static(
    state: &mut EffectState,
    config: &FilterConfig,
    b: Block,
    left: &[f32],
    right: &[f32],
    out: &mut Outcome,
) {
    let effect_id = b.effect_id;
    if !state.logged_start {
        state.logged_start = true;
        if b.sample_total == 0 {
            tracing::error!(
                "静的: オブジェクトの総サンプル数が 0 で解析できない。リアルタイムモードを使ってください (effect_id {effect_id})"
            );
        } else {
            tracing::info!(
                "静的: 解析開始 — {}Hz / 総 {} サンプル / 最初の位置 {} / ブロック {} サンプル (effect_id {effect_id})",
                b.sample_rate,
                b.sample_total,
                b.sample_index,
                left.len()
            );
        }
    }
    if b.sample_total == 0 {
        return;
    }

    let analyzer = state
        .analyzer
        .get_or_insert_with(|| static_mode::StaticAnalyzer::new(b.sample_rate, b.sample_total));
    if let Some(stall) = analyzer.feed(b.sample_index, left, right) {
        // 確定しないまま通しをやり直した。原因の見当ごとに 1 回だけ知らせる。
        let rate = b.sample_rate;
        if stall.tail_unreached {
            if !state.warned_tail {
                state.warned_tail = true;
                tracing::warn!(
                    "静的: 測定が {:.0}% のまま新しい通しが始まった。届いた一番後ろは {:.1} 秒（全長 {:.1} 秒）で、末尾が一度も届いていない。\
                     途中で止めただけなら気にしなくてよい。書き出しでこれが出るなら、オブジェクトの末尾が出力範囲（範囲選択）より後ろにあり、\
                     その部分の音が配られないので測定が確定しない。全体が入る範囲で 1 回通してください (effect_id {effect_id})",
                    stall.coverage * 100.0,
                    stall.furthest_end as f64 / rate,
                    b.sample_total as f64 / rate
                );
            }
        } else if !state.warned_gaps {
            state.warned_gaps = true;
            tracing::warn!(
                "静的: 測定が {:.0}% のまま新しい通しが始まった。途中に未測定の区間が計 {:.1} 秒ある（再生位置を飛ばした所と、飛ばした直後の 0.2 秒）。\
                 頭から終わりまで止めずに 1 回通すと埋まる (effect_id {effect_id})",
                stall.coverage * 100.0,
                stall.missing_parts as f64 * 0.1
            );
        }
    }

    match analyzer.result() {
        Some(m) => {
            let coverage = analyzer.coverage();
            // 確定したので貯めた分は要らない
            state.analyzer = None;
            state.measured = Some(m);
            // この通しの残りは素通しのまま。次の通しから掛ける。
            state.hold = Some((b.sample_index, b.sample_index + left.len() as u64));
            out.persist = Some(Persisted {
                magic: PERSIST_MAGIC,
                has_loudness: u32::from(m.integrated_lufs.is_some()),
                sample_rate: b.sample_rate as u32,
                force_flag: u32::from(config.force_reanalyze),
                sample_total: b.sample_total,
                integrated_lufs: m.integrated_lufs.unwrap_or(f64::NAN),
                true_peak_dbtp: m.true_peak_dbtp,
            });
            match m.integrated_lufs {
                Some(lufs) => tracing::info!(
                    "測定完了: {:.2} LUFS / {:.2} dBTP（カバー率 {:.1}%）。この通しの残りは素通しのまま、次の再生・書き出しから掛かる (effect_id {effect_id})",
                    lufs,
                    m.true_peak_dbtp,
                    coverage * 100.0
                ),
                None => tracing::info!(
                    "測定完了: 実質無音（ゲートを通過するブロックなし） (effect_id {effect_id})"
                ),
            }
        }
        None => {
            // 進捗を 10% 刻みで出す。毎ブロック出すとログが読めなくなる。
            let coverage = analyzer.coverage();
            if coverage >= state.logged_coverage + 0.1 {
                state.logged_coverage = (coverage / 0.1).floor() * 0.1;
                tracing::info!(
                    "解析中: {:.0}%（{}/{} パート。全区間を通すとゲインが決まります） (effect_id {effect_id})",
                    coverage * 100.0,
                    analyzer.filled_parts(),
                    analyzer.part_count()
                );
            }
        }
    }
}

#[aviutl2::plugin(FilterPlugin)]
struct LoudnessNormalize;

/// 測定結果と設定から、実際に掛けるゲイン（dB）を決める。
///
/// ラウドネスを合わせるゲインと、トゥルーピークを超えないゲインの
/// 小さい方を採る。さらに `max_gain_db` で持ち上げすぎを抑える。
fn decide_gain_db(analysis: &Analysis, config: &FilterConfig) -> Option<f64> {
    let measured = analysis.integrated_lufs?;
    let loudness_gain = config.target_lufs - measured;
    let peak_gain = if analysis.true_peak_dbtp.is_finite() {
        config.true_peak_ceiling - analysis.true_peak_dbtp
    } else {
        f64::INFINITY
    };
    // 最大ゲインは持ち上げ側だけを抑える。下げる方向は制限しない。
    Some(loudness_gain.min(peak_gain).min(config.max_gain_db))
}

impl FilterPlugin for LoudnessNormalize {
    // 効果ごとの状態。エフェクトが消えたら本体が片付ける（v2.0.1 までは DashMap に持ち、消えても残っていた）
    type Userdata = EffectSlot;

    fn new(_info: aviutl2::AviUtl2Info) -> aviutl2::AnyResult<Self> {
        aviutl2::tracing_subscriber::fmt()
            .with_max_level(if cfg!(debug_assertions) {
                tracing::Level::DEBUG
            } else {
                tracing::Level::INFO
            })
            .event_format(aviutl2::logger::AviUtl2Formatter)
            .with_writer(aviutl2::logger::AviUtl2LogWriter)
            .init();
        Ok(Self)
    }

    fn plugin_info(&self) -> FilterPluginTable {
        FilterPluginTable {
            name: "ラウドネスノーマライズ_H".to_string(),
            label: None,
            information: format!(
                "ラウドネスノーマライズ_H v{version} / EBU R128 / ITU-R BS.1770-4 by HexBrowns",
                version = env!("CARGO_PKG_VERSION")
            ),
            flags: aviutl2::bitflag!(aviutl2::filter::FilterPluginFlags {
                audio: true,
                filter: true,
            }),
            config_items: FilterConfig::to_config_items(),
        }
    }

    fn proc_audio(
        &self,
        config: &[aviutl2::filter::FilterConfigItem],
        audio: &mut FilterProcAudio<Self::Userdata>,
    ) -> anyhow::Result<()> {
        let config: FilterConfig = config.to_struct();
        if config.bypass {
            return Ok(());
        }

        let block = Block {
            effect_id: audio.object.effect_id,
            sample_rate: audio.scene.sample_rate as f64,
            sample_total: audio.audio_object.sample_total,
            sample_index: audio.audio_object.sample_index,
        };
        let n = audio.audio_object.sample_num as usize;
        let channels = audio.audio_object.channel_num.max(1);
        if n == 0 || block.sample_rate <= 0.0 {
            return Ok(());
        }
        let stored = *config.measurement.read();

        // 状態のロックは書き戻しまで持つ（受け渡しのバッファも状態の中にある）
        let mut guard = audio.userdata.write();
        let slot = &mut *guard;
        slot.left.resize(n, 0.0);
        slot.right.resize(n, 0.0);

        // 実際に鳴っている音を取る。上流のフィルタ効果を通った後のもの。
        audio.get_sample_data(AudioChannel::Left, &mut slot.left);
        if channels >= 2 {
            audio.get_sample_data(AudioChannel::Right, &mut slot.right);
        } else {
            // モノラルはステレオで鳴らしたときの聞こえ方に合わせて左右同じとして測る。
            slot.right.copy_from_slice(&slot.left);
        }

        let outcome = slot.process(&config, stored, block, n, Instant::now());
        if let Some(p) = outcome.persist {
            *config.measurement.write() = p;
        }
        if outcome.modified {
            audio.set_sample_data(AudioChannel::Left, &slot.left);
            if channels >= 2 {
                audio.set_sample_data(AudioChannel::Right, &slot.right);
            }
        }

        Ok(())
    }
}

aviutl2::register_filter_plugin!(LoudnessNormalize);

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn config(target: f64, ceiling: f64, max_gain: f64) -> FilterConfig {
        let mut c: FilterConfig = FilterConfig::to_config_items().as_slice().to_struct();
        c.target_lufs = target;
        c.true_peak_ceiling = ceiling;
        c.max_gain_db = max_gain;
        c
    }

    #[test]
    fn boosts_to_target_when_peak_allows() {
        let a = Analysis {
            integrated_lufs: Some(-23.0),
            true_peak_dbtp: -20.0,
        };
        let g = decide_gain_db(&a, &config(-14.0, -1.0, 12.0)).unwrap();
        assert!((g - 9.0).abs() < 1e-9, "{g}");
    }

    #[test]
    fn peak_ceiling_wins_over_target() {
        // ラウドネス的には +9 dB 欲しいが、ピークは +2 dB しか余裕が無い。
        let a = Analysis {
            integrated_lufs: Some(-23.0),
            true_peak_dbtp: -3.0,
        };
        let g = decide_gain_db(&a, &config(-14.0, -1.0, 12.0)).unwrap();
        assert!((g - 2.0).abs() < 1e-9, "{g}");
    }

    #[test]
    fn max_gain_caps_the_boost() {
        let a = Analysis {
            integrated_lufs: Some(-40.0),
            true_peak_dbtp: -40.0,
        };
        let g = decide_gain_db(&a, &config(-14.0, -1.0, 12.0)).unwrap();
        assert!((g - 12.0).abs() < 1e-9, "{g}");
    }

    #[test]
    fn loud_source_is_turned_down_without_max_gain_limit() {
        // 最大ゲインは持ち上げ側だけを抑える。下げる方向は制限しない。
        let a = Analysis {
            integrated_lufs: Some(-6.0),
            true_peak_dbtp: -0.5,
        };
        let g = decide_gain_db(&a, &config(-14.0, -1.0, 0.0)).unwrap();
        assert!((g - (-8.0)).abs() < 1e-9, "{g}");
    }

    #[test]
    fn silence_gets_no_gain() {
        let a = Analysis {
            integrated_lufs: None,
            true_peak_dbtp: f64::NEG_INFINITY,
        };
        assert!(decide_gain_db(&a, &config(-14.0, -1.0, 12.0)).is_none());
    }

    /// 保存データは既定値だと無効で、測り直しの対象になる。
    #[test]
    fn default_persisted_is_invalid() {
        assert_ne!(Persisted::default().magic, PERSIST_MAGIC);
    }

    /// 保存データの大きさと並びは v2.0.2 と同じ（`_pad` を `force_flag` にしただけ）。
    #[test]
    fn persisted_layout_is_unchanged() {
        assert_eq!(std::mem::size_of::<Persisted>(), 40);
        assert_eq!(std::mem::offset_of!(Persisted, force_flag), 12);
        assert_eq!(std::mem::offset_of!(Persisted, sample_total), 16);
    }

    /// 本体が作った直後のユーザーデータは空で、最初の呼び出しで状態を作る。
    #[test]
    fn slot_is_created_lazily() {
        let mut slot = <EffectSlot as aviutl2::filter::FilterUserdata>::new(42);
        assert!(slot.state.is_none());
        let (state, renewed) = slot.get(Mode::Static, 48000.0, 480_000);
        assert!(renewed.is_none());
        assert_eq!(state.sample_total, 480_000);
        assert!(state.analyzer.is_none(), "測るまで貯める場所は作らない");
        assert!(slot.state.is_some());
    }

    /// 同じ素性なら状態（貯めた測定）を持ち越し、モードだけ追従する。
    #[test]
    fn slot_keeps_state_while_source_is_same() {
        let mut slot = EffectSlot::default();
        slot.get(Mode::Static, 48000.0, 480_000).0.logged_start = true;
        let (state, renewed) = slot.get(Mode::Realtime, 48000.0, 480_000);
        assert!(renewed.is_none());
        assert!(state.logged_start, "作り直していない");
        assert_eq!(state.mode, Mode::Realtime);
    }

    /// 素性（長さ・レート）が変わったら作り直し、前の値を返す。「解析をやり直す」の前回値は引き継ぐ。
    #[test]
    fn slot_renews_when_source_changes() {
        let mut slot = EffectSlot::default();
        {
            let s = slot.get(Mode::Static, 48000.0, 480_000).0;
            s.logged_start = true;
            s.last_force = true;
        }
        let (state, renewed) = slot.get(Mode::Static, 44100.0, 441_000);
        assert_eq!(renewed, Some((48000.0, 480_000)));
        assert!(!state.logged_start, "作り直した");
        assert!(state.last_force, "前回値は引き継ぐ（作り直しを立ち上がりにしない）");
        assert_eq!(state.sample_total, 441_000);
    }

    // ---- 1 回の処理を通したテスト ----

    const FS: f64 = 48000.0;
    const CHUNK: usize = 1600;

    fn sine(freq: f64, peak_dbfs: f64, secs: f64) -> Vec<f32> {
        let amp = 10.0_f64.powf(peak_dbfs / 20.0);
        let n = (FS * secs) as usize;
        (0..n)
            .map(|i| (amp * (2.0 * PI * freq * i as f64 / FS).sin()) as f32)
            .collect()
    }

    /// `proc_audio` の代わり。`stored` は本体に保存されている測定値の代わりで、書き込みも反映する。
    struct Host {
        slot: EffectSlot,
        stored: Persisted,
        cfg: FilterConfig,
        now: Instant,
    }

    impl Host {
        fn new(cfg: FilterConfig) -> Self {
            Self {
                slot: EffectSlot::default(),
                stored: Persisted::default(),
                cfg,
                now: Instant::now(),
            }
        }

        /// 1 ブロック流して、出力（素通しなら入力のまま）と結果を返す。
        fn call(&mut self, sig: &[f32], pos: usize) -> (Vec<f32>, Outcome) {
            let end = (pos + CHUNK).min(sig.len());
            let n = end - pos;
            self.slot.left.clear();
            self.slot.left.extend_from_slice(&sig[pos..end]);
            self.slot.right.clear();
            self.slot.right.extend_from_slice(&sig[pos..end]);
            let b = Block {
                effect_id: 1,
                sample_rate: FS,
                sample_total: sig.len() as u64,
                sample_index: pos as u64,
            };
            self.now += Duration::from_millis(33);
            let out = self.slot.process(&self.cfg, self.stored, b, n, self.now);
            if let Some(p) = out.persist {
                self.stored = p;
            }
            let data = if out.modified {
                self.slot.left[..n].to_vec()
            } else {
                sig[pos..end].to_vec()
            };
            (data, out)
        }

        /// 頭から終わりまで 1 回通す。ブロックごとに「書き換えたか」を返す。
        fn pass(&mut self, sig: &[f32]) -> Vec<bool> {
            let mut pos = 0;
            let mut modified = Vec::new();
            while pos < sig.len() {
                let (_, out) = self.call(sig, pos);
                modified.push(out.modified);
                pos += CHUNK;
            }
            modified
        }

        fn measured(&self) -> bool {
            self.slot.state.as_ref().is_some_and(|s| s.measured.is_some())
        }
    }

    /// 1 回目の通しの途中（98%）で確定しても、その通しは最後まで素通し。次の通しは頭から掛かる。
    #[test]
    fn first_pass_stays_unprocessed_after_confirming() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));

        let first = host.pass(&sig);
        assert!(host.measured(), "1 周で確定していない");
        assert!(first.iter().all(|m| !m), "1 回目の通しで書き換えたブロックがある");
        assert!(host.stored.is_valid(), "測定値を保存していない");

        let second = host.pass(&sig);
        assert!(second.iter().all(|m| *m), "2 回目の通しで掛かっていないブロックがある");
        let (out, _) = host.call(&sig, 0);
        let ratio = out[100] / sig[100];
        assert!((20.0 * (ratio as f64).log10() - 9.0).abs() < 0.2, "ゲイン比 {ratio}");
    }

    /// 確定した直後に同じブロックを描き直しても、通しの続きとして素通しのまま。
    #[test]
    fn redraw_of_confirming_block_keeps_holding() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));
        let mut pos = 0;
        while !host.measured() {
            host.call(&sig, pos);
            pos += CHUNK;
        }
        let confirm = pos - CHUNK;
        assert!(!host.call(&sig, confirm).1.modified, "描き直しで掛かった");
        assert!(!host.call(&sig, pos).1.modified, "続きで掛かった");
        // 途中へ飛んだら通しは終わり
        assert!(host.call(&sig, CHUNK * 10).1.modified, "飛んだ後に掛かっていない");
    }

    /// 「解析をやり直す」を ON のまま保存したプロジェクトを開き直しても、測り直さない。
    #[test]
    fn force_left_on_does_not_reanalyze_after_reopen() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut cfg = config(-14.0, -1.0, 12.0);
        cfg.force_reanalyze = true;
        let mut host = Host::new(cfg.clone());
        host.pass(&sig);
        assert!(host.measured());
        assert_eq!(host.stored.force_flag, 1, "ON を見たことを保存していない");

        // 開き直す（状態は無くなり、保存した値だけが残る）
        let stored = host.stored;
        let mut reopened = Host::new(cfg);
        reopened.stored = stored;
        let (_, out) = reopened.call(&sig, 0);
        assert!(out.modified, "保存済みの値で掛かっていない");
        assert_eq!(out.persist, None, "測定値を捨てた（測り直した）");
        assert!(reopened.measured());
    }

    /// 開き直してから一度も音を流さないうちに OFF → ON にしても、ちゃんと測り直す。
    #[test]
    fn force_turned_on_before_first_block_still_reanalyzes() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));
        host.pass(&sig);
        assert_eq!(host.stored.force_flag, 0);

        let stored = host.stored;
        let mut cfg = config(-14.0, -1.0, 12.0);
        cfg.force_reanalyze = true;
        let mut reopened = Host::new(cfg);
        reopened.stored = stored;
        let (_, out) = reopened.call(&sig, 0);
        assert!(!out.modified, "測り直しの間は素通し");
        assert_eq!(out.persist, Some(Persisted::default()), "保存済みを捨てていない");
        assert!(!reopened.measured());
    }

    /// v2.0.2 で ON のまま保存したもの（force_flag が 0）は、開いたときに 1 回だけ測り直す。
    #[test]
    fn force_on_from_old_version_reanalyzes_once() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut cfg = config(-14.0, -1.0, 12.0);
        cfg.force_reanalyze = true;
        let mut host = Host::new(cfg.clone());
        host.pass(&sig);
        let mut old = host.stored;
        old.force_flag = 0;

        let mut reopened = Host::new(cfg);
        reopened.stored = old;
        let (_, out) = reopened.call(&sig, 0);
        assert_eq!(out.persist, Some(Persisted::default()), "1 回目は測り直す");
        reopened.pass(&sig);
        assert!(reopened.measured());
        assert_eq!(reopened.stored.force_flag, 1);
        let (_, out) = reopened.call(&sig, 0);
        assert!(out.modified && out.persist.is_none(), "2 回目以降は測り直さない");
    }

    /// 長さの違う保存済みの値は使わない（別の素材の値を流用しない）。
    #[test]
    fn stored_measurement_for_other_length_is_ignored() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));
        host.pass(&sig);
        let stored = host.stored;

        let shorter = &sig[..sig.len() - CHUNK * 3];
        let mut other = Host::new(config(-14.0, -1.0, 12.0));
        other.stored = stored;
        let (_, out) = other.call(shorter, 0);
        assert!(!out.modified, "長さの違う測定値を使った");
        assert!(!other.measured());
    }

    /// 素性の変化のログ: 1 回目は出す。短い間に続いたら黙り、何度も続いたら警告を 1 回出して以後は黙る。
    #[test]
    fn renew_log_is_throttled() {
        let mut log = RenewLog::default();
        let t0 = Instant::now();
        assert_eq!(log.on_renew(t0), RenewNotice::Info);
        // ゆっくりなら毎回 1 行
        assert_eq!(log.on_renew(t0 + Duration::from_secs(5)), RenewNotice::Info);
        // 連打
        let mut t = t0 + Duration::from_secs(5);
        let mut notices = Vec::new();
        for _ in 0..100 {
            t += Duration::from_millis(33);
            notices.push(log.on_renew(t));
        }
        assert_eq!(notices.iter().filter(|n| **n == RenewNotice::Warn).count(), 1);
        assert_eq!(notices.iter().filter(|n| **n == RenewNotice::Info).count(), 0);
        // 黙った後は、間が空いても出さない
        assert_eq!(log.on_renew(t + Duration::from_secs(60)), RenewNotice::Silent);
    }

    /// 1 つの効果に長さの違う 2 本の音が交互に来ても（グループ制御の疑い）、落ちず、片方の値を他方に使わない。
    #[test]
    fn alternating_sources_do_not_share_measurement() {
        let a = sine(1000.0, -23.0, 10.0);
        let b = sine(1000.0, -33.0, 12.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));
        let mut pos = 0;
        while pos < b.len() {
            if pos < a.len() {
                assert!(!host.call(&a, pos).1.modified);
            }
            assert!(!host.call(&b, pos).1.modified);
            pos += CHUNK;
        }
        assert!(!host.measured(), "交互に来る音で測定が確定した");
        assert!(host.slot.renew_log.quiet, "連打の警告に入っていない");
    }

    /// リアルタイムのゲインのログはエフェクトごとに数行まで。
    #[test]
    fn realtime_log_is_capped() {
        let sig = sine(1000.0, -33.0, 30.0);
        let mut cfg = config(-14.0, -1.0, 30.0);
        cfg.mode = Mode::Realtime;
        cfg.rate_db_per_sec = 12.0;
        let mut host = Host::new(cfg);
        host.pass(&sig);
        let s = host.slot.state.as_ref().unwrap();
        assert_eq!(s.realtime_log_lines, REALTIME_LOG_LINES);
        assert!(s.leveler.gain_db() > 10.0, "ゲインが動いていない: {}", s.leveler.gain_db());
    }

    /// 測定が確定したら、貯めた分（パートの配列）を放す。
    #[test]
    fn analyzer_is_released_after_confirming() {
        let sig = sine(1000.0, -23.0, 20.0);
        let mut host = Host::new(config(-14.0, -1.0, 12.0));
        host.call(&sig, 0);
        assert!(host.slot.state.as_ref().unwrap().analyzer.is_some());
        host.pass(&sig);
        assert!(host.measured());
        assert!(host.slot.state.as_ref().unwrap().analyzer.is_none(), "確定後も持っている");
    }
}

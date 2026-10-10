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
//! - **レンダースレッドで無制限の I/O をしない**。v2 はファイルを読まないので
//!   そもそも I/O が無い。

mod leveler;
mod r128;
mod static_mode;

use aviutl2::{
    filter::{
        AudioChannel, FilterConfigDataHandle, FilterConfigItemSliceExt, FilterConfigItems,
        FilterPlugin, FilterPluginTable, FilterProcAudio,
    },
    tracing,
};

/// 保存データの版。構造を変えたら上げる。合わなければ捨てて測り直す。
const PERSIST_MAGIC: u32 = 0x4C4E_4832; // "LNH2"

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
    _pad: u32,
    /// 測ったときのオブジェクト総サンプル数。同上。
    sample_total: u64,
    integrated_lufs: f64,
    true_peak_dbtp: f64,
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
    analyzer: static_mode::StaticAnalyzer,
    leveler: leveler::Leveler,
    /// 静的モードで確定した測定値。
    measured: Option<static_mode::Measured>,
    /// 「解析をやり直す」の直近の状態。立ち上がりを拾うために持つ。
    last_force: bool,
    /// 直近にログへ出したゲイン。毎ブロック出さないための記録。
    logged_gain_db: Option<f64>,
    /// 直近にログへ出したカバー率。進捗ログを間引くための記録。
    logged_coverage: f64,
    /// 静的モードの開始ログを出したか。
    logged_start: bool,
}

impl EffectState {
    fn new(mode: Mode, sample_rate: f64, sample_total: u64) -> Self {
        Self {
            mode,
            sample_rate,
            sample_total,
            analyzer: static_mode::StaticAnalyzer::new(sample_rate, sample_total),
            leveler: leveler::Leveler::new(sample_rate),
            measured: None,
            last_force: false,
            logged_gain_db: None,
            logged_coverage: -1.0,
            logged_start: false,
        }
    }

    /// オブジェクトの素性が変わっていたら作り直す。
    /// 長さやレートが変われば、前の測定値は当てにならない。
    fn ensure(&mut self, mode: Mode, sample_rate: f64, sample_total: u64) -> Option<(f64, u64)> {
        if self.sample_rate != sample_rate || self.sample_total != sample_total {
            let old = (self.sample_rate, self.sample_total);
            *self = Self::new(mode, sample_rate, sample_total);
            return Some(old);
        }
        self.mode = mode;
        None
    }

    fn reset_analysis(&mut self) {
        self.analyzer = static_mode::StaticAnalyzer::new(self.sample_rate, self.sample_total);
        self.measured = None;
        self.logged_gain_db = None;
        self.logged_coverage = -1.0;
        self.logged_start = false;
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
}

impl aviutl2::filter::FilterUserdata for EffectSlot {
    fn new(_effect_id: i64) -> Self {
        Self::default()
    }
}

impl EffectSlot {
    /// 状態を取る。無ければ作り、素性（レート・長さ）が変わっていたら作り直す。
    /// 作り直したときは前の (レート, 総サンプル数) を返す。
    fn get(&mut self, mode: Mode, sample_rate: f64, sample_total: u64) -> (&mut EffectState, Option<(f64, u64)>) {
        match self.state {
            Some(ref mut state) => {
                let old = state.ensure(mode, sample_rate, sample_total);
                (state, old)
            }
            None => (self.state.insert(EffectState::new(mode, sample_rate, sample_total)), None),
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

        let effect_id = audio.object.effect_id;
        let sample_rate = audio.scene.sample_rate as f64;
        let sample_total = audio.audio_object.sample_total;
        let sample_index = audio.audio_object.sample_index;
        let n = audio.audio_object.sample_num as usize;
        let channels = audio.audio_object.channel_num.max(1);
        if n == 0 || sample_rate <= 0.0 {
            return Ok(());
        }

        // 実際に鳴っている音を取る。上流のフィルタ効果を通った後のもの。
        let mut left = vec![0.0f32; n];
        let mut right = vec![0.0f32; n];
        audio.get_sample_data(AudioChannel::Left, &mut left);
        if channels >= 2 {
            audio.get_sample_data(AudioChannel::Right, &mut right);
        } else {
            // モノラルはステレオで鳴らしたときの聞こえ方に合わせて左右同じとして測る。
            right.copy_from_slice(&left);
        }

        // 状態のロックはゲインを決め終えるまで持ち、書き戻し（set_sample_data）の前に放す
        let mut slot = audio.userdata.write();
        let (state, renewed) = slot.get(config.mode, sample_rate, sample_total);

        if let Some((old_rate, old_total)) = renewed {
            tracing::info!(
                "オブジェクトの素性が変わったので測り直す: {}Hz {} サンプル → {}Hz {} サンプル (effect_id {effect_id})",
                old_rate,
                old_total,
                sample_rate,
                sample_total
            );
        }

        // 「解析をやり直す」の立ち上がりで、保存済みも含めて捨てる。
        if config.force_reanalyze && !state.last_force {
            state.reset_analysis();
            *config.measurement.write() = Persisted::default();
            tracing::info!("解析をやり直す (effect_id {effect_id})");
        }
        state.last_force = config.force_reanalyze;

        let gain_db = match config.mode {
            Mode::Realtime => {
                let params = leveler::LevelerParams {
                    target_lufs: config.target_lufs,
                    true_peak_ceiling: config.true_peak_ceiling,
                    max_gain_db: config.max_gain_db,
                    min_gain_db: config.min_gain_db,
                    rate_db_per_sec: config.rate_db_per_sec,
                };
                state
                    .leveler
                    .process(sample_index, &mut left, &mut right, &params);

                let g = state.leveler.gain_db();
                // 連続的に動くので、1 dB 以上変わったときだけ出す。
                let changed = match state.logged_gain_db {
                    Some(prev) => (prev - g).abs() >= 1.0,
                    None => true,
                };
                if changed {
                    let target = state.leveler.target_db();
                    state.logged_gain_db = Some(g);
                    tracing::info!(
                        "リアルタイム: ゲイン {:+.2} dB（目標 {:+.2} dB） (effect_id {effect_id})",
                        g,
                        target
                    );
                }
                // レベラーは自分で掛けたので、ここでは追加のゲインは無い。
                None
            }
            Mode::Static => {
                // 保存済みの測定値があれば、素性が一致する限りそれを使う。
                if state.measured.is_none() {
                    let stored = *config.measurement.read();
                    if stored.magic == PERSIST_MAGIC
                        && stored.sample_rate == sample_rate as u32
                        && stored.sample_total == sample_total
                    {
                        state.measured = Some(static_mode::Measured {
                            integrated_lufs: (stored.has_loudness != 0)
                                .then_some(stored.integrated_lufs),
                            true_peak_dbtp: stored.true_peak_dbtp,
                        });
                        tracing::info!(
                            "保存済みの測定値を使う: {:.2} LUFS / {:.2} dBTP (effect_id {effect_id})",
                            stored.integrated_lufs,
                            stored.true_peak_dbtp
                        );
                    }
                }

                if !state.logged_start {
                    state.logged_start = true;
                    if sample_total == 0 {
                        tracing::error!(
                            "静的: オブジェクトの総サンプル数が 0 で解析できない。                             リアルタイムモードを使ってください (effect_id {effect_id})"
                        );
                    } else {
                        tracing::info!(
                            "静的: 解析開始 — {}Hz / 総 {} サンプル / {} パート / 最初の位置 {} / ブロック {} サンプル (effect_id {effect_id})",
                            sample_rate,
                            sample_total,
                            state.analyzer.part_count(),
                            sample_index,
                            n
                        );
                    }
                }

                if state.measured.is_none() {
                    // まだ全区間が揃っていない。実際の音を貯める。
                    state.analyzer.feed(sample_index, &left, &right);

                    if let Some(m) = state.analyzer.result() {
                        state.measured = Some(m);
                        *config.measurement.write() = Persisted {
                            magic: PERSIST_MAGIC,
                            has_loudness: u32::from(m.integrated_lufs.is_some()),
                            sample_rate: sample_rate as u32,
                            _pad: 0,
                            sample_total,
                            integrated_lufs: m.integrated_lufs.unwrap_or(f64::NAN),
                            true_peak_dbtp: m.true_peak_dbtp,
                        };
                        let coverage = state.analyzer.coverage();
                        match m.integrated_lufs {
                            Some(lufs) => tracing::info!(
                                "測定完了: {:.2} LUFS / {:.2} dBTP（カバー率 {:.1}%） (effect_id {effect_id})",
                                lufs,
                                m.true_peak_dbtp,
                                coverage * 100.0
                            ),
                            None => tracing::info!(
                                "測定完了: 実質無音（ゲートを通過するブロックなし） (effect_id {effect_id})"
                            ),
                        }
                    } else {
                        // 進捗を 25% 刻みで出す。毎ブロック出すとログが読めなくなる。
                        let coverage = state.analyzer.coverage();
                        if coverage >= state.logged_coverage + 0.1 {
                            state.logged_coverage = (coverage / 0.1).floor() * 0.1;
                            tracing::info!(
                                "解析中: {:.0}%（{}/{} パート。全区間を通すとゲインが決まります） (effect_id {effect_id})",
                                coverage * 100.0,
                                state.analyzer.filled_parts(),
                                state.analyzer.part_count()
                            );
                        }
                        // 揃うまでは素通し。
                        return Ok(());
                    }
                }

                let m = state.measured.expect("直前で埋めた");
                let analysis = Analysis {
                    integrated_lufs: m.integrated_lufs,
                    true_peak_dbtp: m.true_peak_dbtp,
                };
                let Some(gain_db) = decide_gain_db(&analysis, &config) else {
                    return Ok(());
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
                Some(gain_db)
            }
        };
        drop(slot);

        // 静的モードのゲインをここで掛ける（リアルタイムは既に掛かっている）。
        if let Some(gain_db) = gain_db {
            if gain_db.abs() < 1e-3 {
                return Ok(());
            }
            let gain = 10.0_f64.powf(gain_db / 20.0) as f32;
            for s in left.iter_mut() {
                *s *= gain;
            }
            for s in right.iter_mut() {
                *s *= gain;
            }
        }

        audio.set_sample_data(AudioChannel::Left, &left);
        if channels >= 2 {
            audio.set_sample_data(AudioChannel::Right, &right);
        }

        Ok(())
    }
}

aviutl2::register_filter_plugin!(LoudnessNormalize);

#[cfg(test)]
mod tests {
    use super::*;

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

    /// 本体が作った直後のユーザーデータは空で、最初の呼び出しで状態を作る。
    #[test]
    fn slot_is_created_lazily() {
        let mut slot = <EffectSlot as aviutl2::filter::FilterUserdata>::new(42);
        assert!(slot.state.is_none());
        let (state, renewed) = slot.get(Mode::Static, 48000.0, 480_000);
        assert!(renewed.is_none());
        assert_eq!(state.sample_total, 480_000);
        assert!(slot.state.is_some());
    }

    /// 同じ素性なら状態（貯めた測定）を持ち越し、モードだけ追従する。
    #[test]
    fn slot_keeps_state_while_source_is_same() {
        let mut slot = EffectSlot::default();
        slot.get(Mode::Static, 48000.0, 480_000).0.last_force = true;
        let (state, renewed) = slot.get(Mode::Realtime, 48000.0, 480_000);
        assert!(renewed.is_none());
        assert!(state.last_force, "作り直していない");
        assert_eq!(state.mode, Mode::Realtime);
    }

    /// 素性（長さ・レート）が変わったら作り直し、前の値を返す。
    #[test]
    fn slot_renews_when_source_changes() {
        let mut slot = EffectSlot::default();
        slot.get(Mode::Static, 48000.0, 480_000).0.last_force = true;
        let (state, renewed) = slot.get(Mode::Static, 44100.0, 441_000);
        assert_eq!(renewed, Some((48000.0, 480_000)));
        assert!(!state.last_force, "作り直した");
        assert_eq!(state.sample_total, 441_000);
    }
}

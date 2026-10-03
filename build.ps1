# build.ps1 — ラウドネスノーマライズ_H ビルド & デプロイ
#
# 使い方: .\build.ps1
#         .\build.ps1 -NoDeploy   （配置せずビルドのみ）
#         .\build.ps1 -SkipTest   （テストを飛ばす）

param(
    [switch]$NoDeploy,
    [switch]$SkipTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$SrcDir      = $PSScriptRoot
$AviUtl2Root = "C:\ProgramData\aviutl2"
$PluginDir   = "$AviUtl2Root\Plugin\LoudnessNormalize_H"
$LanguageDir = "$AviUtl2Root\Language"

Write-Host "`n=== ラウドネスノーマライズ_H ビルド (Rust) ===" -ForegroundColor Yellow

Push-Location $SrcDir
try {
    if (-not $SkipTest) {
        Write-Host "`n--- テスト ---" -ForegroundColor Cyan
        cargo test
        if ($LASTEXITCODE -ne 0) {
            throw "cargo test failed (exit $LASTEXITCODE)"
        }
    }

    Write-Host "`n--- リリースビルド ---" -ForegroundColor Cyan
    cargo build --release
    if ($LASTEXITCODE -ne 0) {
        throw "cargo build failed (exit $LASTEXITCODE)"
    }
} finally {
    Pop-Location
}

$DllPath = Join-Path $SrcDir "target\release\loudness_normalize_h.dll"
if (-not (Test-Path -LiteralPath $DllPath)) {
    throw "ビルド成果物が見つかりません: $DllPath"
}

$Auf2Out = Join-Path $SrcDir "LoudnessNormalize_H.auf2"
Copy-Item -LiteralPath $DllPath -Destination $Auf2Out -Force
$size = (Get-Item -LiteralPath $Auf2Out).Length
Write-Host ("  完了: {0} ({1:N0} bytes)" -f $Auf2Out, $size) -ForegroundColor Green

# 外部 DLL 依存の確認（VC ランタイム以外がぶら下がっていないか）
$objdump = "C:\msys64\ucrt64\bin\objdump.exe"
if (Test-Path -LiteralPath $objdump) {
    Write-Host "`n--- DLL 依存 ---" -ForegroundColor Cyan
    & $objdump -p $Auf2Out | Select-String "DLL Name" | ForEach-Object { "  $_" }
} else {
    Write-Host "`n  (objdump が無いので DLL 依存の確認は省略)" -ForegroundColor DarkGray
}

if ($NoDeploy) {
    Write-Host "`n-NoDeploy が指定されたので配置しません。" -ForegroundColor DarkGray
    return
}

Write-Host "`n=== デプロイ ===" -ForegroundColor Yellow

if (-not (Test-Path -LiteralPath $PluginDir)) {
    New-Item -ItemType Directory -Path $PluginDir -Force | Out-Null
}
Copy-Item -LiteralPath $Auf2Out -Destination "$PluginDir\LoudnessNormalize_H.auf2" -Force
Write-Host "  配置: $PluginDir\LoudnessNormalize_H.auf2" -ForegroundColor DarkGreen

$Lang = Join-Path $SrcDir "i18n\English.loudness_normalize_h.aul2"
if (Test-Path -LiteralPath $Lang) {
    Copy-Item -LiteralPath $Lang -Destination "$LanguageDir\English.loudness_normalize_h.aul2" -Force
    Write-Host "  配置: $LanguageDir\English.loudness_normalize_h.aul2" -ForegroundColor DarkGreen
}

Write-Host "`n※ プラグインの差し替えは AviUtl2 の再起動が必要です。" -ForegroundColor Yellow

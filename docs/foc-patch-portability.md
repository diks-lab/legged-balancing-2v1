# SimpleFOC観測パッチのWindows対応と読み取り専用診断

## 調査結果

98248aeのパッチは `Path.read_text()` の既定文字コードで読み、UTF-8へ再エンコードして
SHA-256を計算していた。既定文字コードは環境依存である。
対象のUTF-8ソースには非ASCII文字があり、CP932またはCP1252で解釈すると、
例外にならず別の文字列へ変わってハッシュ不一致となるケースを再現した。
UTF-8モードを使わないWindowsのPythonで発生し得る。

旧 `read_text()` はCRLF/LFを既にLFへ正規化するため、単純な改行差だけではこの不一致は再現しない。
PC上の実際の既定文字コード・ソース内容はまだ取得していないので、今回のPCでの原因を
文字コードだけに断定しない。別コミットの取得、手作業変更、途中までの適用も診断で区別する。

現在の基準はSimpleFOC **2.4.0**、checkout **4f072b365f6e0185adca544071e595834405babc**。
こちらの取得ソースはGit blobと一致するUTF-8/LFだった。

| LFへ正規化したファイル全体 | SHA-256 |
|---|---|
| 未適用 | `c119b0e44313f4e36a3886ba40762b7e4b5244e0f82f9d7497efee3a4b765d1f` |
| 適用済み | `6b970b2ece6416f36f22bf1fd1c91bcbb98c663e6914159485e0b095117412a4` |

## 照合・書き込み

- バイト列で読み、CRLFだけをLFへ変換した全体ハッシュで照合する。
  文字はUTF-8として明示的に検証し、環境の既定コードページに依存しない。
- 未適用ハッシュ一致時だけ既存の8か所の観測追加を行い、生成後の全体ハッシュも確認する。
  変換内容と期待する両SHAは98248aeから変更していない。
- 適用済みハッシュ一致時は書き込みを行わず、内容・改行・更新時刻を保持する。
- 新規適用時も入力がLFならLF、CRLFならCRLFをバイナリ書き込みで保持する。
  Windowsの自動改行変換によるCRCRLFや、既定文字コードへの再変換を避ける。
- 混在改行、単独CR、UTF-8 BOM、不正UTF-8、未知の全体ハッシュ、部分適用、
  アンカー不一致、生成結果不一致は書き換えず拒否する。
  空白・コメント・制御式の差も改行差として黙認しない。
- メタデータのバージョン/checkoutは診断情報であり、それだけではソースを受け入れない。
  メタデータが欠けるアーカイブ等でも、既知の全体ハッシュ一致が照合の根拠となる。
  アンカー一致だけで未知ソースを適用しない。

## PC側の読み取り専用コマンド

更新したブランチのプロジェクトルートで実行する。適用処理やファイル書き込みは行わない。

```powershell
python scripts/patch_foc_observation.py --check
```

パスを明示する場合：

```powershell
python scripts/patch_foc_observation.py --check ".pio/libdeps/esp32dev/Arduino-FOC/src/common/base_classes/FOCMotor.cpp"
```

`python` がPlatformIOのものではない場合は、必要に応じて次を使用できる。

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" scripts/patch_foc_observation.py --check
```

出力は以下に限定する。

- `declared_version`：library.propertiesのversion。取得できなければunknown。
- `checkout_commit`：.git/HEADが直接示す40桁ハッシュ。取得できなければunknown。
- `raw_sha256`：対象ファイルのバイト列そのもののハッシュ。
- `lf_normalized_sha256`：CRLF→LFだけを行ったハッシュ。
- 改行形式、CRLF/LFのみ/単独CRの件数、UTF-8判定。
- 未適用/適用済みの全体SHA一致結果、アンカーごとの出現数、拒否した照合条件。

ソース本文、ライブラリの作者・remote URL、環境変数、Wi-Fi設定、任意のメタデータや
ローカル絶対パスは診断へ表示しない。version/checkoutも許可した書式だけを表示する。
通常のビルドでも不一致時に同じ診断を出してRuntimeErrorで停止する。

| state / 終了コード | 意味 |
|---|---|
| ORIGINAL / 0 | 正しい未適用ソース。次の通常ビルドで適用する |
| PATCHED / 0 | 正しい適用済みソース。再書き込み不要 |
| REJECTED / 1 | encoding、line_endings、source_hash等の不一致。強制適用しない |
| 読み取り失敗 / 2 | 対象がない/読めない。ハッシュは計算できない |

anchorの元文字列が適用後文字列の一部でもある箇所では、元/適用後の両countが1となる。
適用済みかどうかはcount単独ではなくファイル全体のSHAで判定する。

## 検証範囲

`python tests/foc_patch_host/run.py` はPython標準ライブラリだけで実行する。
LF/CRLFそれぞれの未適用/適用済み、繰り返し適用、改行・更新時刻保持、
CP932/CP1252による旧ハッシュ失敗、部分適用・未知ソース・制御式変更の拒否、
混在/単独CR/BOM/不正UTF-8、アンカー・出力ハッシュ検査、診断の秘密情報非表示、
CLIの読み取り専用動作を確認する。

`tests/foc_observation_host/run.py` のライブラリ/ソース読み書きもUTF-8へ明示し、
従来の実FOC関数比較テストを維持する。

Linux環境でWindowsのコードページ解釈とCRLFファイルを再現したテストであり、
ネイティブWindowsでのPlatformIO実行成功はまだ確認していない。
制御・観測コード、パッチの追加内容、電圧・極対数・PID・停止条件は変更していない。

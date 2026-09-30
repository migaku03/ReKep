# 2026-09-30 Isaac Sim 起動クラッシュ/ハング:原因と対処

**状態: 解決済み(2026-09-30)**

`docs/widowxai_bringup_status.md` の「2026-09-30 environment incident」以降の節で調査していた件の
結論です。あちらの節にある仮説(Windows Update・OS・GPU ドライバ・キャッシュ破損など)は、
下記のとおりいずれも原因ではありませんでした。

## 症状

- `main.py` / `tools/check_pencil_holder_settle.py` が、Kit の拡張機能のロード中に落ちる、
  または応答しなくなる(9/24 までは同じ環境で安定動作していた)。
- ログには `hdf5_hl.dll` / `hdf5_cpp.dll` の「指定されたプロシージャが見つかりません」が出ていたが、
  既知の無害なノイズとして扱われていた。
- faulthandler のスレッドダンプ(`Windows fatal exception: code 0xc0000139` / `access violation`)が
  繰り返し出力され、OGN ノードスキャン(ThreadPoolExecutor)中に落ちることが多かった。

## 原因

原因は Windows の **DLL 名の衝突**でした。プロセス内に同じ名前の DLL がすでに読み込まれていると、
Windows はフォルダが違っていてもそれを使い回します。そのため、別バージョンの DLL に結び付いて
しまいます。

### 原因1(本命):hdf5 の衝突 — `main.py` / settle チェックが落ちていた原因

1. `import omnigibson` の時点で **h5py 3.16.0** が読み込まれる。h5py 3.16.0 は **HDF5 2.0.0** の
   `hdf5.dll` を同梱している。
2. その後、Kit が `omni.sensors.nv.common` 拡張を起動し、自分用の **HDF5 1.14.4** の
   `hdf5_hl.dll` / `hdf5_cpp.dll` を読み込もうとする。
3. これらは読み込み済みの HDF5 2.0.0 の `hdf5.dll` に結び付けられ、関数が見つからずに失敗する
   (`0xc0000139` = STATUS_ENTRYPOINT_NOT_FOUND)。
4. この例外を faulthandler が出力している最中に、別スレッドで OGN ノードスキャンが動いていると、
   `python310.dll` 内でアクセス違反が起きてプロセスが死ぬ。

5秒で再現する単体テスト:

```powershell
$py = "C:\Users\smiga\miniconda3\envs\behavior\python.exe"
$b  = "C:\Users\smiga\miniconda3\envs\behavior\Lib\site-packages\isaacsim\extscache\omni.sensors.nv.common-2.5.0-coreapi+wx64.r.cp310\bin"
# h5py を先に import してから Kit の DLL を読み込む(h5py 3.16.0 では WinError 127、3.14.0 では OK)
& $py -c "import os,ctypes,h5py;print(h5py.version.hdf5_version);os.add_dll_directory(r'$b');ctypes.WinDLL(r'$b\hdf5_hl.dll');print('OK')"
```

### 原因2(副次的):msvcp140 の衝突 — `tools/check_kit_bare.py` だけに影響

- Kit は古い `msvcp140.dll`(14.29、`site-packages\omni\msvcp140.dll`)を同梱している。
- **torch より先に Kit を起動すると**、torch 2.11 の `c10.dll` が初期化に失敗する(WinError 1114、
  `MSVCP140.dll` 14.29 のオフセット `0x13020` でアクセス違反)。
- そのたびに WerFault(Windows エラー報告)がプロセスを約23秒凍結する。torch を import する拡張の
  数だけこれが繰り返されるため、CPU 0% でハングしているように見える。
- OmniGibson は `import omnigibson` の時点で torch を先に読み込むため、`main.py` などの通常の経路
  では起きない。起きるのは、Kit を先に起動する `check_kit_bare.py` だけ。

## 対処

`behavior` 環境の h5py を、Kit と同じ HDF5 1.14 系を同梱するバージョンに下げた。

```powershell
C:\Users\smiga\miniconda3\envs\behavior\python.exe -m pip install --no-deps "h5py==3.14.0"
# h5py 3.14.0 = HDF5 1.14.6
```

- OmniGibson の要求は `h5py>=3.10.0`(`BEHAVIOR-1K/OmniGibson/setup.py`)なので、範囲内。
- OmniGibson での h5py の使い方は `h5py.File` の読み書きなど基本的な API だけで、主にデータ収集用の
  コード(`data_wrapper.py`, `iterable_dataset.py`)にある。ReKep の通常のパスにはほぼ影響しない。
- 元に戻す場合:`pip install h5py==3.16.0`

これ以外の変更(リポジトリのコード、OS、ドライバなど)は行っていない。

## 確認結果

| 実行 | 結果 |
|---|---|
| `check_pencil_holder_settle.py --config config_widowxai.yaml --scene_file og_scene_file_pen_widowxai.json` × 3回連続 | 全回完走(54〜57秒)、終了コード0、クラッシュ0件 |
| `check_pencil_holder_settle.py --config config.yaml` × 3回連続 | 全回完走(64〜65秒)、終了コード0、クラッシュ0件 |
| `main.py --use_cached_query --config config_widowxai.yaml`(ユーザーが GUI で実行) | クラッシュせず、9/24 と同じ挙動 |

settle チェックの最後に出る `Simulation App Shutting Down` は、スクリプト末尾の `og.shutdown()` による
正常終了で、クラッシュではない。

## 調査で誤っていた点(今後のための教訓)

- **hdf5 の読み込みエラーを「無害なノイズ」と判断していたのが最大の誤り。** これが引き金だった。
- **faulthandler のダンプはノイズではなかった。** 実際の例外(0xc0000139 / アクセス違反)を示していた。
- **`check_kit_bare.py` を「最小再現」としたのは不適切だった。** このスクリプトは 9/30 に作られて
  一度も成功したことがなく、しかも原因2という別の問題を踏んでいた。そのため、本来の問題の切り分けには
  なっていなかった。
- **KB5129195 の切り分けは成立していなかった。** アンインストール後の 9/30 12:21 に、Windows Update が
  自動で再インストールしていた。ただし、結論としては無関係だった。
- 次に同じようなことが起きたら、最初にこれを見る:
  - アプリケーションイベントログ(Event ID 1000)の「障害が発生しているモジュール名とバージョン」
  - Kit ログの「指定されたプロシージャが見つかりません」を出している DLL

```powershell
Get-WinEvent -FilterHashtable @{LogName='Application'; Id=1000; StartTime=(Get-Date).AddHours(-1)} |
  % { $p=$_.Properties; "{0} {1} {2} {3} {4}" -f $_.TimeCreated,$p[3].Value,$p[4].Value,$p[6].Value,$p[7].Value }
```

## 未解決・注意点

- **なぜ 9/30 から急に落ちるようになったかは未確定。** h5py 3.16.0 は 2026-07-24 のインストール時から
  入っていたので、衝突自体は正常期間にもあったはず。クラッシュに至るのは、faulthandler の出力と
  OGN スキャンが重なったときだけ(タイミング依存)と考えられる。9/30 の調査中に OGN キャッシュを
  操作・消去したことで、スキャンが走るようになり顕在化した可能性があるが、検証はしていない。
- **h5py が再び 3.16 以上に上がると再発する。** OmniGibson の再インストールや `pip install -U` の際は
  注意すること。必要なら `h5py==3.14.0` を固定する。
- **`tools/check_kit_bare.py` は今のままだと原因2を踏む。** 使うなら、`from isaacsim import SimulationApp`
  の前に `import torch` を追加する必要がある。
- シェーダーキャッシュを消去していたため、`check_kit_bare.py`(Isaac Sim 既定アプリ)の起動時に
  RTX シェーダー(RtPso)の再コンパイル待ちが 11 分以上続いた(途中で停止)。OmniGibson 経由の実行では
  この待ちは発生しなかった。原因は調べていない。

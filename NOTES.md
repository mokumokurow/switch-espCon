# Nintendo Switch Pro Controller (ESP32) 実装まとめ

## 詰まったポイントと解決策

---

### 1. スタックオーバーフロー（起動直後クラッシュ）

- **症状：** `bt_hidd_events` タスクで即クラッシュ
- **原因：** `CONFIG_ESPHID_TASK_SIZE_BT` のデフォルト値が 2048 バイトと小さすぎた
- **対処：** `sdkconfig` で 8192 に変更

---

### 2. BT コールバック内での API 呼び出し

- **症状：** 接続・切断時にデッドロック／クラッシュ
- **原因：** `esp_bt_gap_set_scan_mode()` などを HID コールバック内から直接呼んでいた
- **対処：** `scan_mode_task` と `output_handler_task` を別タスクとして作り、キュー経由で処理を委譲

---

### 3. サブコマンドの ACK バイトが間違っていた（0x08 ループ）

- **症状：** Switch が 0x08（Ship Power State）を繰り返し送ってくる
- **原因：** データなしの単純 ACK にも `0x80 | subcmd_id` を返していた
- **対処：** レスポンスデータがある場合のみ `0x80 | subcmd_id`、ない場合は `0x80` を使うよう修正

---

### 4. IMU データが 0 のまま（0x40 ループ）

- **症状：** Switch が 0x40（IMU On/Off）を繰り返し送ってくる
- **原因：** 0x30 入力レポートの IMU サンプルがすべて 0 だった（1G 分の値が必要）
- **対処：** 3 サンプルそれぞれの accel_z を `0x1000`（±8G レンジで 1G 相当）に設定

---

### 5. ハンドシェイク中に 60Hz フラッドで ACK がブロック（0x40 ループ再発）

- **症状：** IMU 修正後も 0x40 ループが続く
- **原因：** `s_input_reporting = true` をサブコマンド 0x03（DevInfo）の直後に立てていたため、
  ハンドシェイク完了前に 60Hz の 0x30 が溢れ、Switch への 0x21 ACK が届かなかった
- **対処：** フラグを立てるタイミングをサブコマンド **0x30（Player Lights）受信後** に変更

---

### 6. Switch 12.0+ で CoD（Class of Device）を認識しない

- **症状：** Switch が「コントローラー」として認識せず、ペアリング画面に出ない
- **原因：** `esp_bt_cod_t` の `service` フィールドが 0 だった
- **対処：** `service=1`（Capturing）をセットし `ESP_BT_SET_COD_ALL` で適用

```c
esp_bt_cod_t cod = { .service=1, .major=5, .minor=2, .reserved_2=0, .reserved_8=0 };
esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
```

---

### 7. Sniff モード中に 60Hz フラッドで切断（30〜70 秒後）

- **症状：** `Connection Timeout (rsn 0x8)` で突然切断
- **原因：** Switch が省電力の Sniff モードに入っても 60Hz で 0x30 を送り続けていた
- **対処（部分的）：** `bt_link_mode` 変数で Active/Sniff を追跡し、Sniff 中はレポート送信を
  スキップまたはレートを低下させる方向で検討中

---

### 8. UART 制御の追加

- **追加処理：** `uart_cmd_task` を実装
- COM4 から 1 文字コマンドを受信し、各ボタンを 80ms タップ

| 文字 | ボタン     |
|------|----------|
| a    | A         |
| b    | B         |
| x    | X         |
| y    | Y         |
| l    | L         |
| r    | R         |
| z    | ZL        |
| c    | ZR        |
| +    | +         |
| -    | −         |
| h    | HOME      |
| p    | Capture   |
| w    | D-pad Up  |
| s    | D-pad Down|
| q    | D-pad Left|
| e    | D-pad Right|

---

### 9. Python ラッパーの実装

- **`controller.py`** — `SwitchController` クラス（`press` / `wait` / コンテキストマネージャ）
  - 画像認識プロジェクト等から `from controller import SwitchController` で利用
- **`sample_app.py`** — ターミナルからリアルタイム操作するサンプルアプリ
  - 矢印キー・Enter・Backspace 等を firmware 文字にマッピング
  - `python sample_app.py --port COM4` で起動

---

### 10. COM4 が開けない（PermissionError 31）

- **症状：** `python sample_app.py` で "デバイスが機能していません"
- **原因：** `idf monitor` をウィンドウ強制終了するなどして CH340 ドライバが不正状態に
- **対処：** USB ケーブルを抜き差しして CH340 を再列挙
- **予防：** `idf monitor` は必ず `Ctrl+]` で正常終了する

---

## 完成したハンドシェイクシーケンス

```
Switch → ESP32     ESP32 → Switch
0x02 (DevInfo req) → 0x21 ACK + デバイス情報
0x08 (Power State) → 0x21 ACK (データなし)
0x10 (SPI Read)    → 0x21 ACK + SPI データ  (複数回)
0x03 (DevMode)     → 0x21 ACK
0x04 (TriggerTime) → 0x21 ACK
0x40 (IMU On)      → 0x21 ACK
0x30 (PlayerLED)   → 0x21 ACK  ← ここで s_input_reporting=true
0x48 (Vibration)   → 0x21 ACK
0x21 (MCU Config)  → 0x21 ACK
0x30 (PlayerLED 2) → 0x21 ACK
→ 0x30 入力レポートを 60Hz で送信開始
```

## 主要設定値

| 項目 | 値 |
|------|----|
| VID  | 0x057E |
| PID  | 0x2009 |
| デバイス名 | Pro Controller |
| CoD service / major / minor | 1 / 5 / 2 |
| ESPHID_TASK_SIZE_BT | 8192 |
| 入力レポートレート | 60Hz (16.7ms) |
| IMU accel_z (静止時) | 0x1000 |
| スティック中心値 | 0x800 |
| UART ボーレート | 115200 |

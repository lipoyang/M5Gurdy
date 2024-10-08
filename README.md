# M5Gurdy
ヨーロッパの古楽器ハーディ・ガーディをM5StackとMIDI音源で再現した電子楽器

[![代替テキスト](https://img.youtube.com/vi/fCes9bUJv6Y/0.jpg)](https://www.youtube.com/watch?v=fCes9bUJv6Y)

## 概要
<img src="image/m5gurdy.jpg" width="512">

- ハンドルの回転をエンコーダで検出。
- 22個のキーにより約2オクターブの音域をカバー。
- 2本の旋律弦による旋律と、4本ドローン弦による持続音を再現。
- MIDIシンセサイザユニットでさまざまな音色を出力可能。

## 構成
![概略図](image/overview.png)
- M5Stack Basic V2.7
- ソフトはPlatformIOで開発 (Arduinoベース)
- Synth Unit (M5Stack用MIDIシンセサイザーユニット)
- ロータリーエンコーダー (2相, 24パルス/回転)
- Kailh Choc V1 ロープロファイルキースイッチ (赤軸)
- MBK Choc ロープロファイルキーキャップ
- 16bit I2C I/OエキスパンダIC MCP23017
- DC/DCコンバータ イーター電機工業 AS6R0-0505 

## コンセプト
ハーディ・ガーディはヨーロッパの古楽器です。バイオリンに似た弦楽器ですが、弓で弦を擦るのではなく、ハンドルでホイールを回転させて弦を擦り、旋律は鍵盤によって演奏します。またハーディガーディには、旋律弦のほかに複数のドローン弦があります。ドローン弦は一定のピッチでメロディーを伴奏し、バグパイプに似た音を生み出します。

<img src="image/hurdygurdy.jpg" width="512">

出典：[Wikipedia](https://ja.wikipedia.org/wiki/%E3%83%8F%E3%83%BC%E3%83%87%E3%82%A3%E3%83%BB%E3%82%AC%E3%83%BC%E3%83%87%E3%82%A3)

M5Gurdyは、M5StackとMIDIシンセサイザユニットでハーディ・ガーディを再現した電子楽器です。本物のハーディ・ガーディと同様にハンドルと鍵盤で演奏し、2本の旋律弦と4本のドローン弦をシミュレートしますが、音色はさまざまに変えることができます。
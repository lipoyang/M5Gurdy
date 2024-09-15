#include <M5Stack.h>
//#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <M5UnitSynth.h>
#include <PollingTimer.h>
#include <driver/pcnt.h>

#include "DisplayUI.h"
#include "SerialCom.h" // 開発用

// ピン
#define PIN_RXD2        16
#define PIN_TXD2        17
#define PIN_ENC_CLK     36
#define PIN_ENC_DT      26
#define PIN_PEG_INT     35
#define PIN_NEOPIXEL    5

// 鍵盤入力のIOエキスパンダのI2Cスレーブアドレス
#define ADDR_KB1       0x20    // 低音側
#define ADDR_KB2       0x21    // 高音側

// ペグ入力のIOエキスパンダのI2Cスレーブアドレス
#define ADDR_PEG       0x22

// 周期[msec]
#define INTERVAL1       20

// ドローン弦のキー
#define NOTE_DRONE1     NOTE_C4
#define NOTE_DRONE2     NOTE_G3
#define NOTE_DRONE3     NOTE_C3
#define NOTE_DRONE4     NOTE_G2
// 1オクターブ
#define ONE_OCTAVE      12

// LEDの数
#define NUM_NEOPIXEL    15

// 鍵盤入力のIOエキスパンダ
Adafruit_MCP23X17 keyboard1;
Adafruit_MCP23X17 keyboard2;

// ペグ入力のIOエキスパンダ
Adafruit_MCP23X17 pegInput;

// シンセユニット
M5UnitSynth synth;

// NeoPixel
Adafruit_NeoPixel pixels(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// インターバルタイマ
IntervalTimer interval1;

// ペグのロータリーエンコーダ用
static const uint8_t CLK_MASK[4] = {0x01, 0x04, 0x10, 0x40};
static const uint8_t DT_MASK [4] = {0x02, 0x08, 0x20, 0x80};
static const uint8_t CLK_PIN [4] = {0, 2, 4, 6};

static const uint8_t TIMBRE[][4] = {
    {Violin,      Violin,      Cello,       Cello      }, // Strings
    {BagPipe,     BagPipe,     BagPipe,     BagPipe    }, // Bagpipes
    {ChurchOrgan, ChurchOrgan, ChurchOrgan, ChurchOrgan}, // Church Organ
    {AccordionFrench, AccordionFrench, AccordionFrench, AccordionFrench}, //Accordion
    {Flute,       Flute,       Flute,       Flute      }, // Flute
    {Trumpet,     Trumpet,     Trumpet,     Trumpet    }, // Trumpet
    {Lead2Sawtooth,   Lead8BassLead,   Pad4Choir,       Pad4Choir      }, // Synthesizer
    {StringEnsemble1, StringEnsemble1, StringEnsemble1, StringEnsemble1}, // Ensamble
    {ChoirAahs,   ChoirAahs,   ChoirAahs,   ChoirAahs  }, // Chorus
};

#define TONE_MAX        8  // 音色番号の最大値

// ドローンモード
enum{
    C2_D4 = 0,  // 旋律2弦, ドローン4弦
    C2_D2,      // 旋律2弦, ドローン2弦
    C2_D0,      // 旋律2弦, ドローンなし
    CH_D2,      // 旋律1弦(高音), ドローン2弦
    CH_D0,      // 旋律1弦(高音), ドローンなし
    CL_D2,      // 旋律1弦(低音), ドローン2弦
    CL_D0       // 旋律1弦(低音), ドローンなし
};
#define DRONE_MODE_MAX  CL_D0 // ドローンモードの最大値

int master_vol = 32; // マスター音量 0-32 (=> 0-127)
int tone_no = 0;     // 音色
int scale = 0;       // 音階(ハ長調からどれだけ上がるか下がるか)
int drone_mode = C2_D4;  // ドローン切り替え
int pos_enc = 0;     // ロータリーエンコーダ位置 (LED表示用)

void    keyboard_begin();
uint8_t keyboard_getKey();
void    crank_begin();
uint8_t crank_getVolume();
void    peg_begin();
bool    peg_get(int steps[]);
void    neopixel_rotate(int diff);
void    neopixel_loop(int key, int vol);

// 初期化
void setup()
{
    // UIの初期化
    DisplayUI_begin();

    pinMode(21, INPUT_PULLUP); //デファルトのSDAピン21　のプルアップの指定
    pinMode(22, INPUT_PULLUP); //デファルトのSCLピン22　のプルアップの指定
    delay(100);

    // ペグの初期化
    peg_begin();
    
    // 鍵盤の初期化
    keyboard_begin();

    // クランクの初期化
    crank_begin();

    // シンセユニットの初期化
    synth.begin(&Serial2, UNIT_SYNTH_BAUD, PIN_RXD2, PIN_TXD2);
    synth.setMasterVolume(127);
    synth.setVolume(0, 127);
    synth.setVolume(1, 127);
    synth.setVolume(2, 127);
    synth.setVolume(3, 127);
    synth.setVolume(4, 127);
    synth.setVolume(5, 127);
    synth.setInstrument(0, 0, TIMBRE[tone_no][0]);
    synth.setInstrument(0, 1, TIMBRE[tone_no][1]);
    synth.setInstrument(0, 2, TIMBRE[tone_no][2]);
    synth.setInstrument(0, 3, TIMBRE[tone_no][2]);
    synth.setInstrument(0, 4, TIMBRE[tone_no][3]);
    synth.setInstrument(0, 5, TIMBRE[tone_no][3]);
    
    // NeoPixelの初期化
    pixels.begin();

    // 制御周期の設定
    interval1.set(INTERVAL1);

    // シリアルコマンドの初期化 (開発用)
    SerialCom_init();
}

// メインループ
void loop()
{
    static uint8_t key_prev = 255;
    static uint8_t exp_prev = 0;
    int peg_steps[4];

    // ペグ操作
    if (!digitalRead(PIN_PEG_INT)) {
        bool changed = peg_get(peg_steps);
        // ドローンモード
        if(peg_steps[0] != 0){
            drone_mode += peg_steps[0];
            if(drone_mode < 0) drone_mode = 0;
            if(drone_mode > DRONE_MODE_MAX) drone_mode = DRONE_MODE_MAX;
            changed = false;
            DisplayUI_droneMode(drone_mode);
        }
        // 音色
        if(peg_steps[1] != 0){
            tone_no += peg_steps[1];
            if(tone_no < 0) tone_no = 0;
            if(tone_no > TONE_MAX) tone_no = TONE_MAX;
            synth.setInstrument(0, 0, TIMBRE[tone_no][0]);
            synth.setInstrument(0, 1, TIMBRE[tone_no][1]);
            synth.setInstrument(0, 2, TIMBRE[tone_no][2]);
            synth.setInstrument(0, 3, TIMBRE[tone_no][2]);
            synth.setInstrument(0, 4, TIMBRE[tone_no][3]);
            synth.setInstrument(0, 5, TIMBRE[tone_no][3]);
        }
        // マスター音量
        if(peg_steps[2] != 0){
            master_vol += peg_steps[2];
            if(master_vol < 0)  master_vol = 0;
            if(master_vol > 32) master_vol = 32;
            uint8_t vol = (uint8_t)((master_vol > 0) ? master_vol * 4 - 1 : 0);
            synth.setMasterVolume(vol);
        }
        // キー
        if(peg_steps[3] != 0){
            scale += peg_steps[3];
            if(scale < -12) scale = -12;
            if(scale >  12) scale =  12; 
        }
        // 表示更新
        if(changed){
            DisplayUI_settings();
        }
    }

    if(interval1.elapsed())
    {
        static uint8_t note[6];
        uint8_t expression = crank_getVolume();
        uint8_t key = keyboard_getKey();

        key += scale;
        uint8_t note_drone1 = NOTE_DRONE1 + scale;
        uint8_t note_drone2 = NOTE_DRONE2 + scale;
        uint8_t note_drone3 = NOTE_DRONE3 + scale;
        uint8_t note_drone4 = NOTE_DRONE4 + scale;

        if(expression > 0){
            synth.setExpression(0, expression);
            synth.setExpression(1, expression);
            synth.setExpression(2, expression);
            synth.setExpression(3, expression);
            synth.setExpression(4, expression);
            synth.setExpression(5, expression);
        }

        // 鳴り始め
        if(exp_prev == 0 && expression > 0){
            // 旋律弦(H)
            if(drone_mode <= CH_D0){
                synth.setNoteOn(0, key + ONE_OCTAVE, 127);
            }
            // 旋律弦(L)
            if(drone_mode <= C2_D0 || drone_mode >= CL_D2){
                synth.setNoteOn(1, key, 127);
            }
            // ドローン弦(H)
            if(drone_mode != C2_D0 && drone_mode != CH_D0 && drone_mode != CL_D0){
                synth.setNoteOn(2, note_drone1, 127);
                synth.setNoteOn(3, note_drone2, 127);
            }
            // ドローン弦(L)
            if(drone_mode == C2_D4){
                synth.setNoteOn(4, note_drone3, 127);
                synth.setNoteOn(5, note_drone4, 127);
            }
            note[0] = key + ONE_OCTAVE;
            note[1] = key;
            note[2] = note_drone1;
            note[3] = note_drone2;
            note[4] = note_drone3;
            note[5] = note_drone4;
            Serial.printf("ON(1) %d %d\n", key, expression);
        }
        // 鳴り終わり
        else if(exp_prev > 0 && expression == 0){
            for(int i = 0; i < 6; i++){
                synth.setNoteOff(i, note[i], 0);
            }
            Serial.printf("OFF %d\n", note[1]);
        }
        // 鳴り途中
        else if(expression > 0){
            if(key != key_prev){
                synth.setNoteOff(0, note[0], 0);
                synth.setNoteOff(1, note[1], 0);
                // 旋律弦(H)
                if(drone_mode <= CH_D0){
                    synth.setNoteOn(0, key + ONE_OCTAVE, 127);
                }
                // 旋律弦(L)
                if(drone_mode <= C2_D0 || drone_mode >= CL_D2){
                    synth.setNoteOn(1, key, 127);
                }
                note[0] = key + ONE_OCTAVE;
                note[1] = key;
                Serial.printf("ON(2) %d %d\n", key, expression);
            }
        }
        key_prev = key;
        exp_prev = expression;

        // UIの処理
        int key12  = key % 12;
        int octave = key / 12 - 1;
        DisplayUI_loop(octave, key12, expression);

        // NeoPixelの表示
        neopixel_loop(key12, expression);
    }

    // シリアルコマンド処理 (開発用)
    SerialCom_loop();
}

// 鍵盤の初期化
void keyboard_begin()
{
    // 低音側
    if (!keyboard1.begin_I2C(ADDR_KB1)) {
        Serial.println("Keyboard(L) Error.");
        DisplayUI_error("Keyboard(L)");
        while (1);
    }else{
        Serial.println("Keyboard(L) OK.");
    }
    for(int i = 0; i < 16; i++){
        keyboard1.pinMode(i, INPUT_PULLUP);
    }
    // 高音側
    if (!keyboard2.begin_I2C(ADDR_KB2)) {
        Serial.println("Keyboard(H) Error.");
        DisplayUI_error("Keyboard(H)");
        while (1);
    }else{
        Serial.println("Keyboard(H) OK.");
    }
    for(int i = 0; i < 16; i++){
        keyboard2.pinMode(i, INPUT_PULLUP);
    }
}

// 鍵盤の入力を取得
uint8_t keyboard_getKey()
{
    // 配線の都合に応じてビットマスクを修正
    static const uint32_t KB_MASK[] = {
        0x00000100, // GS3
        0x00000200, // A3 
        0x00000400, // AS3
        0x00000800, // B3 
        0x00001000, // C4 
        0x00002000, // CS4
        0x00004000, // D4 
        0x00008000, // DS4
        0x00000008, // E4 
        0x00000004, // F4 
        0x00000002, // FS4
        0x00000001, // G4 
        0x00800000, // GS4
        0x00400000, // A4 
        0x00200000, // AS4
        0x00100000, // B4 
        0x00080000, // C5 
        0x00040000, // CS5
        0x00020000, // D5 
        0x00010000, // DS5
        0x40000000, // E5 
        0x80000000, // F5 
    };
    
    uint16_t kb1 = keyboard1.readGPIOAB();
    uint16_t kb2 = keyboard2.readGPIOAB();
    uint32_t kb = (uint32_t)kb1 | ((uint32_t)kb2 << 16);
    kb = ~kb;
    // Serial.printf("keyboard_getKey: %08X\n", kb);
    
    uint8_t key = NOTE_G3;
    for(int i = 21; i >= 0; i--){
        if((kb & KB_MASK[i]) != 0){
            key = NOTE_GS3 + i;
            break;
        }
    }
    return key;
}

// クランクの初期化
void crank_begin()
{
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);

    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num  = PIN_ENC_CLK;
    pcnt_config.ctrl_gpio_num   = PIN_ENC_DT;
    pcnt_config.lctrl_mode      = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode      = PCNT_MODE_REVERSE;
    pcnt_config.pos_mode        = PCNT_COUNT_INC;
    pcnt_config.neg_mode        = PCNT_COUNT_DEC;
    pcnt_config.counter_h_lim   =  32767;
    pcnt_config.counter_l_lim   = -32768;
    pcnt_config.unit            = PCNT_UNIT_0;
    pcnt_config.channel         = PCNT_CHANNEL_0;

    pcnt_unit_config(&pcnt_config);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

// クランクの入力を取得
uint8_t crank_getVolume()
{
    static const float GAIN = 100.0f;
    static const float IIR = 0.1f;
    static int16_t count_old = 0;
    static float volume = 0;

    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    int16_t diff = count - count_old;
    neopixel_rotate(diff); // NeoPixe回転表示用
    if(diff < 0) diff = -diff;
    count_old = count;

    static int ave_cnt = 0;
    static int16_t ave_buff[8] ={0};
    ave_buff[ave_cnt] = diff;
    ave_cnt++;
    if(ave_cnt >= 8) ave_cnt = 0;
    int16_t ave_acc = 0;
    for(int i = 0; i < 8; i++) ave_acc += ave_buff[i];
    float ave = (float)ave_acc / 8.0f; 

//  volume = (1.0f - IIR) * volume + IIR * GAIN * (float)diff;
    volume = (1.0f - IIR) * volume + IIR * GAIN * ave;
    int i_volume = (int)volume;
    if(i_volume <   0) i_volume = 0;
    if(i_volume > 127) i_volume = 127;

//  Serial.printf("crank_getVolume: %d %d\n", diff, i_volume);
//  Serial.printf("crank_getVolume: %d %.2f %d\n", diff, ave, i_volume);

    return (uint8_t)i_volume;
}

// ペグの初期化
void peg_begin()
{
    if (!pegInput.begin_I2C(ADDR_PEG)) {
        Serial.println("Peg Input Error.");
        DisplayUI_error("Peg Input");
        return; // while (1);
    }else{
        Serial.println("Peg Input OK.");
    }
    for(int i = 0; i < 16; i++){
        pegInput.pinMode(i, INPUT_PULLUP);
    }

    pegInput.setupInterrupts(true, false, LOW);
    pegInput.setupInterruptPin(0, CHANGE);
    pegInput.setupInterruptPin(2, CHANGE);
    pegInput.setupInterruptPin(4, CHANGE);
    pegInput.setupInterruptPin(6, CHANGE);
    pinMode(PIN_PEG_INT, INPUT);
//  attachInterrupt(digitalPinToInterrupt(PIN_PEG_INT), peg_int, FALLING);
}

// ペグの回転を取得
bool peg_get(int steps[])
{
    bool changed = false;

    //uint8_t val = pegInput.readGPIOA();
    uint8_t last_pin = pegInput.getLastInterruptPin(); 
    uint8_t val  = (uint8_t)(pegInput.getCapturedInterrupt() & 0x00FF);

    for(int i = 0; i < 4; i++) {
        steps[i] = 0;
        if(last_pin == CLK_PIN[i]) {
            int clk = (val & CLK_MASK[i]) ? 1 : 0;
            int dt  = (val & DT_MASK [i]) ? 1 : 0;
            if(clk == LOW){
                changed = true;
                if (dt != clk) {
                    Serial.printf("Peg %d +\n", i);
                    steps[i] = 1;
                } else {
                    Serial.printf("Peg %d -\n", i);
                    steps[i] = -1;
                }
            }
        }
    }
    return changed;
}

// NeoPixel表示位置回転
void neopixel_rotate(int diff)
{
    //pos_enc += diff;
    pos_enc -= diff;
    if(pos_enc >= 24) pos_enc -= 24;
    if(pos_enc <   0) pos_enc += 24;
}

// NeoPixel表示
void neopixel_loop(int key, int vol)
{
    // 表示色のRGB値
    static const int COLOR_TABLE[][3] = {
        { 0xFF, 0x00, 0x00 },   // C  赤
        { 0xE2, 0x1C, 0x00 },   // C# 赤橙
        { 0xCB, 0x33, 0x00 },   // D  橙
        { 0xA9, 0x55, 0x00 },   // D# 黄橙
        { 0x7F, 0x7F, 0x00 },   // E  黄
        { 0x00, 0xFF, 0x00 },   // F  緑
        { 0x00, 0xCB, 0x33 },   // F# 青緑
        { 0x00, 0x7F, 0x7F },   // G  水色
        { 0x00, 0x33, 0xCB },   // G# 空色
        { 0x00, 0x00, 0xFF },   // A  青   
        { 0x33, 0x00, 0xCB },   // A# 紫
        { 0x7F, 0x00, 0x7F },   // B  赤紫
    };

    int r = COLOR_TABLE[key][0];
    int g = COLOR_TABLE[key][1];
    int b = COLOR_TABLE[key][2];

    const int VOL_SAT = 64; // 127以下の値で調整
    if(vol > VOL_SAT) vol = VOL_SAT;
    r = r * vol / VOL_SAT;
    g = g * vol / VOL_SAT;
    b = b * vol / VOL_SAT;
    r /= 2;
    g /= 2;
    b /= 2;

    int pos_deg = pos_enc * 15; // [0, 360) deg
    int pos_seg = pos_deg % 60; // [0, 60) deg
    int pos_pix = pos_seg * 4 / 60; // [0, 3] pixel

    pixels.clear();
    for(int i = 0; i < 15; i++){
        int seg_i = i % 4; // [0, 3)
        if(seg_i == pos_pix){
            pixels.setPixelColor(i, pixels.Color(r, g, b));
        }else{
            pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        }
    }
    pixels.show();
}

// シリアル受信コマンド処理 (開発用)
void SerialCom_callback(char* buff)
{
    // 音色変更イベント
    if(buff[0] == 'T'){
        int ch, val;
        int ret = sscanf(&buff[1], "%d,%d", &ch, &val);
        if(ret == 2){
            if(ch < 0 || ch > 3){
                Serial.printf("invalid channel %d\n", ch);
            }
            else if(val < 0 || val > 127){
                Serial.printf("invalid channel %d\n", val);
            }
            else{
                Serial.printf("setInstrument ch = %d, val = %d\n", ch, val);

                switch(ch){
                    case 0:
                        synth.setInstrument(0, 0, val);
                        break;
                    case 1:
                        synth.setInstrument(0, 1, val);
                        break;
                    case 2:
                        synth.setInstrument(0, 2, val);
                        synth.setInstrument(0, 3, val);
                        break;
                    case 3:
                        synth.setInstrument(0, 4, val);
                        synth.setInstrument(0, 5, val);
                        break;
                }
            }
        }else{
            Serial.println("syntax error!");
        }
    }else{
        Serial.println("unknown command!");
    }
}

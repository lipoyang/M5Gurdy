#include <M5Stack.h>
//#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <M5UnitSynth.h>
#include <PollingTimer.h>
#include <driver/pcnt.h>

// ピン
#define PIN_RXD2        16
#define PIN_TXD2        17
#define PIN_ENC_CLK     36
#define PIN_ENC_DT      26

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

// 鍵盤入力のIOエキスパンダ
Adafruit_MCP23X17 keyboard1;
Adafruit_MCP23X17 keyboard2;

// ペグ入力のIOエキスパンダ
Adafruit_MCP23X17 pegInput;

// シンセユニット
M5UnitSynth synth;

// インターバルタイマ
IntervalTimer interval1;

// ペグのロータリーエンコーダ用
static int peg_clk[4];
static const uint8_t CLK_MASK[4] = {0x01, 0x04, 0x10, 0x40};
static const uint8_t DT_MASK [4] = {0x02, 0x08, 0x20, 0x80};

void    keyboard_begin();
uint8_t keyboard_getKey();
void    crank_begin();
uint8_t crank_getVolume();
void    peg_begin();
void    peg_get(int steps[]);

// 初期化
void setup()
{
    M5.begin();

    // シリアルポートの初期化(デバッグ用)
    // Serial.begin(115200);

    pinMode(21, INPUT_PULLUP); //デファルトのSDAピン21　のプルアップの指定
    pinMode(22, INPUT_PULLUP); //デファルトのSCLピン22　のプルアップの指定
    delay(1000);

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
    synth.setInstrument(0, 0, Violin);
    synth.setInstrument(0, 1, Violin);
    synth.setInstrument(0, 2, Cello);
    synth.setInstrument(0, 3, Cello);
    synth.setInstrument(0, 4, Contrabass);
    synth.setInstrument(0, 5, Contrabass);
    
    // 制御周期の設定
    interval1.set(INTERVAL1);
}

// メインループ
void loop()
{
    static uint8_t key_prev = 255;
    static uint8_t exp_prev = 0;
    int peg_steps[4];

    if(interval1.elapsed()){
        peg_get(peg_steps);

        uint8_t expression = crank_getVolume();
        uint8_t key = keyboard_getKey();

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
            synth.setNoteOn(0, key, 127);
            synth.setNoteOn(1, key + ONE_OCTAVE, 127);
            synth.setNoteOn(2, NOTE_DRONE1, 127);
            synth.setNoteOn(3, NOTE_DRONE2, 127);
            synth.setNoteOn(4, NOTE_DRONE3, 127);
            synth.setNoteOn(5, NOTE_DRONE4, 127);
            Serial.printf("ON(1) %d %d\n", key, expression);
        }
        // 鳴り終わり
        else if(exp_prev > 0 && expression == 0){
            synth.setNoteOff(0, key_prev, 0);
            synth.setNoteOff(1, key_prev + ONE_OCTAVE, 0);
            synth.setNoteOff(2, NOTE_DRONE1, 0);
            synth.setNoteOff(3, NOTE_DRONE2, 0);
            synth.setNoteOff(4, NOTE_DRONE3, 0);
            synth.setNoteOff(5, NOTE_DRONE4, 0);
            Serial.printf("OFF %d\n", key_prev);
        }
        // 鳴り途中
        else if(expression > 0){
            if(key != key_prev){
                synth.setNoteOff(0, key_prev, 0);
                synth.setNoteOff(1, key_prev + ONE_OCTAVE, 0);
                synth.setNoteOn(0, key, 127);
                synth.setNoteOn(1, key + ONE_OCTAVE, 127);
                Serial.printf("ON(2) %d %d\n", key, expression);
            }
        }
        key_prev = key;
        exp_prev = expression;
    }
}

// 鍵盤の初期化
void keyboard_begin()
{
    // 低音側
    if (!keyboard1.begin_I2C(ADDR_KB1)) {
        Serial.println("Keyboard(L) Error.");
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

static uint8_t val_prev;

// ペグの初期化
void peg_begin()
{
    if (!pegInput.begin_I2C(ADDR_PEG)) {
        Serial.println("Peg Input Error.");
        while (1);
    }else{
        Serial.println("Peg Input OK.");
    }
    for(int i = 0; i < 16; i++){
        pegInput.pinMode(i, INPUT_PULLUP);
    }

    val_prev = pegInput.readGPIOA();
    //Serial.printf("Peg %02X\n", val_prev);
    for(int i = 0; i < 4; i++){
        peg_clk[i] = (val_prev & CLK_MASK[i]) ? 1 : 0;
    }
}

// ペグの回転を取得
void peg_get(int steps[])
{
    uint8_t val = pegInput.readGPIOA();
    if(val != val_prev){
        //Serial.printf("Peg %02X\n", val);
        val_prev = val;
    }

    for(int i = 0; i < 4; i++)
    {
        int clk = (val & CLK_MASK[i]) ? 1 : 0;
        int dt  = (val & DT_MASK [i]) ? 1 : 0;

        if (clk != peg_clk[i]) {
            if (dt != clk) {
                steps[i] = 1;
                Serial.printf("Peg %d +\n", i);
            } else {
                steps[i] = -1;
                Serial.printf("Peg %d -\n", i);
            }
        }else{
            steps[i] = 0;
        }
        peg_clk[i] = clk;
    }
}


/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include "config.hpp"
#include <Arduino.h>

// ================= ピンの定義 =================

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

#define ENC_PPR_SPEC 2048      // エンコーダの設定値
#define PPR (ENC_PPR_SPEC * 4) // 実効PPR（x4カウント）

#define DEG_PER_COUNT (360.0f / PPR)
#define HALF_PPR (PPR / 2)

// ピンの定義 //
// 状態表示LED
#define LED 0

// MD PWM
#define MD1P 4
#define MD2P 5
#define MD3P 6
#define MD4P 7

// MD DIR
#define MD1D 8
#define MD2D 9
#define MD3D 10
#define MD4D 11

// サーボ
#define SERVO1 35
#define SERVO2 36
#define SERVO3 37
#define SERVO4 38

// ソレノイドバルブ
#define TR1 39
#define TR2 40
#define TR3 41
#define TR4 42
#define TR5 47
#define TR6 48
#define TR7 1

// 旧メイン基板用
#define SV1 2

// エンコーダ
#define ENC1_A 12
#define ENC1_B 13
#define ENC2_A 14
#define ENC2_B 15
#define ENC3_A 16
#define ENC3_B 17
#define ENC4_A 18
#define ENC4_B 21

// スイッチ
#define SW1 1
#define SW2 48
#define SW3 47
#define SW4 42
#define SW5 41
#define SW6 38
#define SW7 37
#define SW8 36

// モータ数
#define motor 2

// ロボマス
#define CAN_RX 2
#define CAN_TX 3

// MD用
#define MD_PWM_MAX ((1 << MD_PWM_RESOLUTION) - 1)

// サーボ用
#define SERVO_PWM_PERIOD_US (1000000.0 / SERVO_PWM_FREQ) // 周波数から周期を計算
#define SERVO_PWM_MAX_DUTY ((1 << SERVO_PWM_RESOLUTION) - 1)
#define SERVO_PWM_SCALE (SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US)

// ロボマス
#define NUM_MOTOR 4                    // モーター数
#define gear_m3508 19.2f               // M3508ギア比
#define gear_m2006 36.0f               // M2006ギア比
#define ENCODER_MAX 8192               // エンコーダ分解能
#define HALF_ENCODER (ENCODER_MAX / 2) // エンコーダ分解能の半分

// 状態変数の宣言
extern int encoder_count[NUM_MOTOR];
extern int rpm[NUM_MOTOR];
extern int current[NUM_MOTOR];
extern bool offset_ok[NUM_MOTOR];
extern int encoder_offset[NUM_MOTOR];
extern int last_encoder[NUM_MOTOR];
extern int rotation_count[NUM_MOTOR];
extern long total_encoder[NUM_MOTOR];
// extern float angle[NUM_MOTOR];
// extern float vel[NUM_MOTOR];

// PID関連変数
extern float target_angle[NUM_MOTOR];
extern float pos_error_prev[NUM_MOTOR];
extern float cur_error_prev[NUM_MOTOR];
extern float pos_integral[NUM_MOTOR];
extern float vel_integral[NUM_MOTOR];
extern float cur_integral[NUM_MOTOR];
extern float pos_output[NUM_MOTOR];
extern float cur_output[NUM_MOTOR];
extern float motor_output_current[NUM_MOTOR];

extern float angle_m3508[NUM_MOTOR];
extern float vel_m3508[NUM_MOTOR];
extern float c[NUM_MOTOR];
extern float output[NUM_MOTOR];
extern float angle[NUM_MOTOR];

extern unsigned long lastPidTime;
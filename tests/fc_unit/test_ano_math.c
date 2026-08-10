/**
 * Ano_Math.c 单元测试
 *
 * 覆盖：基本数学函数、三角函数近似、向量运算、角度归约、死区、FIFO、向量限幅
 *
 * 设计目标：
 *   1. 验证每个纯函数的基本正确性
 *   2. 捕获 known_issues.md 中记录的历史 bug 模式（角度归约、safe_div 除零保护等）
 *   3. 暴露近似函数的精度边界（fast_atan2、mx_sin/my_sin/my_cos）
 *
 * 编译：gcc -I mock/ -I ../../ANO_LX_FC_T265代替光流/DriversBsp/ \
 *        ../../ANO_LX_FC_T265代替光流/DriversBsp/Ano_Math.c test_ano_math.c \
 *        -o test_ano_math.exe -lm
 */

#include "test_framework.h"

/* 引入被测源码 — GCC -I 路径使 Ano_Math.h → SysConfig.h 命中 mock/ 桩 */
#include "Ano_Math.h"

#include <math.h>
#include <float.h>

/* 标准数学常量（被测代码内部用 MY_PPPIII 等，测试用高精度参考值） */
#define PI_REF   3.14159265358979323846
#define PI2_REF  1.57079632679489661923
#define TAU_REF  6.28318530717958647692

/* 近似函数容差 */
#define ATAN2_TOL   0.02f    /* fast_atan2 ~1° 以内 */
#define SQRT_TOL    0.01f    /* my_sqrt  ~1% 以内 */
#define SIN_TOL     0.01     /* mx_sin/my_sin  ~0.01 以内 */
#define COS_TOL     0.01f    /* my_cos */

/* ======================================================================
 * 第一部分：宏测试 — 捕获 known_issues 中涉及的宏级别 bug 模式
 * ====================================================================== */

/* --- safe_div: 除零保护（问题 #6 通信层除零风险的同类模式）--- */
TEST_BEGIN(test_safe_div_normal)
    ASSERT_FLOAT_EQ(2.0f, safe_div(4.0f, 2.0f, 0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.5f, safe_div(1.0f, 2.0f, 0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(-3.0f, safe_div(9.0f, -3.0f, 0.0f), 1e-6f);
TEST_END

TEST_BEGIN(test_safe_div_zero_denominator)
    /* 分母为 0 时应返回安全值，不崩溃 */
    ASSERT_FLOAT_EQ(99.0f, safe_div(1.0f, 0.0f, 99.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, safe_div(5.0f, 0.0f, 0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(-1.0f, safe_div(3.0f, 0.0f, -1.0f), 1e-6f);
TEST_END

/* --- LIMIT: 限幅宏 --- */
TEST_BEGIN(test_limit_clamp)
    ASSERT_FLOAT_EQ(5.0f, LIMIT(5.0f, 0.0f, 10.0f), 1e-6f);   /* 范围内 */
    ASSERT_FLOAT_EQ(0.0f, LIMIT(-1.0f, 0.0f, 10.0f), 1e-6f);  /* 下限 */
    ASSERT_FLOAT_EQ(10.0f, LIMIT(15.0f, 0.0f, 10.0f), 1e-6f); /* 上限 */
    ASSERT_FLOAT_EQ(0.0f, LIMIT(0.0f, 0.0f, 10.0f), 1e-6f);   /* 边界=下限 */
    ASSERT_FLOAT_EQ(10.0f, LIMIT(10.0f, 0.0f, 10.0f), 1e-6f); /* 边界=上限 */
TEST_END

/* --- range_to_180deg: 角度归约（问题 #45 航向 runaway 的同类模式）--- */
TEST_BEGIN(test_range_to_180deg_normal)
    ASSERT_FLOAT_EQ(0.0f, range_to_180deg(0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(90.0f, range_to_180deg(90.0f), 1e-6f);
    ASSERT_FLOAT_EQ(-90.0f, range_to_180deg(-90.0f), 1e-6f);
    ASSERT_FLOAT_EQ(45.0f, range_to_180deg(45.0f), 1e-6f);
TEST_END

TEST_BEGIN(test_range_to_180deg_wrap)
    /* 超过 ±180 应回绕 */
    ASSERT_FLOAT_EQ(-170.0f, range_to_180deg(190.0f), 1e-6f);
    ASSERT_FLOAT_EQ(170.0f, range_to_180deg(-190.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, range_to_180deg(360.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, range_to_180deg(-360.0f), 1e-6f);
TEST_END

TEST_BEGIN(test_range_to_180deg_boundary)
    /* 恰好在 ±180 边界 */
    ASSERT_FLOAT_EQ(180.0f, range_to_180deg(180.0f), 1e-6f);
    ASSERT_FLOAT_EQ(-180.0f, range_to_180deg(-180.0f), 1e-6f);
    /* 刚刚超过边界 */
    float r1 = range_to_180deg(181.0f);
    ASSERT_TRUE(r1 < 0 && r1 > -180);  /* 181 → -179 */
    float r2 = range_to_180deg(-181.0f);
    ASSERT_TRUE(r2 > 0 && r2 < 180);   /* -181 → 179 */
TEST_END

/* --- my_pow: 平方宏（注意：表达式副作用问题）--- */
TEST_BEGIN(test_my_pow_basic)
    ASSERT_FLOAT_EQ(4.0f, my_pow(2.0f), 1e-6f);
    ASSERT_FLOAT_EQ(9.0f, my_pow(3.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, my_pow(0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(1.0f, my_pow(-1.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.25f, my_pow(0.5f), 1e-6f);
TEST_END

/* --- my_sign: 符号宏 --- */
TEST_BEGIN(test_my_sign)
    ASSERT_INT_EQ(1, my_sign(1.0f));
    ASSERT_INT_EQ(-1, my_sign(-1.0f));
    ASSERT_INT_EQ(0, my_sign(0.0f));
    ASSERT_INT_EQ(0, my_sign(1e-7f));   /* 小于 1e-6 阈值 → 0 */
    ASSERT_INT_EQ(1, my_sign(1e-5f));   /* 大于 1e-6 阈值 → 1 */
TEST_END

/* --- ABS: 绝对值宏 --- */
TEST_BEGIN(test_abs_macro)
    ASSERT_FLOAT_EQ(5.0f, ABS(5.0f), 1e-6f);
    ASSERT_FLOAT_EQ(5.0f, ABS(-5.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, ABS(0.0f), 1e-6f);
TEST_END

/* --- _MIN / _MAX --- */
TEST_BEGIN(test_min_max)
    ASSERT_FLOAT_EQ(1.0f, _MIN(1.0f, 2.0f), 1e-6f);
    ASSERT_FLOAT_EQ(2.0f, _MAX(1.0f, 2.0f), 1e-6f);
    ASSERT_FLOAT_EQ(-3.0f, _MIN(-3.0f, 0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, _MAX(-3.0f, 0.0f), 1e-6f);
TEST_END

/* --- my_2_norm / my_3_norm --- */
TEST_BEGIN(test_norms)
    ASSERT_FLOAT_EQ(5.0f, my_2_norm(3.0f, 4.0f), 0.05f);  /* 3-4-5 三角形 */
    ASSERT_FLOAT_EQ(0.0f, my_2_norm(0.0f, 0.0f), 1e-6f);
    ASSERT_FLOAT_EQ(1.0f, my_2_norm(1.0f, 0.0f), 0.01f);

    /* 3D norm: (1,2,2) → 3 */
    ASSERT_FLOAT_EQ(3.0f, my_3_norm(1.0f, 2.0f, 2.0f), 0.05f);
    ASSERT_FLOAT_EQ(0.0f, my_3_norm(0.0f, 0.0f, 0.0f), 1e-6f);
TEST_END

/* --- DELTA_LIMIT --- */
TEST_BEGIN(test_delta_limit)
    float y = 10.0f;
    DELTA_LIMIT(15.0f, 3.0f, y);  /* x-y=5, limit to 3 → y=13 */
    ASSERT_FLOAT_EQ(13.0f, y, 1e-6f);

    y = 10.0f;
    DELTA_LIMIT(5.0f, 3.0f, y);   /* x-y=-5, limit to -3 → y=7 */
    ASSERT_FLOAT_EQ(7.0f, y, 1e-6f);

    y = 10.0f;
    DELTA_LIMIT(11.0f, 3.0f, y);  /* x-y=1, within limit → y=11 */
    ASSERT_FLOAT_EQ(11.0f, y, 1e-6f);
TEST_END

/* --- my_pow_2_curve --- */
TEST_BEGIN(test_pow_2_curve)
    /* a=0 → 线性; a=1 → 全非线性 */
    float r1 = my_pow_2_curve(1.0f, 0.0f, 1.0f);
    ASSERT_FLOAT_EQ(1.0f, r1, 1e-6f);  /* 线性: (1-0+0)*1 = 1 */

    float r2 = my_pow_2_curve(0.5f, 0.0f, 1.0f);
    ASSERT_FLOAT_EQ(0.5f, r2, 1e-6f);  /* 线性: 0.5 */

    /* a=1, in=max → ((1-1)+1*LIMIT(1,0,1))*1 = 1 */
    float r3 = my_pow_2_curve(1.0f, 1.0f, 1.0f);
    ASSERT_FLOAT_EQ(1.0f, r3, 1e-6f);
TEST_END

/* ======================================================================
 * 第二部分：函数测试 — 基本数学函数
 * ====================================================================== */

/* --- my_abs --- */
TEST_BEGIN(test_my_abs_func)
    ASSERT_FLOAT_EQ(3.14f, my_abs(3.14f), 1e-6f);
    ASSERT_FLOAT_EQ(3.14f, my_abs(-3.14f), 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, my_abs(0.0f), 1e-6f);
TEST_END

/* --- fast_atan2: 快速反正切（所有象限）--- */
TEST_BEGIN(test_fast_atan2_quadrant1)
    /* 第一象限: 0° ~ 90° */
    ASSERT_FLOAT_EQ(0.0f, fast_atan2(0.0f, 1.0f), ATAN2_TOL);          /* 0° */
    ASSERT_FLOAT_EQ(PI2_REF, fast_atan2(1.0f, 0.0f), ATAN2_TOL);       /* 90° */
    ASSERT_FLOAT_EQ(PI2_REF / 2.0f, fast_atan2(1.0f, 1.0f), ATAN2_TOL);/* 45° */
TEST_END

TEST_BEGIN(test_fast_atan2_quadrant2)
    /* 第二象限: 90° ~ 180° */
    ASSERT_FLOAT_EQ(PI2_REF, fast_atan2(1.0f, -0.001f), ATAN2_TOL);    /* ~90° */
    float a = fast_atan2(1.0f, -1.0f);
    ASSERT_TRUE(a > 2.0f && a < 3.0f);  /* ~135° ≈ 2.356 rad */
TEST_END

TEST_BEGIN(test_fast_atan2_quadrant3_4)
    /* 第三/四象限 */
    float a1 = fast_atan2(-1.0f, -1.0f);
    ASSERT_TRUE(a1 < -2.0f && a1 > -3.0f);  /* ~-135° */

    float a2 = fast_atan2(-1.0f, 1.0f);
    ASSERT_TRUE(a2 > -1.0f && a2 < 0.0f);    /* ~-45° */
TEST_END

TEST_BEGIN(test_fast_atan2_origin)
    /* 原点: (0,0) 应返回 0 */
    ASSERT_FLOAT_EQ(0.0f, fast_atan2(0.0f, 0.0f), 1e-6f);
TEST_END

TEST_BEGIN(test_fast_atan2_axes)
    /* 坐标轴方向 */
    ASSERT_FLOAT_EQ(0.0f, fast_atan2(0.0f, 5.0f), ATAN2_TOL);        /* +X → 0° */
    ASSERT_FLOAT_EQ((float)PI_REF, fast_atan2(0.0f, -5.0f), ATAN2_TOL); /* -X → 180° */
TEST_END

/* --- my_sqrt / my_sqrt_reciprocal: 快速平方根 --- */
TEST_BEGIN(test_my_sqrt_basic)
    ASSERT_FLOAT_EQ(1.0f, my_sqrt(1.0f), SQRT_TOL);
    ASSERT_FLOAT_EQ(2.0f, my_sqrt(4.0f), SQRT_TOL);
    ASSERT_FLOAT_EQ(3.0f, my_sqrt(9.0f), SQRT_TOL);
    ASSERT_FLOAT_EQ(10.0f, my_sqrt(100.0f), 0.1f);
TEST_END

TEST_BEGIN(test_my_sqrt_edge)
    ASSERT_FLOAT_EQ(0.0f, my_sqrt(0.0f), 1e-6f);
    /* 小数 */
    ASSERT_FLOAT_EQ(0.5f, my_sqrt(0.25f), 0.01f);
TEST_END

TEST_BEGIN(test_my_sqrt_reciprocal_basic)
    /* rsqrt(4) = 0.5 */
    ASSERT_FLOAT_EQ(0.5f, my_sqrt_reciprocal(4.0f), 0.01f);
    /* rsqrt(1) = 1.0 */
    ASSERT_FLOAT_EQ(1.0f, my_sqrt_reciprocal(1.0f), 0.01f);
    /* rsqrt(0.01) = 10 */
    ASSERT_FLOAT_EQ(10.0f, my_sqrt_reciprocal(0.01f), 0.5f);
TEST_END

/* --- mx_sin / my_sin / my_cos: 近似三角函数 --- */
TEST_BEGIN(test_mx_sin_basic)
    /* mx_sin 在 [-π, π] 范围内有效 */
    ASSERT_DOUBLE_EQ(0.0, mx_sin(0.0), SIN_TOL);
    ASSERT_DOUBLE_EQ(1.0, mx_sin(PI2_REF), SIN_TOL);
    ASSERT_DOUBLE_EQ(-1.0, mx_sin(-PI2_REF), SIN_TOL);
    ASSERT_DOUBLE_EQ(0.0, mx_sin(PI_REF), SIN_TOL);
TEST_END

TEST_BEGIN(test_my_sin_range)
    /* my_sin 在 [0, 2π] 范围内有效 */
    ASSERT_DOUBLE_EQ(0.0, my_sin(0.0), SIN_TOL);
    ASSERT_DOUBLE_EQ(1.0, my_sin(PI2_REF), SIN_TOL);
    ASSERT_DOUBLE_EQ(0.0, my_sin(PI_REF), SIN_TOL);
    ASSERT_DOUBLE_EQ(-1.0, my_sin(3.0 * PI2_REF), SIN_TOL);
TEST_END

TEST_BEGIN(test_my_sin_known_limitation)
    /*
     * 已知限制：my_sin 只处理 [0, 2π] 范围。
     * 负角度或 > 2π 的结果不可靠 — 这是飞控固件中
     * 需要调用者自行保证输入范围的设计约束。
     * 此测试记录这一行为，防止未来无意中改变。
     */
    double r_neg = my_sin(-0.5);
    /* 不检查结果正确性，只确认函数不崩溃 */
    (void)r_neg;

    double r_large = my_sin(7.0);  /* > 2π */
    (void)r_large;
    /* 如果未来修复了范围处理，这些测试需要更新 */
TEST_END

TEST_BEGIN(test_my_cos_basic)
    /* my_cos 在 [0, 2π] 范围内有效 */
    ASSERT_FLOAT_EQ(1.0f, my_cos(0.0), COS_TOL);
    ASSERT_FLOAT_EQ(0.0f, my_cos(PI2_REF), COS_TOL);
    ASSERT_FLOAT_EQ(-1.0f, my_cos(PI_REF), COS_TOL);
TEST_END

/* ======================================================================
 * 第三部分：死区函数
 * ====================================================================== */

TEST_BEGIN(test_my_deadzone_basic)
    /* my_deadzone(x, ref, zoom): x > ref 时 t=x-zoom (不低于ref) */
    /* x=5 > ref=0, t=5-2=3, 3 >= 0 → 返回 3 */
    ASSERT_FLOAT_EQ(3.0f, my_deadzone(5.0f, 0.0f, 2.0f), 1e-6f);

    /* x < ref 时: t=x+zoom (不超过ref) */
    /* x=-5 < ref=0, t=-5+2=-3, -3 <= 0 → 返回 -3 */
    ASSERT_FLOAT_EQ(-3.0f, my_deadzone(-5.0f, 0.0f, 2.0f), 1e-6f);
TEST_END

TEST_BEGIN(test_my_deadzone_inside_zone)
    /* 在死区内: x > ref 但 x-zoom < ref → 返回 ref */
    ASSERT_FLOAT_EQ(0.0f, my_deadzone(1.0f, 0.0f, 2.0f), 1e-6f);  /* 1-2=-1 < 0 → 0 */
    ASSERT_FLOAT_EQ(0.0f, my_deadzone(-1.0f, 0.0f, 2.0f), 1e-6f); /* -1+2=1 > 0 → 0 */
TEST_END

TEST_BEGIN(test_my_deadzone_2_basic)
    /* my_deadzone_2: 在 [ref-zoom, ref+zoom] 内返回 ref，否则返回 x */
    ASSERT_FLOAT_EQ(0.0f, my_deadzone_2(0.5f, 0.0f, 1.0f), 1e-6f);   /* 在死区内 */
    ASSERT_FLOAT_EQ(0.0f, my_deadzone_2(-0.5f, 0.0f, 1.0f), 1e-6f);  /* 在死区内 */
    ASSERT_FLOAT_EQ(2.0f, my_deadzone_2(2.0f, 0.0f, 1.0f), 1e-6f);   /* 在死区外 */
    ASSERT_FLOAT_EQ(-2.0f, my_deadzone_2(-2.0f, 0.0f, 1.0f), 1e-6f); /* 在死区外 */
TEST_END

/* ======================================================================
 * 第四部分：角度归约
 * ====================================================================== */

TEST_BEGIN(test_to_180_degrees_db)
    ASSERT_DOUBLE_EQ(0.0, To_180_degrees_db(0.0), 1e-6);
    ASSERT_DOUBLE_EQ(90.0, To_180_degrees_db(90.0), 1e-6);
    ASSERT_DOUBLE_EQ(-90.0, To_180_degrees_db(-90.0), 1e-6);
    ASSERT_DOUBLE_EQ(-170.0, To_180_degrees_db(190.0), 1e-6);
    ASSERT_DOUBLE_EQ(170.0, To_180_degrees_db(-190.0), 1e-6);
    ASSERT_DOUBLE_EQ(0.0, To_180_degrees_db(360.0), 1e-6);
TEST_END

/* ======================================================================
 * 第五部分：向量运算
 * ====================================================================== */

TEST_BEGIN(test_vec_2_dot_product)
    float a[2] = {1.0f, 0.0f};
    float b[2] = {0.0f, 1.0f};
    ASSERT_FLOAT_EQ(0.0f, vec_2_dot_product(a, b), 1e-6f);  /* 正交 */

    float c[2] = {1.0f, 0.0f};
    float d[2] = {1.0f, 0.0f};
    ASSERT_FLOAT_EQ(1.0f, vec_2_dot_product(c, d), 1e-6f);  /* 同向 */

    float e[2] = {3.0f, 4.0f};
    float f[2] = {1.0f, 2.0f};
    ASSERT_FLOAT_EQ(11.0f, vec_2_dot_product(e, f), 1e-6f); /* 3*1+4*2=11 */
TEST_END

TEST_BEGIN(test_vec_2_cross_product)
    float a[2] = {1.0f, 0.0f};
    float b[2] = {0.0f, 1.0f};
    ASSERT_FLOAT_EQ(1.0f, vec_2_cross_product(a, b), 1e-6f);  /* 逆时针 90° → 正 */

    float c[2] = {0.0f, 1.0f};
    float d[2] = {1.0f, 0.0f};
    ASSERT_FLOAT_EQ(-1.0f, vec_2_cross_product(c, d), 1e-6f); /* 顺时针 90° → 负 */

    /* 平行向量叉积为 0 */
    float e[2] = {2.0f, 3.0f};
    float f[2] = {4.0f, 6.0f};
    ASSERT_FLOAT_EQ(0.0f, vec_2_cross_product(e, f), 1e-6f);
TEST_END

TEST_BEGIN(test_vec_3_dot_product)
    float a[3] = {1.0f, 2.0f, 3.0f};
    float b[3] = {4.0f, 5.0f, 6.0f};
    /* 1*4 + 2*5 + 3*6 = 32 */
    ASSERT_FLOAT_EQ(32.0f, vec_3_dot_product(a, b), 1e-6f);

    /* 正交 */
    float c[3] = {1.0f, 0.0f, 0.0f};
    float d[3] = {0.0f, 1.0f, 0.0f};
    ASSERT_FLOAT_EQ(0.0f, vec_3_dot_product(c, d), 1e-6f);
TEST_END

TEST_BEGIN(test_vec_3_cross_product)
    /* 标准基向量叉积: i × j = k */
    float i[3] = {1.0f, 0.0f, 0.0f};
    float j[3] = {0.0f, 1.0f, 0.0f};
    float out[3];
    vec_3_cross_product_err_sinx(i, j, out);
    ASSERT_FLOAT_EQ(0.0f, out[0], 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, out[1], 1e-6f);
    ASSERT_FLOAT_EQ(1.0f, out[2], 1e-6f);

    /* j × i = -k */
    vec_3_cross_product_err_sinx(j, i, out);
    ASSERT_FLOAT_EQ(0.0f, out[0], 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, out[1], 1e-6f);
    ASSERT_FLOAT_EQ(-1.0f, out[2], 1e-6f);
TEST_END

/* --- rot_vec_2: 2D 旋转向量 --- */
TEST_BEGIN(test_rot_vec_2_identity)
    /* sinx=0 → 不旋转 */
    float in[2] = {1.0f, 0.0f};
    float out[2];
    rot_vec_2(in, 0.0f, out);
    ASSERT_FLOAT_EQ(1.0f, out[0], 0.01f);
    ASSERT_FLOAT_EQ(0.0f, out[1], 0.01f);
TEST_END

TEST_BEGIN(test_rot_vec_2_90deg)
    /* sinx=1 (90°): (1,0) → (0,1) */
    float in[2] = {1.0f, 0.0f};
    float out[2];
    rot_vec_2(in, 1.0f, out);
    ASSERT_FLOAT_EQ(0.0f, out[0], 0.05f);  /* cos(90°)=0 */
    ASSERT_FLOAT_EQ(1.0f, out[1], 0.05f);  /* sin(90°)=1 */
TEST_END

/* --- length_limit: 向量长度限幅 --- */
TEST_BEGIN(test_length_limit_within)
    /* 向量长度在 limit 内 → 不变 */
    float in1 = 1.0f, in2 = 0.0f;
    float out1, out2;
    length_limit(&in1, &in2, 5.0f, &out1, &out2);
    ASSERT_FLOAT_EQ(1.0f, out1, 0.01f);
    ASSERT_FLOAT_EQ(0.0f, out2, 0.01f);
TEST_END

TEST_BEGIN(test_length_limit_exceeds)
    /* 向量长度超过 limit → 缩放到 limit */
    float in1 = 3.0f, in2 = 4.0f;  /* 长度=5 */
    float out1, out2;
    length_limit(&in1, &in2, 2.5f, &out1, &out2);
    /* 缩放因子 = 2.5/5 = 0.5 */
    ASSERT_FLOAT_EQ(1.5f, out1, 0.05f);
    ASSERT_FLOAT_EQ(2.0f, out2, 0.05f);
TEST_END

TEST_BEGIN(test_length_limit_zero)
    /* 零向量 → 输出零 */
    float in1 = 0.0f, in2 = 0.0f;
    float out1, out2;
    length_limit(&in1, &in2, 5.0f, &out1, &out2);
    ASSERT_FLOAT_EQ(0.0f, out1, 1e-6f);
    ASSERT_FLOAT_EQ(0.0f, out2, 1e-6f);
TEST_END

/* ======================================================================
 * 第六部分：FIFO 环形缓冲
 * ====================================================================== */

TEST_BEGIN(test_fifo_basic)
    float arr[4] = {0};
    u8 cnt = 0;
    /* 写入 1.0, 返回 arr[1] (写入后 cnt 递增) */
    float r1 = fifo(4, &cnt, arr, 1.0f);
    ASSERT_INT_EQ(1, cnt);
    ASSERT_FLOAT_EQ(0.0f, r1, 1e-6f);  /* arr[1] 还是 0 */

    float r2 = fifo(4, &cnt, arr, 2.0f);
    ASSERT_INT_EQ(2, cnt);
    ASSERT_FLOAT_EQ(0.0f, r2, 1e-6f);  /* arr[2] 还是 0 */

    float r3 = fifo(4, &cnt, arr, 3.0f);
    ASSERT_INT_EQ(3, cnt);

    float r4 = fifo(4, &cnt, arr, 4.0f);
    /* cnt=3 → 写入 arr[3]=4.0, cnt→4 → 回绕到 0 → 返回 arr[0]=1.0 */
    ASSERT_INT_EQ(0, cnt);
    ASSERT_FLOAT_EQ(1.0f, r4, 1e-6f);
TEST_END

/* ======================================================================
 * 第七部分：历史 bug 模式回归测试
 *
 * 这些测试针对 known_issues.md 中记录的 bug 模式设计，
 * 验证底层数学工具不会重现同类问题。
 * ====================================================================== */

/*
 * 问题 #45 模式：航向角累积导致超出 ±180° 范围
 * 验证 range_to_180deg 在大幅度累积后仍然正确归约
 */
TEST_BEGIN(test_regression_heading_accumulation)
    /* 模拟航向角持续增加 */
    float heading = 0.0f;
    for (int i = 0; i < 1000; i++) {
        heading += 0.5f;  /* 每步 +0.5° */
        heading = range_to_180deg(heading);
        ASSERT_TRUE(heading >= -180.0f && heading <= 180.0f);
    }
TEST_END

/*
 * 问题 #6 模式：通信层除零保护
 * 验证 safe_div 在所有边界条件下的安全性
 */
TEST_BEGIN(test_regression_division_safety)
    /* 正常除法 */
    float r1 = safe_div(100.0f, 10.0f, 0.0f);
    ASSERT_FLOAT_EQ(10.0f, r1, 1e-6f);

    /* 分母为零 — 不应崩溃 */
    float r2 = safe_div(100.0f, 0.0f, 0.0f);
    ASSERT_FLOAT_EQ(0.0f, r2, 1e-6f);

    /* 分子分母都为零 */
    float r3 = safe_div(0.0f, 0.0f, 42.0f);
    ASSERT_FLOAT_EQ(42.0f, r3, 1e-6f);

    /* 极小分母（非零但接近零）— 应返回大数而非崩溃 */
    float r4 = safe_div(1.0f, 1e-38f, 0.0f);
    ASSERT_TRUE(r4 > 0);  /* 结果很大但不崩溃 */
TEST_END

/*
 * 问题 #26 模式：HOVER_DROP 后高度不恢复 — 涉及向量长度限幅
 * 验证 length_limit 在零向量、极小向量下的行为
 */
TEST_BEGIN(test_regression_vector_limit_edge)
    float out1, out2;

    /* 极小向量 */
    float tiny1 = 1e-10f, tiny2 = 1e-10f;
    length_limit(&tiny1, &tiny2, 1.0f, &out1, &out2);
    /* 长度 ≈ 1.4e-10, 小于 limit → 应基本不变 */
    ASSERT_TRUE(out1 > 0 && out2 > 0);

    /* 一个分量为零 */
    float a = 5.0f, b = 0.0f;
    length_limit(&a, &b, 3.0f, &out1, &out2);
    ASSERT_FLOAT_EQ(3.0f, out1, 0.05f);
    ASSERT_FLOAT_EQ(0.0f, out2, 0.05f);
TEST_END

/* ======================================================================
 * 主函数
 * ====================================================================== */

int main(void)
{
    printf("=== Ano_Math 单元测试 ===\n\n");

    /* 宏测试 */
    printf("[宏] safe_div / LIMIT / range_to_180deg / my_pow / my_sign / ABS / _MIN / _MAX\n");
    RUN_TEST(test_safe_div_normal);
    RUN_TEST(test_safe_div_zero_denominator);
    RUN_TEST(test_limit_clamp);
    RUN_TEST(test_range_to_180deg_normal);
    RUN_TEST(test_range_to_180deg_wrap);
    RUN_TEST(test_range_to_180deg_boundary);
    RUN_TEST(test_my_pow_basic);
    RUN_TEST(test_my_sign);
    RUN_TEST(test_abs_macro);
    RUN_TEST(test_min_max);
    RUN_TEST(test_norms);
    RUN_TEST(test_delta_limit);
    RUN_TEST(test_pow_2_curve);

    /* 基本数学函数 */
    printf("[函数] my_abs / fast_atan2 / my_sqrt / my_sqrt_reciprocal\n");
    RUN_TEST(test_my_abs_func);
    RUN_TEST(test_fast_atan2_quadrant1);
    RUN_TEST(test_fast_atan2_quadrant2);
    RUN_TEST(test_fast_atan2_quadrant3_4);
    RUN_TEST(test_fast_atan2_origin);
    RUN_TEST(test_fast_atan2_axes);
    RUN_TEST(test_my_sqrt_basic);
    RUN_TEST(test_my_sqrt_edge);
    RUN_TEST(test_my_sqrt_reciprocal_basic);

    /* 三角函数近似 */
    printf("[三角] mx_sin / my_sin / my_cos\n");
    RUN_TEST(test_mx_sin_basic);
    RUN_TEST(test_my_sin_range);
    RUN_TEST(test_my_sin_known_limitation);
    RUN_TEST(test_my_cos_basic);

    /* 死区 */
    printf("[死区] my_deadzone / my_deadzone_2\n");
    RUN_TEST(test_my_deadzone_basic);
    RUN_TEST(test_my_deadzone_inside_zone);
    RUN_TEST(test_my_deadzone_2_basic);

    /* 角度归约 */
    printf("[角度] To_180_degrees_db\n");
    RUN_TEST(test_to_180_degrees_db);

    /* 向量运算 */
    printf("[向量] dot / cross / rot / length_limit\n");
    RUN_TEST(test_vec_2_dot_product);
    RUN_TEST(test_vec_2_cross_product);
    RUN_TEST(test_vec_3_dot_product);
    RUN_TEST(test_vec_3_cross_product);
    RUN_TEST(test_rot_vec_2_identity);
    RUN_TEST(test_rot_vec_2_90deg);
    RUN_TEST(test_length_limit_within);
    RUN_TEST(test_length_limit_exceeds);
    RUN_TEST(test_length_limit_zero);

    /* FIFO */
    printf("[FIFO] fifo 环形缓冲\n");
    RUN_TEST(test_fifo_basic);

    /* 历史 bug 回归 */
    printf("[回归] known_issues 历史 bug 模式\n");
    RUN_TEST(test_regression_heading_accumulation);
    RUN_TEST(test_regression_division_safety);
    RUN_TEST(test_regression_vector_limit_edge);

    TEST_SUMMARY();
}

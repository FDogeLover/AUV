/**
 * 最小单元测试框架 — 纯 C，零依赖，单头文件
 *
 * 用法：
 *   #include "test_framework.h"
 *   TEST_BEGIN(test_name) { ASSERT_FLOAT_EQ(1.0, 1.0, 1e-6); } TEST_END
 *   int main() { RUN(test_name); return TEST_SUMMARY(); }
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>
#include <string.h>

static int _test_pass_count = 0;
static int _test_fail_count = 0;
static const char *_current_test = "";

/* ---- 断言宏 ---- */

#define ASSERT_TRUE(expr) do { \
    if (!(expr)) { \
        printf("  FAIL: %s — ASSERT_TRUE(%s) @ %s:%d\n", \
               _current_test, #expr, __FILE__, __LINE__); \
        _test_fail_count++; return; \
    } \
} while (0)

#define ASSERT_FALSE(expr) do { \
    if (expr) { \
        printf("  FAIL: %s — ASSERT_FALSE(%s) @ %s:%d\n", \
               _current_test, #expr, __FILE__, __LINE__); \
        _test_fail_count++; return; \
    } \
} while (0)

#define ASSERT_INT_EQ(expected, actual) do { \
    int _e = (expected), _a = (actual); \
    if (_e != _a) { \
        printf("  FAIL: %s — ASSERT_INT_EQ(%d, %d) @ %s:%d\n", \
               _current_test, _e, _a, __FILE__, __LINE__); \
        _test_fail_count++; return; \
    } \
} while (0)

#define ASSERT_FLOAT_EQ(expected, actual, tol) do { \
    float _e = (float)(expected), _a = (float)(actual); \
    float _t = (float)(tol); \
    if (fabsf(_e - _a) > _t) { \
        printf("  FAIL: %s — ASSERT_FLOAT_EQ(%g, %g, tol=%g) diff=%g @ %s:%d\n", \
               _current_test, (double)_e, (double)_a, (double)_t, \
               (double)fabsf(_e - _a), __FILE__, __LINE__); \
        _test_fail_count++; return; \
    } \
} while (0)

#define ASSERT_DOUBLE_EQ(expected, actual, tol) do { \
    double _e = (double)(expected), _a = (double)(actual); \
    double _t = (double)(tol); \
    if (fabs(_e - _a) > _t) { \
        printf("  FAIL: %s — ASSERT_DOUBLE_EQ(%g, %g, tol=%g) diff=%g @ %s:%d\n", \
               _current_test, _e, _a, _t, fabs(_e - _a), __FILE__, __LINE__); \
        _test_fail_count++; return; \
    } \
} while (0)

/* ---- 测试定义/运行 ---- */

#define TEST_BEGIN(name) \
    static void name(void) { \
        _current_test = #name;

#define TEST_END \
        _test_pass_count++; \
    }

#define RUN_TEST(name) do { \
    name(); \
} while (0)

/* ---- 汇总 ---- */

#define TEST_SUMMARY() \
    printf("\n========================================\n"); \
    if (_test_fail_count == 0) \
        printf("ALL %d TESTS PASSED\n", _test_pass_count); \
    else \
        printf("FAILED: %d / %d tests failed\n", \
               _test_fail_count, _test_pass_count + _test_fail_count); \
    printf("========================================\n"); \
    return _test_fail_count > 0 ? 1 : 0

#endif /* TEST_FRAMEWORK_H */

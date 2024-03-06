//
// Created by LiuDongPeng on 2023/11/16.
//

#include "foc.h"
#include "arm_math.h"
#include "utils.h"

/**
 * @brief Clark transform
 * @param[in]   ia
 * @param[in]   ib
 * @param[out]  ialpha
 * @param[out]  ibeta
 */
void foc_clark(float ia, float ib, float *ialpha, float *ibeta)
{
    /* Calculate pIalpha using the equation, pIalpha = Ia */
    if (ialpha)
        *ialpha = ia;

    /* Calculate pIbeta using the equation, pIbeta = (1/sqrt(3)) * Ia + (2/sqrt(3)) * Ib */
    if (ibeta)
        *ibeta = ((float32_t) 0.57735026919 * ia + (float32_t) 1.15470053838 * ib);

//    arm_clarke_f32(ia, ib, ialpha, ibeta);
}


/**
 * @brief Park transform
 * @param[in]   ialpha
 * @param[in]   ibeta
 * @param[in]   sinTheta
 * @param[in]   cosTheta
 * @param[out]  id
 * @param[out]  iq
 */
void foc_park(float ialpha, float ibeta, float sinTheta, float cosTheta, float *id, float *iq)
{
    if (id)
        *id = ialpha * cosTheta + ibeta * sinTheta;

    if (iq)
        *iq = -ialpha * sinTheta + ibeta * cosTheta;

//    arm_park_f32(ialpha, ibeta, id, iq, sinTheta, cosTheta);
}

/**
 * @brief Inv park transform
 * @param[in]   ud
 * @param[in]   uq
 * @param[in]   sinTheta
 * @param[in]   cosTheta
 * @param[out]  ualpha
 * @param[out]  ubeta
 */
void foc_inv_park(float ud, float uq, float sinTheta, float cosTheta, float *ualpha, float *ubeta)
{
    if (ualpha)
        *ualpha = ud * cosTheta - uq * sinTheta;

    if (ubeta)
        *ubeta = ud * sinTheta + uq * cosTheta;

//    arm_inv_park_f32(ud, uq, ualpha, ubeta, sinTheta, cosTheta);
}


/**
 * @brief Calc 3 phase pwm duty
 * @param[in]   ualpha
 * @param[in]   ubeta
 * @param[in]   udc
 * @param[in]   tpwm
 * @param[out]  ta
 * @param[out]  tb
 * @param[out]  tc
 */
void foc_svpwm(float ualpha, float ubeta, float udc, float tpwm,
               float *ta, float *tb, float *tc)
{
    if (ta == NULL || tb == NULL || tc == NULL)
        return;

    // 0. 预备工作
    const float sqrt3 = SQRT_3;
    const float sqrt3TpwmUdc = sqrt3 * tpwm / udc;

    /* 1. 计算扇区 */
    const int sectorTable[8] = {0, 2, 6, 1, 4, 3, 5, 0};
    int sector;
    int A, B, C, N;
    float u1, u2, u3;

    u1 = ubeta;
    u2 = (sqrt3 * ualpha - ubeta) * 0.5f;
    u3 = (-sqrt3 * ualpha - ubeta) * 0.5f;

    A = u1 > 0 ? 1 : 0;
    B = u2 > 0 ? 1 : 0;
    C = u3 > 0 ? 1 : 0;
    N = 4 * C + 2 * B + A;
    sector = sectorTable[N];


    // 2. 根据扇区计算Tx Ty
    float Tx, Ty;
    switch (sector)
    {
        case 1:
            Tx = sqrt3TpwmUdc * u2;
            Ty = sqrt3TpwmUdc * u1;
            break;

        case 2:
            Tx = sqrt3TpwmUdc * -u2;
            Ty = sqrt3TpwmUdc * -u3;
            break;

        case 3:
            Tx = sqrt3TpwmUdc * u1;
            Ty = sqrt3TpwmUdc * u3;
            break;

        case 4:
            Tx = sqrt3TpwmUdc * -u1;
            Ty = sqrt3TpwmUdc * -u2;
            break;

        case 5:
            Tx = sqrt3TpwmUdc * u3;
            Ty = sqrt3TpwmUdc * u2;
            break;

        case 6:
        default:
            Tx = sqrt3TpwmUdc * -u3;
            Ty = sqrt3TpwmUdc * -u1;
            break;
    }

    float Tsum = Tx + Ty;
    if (Tsum > tpwm)
    {
        Tx = Tx / Tsum * tpwm;
        Ty = Ty / Tsum * tpwm;
    }

    // 3. 根据扇区计算三路PWM占空比
    float Sa, Sb, Sc;

    float val1 = (tpwm - Tx - Ty) * 0.25f;
    float val2 = val1 + Tx * 0.5f;
    float val3 = val2 + Ty * 0.5f;

    switch (sector)
    {
        case 1:
            Sa = val1;
            Sb = val2;
            Sc = val3;
            break;

        case 2:
            Sa = val2;
            Sb = val1;
            Sc = val3;
            break;

        case 3:
            Sa = val3;
            Sb = val1;
            Sc = val2;
            break;

        case 4:
            Sa = val3;
            Sb = val2;
            Sc = val1;
            break;

        case 5:
            Sa = val2;
            Sb = val3;
            Sc = val1;
            break;

        case 6:
        default:
            Sa = val1;
            Sb = val3;
            Sc = val2;
            break;
    }

    *ta = Sa;
    *tb = Sb;
    *tc = Sc;
}

/**
 * @brief Calc 3 phase pwm duty
 * @param ualpha
 * @param ubeta
 * @param VBus
 * @param ta
 * @param tb
 * @param tc
 * @return
 */
int foc_mid_point_svm(float ualpha, float ubeta, float VBus, float *ta, float *tb, float *tc)
{
    float alpha = ualpha / VBus;
    float beta = ubeta / VBus;

    float ua = alpha;
    float ub = -0.5f * alpha + SQRT3_DIV_2 * beta;
    float uc = -0.5f * alpha - SQRT3_DIV_2 * beta;

    float umax = fmaxf(fmaxf(ua, ub), uc);
    float umin = fminf(fminf(ua, ub), uc);
    float umid = 0.5f * (umax + umin);

    float tu = umid - ua + 0.5f;
    float tv = umid - ub + 0.5f;
    float tw = umid - uc + 0.5f;

    int result_valid =
            tu >= 0.0f && tu <= 1.0f &&
            tv >= 0.0f && tv <= 1.0f &&
            tw >= 0.0f && tw <= 1.0f;

    if (result_valid)
    {
        if (ta)
            *ta = tu;
        if (tb)
            *tb = tv;
        if (tc)
            *tc = tw;
    }

    return result_valid;
}


/**
 * @brief Calc 3 phase pwm duty
 * @param ualpha
 * @param ubeta
 * @param ta
 * @param tb
 * @param tc
 * @param sector
 * @return
 */
int odriver_svm(float ualpha, float ubeta, float *ta, float *tb, float *tc, int *sector)
{
    float tA, tB, tC;
    int Sextant;

    float alpha = ualpha;
    float beta = ubeta;

    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else
        {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else
    {
        if (alpha >= 0.0f)
        {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else
        {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant)
    {
        // sextant v1-v2
        case 1:
        {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        }
            break;

            // sextant v2-v3
        case 2:
        {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        }
            break;

            // sextant v3-v4
        case 3:
        {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        }
            break;

            // sextant v4-v5
        case 4:
        {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        }
            break;

            // sextant v5-v6
        case 5:
        {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        }
            break;

            // sextant v6-v1
        case 6:
        default:
        {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        }
            break;
    }

    int result_valid =
            tA >= 0.0f && tA <= 1.0f
            && tB >= 0.0f && tB <= 1.0f
            && tC >= 0.0f && tC <= 1.0f;

    if (result_valid)
    {
        if (ta)
            *ta = tA;
        if (tb)
            *tb = tB;
        if (tc)
            *tc = tC;
    }

    if (sector)
        *sector = Sextant;

    return result_valid;
}


/**
 *
 * @param pll
 * @param p
 * @param i
 * @param d
 * @param ka
 * @param lowLimit
 * @param upLimit
 * @return
 */
int foc_pll_init(foc_pll_t *pll, float p, float i, float d, float ka, float lowLimit, float upLimit)
{
    if (pll == 0)
        return -1;

    memset(pll, 0, sizeof(foc_pll_t));

    pll->pid.kp = p;
    pll->pid.ki = i;
    pll->pid.ka = ka;
    pll->pid.outLowLimit = lowLimit;
    pll->pid.outUpLimit = upLimit;

    pll->pid.ti = 1.0f / 20000.0f;

    return 0;
}

/**
 *
 * @param pll
 * @param rawAngle
 * @return
 */
int foc_pll_calc(foc_pll_t *pll, int rawAngle)
{
    int diff = foc_pid_diff(rawAngle, pll->lastAngle, 16384);
    pll->lastAngle = rawAngle;
    pll->speed += ((float) diff / 16384.0f - pll->speed) * pll->pid.kp;
    pll->speed = FOC_CLAMP(pll->speed, pll->pid.outLowLimit, pll->pid.outUpLimit);

    return 0;
}


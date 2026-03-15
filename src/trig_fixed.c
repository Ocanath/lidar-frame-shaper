#include "trig_fixed.h"

#define MAX_INT32 	0x7FFFFFFF
#define MIN_INT32	-0x80000000

/*Poly Coefficients used for sin calculation, scaled up by 2^12*/
static const int32_t sc1_12b = 117;
static const int32_t sc2_12b = -834;
static const int32_t sc3_12b = 85;
static const int32_t sc4_12b = 4078;
static const int32_t sc5_12b = 1;

/*Poly Coefficients for sin calculation, 14b*/
static const int32_t sc1_14b = 469;
static const int32_t sc2_14b = -3339;
static const int32_t sc3_14b = 342;
static const int32_t sc4_14b = 16310;
static const int32_t sc5_14b = 4;


/*Poly coefficients, scaled up by 2^12*/
static const int32_t tc1_12b = 580;    //2^12 scale
static const int32_t tc2_12b = -1406;    //2^12
static const int32_t tc3_12b = -66;    //2^12
static const int32_t tc4_12b = 4112;    //2^12
static const int32_t tc5_12b = -1*4096;    //2^12, with extra scaling factor applied for speed


static const int32_t tc1_14b = 2310;    //2^14 scale
static const int32_t tc2_14b = -5625;    //2^14
static const int32_t tc3_14b = -266;    //2^14
static const int32_t tc4_14b = 16447;    //2^14
static const int32_t tc5_14b = -3*16384;    //2^14, with extra scaling factor applied for speed




/*
*   Input:
        theta. MUST BE EXTERNALLY CONSTRAINED TO BE BETWEEN -PI_12B and PI_2B
    OUTPUT:
        ~sin(theta). range -4096 to 4096
*/
int32_t sin_12b(int32_t theta)
{
    //Preprocess theta to force it in the range 0-pi/2 for poly calculation
    uint8_t is_neg = 0;
    if(theta == 0)
    	return 0;	//sin of 0 is 0
    if(theta == PI_12B || theta == -PI_12B)
    	return 0;

    if (theta > HALF_PI_12B && theta <= PI_12B) // if positiveand in quadrant II, put in quadrant I(same)
    {
        theta = PI_12B - theta;
    }
    else if (theta >= PI_12B && theta < THREE_BY_TWO_PI_12B)
    {
        is_neg = 1;
        theta = theta - PI_12B;
    }
    else if (theta > THREE_BY_TWO_PI_12B && theta < TWO_PI_12B)
    {
        theta = theta - TWO_PI_12B;
    }
    else if (theta < -HALF_PI_12B && theta >= -PI_12B) // if negativeand in quadrant III,
    {
        is_neg = 1;
        theta = theta + PI_12B;
    }
    else if (theta < 0 && theta >= -HALF_PI_12B) // necessary addition for 4th order asymmetry
    {
        is_neg = 1;
        theta = -theta;
    }

    // 7 fixed point multiplies. compute polynomial output 0-1 for input 0-pi/2
    int32_t theta2 = (theta * theta) >> 12;
    int32_t theta3 = (theta2 * theta) >> 12;
    int32_t theta4 = (theta3 * theta) >> 12;
    int32_t res = sc1_12b * theta4 + sc2_12b * theta3 + sc3_12b * theta2 + sc4_12b * theta + (sc5_12b << 12);
    res = res >> 12;

    int32_t y;
    if (is_neg == 1)
        y = -res;
    else
        y = res;

    return y;
}

/*
*   Input:
        theta. MUST BE EXTERNALLY CONSTRAINED TO BE BETWEEN -PI_14B and PI_14B
    OUTPUT:
        ~sin(theta). range -16384 to 16384
*/
int32_t sin_14b(int32_t theta)
{
    //Preprocess theta to force it in the range 0-pi/2 for poly calculation
    uint8_t is_neg = 0;
    if(theta == 0)
    	return 0;	//sin of 0 is 0
    if(theta == PI_14B || theta == -PI_14B)
    	return 0;

    if (theta > HALF_PI_14B && theta <= PI_14B) // if positiveand in quadrant II, put in quadrant I(same)
    {
        theta = PI_14B - theta;
    }
    else if (theta >= PI_14B && theta < THREE_BY_TWO_PI_14B)
    {
        is_neg = 1;
        theta = theta - PI_14B;
    }
    else if (theta > THREE_BY_TWO_PI_14B && theta < TWO_PI_14B)
    {
        theta = theta - TWO_PI_14B;
    }
    else if (theta < -HALF_PI_14B && theta >= -PI_14B) // if negativeand in quadrant III,
    {
        is_neg = 1;
        theta = theta + PI_14B;
    }
    else if (theta < 0 && theta >= -HALF_PI_14B) // necessary addition for 4th order asymmetry
    {
        is_neg = 1;
        theta = -theta;
    }

    // 7 fixed point multiplies. compute polynomial output 0-1 for input 0-pi/2
    int32_t theta2 = (theta * theta) >> 14;
    int32_t theta3 = (theta2 * theta) >> 14;
    int32_t theta4 = (theta3 * theta) >> 14;
    int32_t res = sc1_14b * theta4 + sc2_14b * theta3 + sc3_14b * theta2 + sc4_14b * theta + (sc5_14b << 14);
    res = res >> 14;

    int32_t y;
    if (is_neg == 1)
        y = -res;
    else
        y = res;

    return y;
}


int32_t cos_12b(int32_t theta)
{
    return sin_12b(theta + HALF_PI_12B);
}

int32_t cos_14b(int32_t theta)
{
    return sin_14b(theta + HALF_PI_14B);
}



int32_t atan2_12b(int32_t y, int32_t x)
{
    //assert(isa(y, 'int32') & isa(x, 'int32'), 'Error: inputs must be of type int32');

    //capture edge cases, prevent div by 0
    if (x == 0)
    {
        if (y == 0)
            return 0;
        else if (y > 0)
            return HALF_PI_12B;// pi / 
        else
            return -HALF_PI_12B; // -pi / 2
    }
    if (y == 0)
    {
        if (x > 0)
            return 0;
        else
            return PI_12B;// pi
    }

    // get absolute value of both
    int32_t abs_s = y;
    if (abs_s < 0)
        abs_s = -abs_s;
    int32_t abs_c = x;
    if (abs_c < 0)
        abs_c = -abs_c;

    // get min value of both
    int32_t minv = abs_c;
    int32_t maxv = abs_s;
    if (maxv < minv)
    {
        minv = abs_s;
        maxv = abs_c;
    }

    //// do a fixed point division...
    // pre-apply a fixed point scaling factor of 2 ^ 12 with a shift
    int32_t a = (minv << 12) / maxv;// this is guaranteed to range from 0 - 4096, as minv-maxv is constrained from 0-1 by above logic

    //compute some exponents for the polyomial approximation
    int32_t a2 = (a * a) >> 12;// remove double application of scaling factor 2 ^ 12 with a rightshift
    int32_t a3 = (a2 * a) >> 12;
    int32_t a4 = (a3 * a) >> 12;

    // best, most multiplies, 4th order
    int32_t r = a4 * tc1_12b + a3 * tc2_12b + a2 * tc3_12b + a * tc4_12b + tc5_12b; //compute polynomial result. Shift the last coefficient, to apply the factor of 2^12 present in all the exponents
    r = r >> 12;    //remove the factor of 2^15 added by the coefficints, leaving only the factor of 2^12 that was preserved in the exponents

    // fp_coef from fixed_point_foc
    if (abs_s > abs_c)
        r = HALF_PI_12B - r;
    if (x < 0)
        r = PI_12B - r;
    if (y < 0)
        r = -r;

    return r;   //output range -PI_12B to PI_12B
}

int32_t atan2_14b(int32_t y, int32_t x)
{
    //assert(isa(y, 'int32') & isa(x, 'int32'), 'Error: inputs must be of type int32');

    //capture edge cases, prevent div by 0
    if (x == 0)
    {
        if (y == 0)
            return 0;
        else if (y > 0)
            return HALF_PI_14B;// pi /
        else
            return -HALF_PI_14B; // -pi / 2
    }
    if (y == 0)
    {
        if (x > 0)
            return 0;
        else
            return PI_14B;// pi
    }

    // get absolute value of both
    int32_t abs_s = y;
    if (abs_s < 0)
        abs_s = -abs_s;
    int32_t abs_c = x;
    if (abs_c < 0)
        abs_c = -abs_c;

    // get min value of both
    int32_t minv = abs_c;
    int32_t maxv = abs_s;
    if (maxv < minv)
    {
        minv = abs_s;
        maxv = abs_c;
    }

    //// do a fixed point division...
    // pre-apply a fixed point scaling factor of 2 ^ 14 with a shift
    int32_t a = (minv << 14) / maxv;// this is guaranteed to range from 0 - 4096, as minv-maxv is constrained from 0-1 by above logic

    //compute some exponents for the polyomial approximation
    int32_t a2 = (a * a) >> 14;// remove double application of scaling factor 2 ^ 14 with a rightshift
    int32_t a3 = (a2 * a) >> 14;
    int32_t a4 = (a3 * a) >> 14;

    // best, most multiplies, 4th order
    int32_t r = a4 * tc1_14b + a3 * tc2_14b + a2 * tc3_14b + a * tc4_14b + tc5_14b; //compute polynomial result. Shift the last coefficient, to apply the factor of 2^14 present in all the exponents
    r = r >> 14;    //remove the factor of 2^15 added by the coefficints, leaving only the factor of 2^14 that was preserved in the exponents

    // fp_coef from fixed_point_foc
    if (abs_s > abs_c)
        r = HALF_PI_14B - r;
    if (x < 0)
        r = PI_14B - r;
    if (y < 0)
        r = -r;

    return r;   //output range -PI_14B to PI_14B
}


/*This function performs 'wrapping' operations on input angles. Can have arbitrary scale.
The conditionals are there to capture the fact that mod() in matlab has different behavior than % operator
on negative numbers. We are capturing the cases where (in + PI_12B) is less than 0
*/
int32_t wrap_2pi_12b(int32_t in)
{
    int32_t result = ((in + PI_12B) % TWO_PI_12B) - PI_12B;
    if (in < -PI_12B) //if( (in + PI_12B) < 0 )
        return TWO_PI_12B + result;
    else 
        return result;
}

int32_t wrap_2pi_fixed(int32_t in, int32_t two_pi_fixed)
{
	int32_t pi_fixed = two_pi_fixed / 2;
    int32_t result = ((in + pi_fixed) % two_pi_fixed) - pi_fixed;
    if (in < -pi_fixed) //if( (in + pi_fixed) < 0 )
        return two_pi_fixed + result;
    else
        return result;
}

/*This function performs 'wrapping' operations on input angles. Can have arbitrary scale.
The conditionals are there to capture the fact that mod() in matlab has different behavior than % operator
on negative numbers. We are capturing the cases where (in + PI_14B) is less than 0
*/
int32_t wrap_2pi_14b(int32_t in)
{
    int32_t result = ((in + PI_14B) % TWO_PI_14B) - PI_14B;
    if (in < -PI_14B) //if( (in + PI_14B) < 0 )
        return TWO_PI_14B + result;
    else
        return result;
}



/*64 bit analogue which we need for unwrap_64*/
int64_t wrap_2pi12b_64(int64_t in)
{
	int64_t result = ((in + PI_12B) % TWO_PI_12B) - PI_12B;
    if (in < -PI_12B) //if( (in + PI_12B) < 0 )
        return TWO_PI_12B + result;
    else
        return result;
}

// sqrt_i32 computes the squrare root of a 32bit integer and returns
// a 32bit integer value. It requires that v is positive.
int32_t sqrt_i32(int32_t v)
{
	uint32_t b = 1 << 30, q = 0, r = v;
	while (b > r)
		b >>= 2;
	while (b > 0)
	{
		uint32_t t = q + b;
		q >>= 1;
		if (r >= t)
		{
			r -= t;
			q += b;
		}
		b >>= 2;
	}
	return q;
}


// sqrt_i64 computes the squrare root of a 64bit integer and returns
// a 64bit integer value. It requires that v is positive.
int64_t sqrt_i64(int64_t v)
{
	uint64_t b = ((uint64_t)1) << 62, q = 0, r = v;
	while (b > r)
		b >>= 2;
	while (b > 0)
	{
		uint64_t t = q + b;
		q >>= 1;
		if (r >= t)
		{
			r -= t;
			q += b;
		}
		b >>= 2;
	}
	return q;
}




/*
 *
 * */
int32_t check_overflow32(int32_t a, int32_t b)
{

	if(a > 0)
	{
		int32_t upperbound = (MAX_INT32 - a);
		if(b > upperbound)
		{
			return 1;
		}
		else
			return 0;
	}
	else if (a < 0)
	{
		int32_t lowerbound = (MIN_INT32 - a);
		if(b < lowerbound)
		{
			return -1;
		}
		else
			return 0;
	}
	else
		return 0;
}

/*
 *
 * */
int32_t unwrap_angle_32b_overflow(int32_t theta, int32_t two_pi_fixed, unwrap_state_t * st)
{
	int32_t dif_wrapped = wrap_2pi_fixed(theta - st->prev_theta_wrapped, two_pi_fixed);
	st->prev_theta_wrapped = theta;
	int32_t ovfl = check_overflow32(st->unwrapped_angle, dif_wrapped);
	st->unwrapped_angle += dif_wrapped;
	st->ovfl_cnt += ovfl;

	return st->unwrapped_angle;
}




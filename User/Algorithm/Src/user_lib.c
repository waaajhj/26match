/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file	user_lib.c/h
 * @brief	用户自定义函数
 * @history
 * Date            Author          Modification
 * @attention
 ==============================================================================
 ==============================================================================
 *************************(C) COPYRIGHT 2024 DragonBot*************************
*/
#include "user_lib.h"

// 快速开方
float Sqrt(float x)
{
	float y;
	float delta;
	float maxError;

	if (x <= 0)
	{
		return 0;
	}

	// initial guess
	y = x / 2;

	// refine
	maxError = x * 0.001f;

	do
	{
		delta = (y * y) - x;
		y -= delta / (2 * y);
	} while (delta > maxError || delta < -maxError);

	return y;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
	if (Value < minValue)
		return minValue;
	else if (Value > maxValue)
		return maxValue;
	else
		return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
	if (Value < minValue)
		return minValue;
	else if (Value > maxValue)
		return maxValue;
	else
		return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
	if (maxValue < minValue)
	{
		return Input;
	}

	if (Input > maxValue)
	{
		float len = maxValue - minValue;
		while (Input > maxValue)
		{
			Input -= len;
		}
	}
	else if (Input < minValue)
	{
		float len = maxValue - minValue;
		while (Input < minValue)
		{
			Input += len;
		}
	}
	return Input;
}

// 弧度格式化为-PI~PI
float rad_format(float Ang)
{
	return loop_float_constrain(Ang, -PI, PI);
}

float Max(float Value1, float Value2)
{
	if (Value1 > Value2)
		return Value1;
	else
		return Value2;
}

float Min(float Value1, float Value2)
{
	if (Value1 < Value2)
		return Value1;
	else
		return Value2;
}

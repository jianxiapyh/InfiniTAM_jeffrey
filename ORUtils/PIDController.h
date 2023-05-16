// Copyright 2022 RSIM Group

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>

#include "MathUtils.h"

namespace ORUtils
{
	// Based on https://dl.acm.org/doi/pdf/10.1145/3410463.3414636
	// std::is_arithmetic<T>::value **MUST** be true
	template<class T>
	class GenericPIDController
	{
	private:
		// Control coefficients
		const T Kp;
		const T Ki;
		const T Kd;

		// Dynamic setpoint info
		const T alpha;
		T count{};
		T sumVal{};
		T minVal{std::numeric_limits<T>::max()};
		T maxVal{std::numeric_limits<T>::min()};
		T lowPoint{};
		T highPoint{};
		T binSize{};

		// Valid outputs
		const unsigned minOutputIdx;
		const unsigned maxOutputIdx;
        //pyh add default freq index
        const unsigned default_freq_idx;
		const T numOutputs;
		const std::vector<T> outputs;

		// Sliding window
		std::vector<T> window;
		static constexpr unsigned WINDOW_SIZE{30};

		// Bootstrapping -- lasts 1 second at 30 fps
		bool bootstrap{true};
		static constexpr unsigned BOOTSTRAP_LENGTH{30};

	private:
		void enqueue(const T val)
		{
			window.push_back(val);
			if (window.size() == (WINDOW_SIZE + 1))
				window.erase(window.begin());
		}

		T average() const
		{
			return std::accumulate(window.begin(), window.end(), T{}) / static_cast<double>(window.size());
		}

		void updateSetpoints(const T val)
		{
			// Keep track of min, max, and sum of vals
			minVal = MIN(minVal, val);
			maxVal = MAX(maxVal, val);
			sumVal += val;

			// Update dynamic setpoints. Refer to Section 4.2 of https://dl.acm.org/doi/pdf/10.1145/3410463.3414636 for more details.
			const T avgVal = sumVal / count;
			lowPoint  = ((1.0f - alpha) * minVal) + (alpha * avgVal);
			highPoint = ((1.0f - alpha) * avgVal) + (alpha * maxVal);
			binSize = (highPoint - lowPoint) / numOutputs;
		}

		T calculatePIdx(const T val) const
		{
			// Linear interpolation between low and high, with the range divided into equal sized 'numOutputs' bins
			return (val - lowPoint) / binSize;
		}

		T calculateIIdx() const
		{
			// Average of 'val - lowPoint'
			const T avgDiff = average();

			// Go down a bin if we were, on average, lower than the low point
			if (avgDiff < 0)
				return T{-1};

			// Go up a bin if we were, on average, higher than the high point
			if ((lowPoint + avgDiff) > highPoint)
				return T{1};

			// Proportional change
			return avgDiff / binSize;
		}

		T calculateDIdx() const
		{
			bool ascending = std::is_sorted(window.begin(), window.end());

			// Go up a bin
			if (ascending)
				return T{1};
			else
				return T{};
		}

		T calculate(const T val) const
		{
            //if(Kp == 0 && Ki ==0 && Kd==0 && alpha==100)
            //{
            //    //pyh kind of stupid but I need to find the index for the nearest frequency
            //    printf("constant frequency triggered, default freq: %f \n", default_freq);
            //    //                                   30   15  10     9     8       7     6    5   4    3    2     1
            //    const std::vector<double> avail_freq{1.0, 2.0, 3.0, 3.333, 3.75 , 4.29, 5.0, 6.0, 7.5, 10.0, 15.0, 30.0};
            //    double min_diff = 100;
            //    double closest_index=0;
            //    for(unsigned i=0; i< avail_freq.size(); i++)
            //    {
            //        double difference = std::abs(avail_freq[i]-default_freq);
            //        if(difference < min_diff)
            //        {
            //            min_diff = difference;
            //            closest_index = i;
            //        }
            //    }
            //    printf("selected freq %f\n", outputs[closest_index]);
            //    return outputs[closest_index];
            //}
            //printf("non-constant freq triggered\n");
			// If the value is too low, run at min
			if (val < lowPoint){
                //printf("pyhtest: lower than lowpoint detected printing %f\n", outputs[minOutputIdx]);
                if(Kp == 0 && Ki ==0 && Kd ==0)
                    return outputs[default_freq_idx];

				return outputs[minOutputIdx];
            }

			//// If the value is too high, run at max
			if (val > highPoint){
                //printf("pyhtest: higher than lowpoint detected printing %f\n", outputs[maxOutputIdx]);
                if(Kp == 0 && Ki ==0 && Kd ==0)
                    return outputs[default_freq_idx];
				
                return outputs[maxOutputIdx];
            }
            // Proportional term
			T pIdx = Kp * calculatePIdx(val);

			// Integral term
			T iIdx = Ki * calculateIIdx();

			// Derivative term
			T dIdx = Kd * calculateDIdx();

			// Final output index
			T outputIdx = default_freq_idx + pIdx + iIdx + dIdx;
			outputIdx = CLAMP(std::floor(outputIdx), minOutputIdx, maxOutputIdx);

			return outputs[outputIdx];
		}

	public:
		GenericPIDController(const T Kp_, const T Ki_, const T Kd_, const T alpha_, const std::vector<T> &outputs_)
        //pyh: add a parameter to cap min frequency, 0 => 30 to 1 where 1=> 15 to 1, 
        //this allows me to have PID operate at constant frequency since it will always output the minOutputIdx 
		//GenericPIDController(const T Kp_, const T Ki_, const T Kd_, const T alpha_, const std::vector<T> &outputs_, const T default_frequency)
			: Kp{Kp_}
			, Ki{Ki_}
			, Kd{Kd_}
			, alpha{alpha_}
			, minOutputIdx{0}
			, maxOutputIdx{static_cast<unsigned>(outputs_.size() - 1)}
            //, default_freq{default_frequency}
			, numOutputs{static_cast<double>(outputs_.size())}
			, outputs{outputs_}
            //8 for 7.5
            //6 for 5
            //10 for 15
            //2 for 2
            , default_freq_idx{1}
		{
			static_assert(std::is_arithmetic<T>::value, "Type must be an arithmetic type!");
		}

		T Calculate(T val)
		{
			count++;
			updateSetpoints(val);
			enqueue(val - lowPoint);

			// Bootstrap phase ends when the sliding window fills up
			if (count == BOOTSTRAP_LENGTH)
				bootstrap = false;

			// Run at max until bootstrap phase finishes
			if (bootstrap)
				return outputs[maxOutputIdx];

			return calculate(val);
		}
	};

	using PIDController = GenericPIDController<double>;
}

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace Bepu
{
	struct SpringSettings
	{
		/// <summary>
		/// Target number of undamped oscillations per unit of time, scaled by 2 * PI.
		/// </summary>
		float AngularFrequency;
		/// <summary>
		/// Twice the ratio of the spring's actual damping to its critical damping.
		/// </summary>
		float TwiceDampingRatio;

		/// <summary>
		/// Gets or sets the target number of undamped oscillations per unit of time.
		/// </summary>
		float GetFrequency() { return AngularFrequency / (2 * M_PI); }
		void SetFrequency(float value) { AngularFrequency = value * (2 * M_PI); }

		/// <summary>
		/// Gets or sets the ratio of the spring's actual damping to its critical damping. 0 is undamped, 1 is critically damped, and higher values are overdamped.
		/// </summary>
		float GetDampingRatio() { return TwiceDampingRatio / 2.0f; }
		void SetDampingRatio(float value) { TwiceDampingRatio = value * 2.0f; }

		SpringSettings() { AngularFrequency = 0; TwiceDampingRatio = 0; }

		/// <summary>
		/// Constructs a new spring settings instance.
		/// </summary>
		/// <param name="frequency">Target number of undamped oscillations per unit of time.</param>
		/// <param name="dampingRatio">Ratio of the spring's actual damping to its critical damping. 0 is undamped, 1 is critically damped, and higher values are overdamped.</param>
		SpringSettings(float frequency, float dampingRatio)
		{
			AngularFrequency = frequency * (2 * M_PI);
			TwiceDampingRatio = dampingRatio * 2;
		}
	};
}
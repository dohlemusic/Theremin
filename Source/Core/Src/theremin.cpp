/*
 * theremin.cpp
 *
 *  Created on: Jul 3, 2024
 *      Author: Max
 */

#include "theremin.hpp"

#include <daisysp.h>

namespace theremin {
daisysp::Oscillator oscilator;
daisysp::LadderFilter filter;
daisysp::OnePole freqFilter;

float smoothstep (float x) {
   return x * x * (3.0f - 2.0f * x);
}


void init() {
	oscilator.Init(44100);
	oscilator.SetFreq(440.f);
	oscilator.SetWaveform(oscilator.WAVE_POLYBLEP_TRI);
	oscilator.SetAmp(1);

	filter.Init(44100);
	filter.SetFilterMode(filter.FilterMode::LP24);
	filter.SetFreq(2500.f);
	filter.SetRes(0.4f);

	freqFilter.Init();
	freqFilter.SetFilterMode(daisysp::OnePole::FILTER_MODE_LOW_PASS);
	freqFilter.SetFrequency(0.05);
}

void updatePitch(float frequencyHz) {
	oscilator.SetFreq(freqFilter.Process(frequencyHz));
}

void updateCutoff(float cutoff)
{
	filter.SetFreq(cutoff);
}

void updateVolume(float volume) {
	oscilator.SetAmp(volume);
}

float processSample() {
	float sample = oscilator.Process();

	// model assymetric clipping of the bottom of the triangle wave, adding some warm fuzz to the signal
	sample = 0.25f * sample + 0.75f;
	sample = smoothstep(sample);
	sample = 4.f * (sample - 0.75f);

	return 2.f * filter.Process(sample);
}
}

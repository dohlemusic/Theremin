/*
 * theremin.cpp
 *
 *  Created on: Jul 3, 2024
 *      Author: Max
 */

#include "theremin.hpp"

#include <daisysp.h>

namespace theremin {
daisysp::VariableShapeOscillator oscilator;

daisysp::OnePole filter1;
daisysp::OnePole filter2;

daisysp::OnePole freqFilter;
float outputVolume = 1.f;

float smoothstep (float x) {
   return x * x * (3.0f - 2.0f * x);
}


void init() {
	oscilator.Init(44100);
	oscilator.SetFreq(440.f);
	oscilator.SetSync(false);
	oscilator.SetPW(0.5);

	filter1.Init();
	filter1.SetFilterMode(daisysp::OnePole::FILTER_MODE_LOW_PASS);
	filter1.SetFrequency(0.497f);
	filter2.Init();
	filter2.SetFilterMode(daisysp::OnePole::FILTER_MODE_LOW_PASS);
	filter2.SetFrequency(0.497f);

	freqFilter.Init();
	freqFilter.SetFilterMode(daisysp::OnePole::FILTER_MODE_LOW_PASS);
	freqFilter.SetFrequency(0.05);
}

void updatePitch(float frequencyHz) {
	oscilator.SetSyncFreq(freqFilter.Process(frequencyHz));
}

void updateCutoff(float cutoff)
{
	filter1.SetFrequency(0.497f * cutoff);
	filter2.SetFrequency(0.497f * cutoff);
}

void updateVolume(float volume) {
	outputVolume = volume;
}

void updateWaveShape(float waveShape) {
	oscilator.SetWaveshape(waveShape);
}

float processSample() {
	float sample = oscilator.Process();

	// model assymetric clipping of the bottom of the triangle wave, adding some warm fuzz to the signal
	sample = 0.25f * sample + 0.75f;
	sample = smoothstep(sample);
	sample = 4.f * (sample - 0.75f);

	return filter2.Process(filter1.Process(sample)) * outputVolume;
}
}

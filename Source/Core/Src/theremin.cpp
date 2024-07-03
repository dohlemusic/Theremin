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

void init() {
	oscilator.Init(44100);
	oscilator.SetFreq(440.f);
	oscilator.SetWaveform(oscilator.WAVE_POLYBLEP_TRI);
	oscilator.SetAmp(1);

	filter.Init(44100);
	filter.SetFilterMode(filter.FilterMode::LP24);
	filter.SetFreq(1500.f);
	filter.SetRes(0.6f);

	freqFilter.Init();
	freqFilter.SetFilterMode(daisysp::OnePole::FILTER_MODE_LOW_PASS);
	freqFilter.SetFrequency(0.05);
}

void updatePitch(float frequencyHz) {
	oscilator.SetFreq(frequencyHz);
}

void updateVolume(float volume) {
	oscilator.SetAmp(volume);
}

float processSample() {
	//oscillator.SetSyncFreq(440.f + lfoOut * 10);
	//filter.SetFreq(600.f + 4000.f * (lfoOut + 0.2));
	//oscillator.SetWaveshape(0.5f * (lfoOut + 1.f));
	return filter.Process(oscilator.Process());
}
}

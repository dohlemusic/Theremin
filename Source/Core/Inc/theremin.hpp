/*
 * theremin.hpp
 *
 *  Created on: Jul 3, 2024
 *      Author: Max
 */

#ifndef INC_THEREMIN_HPP_
#define INC_THEREMIN_HPP_


namespace theremin
{
void init();
void updatePitch(float frequencyHz);
void updateCutoff(float cutoff);
void updateVolume(float volume);
void updateWaveShape(float waveShape);
float processSample();
}


#endif /* INC_THEREMIN_HPP_ */

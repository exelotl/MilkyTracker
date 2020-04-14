/*
 * Copyright (c) 2009, The MilkyTracker Team.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the <ORGANIZATION> nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  AudioDriver_3DS.cpp
 *  MilkyPlay
 *
 *  Created by Jeremy Clarke on 11.4.20
 *
 */

#include "AudioDriver_3DS.h"

#include <iostream>
#include <cstdlib>
using namespace std;

void AudioDriver_3DS::fill_audio(void *userdata)
{
	AudioDriver_3DS* driver = (AudioDriver_3DS*)userdata;
	ndspWaveBuf *buf = &driver->audioBuf[driver->currentBufIndex];
	
	int bufferSizeInBytes = driver->bufferSize * sizeof(u16);
	
	if (buf->status == NDSP_WBUF_DONE) {
		if (driver->mono) {
			// ?
		} else {
			driver->fillAudioWithCompensation((char*)buf->data_pcm16, bufferSizeInBytes/2);
		}

		DSP_FlushDataCache(buf->data_pcm8, bufferSizeInBytes);

		ndspChnWaveBufAdd(0, buf);
		driver->currentBufIndex ^= 1;
	}
	
}

AudioDriver_3DS::AudioDriver_3DS() :
	AudioDriver_COMPENSATE()
{
}

AudioDriver_3DS::~AudioDriver_3DS()
{
}

// On error return a negative value
// If the requested buffer size can be served return MP_OK,
// otherwise return the number of 16 bit words contained in the obtained buffer
mp_sint32 AudioDriver_3DS::initDevice(mp_sint32 bufferSizeInWords, mp_uint32 mixFrequency, MasterMixer* mixer)
{
	mp_sint32 res = AudioDriverBase::initDevice(bufferSizeInWords, mixFrequency, mixer);
	if (res < 0)
	{
		return res;
	}
	
	s32 bufferSizeInBytes = bufferSize * sizeof(u16);

	audioData = (u8*) linearAlloc(2 * bufferSizeInBytes);
	clearAudioData();

	Result initRes = ndspInit();
	start();
	return MP_OK;
}

void AudioDriver_3DS::clearAudioData()
{
	s32 bufferSizeInBytes = bufferSize * sizeof(u16);
	memset(audioData, 0, 2 * bufferSizeInBytes);
	currentBufIndex = 0;
}

mp_sint32 AudioDriver_3DS::stop()
{
	ndspChnWaveBufClear(0);
	clearAudioData();
	deviceHasStarted = false;
	return MP_OK;
}

mp_sint32 AudioDriver_3DS::closeDevice()
{
	linearFree(audioData);
	ndspExit();
	deviceHasStarted = false;
	return MP_OK;
}

mp_sint32 AudioDriver_3DS::start()
{

	float mixerParams3ds[12] = {1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	ndspSetOutputMode(mono ? NDSP_OUTPUT_MONO : NDSP_OUTPUT_STEREO);
	ndspSetOutputCount(1);

	ndspChnReset(0);
	// ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
	ndspChnSetInterp(0, NDSP_INTERP_NONE);
	ndspChnSetRate(0, mixFrequency);
	ndspChnSetFormat(0, NDSP_CHANNELS(mono ? 1 : 2) | NDSP_ENCODING(NDSP_ENCODING_PCM16));
	ndspChnSetMix(0, mixerParams3ds);

	ndspSetMasterVol(1.0f);
	ndspSetCallback(fill_audio, this);
	
	s32 bufferSizeInBytes = bufferSize * sizeof(u16);

	audioBuf[0].data_vaddr = &audioData[0];
	audioBuf[0].nsamples = bufferSize/2;
	audioBuf[1].data_vaddr = &audioData[bufferSizeInBytes];
	audioBuf[1].nsamples = bufferSize/2;

	clearAudioData();

	ndspChnWaveBufAdd(0, &audioBuf[0]);
	ndspChnWaveBufAdd(0, &audioBuf[1]);

	deviceHasStarted = true;
	return MP_OK;
}

mp_sint32 AudioDriver_3DS::pause()
{
	return MP_OK;
}

mp_sint32 AudioDriver_3DS::resume()
{
	return MP_OK;
}

#include "daisy_pod.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyPod hw;

// Constants
const int NUM_DELAYS = 6;
const int NUM_PATCHES = 4;
const float MAX_DELAY_TIME = 1.0f; // 1 second max delay

// Delay lines
DelayLine<float, 12000> delays[NUM_DELAYS]; // 1 second max delay at 48kHz

// Modulation
Oscillator lfo[NUM_DELAYS];
float modDepth[NUM_DELAYS];
float modRate[NUM_DELAYS];
float baseDelayTime[NUM_DELAYS];
float feedback[NUM_DELAYS];
float delayLevel[NUM_DELAYS]; // Individual level for each delay

// Global parameters
float dryWet; // 0-1
int currentPatch;

// Helper functions
void UpdateKnobs();
void UpdateLeds();
void ProcessAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);
void InitDelays();
void SetPatch(int patch);

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    UpdateKnobs();
    ProcessAudio(in, out, size);
    UpdateLeds();
}

void UpdateKnobs()
{
    hw.ProcessAllControls();
    
    // Large knob changes patches
    if(hw.encoder.Increment() != 0)
    {
        currentPatch += hw.encoder.Increment();
        if(currentPatch < 0) currentPatch = NUM_PATCHES - 1;
        if(currentPatch >= NUM_PATCHES) currentPatch = 0;
        SetPatch(currentPatch);
    }
    
    // Small knob 1 controls mod depth and dry/wet
    float knob1 = hw.knob1.Process();

    // First half of knob controls dry/wet (0-100%)
    dryWet = knob1 * 2.0f;
    
    // for(int i = 0; i < NUM_DELAYS; i++)
    // {
    //     modDepth[i] = (knob1 - 0.5f) * 1.4f;
    // }
    
    // Small knob 2 controls mod rate
    float knob2 = hw.knob2.Process();

    switch(currentPatch)
    {
        case 0: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                
                lfo[i].SetFreq(modRate[i] *knob2 * 0.5f);
            }
            break;
                
        case 1: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                feedback[i] = knob2 * 2.0f; // Reduced range for reverb modulation
                
            }
            break;

        case 2: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                modRate[i] = knob2 * 0.5f; // Full range for combined patch
                lfo[i].SetFreq(modRate[i]);
            }
            break;

        case 3: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                
                lfo[i].SetFreq(modRate[i] *knob2 * 0.5f);
            }
            break;
    }
}

void UpdateLeds()
{
    // Set LEDs based on current patch
    switch(currentPatch)
    {
        case 0: // Chorus
            hw.led1.Set(0, 1, 0); // Green
            hw.led2.Set(0, 0, 0);
            break;
        case 3: // Chorus
            hw.led1.Set(0, 1, 0); // Green
            hw.led2.Set(0, 1, 0);
            break;
        case 1: // Reverb
            hw.led1.Set(0, 0, 0);
            hw.led2.Set(1, 0, 0); // Red
            break;
        case 2: // Combined
            hw.led1.Set(0, 1, 0); // Green
            hw.led2.Set(1, 0, 0); // Red
            break;
    }
    hw.UpdateLeds();
}

void ProcessAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        float dryL = in[0][i];
        float dryR = in[1][i];
        float wetL = 0.0f;
        float wetR = 0.0f;
        int activeDelaysL = 0;
        int activeDelaysR = 0;
        
        // Process each delay line
        for(int d = 0; d < NUM_DELAYS; d++)
        {
            // Skip if delay level is zero
            if(delayLevel[d] <= 0.0f) continue;
            
            // Calculate modulated delay time
            float mod = lfo[d].Process() * modDepth[d];
            float delayTime = baseDelayTime[d] * (1.0f + mod);
            delayTime = fclamp(delayTime, 0.0f, MAX_DELAY_TIME);
            
            delays[d].SetDelay(delayTime * hw.AudioSampleRate());
            
            // Read from delay
            float delayed = delays[d].Read() * delayLevel[d];
            
            // Write to delay (with feedback)
            float input = (d < 3) ? dryL : dryR; // First 3 delays are L, next 3 are R
            delays[d].Write(input + delayed * feedback[d]);
            
            // Accumulate to wet signal and count active delays
            if(d < 3) {
                wetL += delayed;
                activeDelaysL++;
            }
            else {
                wetR += delayed;
                activeDelaysR++;
            }
        }
        
        // Mix dry and wet (only average active delays)
        if(activeDelaysL > 0) wetL /= activeDelaysL;
        if(activeDelaysR > 0) wetR /= activeDelaysR;
        
        out[0][i] = dryL * (1.0f - dryWet) + wetL * dryWet;
        out[1][i] = dryR * (1.0f - dryWet) + wetR * dryWet;
    }
}

void InitDelays()
{
    for(int i = 0; i < NUM_DELAYS; i++)
    {
        delays[i].Init();
        lfo[i].Init(hw.AudioSampleRate());
        lfo[i].SetWaveform(Oscillator::WAVE_SIN);
        delayLevel[i] = 1.0f; // Default to full level
    }
    
    // Set initial patch
    currentPatch = 0;
    SetPatch(currentPatch);
    
    // Initialize global parameters
    dryWet = 0.5f;
}

void SetPatch(int patch)
{
    switch(patch)
    {
        case 0: // Chorus patch - using 2 left and 2 right delays
            // Left channel delays (0-2)
            baseDelayTime[0] = 0.0236f;  // 5ms
            baseDelayTime[1] = 0.030f;  // 7ms
            baseDelayTime[2] = 0.0f;    // Disabled
            
            // Right channel delays (3-5)
            baseDelayTime[3] = 0.0409f;  // 6ms
            baseDelayTime[4] = 0.0482f;  // 8ms
            baseDelayTime[5] = 0.0f;    // Disabled
            
            // Modulation settings
            modDepth[0] = 0.2f;   modRate[0] = 3.9f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.3f;   modRate[1] = 5.2f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.2f;  modRate[3] = 4.0f; feedback[3] = 0.0f; delayLevel[3] = 1.0f;
            modDepth[4] = 0.3f;  modRate[4] = 4.9f; feedback[4] = 0.0f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.0f;   modRate[5] = 0.0f; feedback[5] = 0.0f; delayLevel[5] = 0.0f;
            break;

        case 3: // Chorus patch - using 2 left and 2 right delays
            // Left channel delays (0-2)
            baseDelayTime[0] = 0.0236f;  // 5ms
            baseDelayTime[1] = 0.030f;  // 7ms
            baseDelayTime[2] = 0.0f;    // Disabled
            
            // Right channel delays (3-5)
            baseDelayTime[3] = 0.036f;  // 6ms
            baseDelayTime[4] = 0.028f;  // 8ms
            baseDelayTime[5] = 0.0f;    // Disabled
            
            // Modulation settings
            modDepth[0] = 0.2f;   modRate[0] = 6.5f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.2f;   modRate[1] = 5.7f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.2f;  modRate[3] = 4.8f; feedback[3] = 0.0f; delayLevel[3] = 1.0f;
            modDepth[4] = 0.2f;  modRate[4] = 4.4f; feedback[4] = 0.0f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.0f;   modRate[5] = 0.0f; feedback[5] = 0.0f; delayLevel[5] = 0.0f;
            break;
            
        case 1: // Reverb patch - using 2 left and 2 right delays
            // Left channel delays
            baseDelayTime[0] = 0.25f; // 25ms
            baseDelayTime[1] = 0.35f; // 35ms
            baseDelayTime[2] = 0.0f;   // Disabled
            
            // Right channel delays
            baseDelayTime[3] = 0.40f; // 30ms
            baseDelayTime[4] = 0.50f; // 40ms
            baseDelayTime[5] = 0.0f;   // Disabled
            
            // Modulation settings
            modDepth[0] = 0.00f;  modRate[0] = 0.0f; feedback[0] = 0.07f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.00f;  modRate[1] = 0.0f; feedback[1] = 0.065f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.0f; modRate[3] = 0.00f; feedback[3] = 0.068f; delayLevel[3] = 1.0f;
            modDepth[4] = 0.0f; modRate[4] = 0.00f; feedback[4] = 0.072f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.0f;    modRate[5] = 0.0f; feedback[5] = 0.0f; delayLevel[5] = 0.0f;
            break;
            
        case 2: // Combined patch - using all delays
            // Left channel - first 2 chorus, 1 reverb
            baseDelayTime[0] = 0.0236f;  // 5ms
            baseDelayTime[1] = 0.030f;  // 7ms
            baseDelayTime[2] = 0.5f;    // Disabled
            
            // Right channel delays (3-5)
            baseDelayTime[3] = 0.036f;  // 6ms
            baseDelayTime[4] = 0.35f;  // 8ms
            baseDelayTime[5] = 0.6f;    // Disabled
            
            // Modulation settings
            modDepth[0] = 0.2f;   modRate[0] = 6.5f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.2f;   modRate[1] = 5.7f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.1f; delayLevel[2] = 0.2f;
            modDepth[3] = 0.2f;  modRate[3] = 4.8f; feedback[3] = 0.0f; delayLevel[3] = 1.0f;
            modDepth[4] = 0.0f;  modRate[4] = 4.4f; feedback[4] = 0.1f; delayLevel[4] = 0.2f;
            modDepth[5] = 0.0f;   modRate[5] = 0.0f; feedback[5] = 0.1f; delayLevel[5] = 0.2f;
            
            break;
    }
    
    // Update LFO rates based on current settings
    for(int i = 0; i < NUM_DELAYS; i++)
    {
        lfo[i].SetFreq(modRate[i]);
    }
}

int main(void)
{
    hw.Init();
    //hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_96KHZ);
    //hw.SetAudioBlockSize(4); // You may want to adjust this for 96kHz
    hw.SetAudioBlockSize(8); // Small block size for better responsiveness
    
    InitDelays();
    
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    
    while(1) {}
}

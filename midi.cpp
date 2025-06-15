#include "daisy_pod.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyPod hw;

// Constants
const int NUM_SHORT_DELAYS = 4;  // 100ms max
const int NUM_LONG_DELAYS = 3;   // 1s max
const int NUM_DELAYS = NUM_SHORT_DELAYS + NUM_LONG_DELAYS;
const int NUM_PATCHES = 4;
const float MAX_SHORT_DELAY = 0.1f; // 100ms
const float MAX_LONG_DELAY = 1.0f;  // 1s

// Delay lines - separate short and long delays
DelayLine<float, 2000> shortDelays[NUM_SHORT_DELAYS];  // 100ms at 48kHz
DelayLine<float, 36000> longDelays[NUM_LONG_DELAYS];   // 1s at 48kHz

// Modulation
Oscillator lfo[NUM_DELAYS];
float modDepth[NUM_DELAYS];
float modRate[NUM_DELAYS];
float baseDelayTime[NUM_DELAYS];
float feedback[NUM_DELAYS];
float delayLevel[NUM_DELAYS];

// Global parameters
float dryWet;
int currentPatch;

// Helper function to get appropriate delay line reference
template <size_t N>
DelayLine<float, N>& getDelayRef(DelayLine<float, N>* array, int index) {
    return array[index];
}

// Helper function to get max delay time for a delay line
float getMaxDelay(int index) {
    return (index < NUM_SHORT_DELAYS) ? MAX_SHORT_DELAY : MAX_LONG_DELAY;
}

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
    
    // Small knob 1 controls dry/wet
    dryWet = hw.knob1.Process();
    
    // Small knob 2 controls parameters based on patch
    float knob2 = hw.knob2.Process();

    switch(currentPatch)
    {
        case 0: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                lfo[i].SetFreq(modRate[i] * knob2 * 0.5f);
            }
            break;
                
        case 1: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                feedback[i] = knob2 * 2.0f;
            }
            break;

        case 2: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                lfo[i].SetFreq(modRate[i] * knob2 * 2.0f);
            }
            break;

        case 3: 
            for(int i = 0; i < NUM_DELAYS; i++)
            {
                lfo[i].SetFreq(modRate[i] * knob2 * 0.5f);
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
            delayTime = fclamp(delayTime, 0.0f, getMaxDelay(d));
            
            if(d < NUM_SHORT_DELAYS) {
                shortDelays[d].SetDelay(delayTime * hw.AudioSampleRate());
                float delayed = shortDelays[d].Read() * delayLevel[d];
                float input = (d < 2) ? dryL : dryR;
                shortDelays[d].Write(input + delayed * feedback[d]);
                
                if(d < 2) {
                    wetL += delayed;
                    activeDelaysL++;
                } else {
                    wetR += delayed;
                    activeDelaysR++;
                }
            } else {
                int longDelayIdx = d - NUM_SHORT_DELAYS;
                longDelays[longDelayIdx].SetDelay(delayTime * hw.AudioSampleRate());
                float delayed = longDelays[longDelayIdx].Read() * delayLevel[d];
                float input = (d < 6) ? dryL : dryR;
                longDelays[longDelayIdx].Write(input + delayed * feedback[d]);
                
                if(d < 5) {
                    wetL += delayed;
                    activeDelaysL++;}
                if (d == 6) {
                    wetL += 0.5*delayed;
                    wetR += 0.5*delayed;
                    activeDelaysL++;
                }
                else {
                    wetR += delayed;
                    activeDelaysR++;
                }
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
    // Initialize short delays (0-3)
    for(int i = 0; i < NUM_SHORT_DELAYS; i++) {
        shortDelays[i].Init();
        lfo[i].Init(hw.AudioSampleRate());
        lfo[i].SetWaveform(Oscillator::WAVE_SIN);
        delayLevel[i] = 1.0f;
    }
    
    // Initialize long delays (4-6)
    for(int i = 0; i < NUM_LONG_DELAYS; i++) {
        longDelays[i].Init();
        lfo[NUM_SHORT_DELAYS + i].Init(hw.AudioSampleRate());
        lfo[NUM_SHORT_DELAYS + i].SetWaveform(Oscillator::WAVE_SIN);
        delayLevel[NUM_SHORT_DELAYS + i] = 1.0f;
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
        case 0: // Chorus - uses only short delays
            // Left channel delays (0-1)
            baseDelayTime[0] = 0.0236f;
            baseDelayTime[1] = 0.030f;
            baseDelayTime[2] = 0.0f;    // Disabled short delay 2
            baseDelayTime[3] = 0.0f;    // Disabled short delay 3
            
            // Right channel delays (4-5) - using long delays as shorts
            baseDelayTime[4] = 0.0409f;
            baseDelayTime[5] = 0.0482f;
            baseDelayTime[6] = 0.0f;    // Disabled long delay 2
            
            // Modulation settings (maintain your original values)
            modDepth[0] = 0.2f;   modRate[0] = 3.9f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.3f;   modRate[1] = 5.2f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.0f;   modRate[3] = 0.0f; feedback[3] = 0.0f; delayLevel[3] = 0.0f;
            modDepth[4] = 0.2f;   modRate[4] = 4.0f; feedback[4] = 0.0f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.3f;   modRate[5] = 4.9f; feedback[5] = 0.0f; delayLevel[5] = 1.0f;
            modDepth[6] = 0.0f;   modRate[6] = 0.0f; feedback[6] = 0.0f; delayLevel[6] = 0.0f;
            break;

        case 3: // Chorus - uses only short delays
            // Left channel delays (0-1)
            baseDelayTime[0] = 0.0236f;
            baseDelayTime[1] = 0.030f;
            baseDelayTime[2] = 0.0f;    // Disabled short delay 2
            baseDelayTime[3] = 0.0f;    // Disabled short delay 3
            
            // Right channel delays (4-5) - using long delays as shorts
            baseDelayTime[4] = 0.036f;
            baseDelayTime[5] = 0.028f;
            baseDelayTime[6] = 0.0f;    // Disabled long delay 2
            
            // Modulation settings
            modDepth[0] = 0.2f;   modRate[0] = 6.5f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.2f;   modRate[1] = 5.7f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.0f;   modRate[3] = 0.0f; feedback[3] = 0.0f; delayLevel[3] = 0.0f;
            modDepth[4] = 0.2f;   modRate[4] = 4.8f; feedback[4] = 0.0f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.2f;   modRate[5] = 4.4f; feedback[5] = 0.0f; delayLevel[5] = 1.0f;
            modDepth[6] = 0.0f;   modRate[6] = 0.0f; feedback[6] = 0.0f; delayLevel[6] = 0.0f;
            break;
            
        case 1: // Reverb - uses only long delays
            // Left channel delays (4-5)
            baseDelayTime[0] = 0.0f;    // Disabled short delay 0
            baseDelayTime[1] = 0.0f;    // Disabled short delay 1
            baseDelayTime[2] = 0.0f;    // Disabled short delay 2
            baseDelayTime[3] = 0.0f;    // Disabled short delay 3
            
            baseDelayTime[4] = 0.30f;
            baseDelayTime[5] = 0.7f;
            baseDelayTime[6] = 0.47f;    // Disabled long delay 2
            
            // Right channel delays (using some long delays)
            // Modulation settings
            modDepth[0] = 0.0f;   modRate[0] = 0.0f; feedback[0] = 0.0f; delayLevel[0] = 0.0f;
            modDepth[1] = 0.0f;   modRate[1] = 0.0f; feedback[1] = 0.0f; delayLevel[1] = 0.0f;
            modDepth[2] = 0.0f;   modRate[2] = 0.0f; feedback[2] = 0.0f; delayLevel[2] = 0.0f;
            modDepth[3] = 0.0f;   modRate[3] = 0.0f; feedback[3] = 0.0f; delayLevel[3] = 0.0f;
            modDepth[4] = 0.00f;  modRate[4] = 0.0f; feedback[4] = 0.07f; delayLevel[4] = 1.0f;
            modDepth[5] = 0.00f;  modRate[5] = 0.0f; feedback[5] = 0.065f; delayLevel[5] = 1.0f;
            modDepth[6] = 0.0f;   modRate[6] = 0.0f; feedback[6] = 0.05f; delayLevel[6] = 1.0f;
            break;
            
        case 2: // Combined - uses both short and long delays
            // Left channel - short delays (0-1)
            baseDelayTime[0] = 0.0236f;
            baseDelayTime[1] = 0.030f;
            baseDelayTime[2] = 0.0409f;    // Disabled short delay 2
            baseDelayTime[3] = 0.0402f;    // Disabled short delay 3
            
            baseDelayTime[4] = 0.30f;
            baseDelayTime[5] = 0.7f;
            baseDelayTime[6] = 0.47f;    // Disabled long delay 2
            
            // Modulation settings
            modDepth[0] = 0.2f;   modRate[0] = 6.5f; feedback[0] = 0.0f; delayLevel[0] = 1.0f;
            modDepth[1] = 0.2f;   modRate[1] = 5.7f; feedback[1] = 0.0f; delayLevel[1] = 1.0f;
            modDepth[2] = 0.2f;   modRate[2] = 6.0f; feedback[2] = 0.0f; delayLevel[2] = 1.0f;
            modDepth[3] = 0.2f;   modRate[3] = 6.0f; feedback[3] = 0.0f; delayLevel[3] = 1.0f;
            modDepth[4] = 0.0f;   modRate[4] = 0.0f; feedback[4] = 0.1f; delayLevel[4] = 0.2f;
            modDepth[5] = 0.0f;   modRate[5] = 4.4f; feedback[5] = 0.1f; delayLevel[5] = 0.2f;
            modDepth[6] = 0.0f;   modRate[6] = 0.0f; feedback[6] = 0.1f; delayLevel[6] = 0.2f;
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
    hw.SetAudioBlockSize(4);
    
    InitDelays();
    
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    
    while(1) {}
}

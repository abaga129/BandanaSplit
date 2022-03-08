/**
Copyright: Ethan Reker 2017.
License:   MIT
*/
module main;

import std.math;
import std.algorithm;

import dplug.core,
       dplug.client;

import core.stdc.math : tanhf;
import std.math : abs;

import ddsp.filter;


// This define entry points for plugin formats, 
// depending on which version identifiers are defined.
mixin(pluginEntryPoints!BandanaSplitClient);

enum : int
{
    paramInputGain,
    paramCrossoverFreq1,
    paramCrossoverFreq2,
    paramGainBand1,
    paramGainBand2,
    paramGainBand3,
    paramOutputGain
}


/// Example mono/stereo distortion plugin.
final class BandanaSplitClient : dplug.client.Client
{
public:
nothrow:
@nogc:

    this()
    {
        _lp1 = mallocNew!(LinkwitzRileyLPNthOrder!float)(8);
        _lp2 = mallocNew!(LinkwitzRileyLPNthOrder!float)(8);
        _hp1 = mallocNew!(LinkwitzRileyHPNthOrder!float)(8);
        _hp2 = mallocNew!(LinkwitzRileyHPNthOrder!float)(8);
        _ap1 = mallocNew!(AllpassNthOrder!float)(8);
    }

    ~this()
    {
        _lp1.destroyFree();
        _lp2.destroyFree();
        _hp1.destroyFree();
        _hp2.destroyFree();
        _ap1.destroyFree();
    }

    override PluginInfo buildPluginInfo()
    {
        // Plugin info is parsed from plugin.json here at compile time.
        // Indeed it is strongly recommended that you do not fill PluginInfo 
        // manually, else the information could diverge.
        static immutable PluginInfo pluginInfo = parsePluginInfo(import("plugin.json"));
        return pluginInfo;
    }

    // This is an optional overload, default is zero parameter.
    // Caution when adding parameters: always add the indices
    // in the same order as the parameter enum.
    override Parameter[] buildParameters()
    {
        auto params = makeVec!Parameter();
        params ~= mallocNew!LinearFloatParameter(paramInputGain, "input gain", "dB", -12.0f, 12.0f, 0.0f) ;
        params ~= mallocNew!LinearFloatParameter(paramCrossoverFreq1, "Crossover Frequency 1", "Hz", 20.0f, 22_000.0f, 500.0f);
        params ~= mallocNew!LinearFloatParameter(paramCrossoverFreq2, "Crossover Frequency 2", "Hz", 20.0f, 22_000.0f, 5000.0f);
        params ~= mallocNew!LinearFloatParameter(paramGainBand1, "Volume Band 1", "%", 0, 100, 0);
        params ~= mallocNew!LinearFloatParameter(paramGainBand2, "Volume Band 2", "%", 0, 100, 0);
        params ~= mallocNew!LinearFloatParameter(paramGainBand3, "Volume Band 3", "%", 0, 100, 0);
        params ~= mallocNew!LinearFloatParameter(paramOutputGain, "output gain", "db", -12.0f, 12.0f, 0.0f) ;
        return params.releaseData();
    }

    override LegalIO[] buildLegalIO()
    {
        auto io = makeVec!LegalIO();
        io ~= LegalIO(1, 1);
        io ~= LegalIO(2, 2);
        return io.releaseData();
    }

    // This override is optional, the default implementation will
    // have one default preset.
    override Preset[] buildPresets() nothrow @nogc
    {
        auto presets = makeVec!Preset();
        presets ~= makeDefaultPreset();
        return presets.releaseData();
    }

    // This override is also optional. It allows to split audio buffers in order to never
    // exceed some amount of frames at once.
    // This can be useful as a cheap chunking for parameter smoothing.
    // Buffer splitting also allows to allocate statically or on the stack with less worries.
    override int maxFramesInProcess() const //nothrow @nogc
    {
        return 512;
    }

    override void reset(double sampleRate, int maxFrames, int numInputs, int numOutputs) nothrow @nogc
    {
        // Clear here any state and delay buffers you might have.
        _lp1.setSampleRate(sampleRate);
        _lp2.setSampleRate(sampleRate);
        _hp1.setSampleRate(sampleRate);
        _hp2.setSampleRate(sampleRate);
        _ap1.setSampleRate(sampleRate);

        assert(maxFrames <= 512); // guaranteed by audio buffer splitting
    }

    override void processAudio(const(float*)[] inputs, float*[]outputs, int frames,
                               TimeInfo info) nothrow @nogc
    {
        assert(frames <= 512); // guaranteed by audio buffer splitting

        int numInputs = cast(int)inputs.length;
        int numOutputs = cast(int)outputs.length;

        int minChan = numInputs > numOutputs ? numOutputs : numInputs;

        /// Read parameter values
        /// Convert decibel values to floating point
        immutable float inputGain = pow(10, readParam!float(paramInputGain) /20);
        immutable float outputGain = pow(10, readParam!float(paramOutputGain) /20);
        immutable float freq1 = readParam!float(paramCrossoverFreq1);
        immutable float freq2 = readParam!float(paramCrossoverFreq2);
        immutable float gainBand1 = readParam!float(paramGainBand1) / 100.0f;
        immutable float gainBand2 = readParam!float(paramGainBand2) / 100.0f;
        immutable float gainBand3 = readParam!float(paramGainBand3) / 100.0f;

        _lp1.setFrequency(freq1);
        _lp2.setFrequency(freq2);
        _hp1.setFrequency(freq1);
        _hp2.setFrequency(freq2);
        _ap1.setFrequency(freq2);

        for (int f = 0; f < frames; ++f)
        {
            // Make MONO
            float inputSample = 0.0f;
            for (int chan = 0; chan < minChan; ++chan)
            {
                inputSample += inputs[chan][f];
            }
            inputSample = inputSample / minChan * inputGain;

            immutable float band1 = _ap1.getNextSample(_lp1.getNextSample(inputSample));
           
            immutable float highPassFreq1 = _hp1.getNextSample(inputSample);
            immutable float band2 = _lp2.getNextSample(highPassFreq1);
            immutable float band3 = _hp2.getNextSample(highPassFreq1);

            //Process Sample
            float outputSample = (band1 * gainBand1) + (band2 * gainBand2) + (band3 * gainBand3);

            // Assign output
            for (int chan = 0; chan < minChan; ++chan)
            {
                outputs[chan][f] = outputSample * outputGain;
            }
        }


        // fill with zero the remaining channels
        for (int chan = minChan; chan < numOutputs; ++chan)
            outputs[chan][0..frames] = 0; // D has array slices assignments and operations

    }

private:
    LinkwitzRileyLPNthOrder!float _lp1;
    LinkwitzRileyLPNthOrder!float _lp2;
    LinkwitzRileyHPNthOrder!float _hp1;
    LinkwitzRileyHPNthOrder!float _hp2;

    AllpassNthOrder!float _ap1;

    float waveShaper(float x, float amount)
    {
        immutable float k = 2*amount/(1-amount);

        return (1+k)*x/(1+ k * abs(x));
    }
}


#pragma once

#include <FMTranscoder.hpp>
#include <cmath>
#include <iostream>

struct PLLParameters{
    double dampingRatio;
    double naturalFrequency;
    double symbolTime;
    double G1; // Gain of the upper branch of the loop filter
    double G2; // Gain of the lower branch of the loop filter
};

class GMSKTranscoder {
private:
    double internalBufferInPhase[60000]; // TODO: Determine size (signal_length * max_samplesPerSymbol)
    double internalBufferQuadrature[60000];
    double wienerTaps[3] = {-0.0859984, 1.0116342, -0.0859984};
    double delayedTaps[60];    // TODO: Determine size as 6 * samples/symbol, here max samples/symbol considered 10
    bool equalize;
    double convolvedFilters[109]; //TODO: Length is the length of gmsk_mod and delayed taps added (Now 60 + 49)
    uint32_t samplingFrequency;
    uint32_t samplingRate;
    uint32_t samplesPerSymbol;
    uint32_t maxFrequency;
    uint32_t symbolRate;
    uint32_t maxDeviation;
    FMTranscoder fmTranscoder;
    PLLParameters pllParams;

public:
    GMSKTranscoder(uint32_t sampleFrequency, uint32_t symbRate, bool equalize) :
            fmTranscoder(FMTranscoder(sampleFrequency, symbRate / 2, 0,
                                      symbRate / 4, 0, 0)),
            symbolRate(symbRate), samplingFrequency(sampleFrequency) {
        samplesPerSymbol = sampleFrequency / symbolRate;
        maxFrequency = symbolRate / 2;
        maxDeviation = symbolRate / 4;

        // Adding 2-symbol delay between taps
        std::fill(std::begin(delayedTaps), std::begin(delayedTaps) + 2*samplesPerSymbol, wienerTaps[0]);
        std::fill(std::begin(delayedTaps) + 2*samplesPerSymbol, std::begin(delayedTaps) + 4*samplesPerSymbol, wienerTaps[1]);
        std::fill(std::begin(delayedTaps) + 4*samplesPerSymbol, std::begin(delayedTaps) + 6*samplesPerSymbol, wienerTaps[2]);
        this->equalize = equalize;

        double zeta = 0.707;
        double wn = (2.0*10*M_PI*symbolRate)/4800;
        double an = exp(-zeta*wn/samplingFrequency);
        double g1 = 1 - an*an;
        double g2 = 1 + an*an - 2*an* cos(wn*sqrt(1-zeta*zeta)/samplingFrequency);

        pllParams = {zeta,
                                    wn,
                                    1.0/samplingFrequency,
                                    g1*M_PI/2,
                                    g2*M_PI/2
                                    };
    }

    // TODO: signal_length should be a pre-determined number
    void modulate(bool *signal, uint16_t signalLength, double *inPhaseSignal, double *quadratureSignal);

    // Consider changing input IQ signal to const
    void
    demodulate(double *inputInPhaseSignal, double *inputQuadratureSignal, uint16_t signalLength, bool *signal);

};
for clkrate in [64000, 48000, 44000, 32000, 24000, 22000]:
    for adcbits in [12, 10, 8, 6]:
        for sampletime in [1,2,4,7,19,61] + []:
            if clkrate % ((adcbits+1+sampletime) * 125) == 0 and 16 <= clkrate / ((adcbits+1+sampletime) * 125) <= 40:
                print (clkrate, adcbits, sampletime, clkrate / ((adcbits+1+sampletime) * 125))

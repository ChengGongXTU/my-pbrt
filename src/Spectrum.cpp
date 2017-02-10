#include"Spectrum.h"

float AverageSpectrumSamples(const float *lambda, const float *vals,
	int n, float lambdaStart, float lambdaEnd) {
	
	//if sample is out of range,ro is single in th range.
	if (lambdaEnd <= lambda[0]) return vals[0];
	if (lambdaStart >= lambda[n - 1]) return vals[n - 1];
	if (n == 1) return vals[0];
	float sum = 0.f;
	
	if (lambdaStart < lambda[0])
		sum += vals[0] * (lambda[0] - lambdaStart);
	if (lambdaEnd > lambda[n - 1])
		sum += vals[n - 1] * (lambdaEnd - lambda[n - 1]);

	int i = 0;
	while (lambdaStart > lambda[i + 1]) ++i;

	#define INTERP(w, i) \
		Lerp(((w) - lambda[i]) / (lambda[(i)+1] - lambda[i]), \
			vals[i], vals[(i)+1])
	#define SEG_AVG(wl0, wl1, i) (0.5f * (INTERP(wl0, i) + INTERP(wl1, i)))
	for (; i + 1 < n && lambdaEnd >= lambda[i]; ++i) {
		float segStart = max(lambdaStart, lambda[i]);
		float segEnd = min(lambdaEnd, lambda[i + 1]);
		sum += SEG_AVG(segStart, segEnd, i) * (segEnd - segStart);
	}
	#undef INTERP
	#undef SEG_AVG

	return sum / (lambdaEnd - lambdaStart);
}
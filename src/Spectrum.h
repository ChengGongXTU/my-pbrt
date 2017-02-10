#include"pbrt.h"


template <int nSamples> class CoefficientSpectrum {
public:
	//  initializes
	CoefficientSpectrum(float v = 0.f) {
		for (i = 0; i < nSamples; ++i)
			c[i] = v;
	}

	// add
	CoefficientSpectrum &operator+=(const CoefficientSpectrum &s2) {
		for (i = 0; i < nSamples; ++i)
			c[i] += s2[i];
		return *this;
	}

	CoefficientSpectrum operator+(const CoefficientSpectrum &s2) const {
		CoefficientSpectrum ret = *this;
		for (int i = 0; i < nSamples; ++i)
			ret.c[i] += s2.c[i];
		return ret;
	}

	// check if these samples have 0 value for SPD.
	bool IsBlack()const {
		for (int i = 0; i < nSamples; ++i)
			if (c[i] != 0.) return false;
		return true;
	}
	
	//  compute square root
	friend CoefficientSpectrum Sqrt(const CoefficientSpectrum &s) {
		CoefficientSpectrum ret;
		for (int i = 0; i < nSamples; ++i)
			ret.c[i] = sqrtf(s.c[i]);
		return ret;
	}

	// check debug
	bool HasNaNs() const {
		for (int i = 0; i < nSamples; ++i)
			if (isnan(c[i])) return true;
		return false;
	}



protected:
	float c[nSample];

};


//  start and end of wavelength, and number of sample
static const int sampledLambdaStart = 400;
static const int sampledLambdaEnd = 700;
static const int nSpectralSamples = 30;

static const int nCIESamples = 471;
extern const float CIE_X[nCIESamples];
extern const float CIE_Y[nCIESamples];
extern const float CIE_Z[nCIESamples];
extern const float CIE_lambda[nCIESamples];


class SampledSpectrum 
	: public CoefficientSpectrum<nSpectralSamples> {
public:
	// initialize
	SampledSpectrum(float v = 0.f) {
		for (int i = 0; i < nSpectralSamples; ++i)
			c[i] = v;
	}

	static SampledSpectrum FromSampled(const float *lambda,const float *v, int n)
	{	
		//  sort the  sample in the range
		if (!SpectrumSamplesSorted(lambda, v, n)) 
		{	vector<float> slambda(&lambda[0], &lambda[n]);
			vector<float> sv(&v[0], &v[n]);
			SortSpectrumSamples(&slambda[0], &sv[0], n);
			return FromSampled(&slambda[0], &sv[0], n);
		}
		SampledSpectrum r;
		// compute the average of SPD in this range
		for (int i = 0; i < nSpectralSamples; ++i) 
		{	float lambda0 = Lerp(float(i) / float(nSpectralSamples),sampledLambdaStart, sampledLambdaEnd);
			float lambda1 = Lerp(float(i + 1) / float(nSpectralSamples),sampledLambdaStart, sampledLambdaEnd);
			r.c[i] = AverageSpectrumSamples(lambda, v, n, lambda0, lambda1);
			
		}
		return r;
	}

	// method for Init() function
	void ToXYZ(float xyz[3]) const {
		xyz[0] = xyz[1] = xyz[2] = 0.f;
		for (int i = 0; i < nSpectralSamples; ++i) {
			xyz[0] += X.c[i] * c[i];
			xyz[1] += Y.c[i] * c[i];
			xyz[2] += Z.c[i] * c[i];
		}
		xyz[0] /= yint;
		xyz[1] /= yint;
		xyz[2] /= yint;
	}

	float y() const {
		float yy = 0.f;
		for (int i = 0; i < nSpectralSamples; ++i)
			yy += Y.c[i] * c[i];
		return yy / yint;
	}


	static void Init() {
		// get the xyz matching function base on spectrumsample.
		for (int i = 0; i < nSpectralSamples; ++i) {
			float wl0 = Lerp(float(i) / float(nSpectralSamples),
				sampledLambdaStart, sampledLambdaEnd);
			float wl1 = Lerp(float(i + 1) / float(nSpectralSamples),
				sampledLambdaStart, sampledLambdaEnd);
			X.c[i] = AverageSpectrumSamples(CIE_lambda, CIE_X, nCIESamples,
				wl0, wl1);
			Y.c[i] = AverageSpectrumSamples(CIE_lambda, CIE_Y, nCIESamples,
				wl0, wl1);
			Z.c[i] = AverageSpectrumSamples(CIE_lambda, CIE_Z, nCIESamples,
				wl0, wl1);
			yint += Y.c[i];
		}

		//

	}

	void ToRGB(float rgb[3]) const {
		float xyz[3];
		ToXYZ(xyz);
		XYZToRGB(xyz, rgb);
	}

	static SampledSpectrum FromXYZ(const float xyz[3],
		SpectrumType type = SPECTRUM_REFLECTANCE) {
		float rgb[3];
		XYZToRGB(xyz, rgb);
		return FromRGB(rgb, type);
	}RGBSpectrum ToRGBSpectrum() const;



private:
	static SampledSpectrum X, Y, Z;
	static float yint;

	static SampledSpectrum rgbRefl2SpectWhite, rgbRefl2SpectCyan;
	static SampledSpectrum rgbRefl2SpectMagenta, rgbRefl2SpectYellow;
	static SampledSpectrum rgbRefl2SpectRed, rgbRefl2SpectGreen;
	static SampledSpectrum rgbRefl2SpectBlue;

	static SampledSpectrum rgbIllum2SpectWhite, rgbIllum2SpectCyan;
	static SampledSpectrum rgbIllum2SpectMagenta, rgbIllum2SpectYellow;
	static SampledSpectrum rgbIllum2SpectRed, rgbIllum2SpectGreen;
	static SampledSpectrum rgbIllum2SpectBlue;
};

static const int nRGB2SpectSamples = 32;
extern const float RGB2SpectLambda[nRGB2SpectSamples];
extern const float RGBRefl2SpectWhite[nRGB2SpectSamples];
extern const float RGBRefl2SpectCyan[nRGB2SpectSamples];
extern const float RGBRefl2SpectMagenta[nRGB2SpectSamples];
extern const float RGBRefl2SpectYellow[nRGB2SpectSamples];
extern const float RGBRefl2SpectRed[nRGB2SpectSamples];
extern const float RGBRefl2SpectGreen[nRGB2SpectSamples];
extern const float RGBRefl2SpectBlue[nRGB2SpectSamples];

extern const float RGBIllum2SpectWhite[nRGB2SpectSamples];
extern const float RGBIllum2SpectCyan[nRGB2SpectSamples];
extern const float RGBIllum2SpectMagenta[nRGB2SpectSamples];

extern const float RGBIllum2SpectYellow[nRGB2SpectSamples];
extern const float RGBIllum2SpectRed[nRGB2SpectSamples];
extern const float RGBIllum2SpectGreen[nRGB2SpectSamples];
extern const float RGBIllum2SpectBlue[nRGB2SpectSamples];
//-----------------------------inline function------------------------
// interpolate for spectrum
inline Spectrum Lerp(float t, const Spectrum &s1, const Spectrum &s2) {
	return (1.f - t) * s1 + t * s2;
}

// transform matrix for xyz and rgb system.
inline void XYZToRGB(const float xyz[3], float rgb[3]) {
	rgb[0] = 3.240479f*xyz[0] - 1.537150f*xyz[1] - 0.498535f*xyz[2];
	rgb[1] = -0.969256f*xyz[0] + 1.875991f*xyz[1] + 0.041556f*xyz[2];
	rgb[2] = 0.055648f*xyz[0] - 0.204043f*xyz[1] + 1.057311f*xyz[2];
}

inline void RGBToXYZ(const float rgb[3], float xyz[3]) {
	xyz[0] = 0.412453f*rgb[0] + 0.357580f*rgb[1] + 0.180423f*rgb[2];
	xyz[1] = 0.212671f*rgb[0] + 0.715160f*rgb[1] + 0.072169f*rgb[2];
	xyz[2] = 0.019334f*rgb[0] + 0.119193f*rgb[1] + 0.950227f*rgb[2];
}

enum SpectrumType { SPECTRUM_REFLECTANCE, SPECTRUM_ILLUMINANT };

float InterpolateSpectrumSamples(const float *lambda, const float *vals,
	int n, float l) {
	if (l <= lambda[0]) return vals[0];
	if (l >= lambda[n - 1]) return vals[n - 1];
	for (int i = 0; i < n - 1; ++i) {
		if (l >= lambda[i] && l <= lambda[i + 1]) {
			float t = (l - lambda[i]) / (lambda[i + 1] - lambda[i]);
			return Lerp(t, vals[i], vals[i + 1]);
		}
	}
}




class RGBSpectrum :public CoefficientSpectrum<3> {
public:
	RGBSpectrum(float v =0.f)
		:CoefficientSpectrum<3>(v){}

	RGBSpectrum(const CoefficientSpectrum<3> &v)
		:CoefficientSpectrum<3>(v){}

	// method : convert array or rgb's c[i] to RGB
	static RGBSpectrum FromRGB(const float rgb[3],
		SpectrumType type = SPECTRUM_REFLECTANCE) {
		RGBSpectrum s;
		s.c[0] = rgb[0];
		s.c[1] = rgb[1];
		s.c[2] = rgb[2];
		return s;
	}

	void ToRGB(float *rgb) const {
		rgb[0] = c[0];
		rgb[1] = c[1];
		rgb[2] = c[2];
	}

	const RGBSpectrum &ToRGBSpectrum() const {
		return *this;
	}

	//convert spectrum to rgb
	static RGBSpectrum FromSampled(const float *lambda, const float *v,int n) {
		
		float xyz[3] = { 0, 0, 0 };
		float yint = 0.f;
		for (int i = 0; i < nCIESamples; ++i) {
			yint += CIE_Y[i];
			float val = InterpolateSpectrumSamples(lambda, v, n,CIE_lambda[i]);
			xyz[0] += val * CIE_X[i];
			xyz[1] += val * CIE_Y[i];
			xyz[2] += val * CIE_Z[i];
		}
		xyz[0] /= yint;
		xyz[1] /= yint;
		xyz[2] /= yint;
		return FromXYZ(xyz);
	}

};


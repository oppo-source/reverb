/* libSoX effect: stereo reverberation
 * Copyright (c) 2007 robs@users.sourceforge.net
 * Filter design based on freeverb by Jezar at Dreampoint.
 * Copyright (C) 2020 Oplus. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <cstdlib>
#include <algorithm>
#include <math.h>
#include "reverb.h"
/*commom include*/
#include "config.h"
#include "memory_req_free.h"


 
using std::min;
using std::max;
#ifndef M_LN10
#define M_LN10   2.30258509299404568402 /* log_e 10 */
#endif // M_LN10

#ifndef M_PI
#define M_PI     3.1415926535897932384626433832795
#endif // M_PI

#define array_length(a) (sizeof(a)/sizeof(a[0]))
#define dB_to_linear(x) exp((x) * M_LN10 * 0.05)
#define midi_to_freq(n) (440 * pow(2,((n)-69)/12.))
#define FIFO_SIZE_T int
#define FIFO_MIN (16384)
#define fifo_read_ptr(f) fifo_read(f, (FIFO_SIZE_T)0, NULL)
#define lsx_zalloc(var, n) var = (float *)calloc(n, sizeof(*var))
#define filter_advance(p) if (--(p)->ptr < (p)->buffer) (p)->ptr += (p)->size
#define filter_delete(p) memFree((p)->buffer)

static const int /* Filter delay lengths in samples (44100Hz sample-rate) */
    comb_lengths[] = { 1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617 },
    allpass_lengths[] = { 225, 341, 441, 556 };

typedef struct { double b0, b1, a1, i1, o1; } one_pole_t;

typedef struct {
    int  size;
    float   * buffer, *ptr;
    float   store;
} filter_t;

typedef struct {
    char * data;
    int allocation;   /* Number of bytes allocated for data. */
    int item_size;    /* Size of each item in data */
    int begin;        /* Offset of the first byte to read. */
    int end;          /* 1 + Offset of the last byte byte to read. */
} fifo_t;

typedef struct {
    filter_t comb[array_length(comb_lengths)];
    filter_t allpass[array_length(allpass_lengths)];
    one_pole_t one_pole[2];
} filter_array_t;

typedef struct {
    float feedback;
    float hf_damping;
    float gain;
    fifo_t input_fifo;
    filter_array_t chan[2];
    float * out[2];
} reverb_t;

typedef struct {
	reverb_t reverb;
    float *dry = NULL;
    float *wet[2];
} Reverb_priv_t;

typedef struct {
    double mRoomSize;
    double mPreDelay;
    double mReverberance;
    double mHfDamping;
    double mToneLow;
    double mToneHigh;
    double mWetGain;
    double mDryGain;
    double mStereoWidth;
    bool mWetOnly;
} Params;

static Reverb_priv_t *mP1 = NULL;
static Reverb_priv_t *mP2 = NULL;
static Reverb_priv_t *mP3 = NULL;
static Reverb_priv_t *mP4 = NULL;

/*                                  Room  Pre            Hf       Tone Tone  Wet   Dry   Stereo Wet
                      Name          Size, Delay, Reverb, Damping, Low, High, Gain, Gain, Width, Only*/
static const Params ParamsMode1 = { 50,   0,     65,    100,      10,  80,   -1,   0,    100,   false };
static const Params ParamsMode2 = { 88,   56,    70,    86,       20,  30,   -1,   0,    100,   false };
static const Params ParamsMode3 = { 100,  13,    55,    50,       0,   100,  -1,   0,    80,    false };
static const Params ParamsMode4 = { 100,  58,    21,    100,      84,  10,   -1,   0,    0,     false };

/* Filter delay lengths in samples (44100Hz sample-rate) */
static const int stereo_adjust = 12;


/***************************************************************************************************
**                                             fifo
***************************************************************************************************/
static void fifo_clear(fifo_t * f)
{
   f->end = f->begin = 0;
}

static void * fifo_reserve(fifo_t * f, FIFO_SIZE_T n)
{
   n *= f->item_size;

   if (f->begin == f->end)
      fifo_clear(f);

   while (1) {
      if (f->end + n <= f->allocation) {          
         void *p = f->data + f->end;
         f->end += n;
         return p;
      }
      if (f->begin > FIFO_MIN) {
         memmove(f->data, f->data + f->begin, f->end - f->begin);
         f->end -= f->begin;
         f->begin = 0;
         continue;
      }
      f->allocation += n;
      f->data = (char *)realloc(f->data, f->allocation);
      if (f->data == NULL) {
          oplusAudioLogPrint("ktv2.0: error!!! memory request fail in file %s on line %d\n", __FILE__, __LINE__, \
              0, 0, 0, 0, 0, 0, 0, 0);
      }
   }
}

static void * fifo_write(fifo_t * f, FIFO_SIZE_T n, void const * data)
{
   void * s = fifo_reserve(f, n);
   if (data)
      memcpy(s, data, n * f->item_size);
   return s;
}

static void * fifo_read(fifo_t * f, FIFO_SIZE_T n, void * data)
{
   char * ret = f->data + f->begin;
   n *= f->item_size;
   if (n > (FIFO_SIZE_T)(f->end - f->begin))
      return NULL;
   if (data)
      memcpy(data, ret, (int)n);
   f->begin += n;
   return ret;
}

static void fifo_delete(fifo_t * f)
{
   memFree(f->data);
}

static int fifo_create(fifo_t * f, FIFO_SIZE_T item_size)
{
    int result = SUCCESS;
    f->item_size = item_size;
    f->allocation = FIFO_MIN;
    memLeakConfirm(f->data);
    f->data = (char *)calloc(1, f->allocation);
    memReqConfirm(f->data);
    if (f->data == NULL) {
        return MEM_REQ_FAIL;
    }
    fifo_clear(f);
    return result;
}

/***************************************************************************************************
**                                           filter define
***************************************************************************************************/
static float comb_process(filter_t * p,  /* gcc -O2 will inline this */
      float const * input, float const * feedback, float const * hf_damping)
{
   float output = *p->ptr;
   p->store = output + (p->store - output) * *hf_damping;
   *p->ptr = *input + p->store * *feedback;
   filter_advance(p);
   return output;
}

static float allpass_process(filter_t * p,  /* gcc -O2 will inline this */
      float const * input)
{
   float output = *p->ptr;
   *p->ptr = *input + output * .5;
   filter_advance(p);
   return output - *input;
}

static float one_pole_process(one_pole_t * p, float i0)
{
   float o0 = i0*p->b0 + p->i1*p->b1 - p->o1*p->a1;
   p->i1 = i0;
   return p->o1 = o0;
}

static int filter_array_create(filter_array_t * p, double rate, double scale, double offset,
    double fc_highpass, double fc_lowpass) {
    int result = SUCCESS;
    unsigned int i;
    double r = rate * (1 / 44100.); /* Compensate for actual sample-rate */

    for (i = 0; i < array_length(comb_lengths); ++i, offset = -offset)
    {
        filter_t * pcomb = &p->comb[i];
        pcomb->size = (int)(scale * r * (comb_lengths[i] + stereo_adjust * offset) + .5);
        memLeakConfirm(pcomb->buffer);
        pcomb->ptr = lsx_zalloc(pcomb->buffer, pcomb->size);
        memReqConfirm(pcomb->buffer);
        if (pcomb->buffer == NULL) {
            return MEM_REQ_FAIL;
        }
    }
    for (i = 0; i < array_length(allpass_lengths); ++i, offset = -offset)
    {
        filter_t * pallpass = &p->allpass[i];
        pallpass->size = (int)(r * (allpass_lengths[i] + stereo_adjust * offset) + .5);
        memLeakConfirm(pallpass->buffer);
        pallpass->ptr = lsx_zalloc(pallpass->buffer, pallpass->size);
        memReqConfirm(pallpass->ptr);
        if (pallpass->ptr == NULL) {
            return MEM_REQ_FAIL;
        }
    }
    { /* EQ: highpass */
        one_pole_t * q = &p->one_pole[0];
        q->a1 = -exp(-2 * M_PI * fc_highpass / rate);
        q->b0 = (1 - q->a1)/2, q->b1 = -q->b0;
    }
    { /* EQ: lowpass */
        one_pole_t * q = &p->one_pole[1];
        q->a1 = -exp(-2 * M_PI * fc_lowpass / rate);
        q->b0 = 1 + q->a1, q->b1 = 0;
    }
    return result;
}

static void filter_array_process(filter_array_t * p,
      int length, float const * input, float * output,
      float const * feedback, float const * hf_damping, float const * gain)
{
   while (length--) {
      float out = 0, in = *input++;

      int i = array_length(comb_lengths) - 1;
      do out += comb_process(p->comb + i, &in, feedback, hf_damping);
      while (i--);

      i = array_length(allpass_lengths) - 1;
      do out = allpass_process(p->allpass + i, &out);
      while (i--);

      out = one_pole_process(&p->one_pole[0], out);
      out = one_pole_process(&p->one_pole[1], out);
      *output++ = out * *gain;
   }
}

static void filter_array_delete(filter_array_t * p)
{
   unsigned int i;
   for (i = 0; i < array_length(allpass_lengths); ++i) {
       filter_delete(&p->allpass[i]);
   }
   for (i = 0; i < array_length(comb_lengths); ++i) {
       filter_delete(&p->comb[i]);
   }      
}


/***************************************************************************************************
**                                  reverb create, process and delete
***************************************************************************************************/
static int reverb_create(reverb_t * p, double sample_rate_Hz,
      double wet_gain_dB,
      double room_scale,     /* % */
      double reverberance,   /* % */
      double hf_damping,     /* % */
      double pre_delay_ms,
      double stereo_depth,
      double tone_low,       /* % */
      double tone_high,      /* % */
      int buffer_size,
      float * * out)
{
    int result = SUCCESS;
    int i, delay = pre_delay_ms / 1000 * sample_rate_Hz + .5;
    double scale = room_scale / 100 * .9 + .1;
    double depth = stereo_depth / 100;
    double a =  -1 /  log(1 - /**/.3 /**/);           /* Set minimum feedback */
    double b = 100 / (log(1 - /**/.98/**/) * a + 1);  /* Set maximum feedback */
    double fc_highpass = midi_to_freq(72 - tone_low / 100 * 48);
    double fc_lowpass  = midi_to_freq(72 + tone_high/ 100 * 48); 

    memset(p, 0, sizeof(*p));
    p->feedback = 1 - exp((reverberance - b) / (a * b));
    p->hf_damping = hf_damping / 100 * .3 + .2;
    p->gain = dB_to_linear(wet_gain_dB) * .015;
    result = fifo_create(&p->input_fifo, sizeof(float));
    if (result != SUCCESS) {
        return result;
    }
    memset(fifo_write(&p->input_fifo, delay, 0), 0, delay * sizeof(float));
    for (i = 0; i <= ceil(depth); ++i) {
        result = filter_array_create(p->chan + i, sample_rate_Hz, scale, i * depth, fc_highpass, fc_lowpass);
        if (result != SUCCESS) {
            return result;
        }
        memLeakConfirm(p->out[i]);
        out[i] = lsx_zalloc(p->out[i], buffer_size);
        memReqConfirm(p->out[i]);
        if (p->out[i] == NULL) {
            return MEM_REQ_FAIL;
        }
    }
    return result;
}

static void reverb_process(reverb_t * p, int length)
{
    int i;
    for (i = 0; i < 2 && p->out[i]; ++i) {
        filter_array_process(p->chan + i, length, (float *)fifo_read_ptr(&p->input_fifo), p->out[i], &p->feedback, &p->hf_damping, &p->gain);
    }
    fifo_read(&p->input_fifo, length, NULL);
}

static void reverb_delete(reverb_t * p)
{
   int i;
   for (i = 0; i < 2 && p->out[i]; ++i) {
      memFree(p->out[i]);
      filter_array_delete(p->chan + i);
   }
   fifo_delete(&p->input_fifo);
}



int reverb_init()
{
    int result = SUCCESS;
    int Chans = 1;
    int SampleRate = 48000;
    bool isStereo = false;

    /*************************************** init mP1 *******************************************/
    memLeakConfirm(mP1);
    mP1 = (Reverb_priv_t *)calloc(Chans, sizeof(*mP1));
    memReqConfirm(mP1);
    if (mP1 == NULL) {
        return MEM_REQ_FAIL;
    }
    result = reverb_create(&mP1[0].reverb,
        SampleRate,
        ParamsMode1.mWetGain,
        ParamsMode1.mRoomSize,
        ParamsMode1.mReverberance,
        ParamsMode1.mHfDamping,
        ParamsMode1.mPreDelay,
        ParamsMode1.mStereoWidth * (isStereo ? 1 : 0),
        ParamsMode1.mToneLow,
        ParamsMode1.mToneHigh,
        1024, // BLOCK
        mP1[0].wet);

    /*************************************** init mP2 *******************************************/
    memLeakConfirm(mP2);
    mP2 = (Reverb_priv_t *)calloc(Chans, sizeof(*mP2));
    memReqConfirm(mP2);
    if (mP2 == NULL) {
        return MEM_REQ_FAIL;
    }
    result = reverb_create(&mP2[0].reverb,
        SampleRate,
        ParamsMode2.mWetGain,
        ParamsMode2.mRoomSize,
        ParamsMode2.mReverberance,
        ParamsMode2.mHfDamping,
        ParamsMode2.mPreDelay,
        ParamsMode2.mStereoWidth * (isStereo ? 1 : 0),
        ParamsMode2.mToneLow,
        ParamsMode2.mToneHigh,
        1024, // BLOCK
        mP2[0].wet);

    /*************************************** init mP3 *******************************************/
    memLeakConfirm(mP3);
    mP3 = (Reverb_priv_t *)calloc(Chans, sizeof(*mP3));
    memReqConfirm(mP3);
    if (mP3 == NULL) {
        return MEM_REQ_FAIL;
    }
    result = reverb_create(&mP3[0].reverb,
        SampleRate,
        ParamsMode3.mWetGain,
        ParamsMode3.mRoomSize,
        ParamsMode3.mReverberance,
        ParamsMode3.mHfDamping,
        ParamsMode3.mPreDelay,
        ParamsMode3.mStereoWidth * (isStereo ? 1 : 0),
        ParamsMode3.mToneLow,
        ParamsMode3.mToneHigh,
        1024, // BLOCK
        mP3[0].wet);

    /*************************************** init mP4 *******************************************/
    memLeakConfirm(mP4);
    mP4 = (Reverb_priv_t *)calloc(Chans, sizeof(*mP4));
    memReqConfirm(mP4);
    if (mP4 == NULL) {
        return MEM_REQ_FAIL;
    }
    result = reverb_create(&mP4[0].reverb,
        SampleRate,
        ParamsMode4.mWetGain,
        ParamsMode4.mRoomSize,
        ParamsMode4.mReverberance,
        ParamsMode4.mHfDamping,
        ParamsMode4.mPreDelay,
        ParamsMode4.mStereoWidth * (isStereo ? 1 : 0),
        ParamsMode4.mToneLow,
        ParamsMode4.mToneHigh,
        1024, // BLOCK
        mP4[0].wet);

    return result;
}


int reverb_exit()
{
	reverb_delete(&mP1[0].reverb);
    memFree(mP1);
    mP1 = NULL;

    reverb_delete(&mP2[0].reverb);
    memFree(mP2);
    mP2 = NULL;

    reverb_delete(&mP3[0].reverb);
    memFree(mP3);
    mP3 = NULL;

    reverb_delete(&mP4[0].reverb);
    memFree(mP4);
    mP4 = NULL;

	return SUCCESS;
}

void reverb_proc(float *ichans, unsigned int inlen, float *ochans, unsigned int outlen, int *params)
{
    float  dryMult = 1.0;
    static Reverb_priv_t *mP_tmp = NULL;
    if (params[0] == 0)
    {
        switch (params[1]) {
            case 0:
                memcpy(ochans, ichans, sizeof(float) * outlen);
                return;
            case 1:
                mP_tmp = mP1;
                dryMult = ParamsMode1.mWetOnly ? 0 : dB_to_linear(ParamsMode1.mDryGain);
                break;
            case 2:
                mP_tmp = mP2;
                dryMult = ParamsMode2.mWetOnly ? 0 : dB_to_linear(ParamsMode2.mDryGain);
                break;
            case 3:
                mP_tmp = mP3;
                dryMult = ParamsMode3.mWetOnly ? 0 : dB_to_linear(ParamsMode3.mDryGain);
                break;
            case 4:
                mP_tmp = mP4;
                dryMult = ParamsMode4.mWetOnly ? 0 : dB_to_linear(ParamsMode4.mDryGain);
                break;
            default:
                memcpy(ochans, ichans, sizeof(float) * outlen);
                 return;
        }
    } else {
        memcpy(ochans, ichans, sizeof(float) * outlen);
        return;
    }

   mP_tmp[0].dry = (float *) fifo_write(&mP_tmp[0].reverb.input_fifo, inlen, ichans);
   reverb_process(&mP_tmp[0].reverb, inlen);
   for (unsigned i = 0; i < inlen; i++)
   {
	   ochans[i] = dryMult * mP_tmp[0].dry[i] + mP_tmp[0].wet[0][i];
   }
}





/*
*
* Finite Impulse Response (FIR) Filter
*
* Implements a discrete-time FIR filter using a set of coefficients and a
 * circular buffer.
* Transfer function in z-domain is as follows:
 * G(z) = c_0 + c_1 * z^-1 + ... + c_(N-1) * z^(N-1)
* Where c_n is the n-th coefficient and N is the filter order.
*
* Written by: Philip M. Salmony @ philsal.co.uk
* Last changed: 01 Dec 2019
* Source:
 * https://github.com/pms67/HadesFCS/blob/master/Filtering/C%20Code/FIR.h
* BSD 3-Clause License
Copyright (c) 2019, Philip Salmony, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdint.h>

class FirFilter{
public:
    FirFilter(std::vector<float> coefficients)
    {
        filterOrder_ = coefficients.size();
        buffer_ = new float[filterOrder_];
        coefficients_ = new float[filterOrder_];
        for (uint8_t idx = 0; idx < filterOrder_; idx++) {
            buffer_[idx] = 0.0f;
            coefficients_[idx] = coefficients[idx];
            //std::cout<<"coeff is "<<coefficients_[idx]<<std::endl;
        }
        //
        output_ = 0.0f;
        putIndex_ = 0;
    };

    ~FirFilter(){
        delete buffer_;
        delete coefficients_;
    };

    float UpdateFilterAndGetOutput(const float& in)
    {
        UpdateFilter(in);
        return output_;
    };

    inline float getOutput() const {
        return output_;
    };

private:
    float output_{};
    uint8_t filterOrder_{};
    float* buffer_;
    float* coefficients_;
    uint8_t putIndex_{};

    void UpdateFilter(const float& in) {
        /* Store newest input value in circular buffer */
        buffer_[putIndex_] = in;

        /* Compute filter output */
        uint8_t getIndex = putIndex_;
        output_ = 0.0f;
        for (uint8_t idx = 0; idx < filterOrder_; idx++) {
            output_ += coefficients_[idx] * buffer_[getIndex];
            if (getIndex == 0) {
                getIndex = filterOrder_ - 1;
            } else {
                getIndex--;
            }
        }

        /* Increment buffer index */
        putIndex_++;
        if (putIndex_ == filterOrder_) {
            putIndex_ = 0;
        }
    };
};
#endif
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_mean_q7.c
 * Description:  Mean value of a Q7 vector
 *
 * $Date:        23 April 2021
 * $Revision:    V1.9.0
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "dsp/statistics_functions.h"

/**
  @ingroup groupStats
 */

/**
  @addtogroup mean
  @{
 */

/**
  @brief         Mean value of a Q7 vector.
  @param[in]     pSrc       points to the input vector
  @param[in]     blockSize  number of samples in input vector
  @param[out]    pResult    mean value returned here

  @par           Scaling and Overflow Behavior
                   The function is implemented using a 32-bit internal accumulator.
                   The input is represented in 1.7 format and is accumulated in a 32-bit
                   accumulator in 25.7 format.
                   There is no risk of internal overflow with this approach, and the
                   full precision of intermediate result is preserved.
                   Finally, the accumulator is truncated to yield a result of 1.7 format.
 */

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

void arm_mean_q7(
  const q7_t * pSrc,
        uint32_t blockSize,
        q7_t * pResult)
{
    uint32_t  blkCnt;           /* loop counters */
    q7x16_t vecSrc;
    q31_t     sum = 0L;


    blkCnt = blockSize >> 4;
    while (blkCnt > 0U)
    {
        vecSrc = vldrbq_s8(pSrc);
        /*
         * sum lanes
         */
        sum = vaddvaq(sum, vecSrc);

        blkCnt--;
        pSrc += 16;
    }

    blkCnt = blockSize & 0xF;
    while (blkCnt > 0U)
    {
      /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
      sum += *pSrc++;
  
      /* Decrement loop counter */
      blkCnt--;
    }

    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
    /* Store the result to the destination */
    *pResult = (q7_t) (sum / (int32_t) blockSize);
}
#else
void arm_mean_q7(
  const q7_t * pSrc,
        uint32_t blockSize,
        q7_t * pResult)
{
        uint32_t blkCnt;                               /* Loop counter */
        q31_t sum = 0;                                 /* Temporary result storage */

#if defined (ARM_MATH_LOOPUNROLL)
        q31_t in;
#endif

#if defined (ARM_MATH_LOOPUNROLL)

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    in = read_q7x4_ia (&pSrc);
    sum += ((in << 24U) >> 24U);
    sum += ((in << 16U) >> 24U);
    sum += ((in <<  8U) >> 24U);
    sum +=  (in >> 24U);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = blockSize % 0x4U;

#else

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

  while (blkCnt > 0U)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *pSrc++;

    /* Decrement loop counter */
    blkCnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
  /* Store result to destination */
  *pResult = (q7_t) (sum / (int32_t) blockSize);
}
#endif /* defined(ARM_MATH_MVEI) */

/**
  @} end of mean group
 */

/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS] = {
  24,   //   1xn
  24,   //   2xn
  24,   //   4xn
  14,   //   8xn
  2,    //  16xn
  0,    //  32xn
  0,    //  64xn
  0     // 128xn
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction() : m_currChromaFormat(NUM_CHROMA_FORMAT)
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  m_piTemp    = nullptr;
  m_pMdlmTemp = nullptr;
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

void IntraPrediction::destroy()
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      delete[] m_yuvExt2[ch][buf];
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
  delete[] m_pMdlmTemp;
  m_pMdlmTemp = nullptr;
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  if (m_yuvExt2[COMPONENT_Y][0] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_yuvExt2[COMPONENT_Y][0] == nullptr)   // check if first is null (in which case, nothing initialised yet)
  {
    m_yuvExtSize2 = (MAX_CU_SIZE) * (MAX_CU_SIZE);

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < 4; buf++)
      {
        m_yuvExt2[ch][buf] = new Pel[m_yuvExtSize2];
      }
    }
  }

  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
  if (m_pMdlmTemp == nullptr)
  {
    m_pMdlmTemp =
      new Pel[(2 * MAX_CU_SIZE + 1) * (2 * MAX_CU_SIZE + 1)];   // MDLM will use top-above and left-below samples.
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
// NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc(const CPelBuf &pSrc, const Size &dstSize)
{
  CHECK(dstSize.width == 0 || dstSize.height == 0, "Empty area provided");

  int        idx, sum = 0;
  Pel        dcVal;
  const int  width     = dstSize.width;
  const int  height    = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width, height);
  const auto divShift  = floorLog2(denom);
  const auto divOffset = (denom >> 1);

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(m_ipaParam.multiRefIndex + 1 + idx, 0);
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(m_ipaParam.multiRefIndex + 1 + idx, 1);
    }
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

int IntraPrediction::getModifiedWideAngle(int width, int height, int predMode)
{
  // The function returns a 'modified' wide angle index, given that it is not necessary
  // in this software implementation to reserve the values 0 and 1 for Planar and DC to generate the prediction signal.
  // It should only be used to obtain the intraPredAngle parameter.
  // To simply obtain the wide angle index, the function PU::getWideAngle should be used instead.
  if (predMode > DC_IDX && predMode <= VDIA_IDX)
  {
    int modeShift[] = { 0, 6, 10, 12, 14, 15 };
    int deltaSize   = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += (VDIA_IDX - 1);
    }
    else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= (VDIA_IDX - 1);
    }
  }
  return predMode;
}

void IntraPrediction::setReferenceArrayLengths(const CompArea &area)
{
  // set Top and Left reference samples length
  const int width  = area.width;
  const int height = area.height;

  m_leftRefLength = (height << 1);
  m_topRefLength  = (width << 1);
}

void IntraPrediction::predIntraAngLIP(const ComponentID compId, PelBuf &piPred, PredictionUnit &pu)
{
  const ComponentID compID      = MAP_CHROMA(compId);
  const ChannelType channelType = toChannelType(compID);
  const int         iWidth      = piPred.width;
  const int         iHeight     = piPred.height;
  const int         loop_all    = (iWidth >= iHeight) ? iHeight : iWidth;
  int               num_loop    = 0;
  for (int w = iWidth, h = iHeight; w >= 1 && h >= 1; w--, h--)
  {
    num_loop++;
    if (w * h < LIP_RESERVE_CNT)
      break;
  }
  assert(num_loop > 1);

  CHECK(iWidth == 2, "Width of 2 is not supported");
  CHECK(PU::isMIP(pu, toChannelType(compId)), "We should not get here for MIP.");
  CHECK(floorLog2(iWidth) < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed");
  CHECK(floorLog2(iWidth) > 7, "Size not allowed");

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  uint32_t LIP_MODE_NUM           = 1 << BitsLoopMode;
  //uint32_t LIP_MODE_NUM           = 3;
  uint32_t LIP_MODE[LIP_MODE_NUM] = LIP_MODE_LIST;

  // TODO
  // const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const CPelBuf &srcBuf = CPelBuf(getPredictorPtrLIPUNFILTERED(compID), srcStride, srcHStride);
  const ClpRng & clpRng(pu.cu->cs->slice->clpRng(compID));

  int bitnum     = 0;
  int Bestbitnum = MAX_INT;
  int BestMode   = 0;
  int xMode      = 0;

  // TODO
  pu.num_loop = num_loop;
  for (xMode = 0; xMode < LIP_MODE_NUM; xMode++)   // for (xMode = 0; xMode < NUM_LUMA_MODE; xMode++)
  {
    int Mode = LIP_MODE[xMode];
    switch (Mode)
    {
    //case (0): bitnum = xPredIntraSape_loop1(srcBuf, piPred); break;//bitnum = xPredIntraPlanar_loop1(srcBuf, piPred); break;
    case (PLANAR_IDX): bitnum = xPredIntraPlanar_loop1(srcBuf, piPred); break;//bitnum = xPredIntraDc_loop1(srcBuf, piPred); break;
    case (DC_IDX): bitnum = xPredIntraDc_loop1(srcBuf, piPred); break;//bitnum = xPredIntraSape_loop1(srcBuf, piPred); break;
    default: bitnum = xPredIntraAng_loop1(srcBuf, piPred, channelType, clpRng, Mode); break;
    }
    if (bitnum < Bestbitnum)
    {
      Bestbitnum = bitnum;
      BestMode   = xMode;
    }
  }
  pu.intraDirLIP[channelType][0] = BestMode;
  pu.intraDir[channelType]       = LIP_MODE[BestMode];
  switch (LIP_MODE[BestMode])
  {
  //case (0): bitnum = xPredIntraSape_loop1(srcBuf, piPred); break;//bitnum = xPredIntraPlanar_loop1(srcBuf, piPred); break;
  case (PLANAR_IDX): bitnum = xPredIntraPlanar_loop1(srcBuf, piPred); break;//bitnum = xPredIntraDc_loop1(srcBuf, piPred); break;
  case (DC_IDX): bitnum = xPredIntraDc_loop1(srcBuf, piPred); break;//bitnum = xPredIntraSape_loop1(srcBuf, piPred); break;
  default: bitnum = xPredIntraAng_loop1(srcBuf, piPred, channelType, clpRng, LIP_MODE[BestMode]); break;
  }
  Bestbitnum = MAX_INT;

  int loop = 1;
  for (; loop < num_loop - 1; loop++)
  {
    for (xMode = 0; xMode < LIP_MODE_NUM; xMode++)
    {
      int Mode = LIP_MODE[xMode];
      switch (Mode)
      {
      //case (0): bitnum = xPredIntraSape_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraPlanar_loop(srcBuf, piPred, loop); break;
      case (PLANAR_IDX): bitnum = xPredIntraPlanar_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraDc_loop(srcBuf, piPred, loop); break;
      case (DC_IDX): bitnum = xPredIntraDc_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraSape_loop(srcBuf, piPred, loop); break;
      default: bitnum = xPredIntraAng_loop(srcBuf, piPred, channelType, clpRng, Mode, loop); break;
      }
      if (bitnum < Bestbitnum)
      {
        Bestbitnum = bitnum;
        BestMode   = xMode;
      }
    }
    pu.intraDirLIP[channelType][loop] = BestMode;
    switch (LIP_MODE[BestMode])
    {
    //case (0): bitnum = xPredIntraSape_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraPlanar_loop(srcBuf, piPred, loop); break;
    case (PLANAR_IDX): bitnum = xPredIntraPlanar_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraDc_loop(srcBuf, piPred, loop); break;
    case (DC_IDX): bitnum = xPredIntraDc_loop(srcBuf, piPred, loop); break;//bitnum = xPredIntraSape_loop(srcBuf, piPred, loop); break;
    default: bitnum = xPredIntraAng_loop(srcBuf, piPred, channelType, clpRng, LIP_MODE[BestMode], loop); break;
    }
    Bestbitnum = MAX_INT;
  }

  if (loop < loop_all)
  {
    Bestbitnum = MAX_INT;
    for (xMode = 0; xMode < LIP_MODE_NUM; xMode++)
    {
      int Mode = LIP_MODE[xMode];
      bitnum   = 0;
      for (int loop_res = loop; loop_res < loop_all; loop_res++)
      {
        switch (Mode)
        {
        //case (0): bitnum += xPredIntraSape_loop(srcBuf, piPred, loop_res); break;//bitnum += xPredIntraPlanar_loop(srcBuf, piPred, loop_res); break;
        case (PLANAR_IDX): bitnum += xPredIntraPlanar_loop(srcBuf, piPred, loop_res); break;//bitnum += xPredIntraDc_loop(srcBuf, piPred, loop_res); break;
        case (DC_IDX): bitnum += xPredIntraDc_loop(srcBuf, piPred, loop_res); break;//bitnum += xPredIntraSape_loop(srcBuf, piPred, loop_res); break;
        default: bitnum += xPredIntraAng_loop(srcBuf, piPred, channelType, clpRng, Mode, loop_res); break;
        }
      }
      if (bitnum < Bestbitnum)
      {
        Bestbitnum                        = bitnum;
        BestMode                          = xMode;
        pu.intraDirLIP[channelType][loop] = BestMode;
      }
    }

    for (int loop_res = loop; loop_res < loop_all; loop_res++)
    {
      switch (LIP_MODE[BestMode])
      {
      //case (0): xPredIntraSape_loop(srcBuf, piPred, loop_res); break;//xPredIntraPlanar_loop(srcBuf, piPred, loop_res); break;
      case (PLANAR_IDX): xPredIntraPlanar_loop(srcBuf, piPred, loop_res); break;//xPredIntraDc_loop(srcBuf, piPred, loop_res); break;
      case (DC_IDX): xPredIntraDc_loop(srcBuf, piPred, loop_res); break;//xPredIntraSape_loop(srcBuf, piPred, loop_res); break;
      default: xPredIntraAng_loop(srcBuf, piPred, channelType, clpRng, LIP_MODE[BestMode], loop_res); break;
      }
    }
  }
}

void IntraPrediction::predIntraAngDecLIP(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
{
  const ComponentID compID      = MAP_CHROMA(compId);
  const ChannelType channelType = toChannelType(compID);
  const int         iWidth      = piPred.width;
  const int         iHeight     = piPred.height;
  const int         loop_all    = (iWidth >= iHeight) ? iHeight : iWidth;
  const int         pstride     = (iWidth + iHeight + 1) * 2;
  int               num_loop    = 0;
  for (int w = iWidth, h = iHeight; w >= 1 && h >= 1; w--, h--)
  {
    num_loop++;
    if (w * h < LIP_RESERVE_CNT)
      break;
  }
  assert(num_loop > 1);

  CHECK(iWidth == 2, "Width of 2 is not supported");
  CHECK(PU::isMIP(pu, toChannelType(compId)), "We should not get here for MIP.");
  CHECK(floorLog2(iWidth) < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed");
  CHECK(floorLog2(iWidth) > 7, "Size not allowed");

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  //uint32_t LIP_MODE_NUM           = 1 << BitsLoopMode;
  //uint32_t LIP_MODE[8] = LIP_MODE_LIST;

  // TODO
  // const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  //const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const CPelBuf &srcBuf = CPelBuf(getPredictorPtrLIPUNFILTERED(compID), srcStride, srcHStride);
  const ClpRng & clpRng(pu.cu->cs->slice->clpRng(compID));

  int bitnum = 0;
  int xMode = 0;
  //int Mode  = 0;

  // TODO
  //pu.num_loop = num_loop;
  xMode       = pu.intraDirLIP[channelType][0];
  //Mode        = LIP_MODE[xMode];

  const uint32_t stride = piPred.stride;
  Pel *          pred   = piPred.buf;
  switch (xMode)
  {
  case (PLANAR_IDX): bitnum = xPredIntraPlanar_loop1(srcBuf, piPred); break;
  case (DC_IDX): bitnum = xPredIntraDc_loop1(srcBuf, piPred); break;
  default: bitnum = xPredIntraAng_loop1(srcBuf, piPred, channelType, clpRng, xMode); break;
  }

  int loop = 1;
  pred     = piPred.buf;
  for (; loop < loop_all; loop++)
  {
    xMode = pu.intraDirLIP[channelType][loop];
    //Mode  = LIP_MODE[xMode];
    //pred += stride + 1;

    switch (xMode)
    {
    case (PLANAR_IDX): bitnum = xPredIntraPlanarDec_loop(srcBuf, piPred, loop, pred); break;
    case (DC_IDX): bitnum = xPredIntraDcDec_loop(srcBuf, piPred, loop, pred); break;
    default: bitnum = xPredIntraAngDec_loop(srcBuf, piPred, channelType, clpRng, xMode, loop, pred); break;
    }
    pred += stride + 1;
  }

  //xMode = pu.intraDirLIP[channelType][num_loop - 1];
  //Mode  = LIP_MODE[xMode];
  /*for (loop = num_loop - 1; loop < loop_all; loop++)
  {
    pred += stride + 1;

    switch (xMode)
    {
    case (PLANAR_IDX): bitnum = xPredIntraPlanarDec_loop(srcBuf, piPred, loop, pred); break;
    case (DC_IDX): bitnum = xPredIntraDcDec_loop(srcBuf, piPred, loop, pred); break;
    default: bitnum = xPredIntraAngDec_loop(srcBuf, piPred, channelType, clpRng, Mode, loop, pred); break;
    }
  }*/
}

void IntraPrediction::predIntraAng(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
{
  const ComponentID compID      = MAP_CHROMA(compId);
  const ChannelType channelType = toChannelType(compID);
  const int         iWidth      = piPred.width;
  const int         iHeight     = piPred.height;
  CHECK(iWidth == 2, "Width of 2 is not supported");
  CHECK(PU::isMIP(pu, toChannelType(compId)), "We should not get here for MIP.");
  const uint32_t uiDirMode = isLuma(compId) && pu.cu->bdpcmMode          ? BDPCM_IDX
                             : !isLuma(compId) && pu.cu->bdpcmModeChroma ? BDPCM_IDX
                                                                         : PU::getFinalIntraMode(pu, channelType);

  CHECK(floorLog2(iWidth) < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed");
  CHECK(floorLog2(iWidth) > 7, "Size not allowed");

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const ClpRng & clpRng(pu.cu->cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
  case (PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred); break;
  case (DC_IDX): xPredIntraDc(srcBuf, piPred, channelType, false); break;
  case (BDPCM_IDX):
    xPredIntraBDPCM(srcBuf, piPred, isLuma(compID) ? pu.cu->bdpcmMode : pu.cu->bdpcmModeChroma, clpRng);
    break;
  default: xPredIntraAng(srcBuf, piPred, channelType, clpRng); break;
  }

  if (m_ipaParam.applyPDPC)
  {
    PelBuf    dstBuf = piPred;
    const int scale  = ((floorLog2(iWidth) - 2 + floorLog2(iHeight) - 2 + 2) >> 2);
    CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

    if (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX)
    {
      for (int y = 0; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(y + 1, 1);
        for (int x = 0; x < iWidth; x++)
        {
          const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top   = srcBuf.at(x + 1, 0);
          const Pel val   = dstBuf.at(x, y);
          dstBuf.at(x, y) = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }
  }
}

void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu,
                                        const CompArea &chromaArea, int intraDir)
{
  int    iLumaStride = 0;
  PelBuf Temp;
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
  {
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    Temp        = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
    iLumaStride = MAX_CU_SIZE + 1;
    Temp        = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  int a, b, iShift;
  xGetLMParameters(pu, compID, chromaArea, a, b, iShift);

  ////// final prediction
  piPred.copyFrom(Temp);
  piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
}

// LIP
int IntraPrediction::LIPgetLoopCost(Pel src, Pel pred)
{
  int cost;
  cost = abs(src - pred);

  return cost;
}

int IntraPrediction::xPredIntraPlanar_loop1(const CPelBuf &pSrc, PelBuf &pDst)
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int      pstride = pDst.width * 4 + pDst.height * 2 + 3;

  int bitnum = 0;

  // const uint32_t log2W = floorLog2(width);
  // const uint32_t log2H = floorLog2(height);

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  // const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int k = 0; k < width; k++)
  {
    topRow[k] = pSrc.at(k + 1, 0);
  }
  topRow[width] = pSrc.at(width, 0);

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 0; k < height; k++)
  {
    leftColumn[k] = pSrc.at(k + 1, 1);
  }
  leftColumn[height] = pSrc.at(height, 1);

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight   = topRow[width];

  for (int k = 0; k < width; k++)
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] * height;
  }

  for (int k = 0; k < height; k++)
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] * width;
  }

  // const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride = pDst.stride;
  Pel *          pred   = pDst.buf;

  int horPred = leftColumn[0];

  for (int x = 0; x < width; x++)
  {
    horPred += rightColumn[0];
    topRow[x] += bottomRow[x];

    int vertPred = topRow[x];
    pred[x]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(x + pstride, 0), pred[x]);
  }

  pred += stride;

  for (int y = 1; y < height; y++, pred += stride)
  {
    int horPred = leftColumn[y];

    horPred += rightColumn[y];
    topRow[0] += bottomRow[0];

    int vertPred = topRow[0];
    pred[0]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(pstride, y), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraPlanar_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop)
{
  const uint32_t width   = pDst.width - loop;
  const uint32_t height  = pDst.height - loop;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;

  int bitnum = 0;

  // const uint32_t log2W = floorLog2(width);
  // const uint32_t log2H = floorLog2(height);

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  // const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int k = 0; k < width; k++)
  {
    topRow[k] = pSrc.at(k + loop + pstride, loop - 1);
  }
  topRow[width] = pSrc.at(width - 1 + loop + pstride, loop - 1);

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 0; k < height; k++)
  {
    leftColumn[k] = pSrc.at(loop - 1 + pstride, k + loop);
  }
  leftColumn[height] = pSrc.at(loop - 1 + pstride, height - 1 + loop);

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight   = topRow[width];

  for (int k = 0; k < width; k++)
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] * height;
  }

  for (int k = 0; k < height; k++)
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] * width;
  }

  // const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride = pDst.stride;
  Pel *          pred   = pDst.buf;
  pred += loop;
  pred += loop * stride;

  int horPred = leftColumn[0];

  for (int x = 0; x < width; x++)
  {
    horPred += rightColumn[0];
    topRow[x] += bottomRow[x];

    int vertPred = topRow[x];
    pred[x]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(x + loop + pstride, loop), pred[x]);
  }

  pred += stride;

  for (int y = 1; y < height; y++, pred += stride)
  {
    int horPred = leftColumn[y];

    horPred += rightColumn[y];
    topRow[0] += bottomRow[0];

    int vertPred = topRow[0];
    pred[0]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, y + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraDc_loop1(const CPelBuf &pSrc, PelBuf &pDst)
{
  const int width   = pDst.width;
  const int height  = pDst.height;
  const int stride  = pDst.stride;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;
  const int denom   = (width == height) ? (width * 2) : std::max(width, height);

  int idx, sum = 0;
  int bitnum = 0;
  Pel dcVal;

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(1 + idx, 0);
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(1 + idx, 1);
    }
  }

  dcVal     = sum / denom;
  Pel *pred = pDst.buf;

  for (int l = 0; l < width; l++)
  {
    pred[l] = dcVal;
    // bitnum += abs(pSrc.at(l + pstride, 0) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + pstride, 0), pred[l]);
  }

  for (int k = 1; k < height; k++)
  {
    pred += stride;
    pred[0] = dcVal;
    // bitnum += abs(pSrc.at(pstride, k) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(pstride, k), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraDc_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop)
{
  const int width   = pDst.width - loop;
  const int height  = pDst.height - loop;
  const int stride  = pDst.stride;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;
  const int denom   = (width == height) ? (width * 2) : std::max(width, height);

  int idx, sum = 0;
  int bitnum = 0;
  Pel dcVal;

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(idx + loop + pstride, loop - 1);
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(loop - 1 + pstride, idx + loop);
    }
  }

  dcVal     = sum / denom;
  Pel *pred = pDst.buf;
  pred += loop;
  pred += loop * stride;

  for (int l = 0; l < width; l++)
  {
    pred[l] = dcVal;
    // TODO
    // bitnum += abs(pSrc.at(l + loop + pstride, loop) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + loop + pstride, loop), pred[l]);
  }

  for (int k = 1; k < height; k++)
  {
    pred += stride;
    pred[0] = dcVal;
    // bitnum += abs(pSrc.at(loop + pstride, k + loop) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, k + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraSape_loop1(const CPelBuf &pSrc, PelBuf &pDst)
{
  const uint32_t width   = pDst.width;
  const uint32_t height  = pDst.height;
  const int      stride  = pDst.stride;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;

  int  bitnum = 0;
  Pel *pred   = pDst.buf;

  Pel left    = pSrc.at(1, 1);
  Pel top     = pSrc.at(1, 0);
  Pel lefttop = pSrc.at(0, 0);
  Pel max     = (left >= top) ? left : top;
  Pel min     = (left >= top) ? top : left;
  if (lefttop >= max)
  {
    pred[0] = min;
  }
  else if (lefttop <= min)
  {
    pred[0] = max;
  }
  else
  {
    pred[0] = left + top - lefttop;
  }
  // bitnum += abs(pSrc.at(pstride, 0) - pred[0]);
  bitnum += LIPgetLoopCost(pSrc.at(pstride, 0), pred[0]);

  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int l = 1; l < width; l++)
  {
    Pel left    = pSrc.at(l - 1 + pstride, 0);
    Pel top     = pSrc.at(l + 1, 0);
    Pel lefttop = pSrc.at(l, 0);
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[l] = min;
    }
    else if (lefttop <= min)
    {
      pred[l] = max;
    }
    else
    {
      pred[l] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(l + pstride, 0) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + pstride, 0), pred[l]);
  }

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 1; k < height; k++)
  {
    pred += stride;
    Pel left    = pSrc.at(k + 1, 1);
    Pel top     = pSrc.at(pstride, k - 1);
    Pel lefttop = pSrc.at(k, 1);
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[0] = min;
    }
    else if (lefttop <= min)
    {
      pred[0] = max;
    }
    else
    {
      pred[0] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(pstride, k) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(pstride, k), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraSape_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop)
{
  const uint32_t width   = pDst.width - loop;
  const uint32_t height  = pDst.height - loop;
  const int      stride  = pDst.stride;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int      pstride = pDst.width * 4 + pDst.height * 2 + 3;

  int  bitnum = 0;
  Pel *pred   = pDst.buf;
  pred += loop;
  pred += loop * stride;

  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int l = 0; l < width; l++)
  {
    Pel left    = pSrc.at(l - 1 + loop + pstride, loop);
    Pel top     = pSrc.at(l + loop + pstride, loop - 1);
    Pel lefttop = pSrc.at(l - 1 + loop + pstride, loop - 1);
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[l] = min;
    }
    else if (lefttop <= min)
    {
      pred[l] = max;
    }
    else
    {
      pred[l] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(l + loop + pstride, loop) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + loop + pstride, loop), pred[l]);
  }

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 1; k < height; k++)
  {
    pred += stride;
    Pel left    = pSrc.at(loop - 1 + pstride, k + loop);
    Pel top     = pSrc.at(loop + pstride, k - 1 + loop);
    Pel lefttop = pSrc.at(loop - 1 + pstride, k - 1 + loop);
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[0] = min;
    }
    else if (lefttop <= min)
    {
      pred[0] = max;
    }
    else
    {
      pred[0] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(loop + pstride, k + loop) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, k + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraAng_loop1(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                         const ClpRng &clpRng, int Mode)
{
  int width  = int(pDst.width);
  int height = int(pDst.height);

  const bool bIsModeVer = Mode >= DIA_IDX;
  // const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;

  const int intraPredAngleMode = (bIsModeVer) ? Mode - VER_IDX : -(Mode - HOR_IDX);

  int              absAng          = 0;
  static const int angTable[32]    = { 0,  1,  2,  3,  4,  6,  8,  10, 12, 14,  16,  18,  20,  23,  26,  29,
                                    32, 35, 39, 45, 51, 57, 64, 73, 86, 102, 128, 171, 256, 341, 512, 1024 };
  static const int invAngTable[32] = { 0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024,
                                       910, 819,   712,  630,  565,  512,  468,  420,  364,  321,  287,
                                       256, 224,   191,  161,  128,  96,   64,   48,   32,   16 };

  const int absAngMode = abs(intraPredAngleMode);
  const int signAng    = intraPredAngleMode < 0 ? -1 : 1;
  absAng               = angTable[absAngMode];

  const int absInvAngle    = invAngTable[absAngMode];
  const int intraPredAngle = signAng * absAng;

  // const int sideSize = bIsModeVer ? height : width;
  // const int maxScale = 2;

  // const int angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * absInvAngle - 2) - 8));

  int bitnum = 0;

  Pel *refMain;
  Pel *refSide;

  Pel refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.

  for (int x = 0; x <= width; x++)
  {
    refAbove[x + height] = pSrc.at(x, 0);
  }
  refAbove[width + height + 1] = pSrc.at(width, 0);
  for (int y = 0; y <= height; y++)
  {
    refLeft[y + width] = pSrc.at(y, 1);
  }
  refLeft[height + width + 1] = pSrc.at(height, 1);
  refMain                     = bIsModeVer ? refAbove + height : refLeft + width;
  refSide                     = bIsModeVer ? refLeft + width : refAbove + height;

  // Extend the Main reference to the left.
  int sizeSide = bIsModeVer ? height : width;
  for (int k = -sizeSide; k <= -1; k++)
  {
    refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
  }

  /*if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    refAbove[width + height + 1] = pSrc.at(width, 0);
    for (int y = 0; y <= height; y++)
    {
      refLeft[y + width] = pSrc.at(y, 1);
    }
    refLeft[height + width + 1] = pSrc.at(height, 1);
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }*/

  // swap width/height if we are doing a horizontal mode:
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }
  Pel       tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : width;
  Pel *     pDstBuf   = bIsModeVer ? pDst.buf : tempArray;

  // compensate for line offset in reference line buffers
  // refMain += multiRefIdx;
  // refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    for (int x = 0; x < width; x++)
    {
      pDsty[x] = refMain[x + 1];
    }

    /*if (m_ipaParam.applyPDPC)
    {
      const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
      const Pel topLeft = refMain[0];
      const Pel left    = refSide[1];
      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        const int wL  = 32 >> (2 * x >> scale);
        const Pel val = pDsty[x];
        pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
      }
    }*/

    pDsty += dstStride;

    for (int y = 1; y < height; y++)
    {
      pDsty[0] = refMain[1];

      /*if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }*/

      pDsty += dstStride;
    }
  }
  else
  {
    int       deltaPos   = intraPredAngle;
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & 31;

    if (!isIntegerSlope(abs(intraPredAngle)))
    {
      if (isLuma(channelType))
      {
        const bool useCubicFilter = true;

        const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                       TFilterCoeff(32 - (deltaFract >> 1)),
                                                       TFilterCoeff(16 + (deltaFract >> 1)),
                                                       TFilterCoeff(deltaFract >> 1) };
        const TFilterCoeff *const f =
          (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

        for (int x = 0; x < width; x++)
        {
          Pel p[4];

          p[0] = refMain[deltaInt + x];
          p[1] = refMain[deltaInt + x + 1];
          p[2] = refMain[deltaInt + x + 2];
          p[3] = refMain[deltaInt + x + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
        }
      }
      else
      {
        // Do linear filtering
        for (int x = 0; x < width; x++)
        {
          Pel p[2];

          p[0] = refMain[deltaInt + x + 1];
          p[1] = refMain[deltaInt + x + 2];

          pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
    }
    else
    {
      // Just copy the integer samples
      for (int x = 0; x < width; x++)
      {
        pDsty[x] = refMain[x + deltaInt + 1];
      }
    }
    /*if (m_ipaParam.applyPDPC)
    {
      const int scale       = angularScale;
      int       invAngleSum = 256;

      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        invAngleSum += absInvAngle;

        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[(invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }*/

    pDsty += dstStride;
    for (int y = 1, deltaPos = intraPredAngle * 2; y < height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;

      if (!isIntegerSlope(abs(intraPredAngle)))
      {
        if (isLuma(channelType))
        {
          const bool useCubicFilter = true;

          const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                         TFilterCoeff(32 - (deltaFract >> 1)),
                                                         TFilterCoeff(16 + (deltaFract >> 1)),
                                                         TFilterCoeff(deltaFract >> 1) };
          const TFilterCoeff *const f =
            (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          Pel p[4];

          p[0] = refMain[deltaInt];
          p[1] = refMain[deltaInt + 1];
          p[2] = refMain[deltaInt + 2];
          p[3] = refMain[deltaInt + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[0] = ClipPel(val, clpRng);   // always clip even though not always neede
        }
        else
        {
          // Do linear filtering

          Pel p[2];

          p[0] = refMain[deltaInt + 1];
          p[1] = refMain[deltaInt + 2];

          pDsty[0] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
      else
      {
        // Just copy the integer samples
        pDsty[0] = refMain[deltaInt + 1];
      }
      /*if (m_ipaParam.applyPDPC)
      {
        const int scale       = angularScale;
        int       invAngleSum = 256;

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += absInvAngle;

          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }*/
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    for (int x = 0; x < width; x++)
    {
      pDst.at(0, x) = pDstBuf[x];
      // bitnum += abs(pSrc.at(pstride, x) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(pstride, x), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      pDst.at(y, 0) = pDstBuf[0];
      // bitnum += abs(pSrc.at(y + pstride, 0) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(y + pstride, 0), pDstBuf[0]);
    }
  }
  else
  {
    for (int x = 0; x < width; x++)
    {
      // bitnum += abs(pSrc.at(x + pstride, 0) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(x + pstride, 0), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      // bitnum += abs(pSrc.at(pstride, y) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(pstride, y), pDstBuf[0]);
    }
  }

  return bitnum;
}

int IntraPrediction::xPredIntraAng_loop(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                        const ClpRng &clpRng, int Mode, int loop)
{
  int width  = int(pDst.width) - loop;
  int height = int(pDst.height) - loop;

  const bool bIsModeVer = Mode >= DIA_IDX;
  // const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;

  const int intraPredAngleMode = (bIsModeVer) ? Mode - VER_IDX : -(Mode - HOR_IDX);

  int              absAng          = 0;
  static const int angTable[32]    = { 0,  1,  2,  3,  4,  6,  8,  10, 12, 14,  16,  18,  20,  23,  26,  29,
                                    32, 35, 39, 45, 51, 57, 64, 73, 86, 102, 128, 171, 256, 341, 512, 1024 };
  static const int invAngTable[32] = { 0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024,
                                       910, 819,   712,  630,  565,  512,  468,  420,  364,  321,  287,
                                       256, 224,   191,  161,  128,  96,   64,   48,   32,   16 };

  const int absAngMode = abs(intraPredAngleMode);
  const int signAng    = intraPredAngleMode < 0 ? -1 : 1;
  absAng               = angTable[absAngMode];

  const int absInvAngle    = invAngTable[absAngMode];
  const int intraPredAngle = signAng * absAng;

  // const int sideSize = bIsModeVer ? height : width;
  // const int maxScale = 2;

  // const int angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * absInvAngle - 2) - 8));

  int bitnum = 0;

  Pel *refMain;
  Pel *refSide;

  Pel refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  for (int x = 0; x <= width; x++)
  {
    refAbove[x + height] = pSrc.at(x - 1 + loop + pstride, loop - 1);
  }
  refAbove[width + height + 1] = pSrc.at(width - 1 + loop + pstride, loop - 1);
  for (int y = 0; y <= height; y++)
  {
    refLeft[y + width] = pSrc.at(loop - 1 + pstride, y - 1 + loop);
  }
  refLeft[height + width + 1] = pSrc.at(loop - 1 + pstride, height - 1 + loop);
  refMain                     = bIsModeVer ? refAbove + height : refLeft + width;
  refSide                     = bIsModeVer ? refLeft + width : refAbove + height;

  // Extend the Main reference to the left.
  int sizeSide = bIsModeVer ? height : width;
  for (int k = -sizeSide; k <= -1; k++)
  {
    refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
  }
  /*if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width; x++)
    {
      refAbove[x + height] = pSrc.at(x - 1 + loop + pstride, loop - 1);
    }
    refAbove[width + height + 1] = pSrc.at(width - 1 + loop + pstride, loop - 1);
    for (int y = 0; y <= height; y++)
    {
      refLeft[y + width] = pSrc.at(loop - 1 + pstride, y - 1 + loop);
    }
    refLeft[height + width + 1] = pSrc.at(loop - 1 + pstride, height - 1 + loop);
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x - 1 + loop + pstride, loop - 1);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(loop - 1 + pstride, y - 1 + loop);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }*/

  // swap width/height if we are doing a horizontal mode:
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }
  Pel       tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : width;
  Pel *     pDstBuf   = bIsModeVer ? pDst.buf : tempArray;
  pDstBuf += loop;
  pDstBuf += loop * dstStride;

  // compensate for line offset in reference line buffers
  // refMain += multiRefIdx;
  // refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    for (int x = 0; x < width; x++)
    {
      pDsty[x] = refMain[x + 1];
    }

    /*if (m_ipaParam.applyPDPC)
    {
      const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
      const Pel topLeft = refMain[0];
      const Pel left    = refSide[1];
      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        const int wL  = 32 >> (2 * x >> scale);
        const Pel val = pDsty[x];
        pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
      }
    }*/

    pDsty += dstStride;

    for (int y = 1; y < height; y++)
    {
      pDsty[0] = refMain[1];

      /*if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }*/

      pDsty += dstStride;
    }
  }
  else
  {
    int       deltaPos   = intraPredAngle;
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & 31;

    if (!isIntegerSlope(abs(intraPredAngle)))
    {
      if (isLuma(channelType))
      {
        const bool useCubicFilter = true;

        const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                       TFilterCoeff(32 - (deltaFract >> 1)),
                                                       TFilterCoeff(16 + (deltaFract >> 1)),
                                                       TFilterCoeff(deltaFract >> 1) };
        const TFilterCoeff *const f =
          (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

        for (int x = 0; x < width; x++)
        {
          Pel p[4];

          p[0] = refMain[deltaInt + x];
          p[1] = refMain[deltaInt + x + 1];
          p[2] = refMain[deltaInt + x + 2];
          p[3] = refMain[deltaInt + x + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
        }
      }
      else
      {
        // Do linear filtering
        for (int x = 0; x < width; x++)
        {
          Pel p[2];

          p[0] = refMain[deltaInt + x + 1];
          p[1] = refMain[deltaInt + x + 2];

          pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
    }
    else
    {
      // Just copy the integer samples
      for (int x = 0; x < width; x++)
      {
        pDsty[x] = refMain[x + deltaInt + 1];
      }
    }
    /*if (m_ipaParam.applyPDPC)
    {
      const int scale       = angularScale;
      int       invAngleSum = 256;

      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        invAngleSum += absInvAngle;

        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[(invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }*/

    pDsty += dstStride;
    for (int y = 1, deltaPos = intraPredAngle * 2; y < height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;

      if (!isIntegerSlope(abs(intraPredAngle)))
      {
        if (isLuma(channelType))
        {
          const bool useCubicFilter = true;

          const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                         TFilterCoeff(32 - (deltaFract >> 1)),
                                                         TFilterCoeff(16 + (deltaFract >> 1)),
                                                         TFilterCoeff(deltaFract >> 1) };
          const TFilterCoeff *const f =
            (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          Pel p[4];

          p[0] = refMain[deltaInt];
          p[1] = refMain[deltaInt + 1];
          p[2] = refMain[deltaInt + 2];
          p[3] = refMain[deltaInt + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[0] = ClipPel(val, clpRng);   // always clip even though not always neede
        }
        else
        {
          // Do linear filtering

          Pel p[2];

          p[0] = refMain[deltaInt + 1];
          p[1] = refMain[deltaInt + 2];

          pDsty[0] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
      else
      {
        // Just copy the integer samples
        for (int x = 0; x < width; x++)
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
      /*if (m_ipaParam.applyPDPC)
      {
        const int scale       = angularScale;
        int       invAngleSum = 256;

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += absInvAngle;

          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }*/
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    for (int x = 0; x < width; x++)
    {
      pDst.at(loop, x + loop) = pDstBuf[x];
      //bitnum += abs(pSrc.at(loop + pstride, x + loop) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, x + loop), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      pDst.at(y + loop, loop) = pDstBuf[0];
      //bitnum += abs(pSrc.at(y + loop + pstride, loop) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(y + loop + pstride, loop), pDstBuf[0]);
    }
  }
  else
  {
    for (int x = 0; x < width; x++)
    {
      //bitnum += abs(pSrc.at(x + loop + pstride, loop) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(x + loop + pstride, loop), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      //bitnum += abs(pSrc.at(loop + pstride, y + loop) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, y + loop), pDstBuf[0]);
    }
  }

  return bitnum;
}

int IntraPrediction::xPredIntraPlanarDec_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop, Pel *LastPred)
{
  const uint32_t width   = pDst.width - loop;
  const uint32_t height  = pDst.height - loop;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int      pstride = pDst.width * 4 + pDst.height * 2 + 3;
  Pel *          xPred   = LastPred;
  const uint32_t stride  = pDst.stride;

  int bitnum = 0;

  // const uint32_t log2W = floorLog2(width);
  // const uint32_t log2H = floorLog2(height);

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  // const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int k = 0; k < width; k++)
  {
    topRow[k] = pSrc.at(k + loop + pstride, loop - 1) + xPred[k + 1];
  }
  //topRow[width] = pSrc.at(width - 1 + loop + pstride, loop - 1) + xPred[width - 1];
  topRow[width] = topRow[width - 1];

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 0; k < height; k++)
  {
    xPred += stride;
    leftColumn[k] = pSrc.at(loop - 1 + pstride, k + loop) + xPred[0];
  }
  //leftColumn[height] = pSrc.at(loop - 1 + pstride, height - 1 + loop) + xPred[0];
  leftColumn[height] = leftColumn[height - 1];

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight   = topRow[width];

  for (int k = 0; k < width; k++)
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] * height;
  }

  for (int k = 0; k < height; k++)
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] * width;
  }

  // const uint32_t finalShift = 1 + log2W + log2H;
  Pel *          pred   = pDst.buf;
  pred += loop;
  pred += loop * stride;

  int horPred = leftColumn[0];

  for (int x = 0; x < width; x++)
  {
    horPred += rightColumn[0];
    topRow[x] += bottomRow[x];

    int vertPred = topRow[x];
    pred[x]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(x + loop + pstride, loop), pred[x]);
  }

  pred += stride;

  for (int y = 1; y < height; y++, pred += stride)
  {
    int horPred = leftColumn[y];

    horPred += rightColumn[y];
    topRow[0] += bottomRow[0];

    int vertPred = topRow[0];
    pred[0]      = ((horPred * height) + (vertPred * width)) / (2 * width * height);

    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, y + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraDcDec_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop, Pel *LastPred)
{
  const int width   = pDst.width - loop;
  const int height  = pDst.height - loop;
  const int stride  = pDst.stride;
  const int pstride = (pDst.width + pDst.height + 1) * 4;
  //const int      pstride = pDst.width * 4 + pDst.height * 2 + 3;
  const int denom   = (width == height) ? (width * 2) : std::max(width, height);
  Pel *          xPred   = LastPred;

  int idx, sum = 0;
  int bitnum = 0;
  Pel dcVal;

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(idx + loop + pstride, loop - 1) + xPred[idx + 1];
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      xPred += stride;
      sum += pSrc.at(loop - 1 + pstride, idx + loop) + xPred[0];
    }
  }

  dcVal     = sum / denom;
  Pel *pred = pDst.buf;
  pred += loop;
  pred += loop * stride;

  for (int l = 0; l < width; l++)
  {
    pred[l] = dcVal;
    // TODO
    // bitnum += abs(pSrc.at(l + loop + pstride, loop) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + loop + pstride, loop), pred[l]);
  }

  for (int k = 1; k < height; k++)
  {
    pred += stride;
    pred[0] = dcVal;
    // bitnum += abs(pSrc.at(loop + pstride, k + loop) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, k + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraSapeDec_loop(const CPelBuf &pSrc, PelBuf &pDst, int loop, Pel *LastPred)
{
  const uint32_t width   = pDst.width - loop;
  const uint32_t height  = pDst.height - loop;
  const int      stride  = pDst.stride;
  const int      pstride = (pDst.width + pDst.height + 1) * 4;
  //const int      pstride = pDst.width * 4 + pDst.height * 2 + 3;
  Pel *          xPred   = LastPred;

  int  bitnum = 0;
  Pel *pred   = pDst.buf;
  pred += loop;
  pred += loop * stride;

  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int l = 0; l < width; l++)
  {
    Pel left    = pSrc.at(l - 1 + loop + pstride, loop) + xPred[l + stride];
    Pel top     = pSrc.at(l + loop + pstride, loop - 1) + xPred[l + 1];
    Pel lefttop = pSrc.at(l - 1 + loop + pstride, loop - 1) + xPred[l];
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[l] = min;
    }
    else if (lefttop <= min)
    {
      pred[l] = max;
    }
    else
    {
      pred[l] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(l + loop + pstride, loop) - pred[l]);
    bitnum += LIPgetLoopCost(pSrc.at(l + loop + pstride, loop), pred[l]);
  }

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 1; k < height; k++)
  {
    pred += stride;
    Pel left    = pSrc.at(loop - 1 + pstride, k + loop) + xPred[(k + 1) * stride];
    Pel top     = pSrc.at(loop + pstride, k - 1 + loop) + xPred[k * stride + 1];
    Pel lefttop = pSrc.at(loop - 1 + pstride, k - 1 + loop) + xPred[k * stride];
    Pel max     = (left >= top) ? left : top;
    Pel min     = (left >= top) ? top : left;
    if (lefttop >= max)
    {
      pred[0] = min;
    }
    else if (lefttop <= min)
    {
      pred[0] = max;
    }
    else
    {
      pred[0] = left + top - lefttop;
    }
    // bitnum += abs(pSrc.at(loop + pstride, k + loop) - pred[0]);
    bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, k + loop), pred[0]);
  }

  return bitnum;
}

int IntraPrediction::xPredIntraAngDec_loop(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                           const ClpRng &clpRng, int Mode, int loop, Pel *LastPred)
{
  int width  = int(pDst.width) - loop;
  int height = int(pDst.height) - loop;

  const bool bIsModeVer = Mode >= DIA_IDX;
  // const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int pstride = (pDst.width + pDst.height + 1) * 4;
  //const int pstride = pDst.width * 4 + pDst.height * 2 + 3;
  const int stride  = pDst.stride;
  Pel *     xPred   = LastPred;

  const int intraPredAngleMode = (bIsModeVer) ? Mode - VER_IDX : -(Mode - HOR_IDX);

  int              absAng          = 0;
  static const int angTable[32]    = { 0,  1,  2,  3,  4,  6,  8,  10, 12, 14,  16,  18,  20,  23,  26,  29,
                                    32, 35, 39, 45, 51, 57, 64, 73, 86, 102, 128, 171, 256, 341, 512, 1024 };
  static const int invAngTable[32] = { 0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024,
                                       910, 819,   712,  630,  565,  512,  468,  420,  364,  321,  287,
                                       256, 224,   191,  161,  128,  96,   64,   48,   32,   16 };

  const int absAngMode = abs(intraPredAngleMode);
  const int signAng    = intraPredAngleMode < 0 ? -1 : 1;
  absAng               = angTable[absAngMode];

  const int absInvAngle    = invAngTable[absAngMode];
  const int intraPredAngle = signAng * absAng;

  // const int sideSize = bIsModeVer ? height : width;
  // const int maxScale = 2;

  // const int angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * absInvAngle - 2) - 8));

  int bitnum = 0;

  Pel *refMain;
  Pel *refSide;

  Pel refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  for (int x = 0; x <= width; x++)
  {
    refAbove[x + height] = pSrc.at(x - 1 + loop + pstride, loop - 1) + xPred[x];
  }
  //refAbove[width + height + 1] = pSrc.at(width - 1 + loop + pstride, loop - 1) + xPred[width];
  refAbove[width + height + 1] = refAbove[width + height];
  for (int y = 0; y <= height; y++)
  {
    refLeft[y + width] = pSrc.at(loop - 1 + pstride, y - 1 + loop) + xPred[0];
    xPred += stride;
  }
  //refLeft[height + width + 1] = pSrc.at(loop - 1 + pstride, height - 1 + loop);
  refLeft[height + width + 1] = refLeft[height + width];
  refMain                     = bIsModeVer ? refAbove + height : refLeft + width;
  refSide                     = bIsModeVer ? refLeft + width : refAbove + height;

  // Extend the Main reference to the left.
  int sizeSide = bIsModeVer ? height : width;
  for (int k = -sizeSide; k <= -1; k++)
  {
    refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
  }
  /*if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width; x++)
    {
      refAbove[x + height] = pSrc.at(x - 1 + loop + pstride, loop - 1);
    }
    refAbove[width + height + 1] = pSrc.at(width - 1 + loop + pstride, loop - 1);
    for (int y = 0; y <= height; y++)
    {
      refLeft[y + width] = pSrc.at(loop - 1 + pstride, y - 1 + loop);
    }
    refLeft[height + width + 1] = pSrc.at(loop - 1 + pstride, height - 1 + loop);
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x - 1 + loop + pstride, loop - 1);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(loop - 1 + pstride, y - 1 + loop);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }*/

  // swap width/height if we are doing a horizontal mode:
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }
  Pel       tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : width;
  Pel *     pDstBuf   = bIsModeVer ? pDst.buf : tempArray;
  pDstBuf += loop;
  pDstBuf += loop * dstStride;

  // compensate for line offset in reference line buffers
  // refMain += multiRefIdx;
  // refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    for (int x = 0; x < width; x++)
    {
      pDsty[x] = refMain[x + 1];
    }

    /*if (m_ipaParam.applyPDPC)
    {
      const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
      const Pel topLeft = refMain[0];
      const Pel left    = refSide[1];
      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        const int wL  = 32 >> (2 * x >> scale);
        const Pel val = pDsty[x];
        pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
      }
    }*/

    pDsty += dstStride;

    for (int y = 1; y < height; y++)
    {
      pDsty[0] = refMain[1];

      /*if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }*/

      pDsty += dstStride;
    }
  }
  else
  {
    int       deltaPos   = intraPredAngle;
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & 31;

    if (!isIntegerSlope(abs(intraPredAngle)))
    {
      if (isLuma(channelType))
      {
        const bool useCubicFilter = true;

        const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                       TFilterCoeff(32 - (deltaFract >> 1)),
                                                       TFilterCoeff(16 + (deltaFract >> 1)),
                                                       TFilterCoeff(deltaFract >> 1) };
        const TFilterCoeff *const f =
          (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

        for (int x = 0; x < width; x++)
        {
          Pel p[4];

          p[0] = refMain[deltaInt + x];
          p[1] = refMain[deltaInt + x + 1];
          p[2] = refMain[deltaInt + x + 2];
          p[3] = refMain[deltaInt + x + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
        }
      }
      else
      {
        // Do linear filtering
        for (int x = 0; x < width; x++)
        {
          Pel p[2];

          p[0] = refMain[deltaInt + x + 1];
          p[1] = refMain[deltaInt + x + 2];

          pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
    }
    else
    {
      // Just copy the integer samples
      for (int x = 0; x < width; x++)
      {
        pDsty[x] = refMain[x + deltaInt + 1];
      }
    }
    /*if (m_ipaParam.applyPDPC)
    {
      const int scale       = angularScale;
      int       invAngleSum = 256;

      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        invAngleSum += absInvAngle;

        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[(invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }*/

    pDsty += dstStride;
    for (int y = 1, deltaPos = intraPredAngle * 2; y < height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;

      if (!isIntegerSlope(abs(intraPredAngle)))
      {
        if (isLuma(channelType))
        {
          const bool useCubicFilter = true;

          const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                         TFilterCoeff(32 - (deltaFract >> 1)),
                                                         TFilterCoeff(16 + (deltaFract >> 1)),
                                                         TFilterCoeff(deltaFract >> 1) };
          const TFilterCoeff *const f =
            (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          Pel p[4];

          p[0] = refMain[deltaInt];
          p[1] = refMain[deltaInt + 1];
          p[2] = refMain[deltaInt + 2];
          p[3] = refMain[deltaInt + 3];

          Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

          pDsty[0] = ClipPel(val, clpRng);   // always clip even though not always neede
        }
        else
        {
          // Do linear filtering

          Pel p[2];

          p[0] = refMain[deltaInt + 1];
          p[1] = refMain[deltaInt + 2];

          pDsty[0] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
        }
      }
      else
      {
        // Just copy the integer samples
        for (int x = 0; x < width; x++)
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
      /*if (m_ipaParam.applyPDPC)
      {
        const int scale       = angularScale;
        int       invAngleSum = 256;

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += absInvAngle;

          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }*/
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    for (int x = 0; x < width; x++)
    {
      pDst.at(loop, x + loop) = pDstBuf[x];
      // bitnum += abs(pSrc.at(loop + pstride, x + loop) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, x + loop), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      pDst.at(y + loop, loop) = pDstBuf[0];
      // bitnum += abs(pSrc.at(y + loop + pstride, loop) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(y + loop + pstride, loop), pDstBuf[0]);
    }
  }
  else
  {
    for (int x = 0; x < width; x++)
    {
      // bitnum += abs(pSrc.at(x + loop + pstride, loop) - pDstBuf[x]);
      bitnum += LIPgetLoopCost(pSrc.at(x + loop + pstride, loop), pDstBuf[x]);
    }
    for (int y = 1; y < height; y++)
    {
      pDstBuf += dstStride;
      // bitnum += abs(pSrc.at(loop + pstride, y + loop) - pDstBuf[0]);
      bitnum += LIPgetLoopCost(pSrc.at(loop + pstride, y + loop), pDstBuf[0]);
    }
  }

  return bitnum;
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra
 * coding).
 */

// NOTE: Bit-Limit - 24-bit source
void IntraPrediction::xPredIntraPlanar(const CPelBuf &pSrc, PelBuf &pDst)
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;

  const uint32_t log2W = floorLog2(width);
  const uint32_t log2H = floorLog2(height);

  int            leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int k = 0; k < width + 1; k++)
  {
    topRow[k] = pSrc.at(k + 1, 0);
  }

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 0; k < height + 1; k++)
  {
    leftColumn[k] = pSrc.at(k + 1, 1);
  }

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight   = topRow[width];

  for (int k = 0; k < width; k++)
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for (int k = 0; k < height; k++)
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride     = pDst.stride;
  Pel *          pred       = pDst.buf;
  for (int y = 0; y < height; y++, pred += stride)
  {
    int horPred = leftColumn[y];

    for (int x = 0; x < width; x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      int vertPred = topRow[x];
      pred[x]      = ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
    }
  }
}

void IntraPrediction::xPredIntraDc(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                   const bool enableBoundaryFilter)
{
  const Pel dcval = xGetPredValDc(pSrc, pDst);
  pDst.fill(dcval);
}

// Function for initialization of intra prediction parameters
void IntraPrediction::initPredIntraParams(const PredictionUnit &pu, const CompArea area, const SPS &sps)
{
  const ComponentID compId = area.compID;
  const ChannelType chType = toChannelType(compId);

  const bool useISP = NOT_INTRA_SUBPARTITIONS != pu.cu->ispMode && isLuma(chType);

  const Size  cuSize    = Size(pu.cu->blocks[compId].width, pu.cu->blocks[compId].height);
  const Size  puSize    = Size(area.width, area.height);
  const Size &blockSize = useISP ? cuSize : puSize;
  const int   dirMode   = PU::getFinalIntraMode(pu, chType);
  const int   predMode  = getModifiedWideAngle(blockSize.width, blockSize.height, dirMode);

  m_ipaParam.isModeVer         = predMode >= DIA_IDX;
  m_ipaParam.multiRefIndex     = isLuma(chType) ? pu.multiRefIdx : 0;
  m_ipaParam.refFilterFlag     = false;
  m_ipaParam.interpolationFlag = false;
  m_ipaParam.applyPDPC =
    (puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY) && m_ipaParam.multiRefIndex == 0;

  const int intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);

  int absAng = 0;
  if (dirMode > DC_IDX && dirMode < NUM_LUMA_MODE)   // intraPredAngle for directional modes
  {
    static const int angTable[32]    = { 0,  1,  2,  3,  4,  6,  8,  10, 12, 14,  16,  18,  20,  23,  26,  29,
                                      32, 35, 39, 45, 51, 57, 64, 73, 86, 102, 128, 171, 256, 341, 512, 1024 };
    static const int invAngTable[32] = {
      0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
      512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
    };   // (512 * 32) / Angle

    const int absAngMode = abs(intraPredAngleMode);
    const int signAng    = intraPredAngleMode < 0 ? -1 : 1;
    absAng               = angTable[absAngMode];

    m_ipaParam.absInvAngle    = invAngTable[absAngMode];
    m_ipaParam.intraPredAngle = signAng * absAng;
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;

      m_ipaParam.angularScale =
        std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.absInvAngle - 2) - 8));
      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
  }

  // high level conditions and DC intra prediction
  if (sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() || !isLuma(chType) || useISP || PU::isMIP(pu, chType)
      || m_ipaParam.multiRefIndex || DC_IDX == dirMode)
  {
  }
  else if ((isLuma(chType) && pu.cu->bdpcmMode) || (!isLuma(chType) && pu.cu->bdpcmModeChroma))   // BDPCM
  {
    m_ipaParam.refFilterFlag = false;
  }
  else if (dirMode == PLANAR_IDX)   // Planar intra prediction
  {
    m_ipaParam.refFilterFlag = puSize.width * puSize.height > 32 ? true : false;
  }
  else if (!useISP)   // HOR, VER and angular modes (MDIS)
  {
    bool filterFlag = false;
    {
      const int diff     = std::min<int>(abs(predMode - HOR_IDX), abs(predMode - VER_IDX));
      const int log2Size = ((floorLog2(puSize.width) + floorLog2(puSize.height)) >> 1);
      CHECK(log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported");
      filterFlag = (diff > m_aucIntraFilter[log2Size]);
    }

    // Selelection of either ([1 2 1] / 4 ) refrence filter OR Gaussian 4-tap interpolation filter
    if (filterFlag)
    {
      const bool isRefFilter = isIntegerSlope(absAng);
      CHECK(puSize.width * puSize.height <= 32,
            "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB");
      m_ipaParam.refFilterFlag     = isRefFilter;
      m_ipaParam.interpolationFlag = !isRefFilter;
    }
  }
}

/** Function for deriving the simplified angular intra predictions.
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are
 * taken from the extended main reference.
 */
// NOTE: Bit-Limit - 25-bit source

void IntraPrediction::xPredIntraAng(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                    const ClpRng &clpRng)
{
  int width  = int(pDst.width);
  int height = int(pDst.height);

  const bool bIsModeVer     = m_ipaParam.isModeVer;
  const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  absInvAngle    = m_ipaParam.absInvAngle;

  Pel *refMain;
  Pel *refSide;

  Pel refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width + 1 + multiRefIdx; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1 + multiRefIdx; y++)
    {
      refLeft[y + width] = pSrc.at(y, 1);
    }
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }

  // swap width/height if we are doing a horizontal mode:
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }
  Pel       tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : width;
  Pel *     pDstBuf   = bIsModeVer ? pDst.buf : tempArray;

  // compensate for line offset in reference line buffers
  refMain += multiRefIdx;
  refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pDsty[x] = refMain[x + 1];
      }

      if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }

      pDsty += dstStride;
    }
  }
  else
  {
    for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
         y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;

      if (!isIntegerSlope(abs(intraPredAngle)))
      {
        if (isLuma(channelType))
        {
          const bool useCubicFilter = !m_ipaParam.interpolationFlag;

          const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(16 - (deltaFract >> 1)),
                                                         TFilterCoeff(32 - (deltaFract >> 1)),
                                                         TFilterCoeff(16 + (deltaFract >> 1)),
                                                         TFilterCoeff(deltaFract >> 1) };
          const TFilterCoeff *const f =
            (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          for (int x = 0; x < width; x++)
          {
            Pel p[4];

            p[0] = refMain[deltaInt + x];
            p[1] = refMain[deltaInt + x + 1];
            p[2] = refMain[deltaInt + x + 2];
            p[3] = refMain[deltaInt + x + 3];

            Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

            pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
          }
        }
        else
        {
          // Do linear filtering
          for (int x = 0; x < width; x++)
          {
            Pel p[2];

            p[0] = refMain[deltaInt + x + 1];
            p[1] = refMain[deltaInt + x + 2];

            pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for (int x = 0; x < width; x++)
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
      if (m_ipaParam.applyPDPC)
      {
        const int scale       = m_ipaParam.angularScale;
        int       invAngleSum = 256;

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += absInvAngle;

          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pDst.at(y, x) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }
}

void IntraPrediction::xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng &clpRng)
{
  const int wdt = pDst.width;
  const int hgt = pDst.height;

  const int strideP = pDst.stride;
  const int strideS = pSrc.stride;

  CHECK(!(dirMode == 1 || dirMode == 2), "Incorrect BDPCM mode parameter.");

  Pel *pred = &pDst.buf[0];
  if (dirMode == 1)
  {
    Pel val;
    for (int y = 0; y < hgt; y++)
    {
      val = pSrc.buf[(y + 1) + strideS];
      for (int x = 0; x < wdt; x++)
      {
        pred[x] = val;
      }
      pred += strideP;
    }
  }
  else
  {
    for (int y = 0; y < hgt; y++)
    {
      for (int x = 0; x < wdt; x++)
      {
        pred[x] = pSrc.buf[x + 1];
      }
      pred += strideP;
    }
  }
}

void IntraPrediction::geneWeightedPred(const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf)
{
  const int width = pred.width;
  CHECK(width == 2, "Width of 2 is not supported");
  const int height    = pred.height;
  const int srcStride = width;
  const int dstStride = pred.stride;

  Pel *dstBuf = pred.buf;
  int  wIntra, wMerge;

  const Position        posBL         = pu.Y().bottomLeft();
  const Position        posTR         = pu.Y().topRight();
  const PredictionUnit *neigh0        = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  const PredictionUnit *neigh1        = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  bool                  isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
  bool                  isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));

  if (isNeigh0Intra && isNeigh1Intra)
  {
    wIntra = 3;
    wMerge = 1;
  }
  else
  {
    if (!isNeigh0Intra && !isNeigh1Intra)
    {
      wIntra = 1;
      wMerge = 3;
    }
    else
    {
      wIntra = 2;
      wMerge = 2;
    }
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstBuf[y * dstStride + x] = (wMerge * dstBuf[y * dstStride + x] + wIntra * srcBuf[y * srcStride + x] + 2) >> 2;
    }
  }
}

void IntraPrediction::switchBuffer(const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst)
{
  Pel *src        = srcBuff.bufAt(0, 0);
  int  compWidth  = compID == COMPONENT_Y ? pu.Y().width : pu.Cb().width;
  int  compHeight = compID == COMPONENT_Y ? pu.Y().height : pu.Cb().height;
  for (int i = 0; i < compHeight; i++)
  {
    memcpy(dst, src, compWidth * sizeof(Pel));
    src += srcBuff.stride;
    dst += compWidth;
  }
}

void IntraPrediction::geneIntrainterPred(const CodingUnit &cu)
{
  if (!cu.firstPU->ciipFlag)
  {
    return;
  }

  const PredictionUnit *pu = cu.firstPU;

  initIntraPatternChTypeLIP(cu, pu->Y());
  predIntraAng(COMPONENT_Y, cu.cs->getPredBuf(*pu).Y(), *pu);
  int maxCompID = 1;
  if (isChromaEnabled(pu->chromaFormat))
  {
    maxCompID = MAX_NUM_COMPONENT;
    if (pu->chromaSize().width > 2)
    {
      initIntraPatternChTypeLIP(cu, pu->Cb());
      predIntraAng(COMPONENT_Cb, cu.cs->getPredBuf(*pu).Cb(), *pu);

      initIntraPatternChTypeLIP(cu, pu->Cr());
      predIntraAng(COMPONENT_Cr, cu.cs->getPredBuf(*pu).Cr(), *pu);
    }
  }
  for (int currCompID = 0; currCompID < maxCompID; currCompID++)
  {
    if (currCompID > 0 && pu->chromaSize().width <= 2)
    {
      continue;
    }
    ComponentID currCompID2 = (ComponentID) currCompID;
    PelBuf      tmpBuf      = currCompID == 0 ? cu.cs->getPredBuf(*pu).Y()
                                              : (currCompID == 1 ? cu.cs->getPredBuf(*pu).Cb() : cu.cs->getPredBuf(*pu).Cr());
    switchBuffer(*pu, currCompID2, tmpBuf, getPredictorPtr2(currCompID2, 0));
  }
}

inline bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT);
inline int  isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                             const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags);
inline int  isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                            const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags);
inline int  isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT,
                                  const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags);
inline int  isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB,
                                 const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags);

void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag)
{
  CHECK(area.width == 2, "Width of 2 is not supported");
  const CodingStructure &cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths(area);

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered, area, cu);
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    xFilterReferenceSamples(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx);
  }
}

void IntraPrediction::initIntraPatternChTypeISP(const CodingUnit &cu, const CompArea &area, PelBuf &recBuf,
                                                const bool forceRefFilterFlag)
{
  const CodingStructure &cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  const Position posLT       = area;
  bool           isLeftAvail = (cs.getCURestricted(posLT.offset(-1, 0), cu, CHANNEL_TYPE_LUMA) != NULL)
                     && cs.isDecomp(posLT.offset(-1, 0), CHANNEL_TYPE_LUMA);
  bool isAboveAvail = (cs.getCURestricted(posLT.offset(0, -1), cu, CHANNEL_TYPE_LUMA) != NULL)
                      && cs.isDecomp(posLT.offset(0, -1), CHANNEL_TYPE_LUMA);
  // ----- Step 1: unfiltered reference samples -----
  if (cu.blocks[area.compID].x == area.x && cu.blocks[area.compID].y == area.y)
  {
    Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
    // With the first subpartition all the CU reference samples are fetched at once in a single call to
    // xFillReferenceSamples
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height << 1;
      m_topRefLength  = cu.Y().width + area.width;
    }
    else   // if (cu.ispMode == VER_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height + area.height;
      m_topRefLength  = cu.Y().width << 1;
    }

    xFillReferenceSamples(cs.picture->getRecoBuf(cu.Y()), refBufUnfiltered, cu.Y(), cu);

    // After having retrieved all the CU reference samples, the number of reference samples is now adjusted for the
    // current subpartition
    m_topRefLength  = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;
  }
  else
  {
    m_topRefLength  = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;

    const int predSizeHor = m_topRefLength;
    const int predSizeVer = m_leftRefLength;
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      Pel *src = recBuf.bufAt(0, -1);
      Pel *ref = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + m_refBufferStride[area.compID];
      if (isLeftAvail)
      {
        for (int i = 0; i <= 2 * cu.blocks[area.compID].height - area.height; i++)
        {
          ref[i] = ref[i + area.height];
        }
      }
      else
      {
        for (int i = 0; i <= predSizeVer; i++)
        {
          ref[i] = src[0];
        }
      }
      Pel *dst = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + 1;
      dst[-1]  = ref[0];
      for (int i = 0; i < area.width; i++)
      {
        dst[i] = src[i];
      }
      Pel sample = src[area.width - 1];
      dst += area.width;
      for (int i = 0; i < predSizeHor - area.width; i++)
      {
        dst[i] = sample;
      }
    }
    else
    {
      Pel *src = recBuf.bufAt(-1, 0);
      Pel *ref = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
      if (isAboveAvail)
      {
        for (int i = 0; i <= 2 * cu.blocks[area.compID].width - area.width; i++)
        {
          ref[i] = ref[i + area.width];
        }
      }
      else
      {
        for (int i = 0; i <= predSizeHor; i++)
        {
          ref[i] = src[0];
        }
      }
      Pel *dst = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + m_refBufferStride[area.compID] + 1;
      dst[-1]  = ref[0];
      for (int i = 0; i < area.height; i++)
      {
        *dst = *src;
        src += recBuf.stride;
        dst++;
      }
      Pel sample = src[-recBuf.stride];
      for (int i = 0; i < predSizeVer - area.height; i++)
      {
        *dst = sample;
        dst++;
      }
    }
  }
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
    Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];
    xFilterReferenceSamples(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx);
  }
}

void IntraPrediction::xFillReferenceSamplesDECLIP(const CPelBuf &recoBuf, const CPelBuf &resiBuf, Pel *refBufUnfiltered,
                                                  const CompArea &area, const CodingUnit &cu, const TCoeff *coeff)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS &            sps    = *cs.sps;
  const PreCalcValues &  pcv    = *cs.pcv;

  const int multiRefIdx = (area.compID == COMPONENT_Y) ? cu.firstPU->multiRefIdx : 0;

  const int tuWidth              = area.width;
  const int tuHeight             = area.height;
  const int predSize             = m_topRefLength;
  const int predHSize            = m_leftRefLength;
  const int predStride           = predSize + 1 + multiRefIdx;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift   = pcv.noChroma2x2 && area.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  unitWidth = tuWidth <= 2 && cu.ispMode && isLuma(area.compID)
                          ? tuWidth
                          : pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int unitHeight =
    tuHeight <= 2 && cu.ispMode && isLuma(area.compID)
      ? tuHeight
      : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(tuWidth / unitWidth, 1);
  const int numLeftUnits       = std::max<int>(tuHeight / unitHeight, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int  numIntraNeighbor = 0;

  memset(neighborFlags, 0, totalUnits);

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable(cu, chType, posLT);
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor +=
    isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1));
  numIntraNeighbor += isAboveRightAvailable(cu, chType, posRT, numAboveRightUnits, unitWidth,
                                            (neighborFlags + totalLeftUnits + 1 + numAboveUnits));
  numIntraNeighbor +=
    isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
  numIntraNeighbor += isBelowLeftAvailable(cu, chType, posLB, numLeftBelowUnits, unitHeight,
                                           (neighborFlags + totalLeftUnits - 1 - numLeftUnits));

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel *srcBuf    = recoBuf.buf;
  const int  srcStride = recoBuf.stride;
  const Pel *resBuf    = resiBuf.buf;
  const int  resStride = resiBuf.stride;
  Pel *      ptrDst    = refBufUnfiltered;
  const Pel *ptrSrc;
  const Pel  valueDC = 1 << (sps.getBitDepth(chType) - 1);

  if (numIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if (numIntraNeighbor == totalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else   // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0]          = ptrSrc[0];
      ptrDst[predStride] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i]              = ptrSrc[i];
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) + predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst            = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit])   // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < multiRefIdx + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit =
            (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
  //以上是对块外参考像素的填充
  //接下来填充块内像素
  //为了方便滤波一行一列的操作，对于块内像素的填充顺序为先L型的行，再L型的列，接着下一环L型的行，下一环L型的列
  // ptrSrc =
  //  resBuf;   //从当前块的左上角第一个开始填充,但存储空间中由于第一行第一列放了块外的参考像素所以要从第二行第二列开始
  // int    k, l;
  // for (k = 1; k < tuHeight; k++)
  // {
  //   for (l = 1; l < tuWidth; l++)
  //   {
  //     //printf("residual = %d\n", coeff[k * tuWidth + l]);
  //     //amp_hevc=-1;
  //   }
  // }
  int offset = 0;   //一行一列参考像素所占用的位置
  // int maxsize = (tuWidth >= tuHeight) ? tuWidth : tuHeight;//表示当前块中L型的数量
  ptrDst = refBufUnfiltered + (predStride + predHSize + 1) * 2;   //指针指到refBufUnfiltered中填充完毕后一个空位
  for (int q = 0; q < (predHSize + 1); q++)
  {
    if (q < tuHeight)
    {
      for (int p = 0; p < predStride; p++)
      {
        if (p < tuWidth)
        {
          ptrDst[offset] = coeff[q * tuWidth + p];
          //printf("residual = %d\n", ptrDst[offset]);
          offset++;
        }
        else
        {
          ptrDst[offset] = ptrDst[offset - 1];   ////剩下位置全部复制最后一个有效值
          //printf("residual = %d\n", ptrDst[offset]);
          offset++;
        }
      }
    }
    else
    {
      for (int p = 0; p < predStride; p++)
      {
        ptrDst[offset] = ptrDst[offset - predStride];   //列的扩展填充为填充其正上方的那个值
        //printf("residual = %d\n", ptrDst[offset]);
        offset++;
      }
    }
  }
}


void IntraPrediction::xFillReferenceSamplesLIP(const CPelBuf &origBuf, Pel *refBufUnfiltered, const CompArea &area,
                                               const CodingUnit &cu)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS &            sps    = *cs.sps;
  const PreCalcValues &  pcv    = *cs.pcv;

  const int multiRefIdx = (area.compID == COMPONENT_Y) ? cu.firstPU->multiRefIdx : 0;

  const int tuWidth              = area.width;
  const int tuHeight             = area.height;
  const int predSize             = m_topRefLength;
  const int predHSize            = m_leftRefLength;
  const int predStride           = predSize + 1 + multiRefIdx;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift   = pcv.noChroma2x2 && area.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  unitWidth = tuWidth <= 2 && cu.ispMode && isLuma(area.compID)
                           ? tuWidth
                           : pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight =
    tuHeight <= 2 && cu.ispMode && isLuma(area.compID)
       ? tuHeight
       : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(tuWidth / unitWidth, 1);
  const int numLeftUnits       = std::max<int>(tuHeight / unitHeight, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int  numIntraNeighbor = 0;

  memset(neighborFlags, 0, totalUnits);

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable(cu, chType, posLT);
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor +=
    isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1));
  numIntraNeighbor += isAboveRightAvailable(cu, chType, posRT, numAboveRightUnits, unitWidth,
                                            (neighborFlags + totalLeftUnits + 1 + numAboveUnits));
  numIntraNeighbor +=
    isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
  numIntraNeighbor += isBelowLeftAvailable(cu, chType, posLB, numLeftBelowUnits, unitHeight,
                                           (neighborFlags + totalLeftUnits - 1 - numLeftUnits));

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel *srcBuf    = origBuf.buf;
  const int  srcStride = origBuf.stride;
  Pel *      ptrDst    = refBufUnfiltered;
  const Pel *ptrSrc;
  const Pel  valueDC = 1 << (sps.getBitDepth(chType) - 1);

  if (numIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if (numIntraNeighbor == totalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else   // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0]          = ptrSrc[0];
      ptrDst[predStride] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i]              = ptrSrc[i];
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) + predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst            = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit])   // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < multiRefIdx + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit =
            (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
 
  ptrSrc =
    srcBuf;   //
  int offset = 0;   //
  // int maxsize = (tuWidth >= tuHeight) ? tuWidth : tuHeight;//
  ptrDst = refBufUnfiltered + (predStride + predHSize + 1) * 2;   //
  for (int q = 0; q < (predHSize + 1); q++)
  {
    if (q < tuHeight)
    {
      for (int p = 0; p < predStride; p++)
      {
        if (p < tuWidth)
        {
          ptrDst[offset] = ptrSrc[q * srcStride + p];
          offset++;
        }
        else
        {
          ptrDst[offset] = ptrDst[offset - 1];   //
          offset++;
        }
      }
    }
    else
    {
      for (int p = 0; p < predStride; p++)
      {
        ptrDst[offset] = ptrDst[offset - predStride];   //
        offset++;
      }
    }
  }
}

void IntraPrediction::initIntraPatternChTypeDECLIP(TransformUnit &tu, const ComponentID compID, const CodingUnit &cu,
                                                   const CompArea &area, const bool forceRefFilterFlag)
{
  CHECK(area.width == 2, "Width of 2 is not supported");
  const CodingStructure &cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths(area);
  const TCoeff *coeff = tu.getCoeffs(compID).buf;
  // ----- Step 1: unfiltered reference samples -----
  // xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // xFillReferenceSamplesLIP(cs.picture->getOrigBuf(area), refBufUnfiltered, area, cu);
  xFillReferenceSamplesDECLIP(cs.picture->getRecoBuf(area), cs.picture->getResiBuf(area), refBufUnfiltered, area, cu,
                              coeff);
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    xFilterReferenceSamplesLIP(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx);
  }
}

void IntraPrediction::initIntraPatternChTypeLIP(const CodingUnit &cu, const CompArea &area,
                                                const bool forceRefFilterFlag)
{
  CHECK(area.width == 2, "Width of 2 is not supported");
  const CodingStructure &cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths(area);

  // ----- Step 1: unfiltered reference samples -----
  // xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  xFillReferenceSamplesLIP(cs.picture->getOrigBuf(area), refBufUnfiltered, area, cu);
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    xFilterReferenceSamplesLIP(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx);
  }
}

void IntraPrediction::xFillReferenceSamples(const CPelBuf &recoBuf, Pel *refBufUnfiltered, const CompArea &area,
                                            const CodingUnit &cu)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS &            sps    = *cs.sps;
  const PreCalcValues &  pcv    = *cs.pcv;

  const int multiRefIdx = (area.compID == COMPONENT_Y) ? cu.firstPU->multiRefIdx : 0;

  const int tuWidth              = area.width;
  const int tuHeight             = area.height;
  const int predSize             = m_topRefLength;
  const int predHSize            = m_leftRefLength;
  const int predStride           = predSize + 1 + multiRefIdx;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift   = pcv.noChroma2x2 && area.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  unitWidth = tuWidth <= 2 && cu.ispMode && isLuma(area.compID)
                           ? tuWidth
                           : pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight =
    tuHeight <= 2 && cu.ispMode && isLuma(area.compID)
       ? tuHeight
       : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(tuWidth / unitWidth, 1);
  const int numLeftUnits       = std::max<int>(tuHeight / unitHeight, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int  numIntraNeighbor = 0;

  memset(neighborFlags, 0, totalUnits);

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable(cu, chType, posLT);
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor +=
    isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1));
  numIntraNeighbor += isAboveRightAvailable(cu, chType, posRT, numAboveRightUnits, unitWidth,
                                            (neighborFlags + totalLeftUnits + 1 + numAboveUnits));
  numIntraNeighbor +=
    isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
  numIntraNeighbor += isBelowLeftAvailable(cu, chType, posLB, numLeftBelowUnits, unitHeight,
                                           (neighborFlags + totalLeftUnits - 1 - numLeftUnits));

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel *srcBuf    = recoBuf.buf;
  const int  srcStride = recoBuf.stride;
  Pel *      ptrDst    = refBufUnfiltered;
  const Pel *ptrSrc;
  const Pel  valueDC = 1 << (sps.getBitDepth(chType) - 1);

  if (numIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if (numIntraNeighbor == totalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else   // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0]          = ptrSrc[0];
      ptrDst[predStride] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i]              = ptrSrc[i];
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) + predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst            = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit])   // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < multiRefIdx + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit =
            (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
}

void IntraPrediction::xFilterReferenceSamplesLIP(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area,
                                                 const SPS &sps, int multiRefIdx)
{
  if (area.compID != COMPONENT_Y)
  {
    multiRefIdx = 0;
  }
  const int predSize     = m_topRefLength + multiRefIdx;    //
  const int predHSize    = m_leftRefLength + multiRefIdx;   //
  int       tuWidthtemp  = predSize >> 2;
  int       tuHeighttemp = predHSize >> 2;
  // int tuWidthtemp = tuWidth;
  // int tuHeighttemp = tuHeight;
  const size_t predStride = m_refBufferStride[area.compID];   //
  //
  const Pel topLeft =
    (refBufUnfiltered[0] + refBufUnfiltered[1] + refBufUnfiltered[predStride] + refBufUnfiltered[predStride + 1] + 2)
    >> 2;

  refBufFiltered[0] = topLeft;
  //
  for (int i = 1; i < predSize; i++)
  {   //[0.25, 0.5, 0.25]
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predSize] = refBufUnfiltered[predSize];   //
  //
  refBufFiltered += predStride;
  refBufUnfiltered += predStride;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predHSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predHSize] = refBufUnfiltered[predHSize];

  refBufFiltered += predHSize + 1;
  refBufUnfiltered += predHSize + 1;

  //
  int Lnumber = (tuWidthtemp >= tuHeighttemp) ? (tuHeighttemp - 1) : (tuWidthtemp - 1);   //
  // const int minshape = (tuWidth >= tuHeight) ? tuHeight : tuWidth;//
  // const int maxshape = (tuWidth >= tuHeight) ? tuWidth : tuHeight;//
  int predLStride = (int)predStride;

  for (int q = 0; q < Lnumber; q++)   //
  {
    const Pel topLeftL =
      (refBufUnfiltered[0] + refBufUnfiltered[1] + refBufUnfiltered[predStride] + refBufUnfiltered[2 * predStride] + 2)
      >> 2;
    refBufFiltered[0] = topLeftL;   //
    //
    for (int i = 1; i < predLStride; i++)
    {
      if (i < tuWidthtemp)
      {
        //[0.25, 0.5, 0.25]
        refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
      }
      else
      {
        refBufFiltered[i] = refBufUnfiltered[i];
      }
    }
    //
    for (int i = 0; i < predLStride * (int)predStride; i += (int)predStride)
    {
      if (i < tuHeighttemp * predStride)
      {
        if (i == 0)
        {
          continue;
        }
        else
        {   //[0.25, 0.5, 0.25]
          refBufFiltered[i] =
            (refBufUnfiltered[i - predStride] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + predStride] + 2) >> 2;
        }
      }
      else
      {
        refBufFiltered[i] = refBufUnfiltered[i];
      }
    }
    refBufFiltered += predStride + 1;
    refBufUnfiltered += predStride + 1;
    predLStride--;   //
    tuWidthtemp--;
    tuHeighttemp--;
  }
}

void IntraPrediction::xFilterReferenceSamples(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area,
                                              const SPS &sps, int multiRefIdx)
{
  if (area.compID != COMPONENT_Y)
  {
    multiRefIdx = 0;
  }
  const int    predSize   = m_topRefLength + multiRefIdx;
  const int    predHSize  = m_leftRefLength + multiRefIdx;
  const size_t predStride = m_refBufferStride[area.compID];

  const Pel topLeft =
    (refBufUnfiltered[0] + refBufUnfiltered[1] + refBufUnfiltered[predStride] + refBufUnfiltered[predStride + 1] + 2)
    >> 2;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predSize] = refBufUnfiltered[predSize];

  refBufFiltered += predStride;
  refBufUnfiltered += predStride;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predHSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predHSize] = refBufUnfiltered[predHSize];
}

bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure &cs     = *cu.cs;
  const Position         refPos = posLT.offset(-1, -1);

  if (!cs.isDecomp(refPos, chType))
  {
    return false;
  }

  return (cs.getCURestricted(refPos, cu, chType) != NULL);
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                     const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags)
{
  const CodingStructure &cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDx      = uiNumUnitsInPU * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                    const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags)
{
  const CodingStructure &cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDy      = uiNumUnitsInPU * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT,
                          const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags)
{
  const CodingStructure &cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDx      = uiNumUnitsInPU * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB,
                         const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags)
{
  const CodingStructure &cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDy      = uiNumUnitsInPU * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  int  iDstStride    = 0;
  Pel *pDst0         = 0;
  int  curChromaMode = pu.intraDir[1];
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    pDst0      = m_pMdlmTemp + iDstStride + 1;
  }
  else
  {
    iDstStride = MAX_CU_SIZE + 1;
    pDst0      = m_piTemp + iDstStride + 1;   // MMLM_SAMPLE_NEIGHBOR_LINES;
  }
  // assert 420 chroma subsampling
  CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(),
                               recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA,
                                          chromaArea.size()));   // needed for correct pos/size (4x4 Tus)

  CHECK(lumaArea.width == chromaArea.width && CHROMA_444 != pu.chromaFormat, "");
  CHECK(lumaArea.height == chromaArea.height && CHROMA_444 != pu.chromaFormat && CHROMA_422 != pu.chromaFormat, "");

  const SizeType uiCWidth  = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src           = pu.cs->picture->getRecoBuf(lumaArea);
  Pel const *   pRecSrc0      = Src.bufAt(0, 0);
  int           iRecStride    = Src.stride;
  int           logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
  int           logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);

  int iRecStride2 = iRecStride << logSubHeightC;

  const CodingUnit &lumaCU = isChroma(pu.chType) ? *pu.cs->picture->cs->getCU(lumaArea.pos(), CH_L) : *pu.cu;
  const CodingUnit &cu     = *pu.cu;

  const CompArea &area = isChroma(pu.chType) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = (1 << MIN_CU_LOG2);

  const int iUnitWidth  = iBaseUnitSize >> getComponentScaleX(area.compID, area.chromaFormat);
  const int iUnitHeight = iBaseUnitSize >> getComponentScaleY(area.compID, area.chromaFormat);

  const int iTUWidthInUnits     = uiTuWidth / iUnitWidth;
  const int iTUHeightInUnits    = uiTuHeight / iUnitHeight;
  const int iAboveUnits         = iTUWidthInUnits;
  const int iLeftUnits          = iTUHeightInUnits;
  const int chromaUnitWidth     = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int chromaUnitHeight    = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, area.chromaFormat);
  const int topTemplateSampNum  = 2 * uiCWidth;   // for MDLM, the number of template samples is 2W or 2H.
  const int leftTemplateSampNum = 2 * uiCHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  const int totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int totalLeftUnits  = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int totalUnits      = totalLeftUnits + totalAboveUnits + 1;
  const int aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int leftBelowUnits  = totalLeftUnits - iLeftUnits;

  int  avaiAboveRightUnits = 0;
  int  avaiLeftBelowUnits  = 0;
  bool bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);
  bool aboveIsAvailable, leftIsAvailable;

  int availlableUnit = isLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                       iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits + leftBelowUnits - 1));

  leftIsAvailable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                    iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + 1));

  aboveIsAvailable = availlableUnit == iTUWidthInUnits;

  if (leftIsAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                              area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight,
                                              (bNeighborFlags + leftBelowUnits - 1));
  }

  if (aboveIsAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                                area.topRightComp(area.compID), aboveRightUnits, iUnitWidth,
                                                (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }

  Pel *      pDst  = nullptr;
  Pel const *piSrc = nullptr;

  bool isFirstRowOfCtu = (lumaArea.y & ((pu.cs->sps)->getCTUSize() - 1)) == 0;

  if (aboveIsAvailable)
  {
    pDst                = pDst0 - iDstStride;
    int addedAboveRight = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedAboveRight = avaiAboveRightUnits * chromaUnitWidth;
    }
    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
    {
      const bool leftPadding = i == 0 && !leftIsAvailable;
      if (pu.chromaFormat == CHROMA_444)
      {
        piSrc   = pRecSrc0 - iRecStride;
        pDst[i] = piSrc[i];
      }
      else if (isFirstRowOfCtu)
      {
        piSrc   = pRecSrc0 - iRecStride;
        pDst[i] = (piSrc[2 * i] * 2 + piSrc[2 * i - (leftPadding ? 0 : 1)] + piSrc[2 * i + 1] + 2) >> 2;
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        piSrc = pRecSrc0 - iRecStride2;

        int s = 2;
        s += piSrc[2 * i] * 2;
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + 1];
        pDst[i] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        piSrc = pRecSrc0 - iRecStride2;

        int s = 4;
        s += piSrc[2 * i - iRecStride];
        s += piSrc[2 * i] * 4;
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + 1];
        s += piSrc[2 * i + iRecStride];
        pDst[i] = s >> 3;
      }
      else
      {
        piSrc = pRecSrc0 - iRecStride2;
        int s = 4;
        s += piSrc[2 * i] * 2;
        s += piSrc[2 * i + 1];
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + iRecStride] * 2;
        s += piSrc[2 * i + 1 + iRecStride];
        s += piSrc[2 * i + iRecStride - (leftPadding ? 0 : 1)];
        pDst[i] = s >> 3;
      }
    }
  }

  if (leftIsAvailable)
  {
    pDst  = pDst0 - 1;
    piSrc = pRecSrc0 - 1 - logSubWidthC;

    int addedLeftBelow = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedLeftBelow = avaiLeftBelowUnits * chromaUnitHeight;
    }

    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
    {
      if (pu.chromaFormat == CHROMA_444)
      {
        pDst[0] = piSrc[0];
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        int s = 2;
        s += piSrc[0] * 2;
        s += piSrc[-1];
        s += piSrc[1];
        pDst[0] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += piSrc[-(abovePadding ? 0 : iRecStride)];
        s += piSrc[0] * 4;
        s += piSrc[-1];
        s += piSrc[1];
        s += piSrc[iRecStride];
        pDst[0] = s >> 3;
      }
      else
      {
        int s = 4;
        s += piSrc[0] * 2;
        s += piSrc[1];
        s += piSrc[-1];
        s += piSrc[iRecStride] * 2;
        s += piSrc[iRecStride + 1];
        s += piSrc[iRecStride - 1];
        pDst[0] = s >> 3;
      }

      piSrc += iRecStride2;
      pDst += iDstStride;
    }
  }

  // inner part from reconstructed picture buffer
  for (int j = 0; j < uiCHeight; j++)
  {
    for (int i = 0; i < uiCWidth; i++)
    {
      if (pu.chromaFormat == CHROMA_444)
      {
        pDst0[i] = pRecSrc0[i];
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        const bool leftPadding = i == 0 && !leftIsAvailable;

        int s = 2;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        pDst0[i] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        const bool leftPadding  = i == 0 && !leftIsAvailable;
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i - (abovePadding ? 0 : iRecStride)];
        s += pRecSrc0[2 * i] * 4;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i + iRecStride];
        pDst0[i] = s >> 3;
      }
      else
      {
        CHECK(pu.chromaFormat != CHROMA_420, "Chroma format must be 4:2:0 for vertical filtering");
        const bool leftPadding = i == 0 && !leftIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + iRecStride] * 2;
        s += pRecSrc0[2 * i + 1 + iRecStride];
        s += pRecSrc0[2 * i + iRecStride - (leftPadding ? 0 : 1)];
        pDst0[i] = s >> 3;
      }
    }

    pDst0 += iDstStride;
    pRecSrc0 += iRecStride2;
  }
}
void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea,
                                       int &a, int &b, int &iShift)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure & cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth       = chromaArea.width;
  const uint32_t     tuHeight      = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleY(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits      = tuWidth / unitWidth;
  const int tuHeightInUnits     = tuHeight / unitHeight;
  const int aboveUnits          = tuWidthInUnits;
  const int leftUnits           = tuHeightInUnits;
  int       topTemplateSampNum  = 2 * cWidth;   // for MDLM, the template sample number is 2W or 2H;
  int       leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits     = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits      = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits          = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits     = totalAboveUnits - aboveUnits;
  int leftBelowUnits      = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits  = 0;
  int avaiAboveUnits      = 0;
  int avaiLeftUnits       = 0;

  int  curChromaMode = pu.intraDir[1];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit = isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth,
                                       (neighborFlags + leftUnits + leftBelowUnits + 1));
  aboveAvailable    = availableUnit == tuWidthInUnits;

  availableUnit = isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight,
                                  (neighborFlags + leftUnits + leftBelowUnits - 1));
  leftAvailable = availableUnit == tuHeightInUnits;
  if (leftAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftUnits      = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID),
                                              leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1));
  }
  if (aboveAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits =
      isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits,
                            unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1));
  }
  Pel *srcColor0, *curChroma0;
  int  srcStride;

  PelBuf temp;
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp      = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp      = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  srcColor0  = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  unsigned internalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int minLuma[2] = { MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };

  Pel *src                       = srcColor0 - srcStride;
  int  actualTopTemplateSampNum  = 0;
  int  actualLeftTemplateSampNum = 0;
  if (curChromaMode == MDLM_T_IDX)
  {
    leftAvailable            = 0;
    avaiAboveRightUnits      = avaiAboveRightUnits > (cHeight / unitWidth) ? cHeight / unitWidth : avaiAboveRightUnits;
    actualTopTemplateSampNum = unitWidth * (avaiAboveUnits + avaiAboveRightUnits);
  }
  else if (curChromaMode == MDLM_L_IDX)
  {
    aboveAvailable            = 0;
    avaiLeftBelowUnits        = avaiLeftBelowUnits > (cWidth / unitHeight) ? cWidth / unitHeight : avaiLeftBelowUnits;
    actualLeftTemplateSampNum = unitHeight * (avaiLeftUnits + avaiLeftBelowUnits);
  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    actualTopTemplateSampNum  = cWidth;
    actualLeftTemplateSampNum = cHeight;
  }
  int startPos[2];   // 0:Above, 1: Left
  int pickStep[2];

  int aboveIs4 = leftAvailable ? 0 : 1;
  int leftIs4  = aboveAvailable ? 0 : 1;

  startPos[0] = actualTopTemplateSampNum >> (2 + aboveIs4);
  pickStep[0] = std::max(1, actualTopTemplateSampNum >> (1 + aboveIs4));

  startPos[1] = actualLeftTemplateSampNum >> (2 + leftIs4);
  pickStep[1] = std::max(1, actualLeftTemplateSampNum >> (1 + leftIs4));

  Pel selectLumaPix[4]   = { 0, 0, 0, 0 };
  Pel selectChromaPix[4] = { 0, 0, 0, 0 };

  int cntT, cntL;
  cntT = cntL = 0;
  int cnt     = 0;
  if (aboveAvailable)
  {
    cntT           = std::min(actualTopTemplateSampNum, (1 + aboveIs4) << 1);
    src            = srcColor0 - srcStride;
    const Pel *cur = curChroma0 + 1;
    for (int pos = startPos[0]; cnt < cntT; pos += pickStep[0], cnt++)
    {
      selectLumaPix[cnt]   = src[pos];
      selectChromaPix[cnt] = cur[pos];
    }
  }

  if (leftAvailable)
  {
    cntL           = std::min(actualLeftTemplateSampNum, (1 + leftIs4) << 1);
    src            = srcColor0 - 1;
    const Pel *cur = curChroma0 + m_refBufferStride[compID] + 1;
    for (int pos = startPos[1], cnt = 0; cnt < cntL; pos += pickStep[1], cnt++)
    {
      selectLumaPix[cnt + cntT]   = src[pos * srcStride];
      selectChromaPix[cnt + cntT] = cur[pos];
    }
  }
  cnt = cntL + cntT;

  if (cnt == 2)
  {
    selectLumaPix[3]   = selectLumaPix[0];
    selectChromaPix[3] = selectChromaPix[0];
    selectLumaPix[2]   = selectLumaPix[1];
    selectChromaPix[2] = selectChromaPix[1];
    selectLumaPix[0]   = selectLumaPix[1];
    selectChromaPix[0] = selectChromaPix[1];
    selectLumaPix[1]   = selectLumaPix[3];
    selectChromaPix[1] = selectChromaPix[3];
  }

  int  minGrpIdx[2] = { 0, 2 };
  int  maxGrpIdx[2] = { 1, 3 };
  int *tmpMinGrp    = minGrpIdx;
  int *tmpMaxGrp    = maxGrpIdx;
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMinGrp[1]])
  {
    std::swap(tmpMinGrp[0], tmpMinGrp[1]);
  }
  if (selectLumaPix[tmpMaxGrp[0]] > selectLumaPix[tmpMaxGrp[1]])
  {
    std::swap(tmpMaxGrp[0], tmpMaxGrp[1]);
  }
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMaxGrp[1]])
  {
    std::swap(tmpMinGrp, tmpMaxGrp);
  }
  if (selectLumaPix[tmpMinGrp[1]] > selectLumaPix[tmpMaxGrp[0]])
  {
    std::swap(tmpMinGrp[1], tmpMaxGrp[0]);
  }

  minLuma[0] = (selectLumaPix[tmpMinGrp[0]] + selectLumaPix[tmpMinGrp[1]] + 1) >> 1;
  minLuma[1] = (selectChromaPix[tmpMinGrp[0]] + selectChromaPix[tmpMinGrp[1]] + 1) >> 1;
  maxLuma[0] = (selectLumaPix[tmpMaxGrp[0]] + selectLumaPix[tmpMaxGrp[1]] + 1) >> 1;
  maxLuma[1] = (selectChromaPix[tmpMaxGrp[0]] + selectChromaPix[tmpMaxGrp[1]] + 1) >> 1;

  if (leftAvailable || aboveAvailable)
  {
    int diff = maxLuma[0] - minLuma[0];
    if (diff > 0)
    {
      int                  diffC               = maxLuma[1] - minLuma[1];
      int                  x                   = floorLog2(diff);
      static const uint8_t DivSigTable[1 << 4] = { // 4bit significands - 8 ( MSB is omitted )
                                                   0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0
      };
      int normDiff = (diff << 4 >> x) & 15;
      int v        = DivSigTable[normDiff] | 8;
      x += normDiff != 0;

      int y   = floorLog2(abs(diffC)) + 1;
      int add = 1 << y >> 1;
      a       = (diffC * v + add) >> y;
      iShift  = 3 + x - y;
      if (iShift < 1)
      {
        iShift = 1;
        a      = ((a == 0) ? 0 : (a < 0) ? -15 : 15);   // a=Sign(a)*15
      }
      b = minLuma[1] - ((a * minLuma[0]) >> iShift);
    }
    else
    {
      a      = 0;
      b      = minLuma[1];
      iShift = 0;
    }
  }
  else
  {
    a = 0;

    b = 1 << (internalBitDepth - 1);

    iShift = 0;
  }
}

void IntraPrediction::initIntraMip(const PredictionUnit &pu, const CompArea &area)
{
  CHECK(area.width > MIP_MAX_WIDTH || area.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP");

  // prepare input (boundary) data for prediction
  CHECK(m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP");
  Pel *     ptrSrc     = getPredictorPtr(area.compID);
  const int srcStride  = m_refBufferStride[area.compID];
  const int srcHStride = 2;

  m_matrixIntraPred.prepareInputForPred(CPelBuf(ptrSrc, srcStride, srcHStride), area,
                                        pu.cu->slice->getSPS()->getBitDepth(toChannelType(area.compID)), area.compID);
}

void IntraPrediction::predIntraMip(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
{
  CHECK(piPred.width > MIP_MAX_WIDTH || piPred.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP");
  CHECK(piPred.width != (1 << floorLog2(piPred.width)) || piPred.height != (1 << floorLog2(piPred.height)),
        "Error: expecting blocks of size 2^M x 2^N");

  // generate mode-specific prediction
  uint32_t modeIdx       = MAX_NUM_MIP_MODE;
  bool     transposeFlag = false;
  if (compId == COMPONENT_Y)
  {
    modeIdx       = pu.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = pu.mipTransposedFlag;
  }
  else
  {
    const PredictionUnit &coLocatedLumaPU = PU::getCoLocatedLumaPU(pu);

    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "Error: MIP is only supported for chroma with DM_CHROMA.");
    CHECK(!coLocatedLumaPU.cu->mipFlag, "Error: Co-located luma CU should use MIP.");

    modeIdx       = coLocatedLumaPU.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = coLocatedLumaPU.mipTransposedFlag;
  }
  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth(toChannelType(compId));

  CHECK(modeIdx >= getNumModesMip(piPred), "Error: Wrong MIP mode index");

  static_vector<int, MIP_MAX_WIDTH * MIP_MAX_HEIGHT> predMip(piPred.width * piPred.height);
  m_matrixIntraPred.predBlock(predMip.data(), modeIdx, transposeFlag, bitDepth, compId);

  for (int y = 0; y < piPred.height; y++)
  {
    for (int x = 0; x < piPred.width; x++)
    {
      piPred.at(x, y) = Pel(predMip[y * piPred.width + x]);
    }
  }
}

void IntraPrediction::reorderPLT(CodingStructure &cs, Partitioner &partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);

  uint8_t reusePLTSizetmp = 0;
  uint8_t pltSizetmp      = 0;
  Pel     curPLTtmp[MAX_NUM_COMPONENT][MAXPLTSIZE];
  bool    curPLTpred[MAXPLTPREDSIZE];

  for (int idx = 0; idx < MAXPLTPREDSIZE; idx++)
  {
    curPLTpred[idx]              = false;
    cu.reuseflag[compBegin][idx] = false;
  }
  for (int idx = 0; idx < MAXPLTSIZE; idx++)
  {
    curPLTpred[idx] = false;
  }

  for (int predidx = 0; predidx < cs.prevPLT.curPLTSize[compBegin]; predidx++)
  {
    bool match  = false;
    int  curidx = 0;

    for (curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
    {
      if (curPLTpred[curidx])
      {
        continue;
      }
      bool matchTmp = true;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        matchTmp = matchTmp && (cu.curPLT[comp][curidx] == cs.prevPLT.curPLT[comp][predidx]);
      }
      if (matchTmp)
      {
        match = true;
        break;
      }
    }

    if (match)
    {
      cu.reuseflag[compBegin][predidx] = true;
      curPLTpred[curidx]               = true;
      if (cu.isLocalSepTree())
      {
        cu.reuseflag[COMPONENT_Y][predidx] = true;
        for (int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++)
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      else
      {
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      reusePLTSizetmp++;
      pltSizetmp++;
    }
  }
  cu.reusePLTSize[compBegin] = reusePLTSizetmp;
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (!curPLTpred[curidx])
    {
      if (cu.isLocalSepTree())
      {
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
        }
        if (isLuma(partitioner.chType))
        {
          curPLTtmp[COMPONENT_Cb][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
          curPLTtmp[COMPONENT_Cr][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
        }
        else
        {
          curPLTtmp[COMPONENT_Y][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
        }
      }
      else
      {
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
        }
      }
      pltSizetmp++;
    }
  }
  assert(pltSizetmp == cu.curPLTSize[compBegin]);
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (cu.isLocalSepTree())
    {
      for (int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++)
      {
        cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
      }
    }
    else
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
      }
    }
  }
}


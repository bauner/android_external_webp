# Copyright 2010 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
        alpha.c \
        analysis.c \
        backward_references.c \
        config.c \
        cost.c \
        filter.c \
        frame.c\
        histogram.c \
        iterator.c \
        near_lossless.c \
        picture.c \
        picture_csp.c \
        picture_psnr.c \
        picture_rescale.c \
        picture_tools.c \
        quant.c \
        syntax.c \
        tree.c \
        token.c \
        vp8l.c \
        webpenc.c \
        ../dsp/alpha_processing.c \
        ../dsp/alpha_processing_mips_dsp_r2.c \
        ../dsp/alpha_processing_sse2.c \
        ../dsp/cpu.c \
        ../dsp/cpu-features.c \
        ../dsp/enc.c \
        ../dsp/enc_avx2.c \
        ../dsp/enc_mips32.c \
        ../dsp/enc_neon.c \
        ../dsp/enc_sse2.c \
        ../dsp/filters_mips_dsp_r2.c \
        ../dsp/lossless.c \
        ../dsp/lossless_mips32.c \
        ../dsp/lossless_neon.c \
        ../dsp/lossless_sse2.c \
        ../dsp/yuv.c \
        ../dsp/yuv_mips32.c \
        ../dsp/yuv_mips_dsp_r2.c \
        ../dsp/yuv_sse2.c \
        ../utils/bit_writer.c \
        ../utils/color_cache.c \
        ../utils/filters.c \
        ../utils/huffman.c \
        ../utils/huffman_encode.c \
        ../utils/quant_levels.c \
        ../utils/rescaler.c \
        ../utils/random.c \
        ../utils/thread.c \
        ../utils/utils.c

LOCAL_CFLAGS := -DANDROID -DWEBP_SWAP_16BIT_CSP

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH) \
        $(LOCAL_PATH)/../../include

LOCAL_MODULE:= libwebp-encode

include $(BUILD_STATIC_LIBRARY)

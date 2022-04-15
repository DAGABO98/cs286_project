/// @file
/// @author David Pilger <dpilger26@gmail.com>
/// [GitHub Repository](https://github.com/dpilger26/NumCpp)
///
/// License
/// Copyright 2020 David Pilger
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this
/// software and associated documentation files(the "Software"), to deal in the Software
/// without restriction, including without limitation the rights to use, copy, modify,
/// merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
/// permit persons to whom the Software is furnished to do so, subject to the following
/// conditions :
///
/// The above copyright notice and this permission notice shall be included in all copies
/// or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
/// PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
/// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
/// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
/// DEALINGS IN THE SOFTWARE.
///
/// Description
/// Calculate the 2D laplace filter.
///
#pragma once

#include "NumCpp/Filter/Filters/Filters2d/convolve.hpp"
#include "NumCpp/NdArray.hpp"

namespace nc
{
    namespace filter
    {
        //============================================================================
        // Method Description:
        ///						Calculate the 2D laplace filter.
        ///
        ///                     SciPy Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.laplace.html#scipy.ndimage.laplace
        ///
        /// @param				inImageArray
        /// @param              inBoundaryType: boundary mode (default Reflect) options (reflect, constant, nearest, mirror, wrap)
        /// @param				inConstantValue: contant value if boundary = 'constant' (default 0)
        /// @return
        ///				NdArray
        ///
        template<typename dtype>
        NdArray<dtype> laplace(const NdArray<dtype>& inImageArray, Boundary inBoundaryType = Boundary::REFLECT, dtype inConstantValue = 0)
        {
            NdArray<dtype> kernel = { {0, 1, 0}, {1, -4, 1}, {0, 1, 0} };
            return convolve(inImageArray, 3, kernel, inBoundaryType, inConstantValue);
        }
    } // namespace filter
} // namespace nc

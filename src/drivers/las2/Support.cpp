/******************************************************************************
* Copyright (c) 2014, Howard Butler (hobu.inc@gmail.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/drivers/las2/Support.hpp>
#include <pdal/drivers/las2/las.hpp>

namespace pdal
{
namespace drivers
{
namespace las2
{

void setScaling(Schema& schema,
                double scaleX,
                double scaleY,
                double scaleZ,
                double offsetX,
                double offsetY,
                double offsetZ)
{
    Dimension dimX = schema.getDimension("X");
    Dimension dimY = schema.getDimension("Y");
    Dimension dimZ = schema.getDimension("Z");

    dimX.setNumericScale(scaleX);
    dimY.setNumericScale(scaleY);
    dimZ.setNumericScale(scaleZ);

    dimX.setNumericOffset(offsetX);
    dimY.setNumericOffset(offsetY);
    dimZ.setNumericOffset(offsetZ);

    schema.setDimension(dimX);
    schema.setDimension(dimY);
    schema.setDimension(dimZ);

    return;
}

// Schema PointFormat::getSchema(::las::las_file const& l) const
// {
//     getDimensions()
// }

std::vector<pdal::Dimension> PointFormat::getDimensions( Stage const& stage)
{
    std::vector<pdal::Dimension> output;
    
    std::vector<pdal::Dimension> const& d = stage.getDefaultDimensions();

    Schema dimensions(d);

    output.push_back(dimensions.getDimension("X", stage.getName()));
    output.push_back(dimensions.getDimension("Y", stage.getName()));
    output.push_back(dimensions.getDimension("Z", stage.getName()));

    output.push_back(dimensions.getDimension("Intensity", stage.getName()));
    output.push_back(dimensions.getDimension("ReturnNumber", stage.getName())); // 3 bits only
    output.push_back(dimensions.getDimension("NumberOfReturns", stage.getName())); // 3 bits only
    output.push_back(dimensions.getDimension("ScanDirectionFlag", stage.getName()));  // 1 bit only
    output.push_back(dimensions.getDimension("EdgeOfFlightLine", stage.getName())); // 1 bit only

    output.push_back(dimensions.getDimension("Classification", stage.getName()));
    
    if (m_id > PointFormat5)
        output.push_back(dimensions.getDimension("ScanAngleRank2", stage.getName()));
    else
        output.push_back(dimensions.getDimension("ScanAngleRank", stage.getName()));
        
    output.push_back(dimensions.getDimension("UserData", stage.getName()));
    output.push_back(dimensions.getDimension("PointSourceId", stage.getName()));

    if (hasTime())
    {
        output.push_back(dimensions.getDimension("Time", stage.getName()));
    }

    if (hasRGB())
    {
        output.push_back(dimensions.getDimension("Red", stage.getName()));
        output.push_back(dimensions.getDimension("Green", stage.getName()));
        output.push_back(dimensions.getDimension("Blue", stage.getName()));
    }

    if (hasNIR())
    {
        output.push_back(dimensions.getDimension("NIR", stage.getName()));
    }

    if (hasWave())
    {
        output.push_back(dimensions.getDimension("WavePacketDescriptorIndex", stage.getName()));
        output.push_back(dimensions.getDimension("WaveformDataOffset", stage.getName()));
        output.push_back(dimensions.getDimension("ReturnPointWaveformLocation", stage.getName()));
        output.push_back(dimensions.getDimension("WaveformXt", stage.getName()));
        output.push_back(dimensions.getDimension("WaveformYt", stage.getName()));
        output.push_back(dimensions.getDimension("WaveformZt", stage.getName()));
    
    }

    return output;
}

bool PointFormat::hasRGB() const
{
    return ( (m_id == PointFormat2) 
        || (m_id == PointFormat3) 
        || (m_id == PointFormat5) 
        || (m_id == PointFormat7)
        || (m_id == PointFormat8)
            );
}

bool PointFormat::hasNIR() const
{
    return ( m_id == PointFormat7)
        || ( m_id == PointFormat8)
        || ( m_id == PointFormat10)
            ;
}

bool PointFormat::hasTime() const
{
    return (m_id == PointFormat1) 
        || (m_id == PointFormat3) 
        || (m_id == PointFormat4) 
        || (m_id == PointFormat5)
        || (m_id == PointFormat6)
        || (m_id == PointFormat7)
        || (m_id == PointFormat8)
        || (m_id == PointFormat9)
        || (m_id == PointFormat10)
            ;
}

bool PointFormat::hasWave() const
{
    return (m_id == PointFormat4) 
        || (m_id == PointFormat5)
        || (m_id == PointFormat9)
        || (m_id == PointFormat10)
            ;
}


} // las2
} // drivers
} // pdal

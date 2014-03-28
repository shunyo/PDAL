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

#ifndef INCLUDED_DRIVERS_LAS2_SUPPORT_HPP
#define INCLUDED_DRIVERS_LAS2_SUPPORT_HPP

#include <pdal/pdal_export.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Schema.hpp>
#include <string>
#include <vector>

namespace pdal
{
    
namespace drivers
{
namespace las2
{
    // setScaling(  Schema& schema,
    //              double scaleX,
    //              double scaleY,
    //              double scaleZ,
    //              double offsetX,
    //              double offsetY,
    //              double offsetZ);

    enum Id
    {
        PointFormat0 = 0,         // base
        PointFormat1 = 1,         // base + time
        PointFormat2 = 2,         // base + color
        PointFormat3 = 3,         // base + time + color
        PointFormat4 = 4,         // base + time + wave
        PointFormat5 = 5,         // LAS 1.4
        PointFormat6 = 6,         // LAS 1.4
        PointFormat7 = 7,         // LAS 1.4
        PointFormat8 = 8,         // LAS 1.4
        PointFormat9 = 9,         // LAS 1.4
        PointFormat10 = 10,         // LAS 1.4
        PointFormatUnknown = 99
    };

    class PointFormat
    {
        PointFormat(Id id)
            : m_id(id) {}

        std::vector<Dimension> getDimensions( Stage const& stage);    
        
        // Schema getSchema(::las::las_file const& l) const;
        bool hasRGB() const;
        bool hasTime() const;
        bool hasWave() const;
        bool hasNIR() const;
        
    private:
        Id m_id;
    };
    



    class PDAL_DLL PointDimensions
    {
    public:
        PointDimensions(const Schema& schema, std::string const& ns);

        Dimension const* X;
        Dimension const* Y;
        Dimension const* Z;

        Dimension const* Intensity;
        Dimension const* ReturnNumber;
        Dimension const* NumberOfReturns;
        Dimension const* ScanDirectionFlag;
        Dimension const* EdgeOfFlightLine;
        Dimension const* Classification;
        Dimension const* ScanAngleRank; // 1 byte wide
        Dimension const* ScanAngleRank2; // 2 bytes wide 
        Dimension const* UserData;
        Dimension const* PointSourceId;

        Dimension const* Time;

        Dimension const* Red;
        Dimension const* Green;
        Dimension const* Blue;
        Dimension const* NIR;

        Dimension const* WavePacketDescriptor;
        Dimension const* WavePacketOffset;
        Dimension const* WaveReturnPoint;

        Dimension const* XT;
        Dimension const* YT;
        Dimension const* ZT;


    };

} // las2
} // drivers
} // pdal

#endif
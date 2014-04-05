/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include <pdal/drivers/pcd/Reader.hpp>

#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/PCLConversions.hpp>
#include <pdal/drivers/pcd/point_types.hpp>

namespace pdal
{
namespace drivers
{
namespace pcd
{

Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
    setSchema(Schema(getDefaultDimensions()));
}

Reader::~Reader()
{}

void Reader::initialize()
{
    pdal::Reader::initialize();

    pcl::PCLPointCloud2 cloud;
    pcl::PCDReader r;
    r.readHeader(getFileName(), cloud);
    setNumPoints(cloud.height * cloud.width);
}

Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}

std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    Dimension x("X", dimension::Float, 4);
    x.setUUID("6aedb002-d2b6-4480-a307-91dbe2ec61cc");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension y("Y", dimension::Float, 4);
    y.setUUID("e5a37966-566b-4010-855a-2a51fe73f2ee");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension z("Z", dimension::Float, 4);
    z.setUUID("bda63c52-d01d-490d-9543-c574e2ea914c");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension i("Intensity", dimension::Float, 4);
    i.setUUID("27e447ec-5763-4fd9-b183-ee6e009bd9f3");
    i.setNamespace(s_getName());
    output.push_back(i);

    Dimension r("Red", dimension::UnsignedInteger, 1);
    r.setUUID("b49ed2f7-7277-4f88-8534-cb452a739f59");
    r.setNamespace(s_getName());
    output.push_back(r);

    Dimension g("Green", dimension::UnsignedInteger, 1);
    g.setUUID("5ae93242-a4fe-4595-b002-8606dfa695da");
    g.setNamespace(s_getName());
    output.push_back(g);

    Dimension b("Blue", dimension::UnsignedInteger, 1);
    b.setUUID("86d2ae38-e5a5-4c5e-ab2f-8213193c9115");
    b.setNamespace(s_getName());
    output.push_back(b);

    return output;
}

std::string Reader::getFileName() const
{
    return getOptions().getOption("filename").getValue<std::string>();
}

pdal::StageSequentialIterator*
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::pcd::iterators::sequential::Iterator(*this, buffer);
}

namespace iterators
{
namespace sequential
{

Iterator::Iterator(pdal::drivers::pcd::Reader const& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(buffer)
    , m_filename(reader.getFileName())
    , m_numPoints(reader.getNumPoints())
    , m_schema(reader.getSchema())
    , m_buffer(pdal::PointBuffer(m_schema, m_numPoints))
{}

Iterator::~Iterator()
{}

boost::uint64_t Iterator::skipImpl(boost::uint64_t count)
{
    return count;
}

bool Iterator::atEndImpl() const
{
    return getIndex() >= m_numPoints;
}

boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    pcl::PointCloud<XYZIRGBA>::Ptr cloud(new pcl::PointCloud<XYZIRGBA>);

    pcl::PCDReader r;
    r.read<XYZIRGBA>(m_filename, *cloud);

    pdal::PCDtoPDAL(*cloud, data);

    boost::uint32_t numPoints = cloud->points.size();

    data.setNumPoints(numPoints);

    return numPoints;
}

} // sequential
} // iterators

}
}
} // namespaces


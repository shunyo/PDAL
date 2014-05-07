/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/drivers/pipeline/Reader.hpp>

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineWriter.hpp>

namespace pdal
{
namespace drivers
{
namespace pipeline
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_filename(options.getValueOrThrow<std::string>("filename"))
    , m_manager(NULL)
{
    return;
}


Reader::~Reader()
{
    m_manager.reset();
    return;
}


void Reader::initialize()
{
    boost::scoped_ptr<PipelineManager> tmp(new PipelineManager());
    m_manager.swap(tmp);

    PipelineReader xmlreader(*m_manager);
    bool isWriter = xmlreader.readPipeline(m_filename);
    if (isWriter)
    {
        m_manager->removeWriter();
    }
    m_stage = m_manager->getStage();
    m_stage->prepare();

    setSchema(m_stage->getSchema());

    setNumPoints(m_stage->getNumPoints());
    setBounds(m_stage->getBounds());
    setSpatialReference(m_stage->getSpatialReference());
}


Options Reader::getDefaultOptions()
{
    return Options();
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return m_stage->createSequentialIterator(buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return m_stage->createRandomIterator(buffer);
}

Metadata Reader::getMetadata() const
{
    return m_stage->getMetadata();
}

boost::property_tree::ptree Reader::serializePipeline() const
{
    return m_stage->serializePipeline();
}

}
}
} // namespaces

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

#include <boost/test/unit_test.hpp>

#include "Support.hpp"

#include <pdal/Options.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/pipeline/Reader.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PipelineReaderTest)


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test1)
{
    PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(Support::datapath("pipeline/pipeline_read.xml"));
    Stage* stage = manager.getStage();
    BOOST_CHECK(stage != NULL);
    stage->prepare();

    {
        const Schema& schema = stage->getSchema();
        PointBuffer data(schema, 2048);
        StageSequentialIterator* iter = stage->createSequentialIterator(data);
        boost::uint32_t np = iter->read(data);
        BOOST_CHECK_EQUAL(np, 1065u);

        delete iter;
    }
}


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test2)
{
#ifndef PDAL_HAVE_LASZIP
    return;
#endif

    FileUtils::deleteFile("pipeline/pdal-compressed.laz");

    {
        PipelineManager manager;
        PipelineReader reader(manager);

        reader.readPipeline(Support::datapath("pipeline/pipeline_write.xml"));
        Writer* writerStage = manager.getWriter();
        writerStage->prepare();

        const boost::uint64_t np = writerStage->write();
        BOOST_CHECK_EQUAL(np, 1065u);
    }

    FileUtils::deleteFile(Support::datapath("pipeline/pdal-compressed.laz"));
}


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test3)
{
    // missing Type
    PipelineManager manager01;
    PipelineReader reader01(manager01);
    BOOST_CHECK_THROW(reader01.readPipeline(Support::datapath("pipeline/bad/pipeline_bad01.xml")), pipeline_xml_error);

    // missing child of filter
    PipelineManager manager02;
    PipelineReader reader02(manager02);
    BOOST_CHECK_THROW(reader02.readPipeline(Support::datapath("pipeline/bad/pipeline_bad02.xml")), pipeline_xml_error);

    // missing child of multifilter
    PipelineManager manager03;
    PipelineReader reader03(manager03);
    BOOST_CHECK_THROW(reader03.readPipeline(Support::datapath("pipeline/bad/pipeline_bad03.xml")), pipeline_xml_error);

    // missing child of writer
    PipelineManager manager04;
    PipelineReader reader04(manager04);
    BOOST_CHECK_THROW(reader04.readPipeline(Support::datapath("pipeline/bad/pipeline_bad04.xml")), pipeline_xml_error);

    // extra child of filter
    PipelineManager manager05;
    PipelineReader reader05(manager05);
    BOOST_CHECK_THROW(reader05.readPipeline(Support::datapath("pipeline/bad/pipeline_bad05.xml")), pipeline_xml_error);

    // extra child of writer
    PipelineManager manager06;
    PipelineReader reader06(manager06);
    BOOST_CHECK_THROW(reader06.readPipeline(Support::datapath("pipeline/bad/pipeline_bad06.xml")), pipeline_xml_error);

    // child of reader
    PipelineManager manager07;
    PipelineReader reader07(manager07);
    BOOST_CHECK_THROW(reader07.readPipeline(Support::datapath("pipeline/bad/pipeline_bad07.xml")), pipeline_xml_error);

    // unknown element
    PipelineManager manager08;
    PipelineReader reader08(manager08);
    BOOST_CHECK_THROW(reader08.readPipeline(Support::datapath("pipeline/bad/pipeline_bad08.xml")), pipeline_xml_error);

    // no Pipeline for writer xml
    PipelineManager manager09;
    PipelineReader reader09(manager09);
    BOOST_CHECK_THROW(reader08.readPipeline(Support::datapath("pipeline/bad/pipeline_bad09.xml")), pipeline_xml_error);

    // no Pipeline for reader xml
    PipelineManager manager10;
    PipelineReader reader10(manager10);
    BOOST_CHECK_THROW(reader10.readPipeline(Support::datapath("pipeline/bad/pipeline_bad10.xml")), pipeline_xml_error);

    // try to make a reader pipeline from a writer pipeline xml file
    PipelineManager managerW;
    PipelineReader readerW(managerW);
    readerW.readPipeline(Support::datapath("pipeline/pipeline_write.xml"));
    BOOST_CHECK(managerW.isWriterPipeline());

    // try to make a writer pipeline from a reader pipeline xml file
    PipelineManager managerR;
    PipelineReader readerR(managerR);
    readerR.readPipeline(Support::datapath("pipeline/pipeline_read.xml"));
    BOOST_CHECK(!managerR.isWriterPipeline());

    // comments are ok
    PipelineManager managerComments1;
    PipelineReader readerComments1(managerComments1);
    BOOST_CHECK_NO_THROW(readerComments1.readPipeline(Support::datapath("pipeline/pipeline_readcomments.xml")));

    PipelineManager managerComments2;
    PipelineReader readerComments2(managerComments2);
    BOOST_CHECK_NO_THROW(readerComments2.readPipeline(Support::datapath("pipeline/pipeline_writecomments.xml")));
}


#ifdef PDAL_HAVE_GDAL
BOOST_AUTO_TEST_CASE(PipelineReaderTest_test4)
{
    FileUtils::deleteFile("pipeline/out2.las");

    {
        PipelineManager manager;
        PipelineReader reader(manager);

        reader.readPipeline(Support::datapath("pipeline/pipeline_write2.xml"));
        Writer* writerStage = manager.getWriter();
        writerStage->prepare();

        const boost::uint64_t np = writerStage->write();
        BOOST_CHECK(np == 1);
    }

    const double preX = 470692.447538;
    const double preY = 4602888.904642;
    const double preZ = 16.000000;
    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;
    {
        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));
        reader.prepare();
        const pdal::Bounds<double>& bounds = reader.getBounds();
        const pdal::Bounds<double> ref(preX, preY, preZ, preX, preY, preZ);
        Support::compareBounds(bounds, ref);
    }
    {
        pdal::drivers::las::Reader reader(Support::datapath("pipeline/out2.las"));
        reader.prepare();
        const pdal::Bounds<double>& bounds = reader.getBounds();
        const pdal::Bounds<double> ref(postX, postY, postZ, postX, postY, postZ);
        Support::compareBounds(bounds, ref);
    }

    FileUtils::deleteFile(Support::datapath("pipeline/out2.las"));
}
#endif

BOOST_AUTO_TEST_CASE(PipelineReaderTest_Reader)
{
    Option option("filename", Support::datapath("pipeline/pipeline_read.xml"));
    Options options(option);

    pdal::drivers::pipeline::Reader reader(options);

    reader.prepare();

    {
        const Schema& schema = reader.getSchema();
        PointBuffer data(schema, 2048);
        StageSequentialIterator* iter = reader.createSequentialIterator(data);
        boost::uint32_t np = iter->read(data);
        BOOST_CHECK(np == 1065);

        delete iter;
    }
}

BOOST_AUTO_TEST_CASE(PipelineReaderTest_MultiOptions)
{
    Option option("filename", Support::datapath("pipeline/pipeline_multioptions.xml"));
    Options options(option);

    pdal::drivers::pipeline::Reader reader(options);

    reader.prepare();

    // .getStage gets us the filter stage.  .getPrevStage gets us the reader
    // we wrote our extra options on the reader stage.

    Stage const& stage = reader.getManager().getStage()->getPrevStage();
    Options const& opt = stage.getOptions();

    Option fname = opt.getOption("filename");

    boost::optional<Options const&> more = fname.getOptions();

    Option meaning_of_life = more->getOption("somemore");

    BOOST_CHECK_EQUAL(meaning_of_life.getValue<int>(), 42);
}


BOOST_AUTO_TEST_CASE(testNoType)
{
    PipelineManager manager;
    PipelineReader reader(manager);
    reader.readPipeline(Support::datapath("pipeline/pipeline_read_notype.xml"));
}


BOOST_AUTO_TEST_SUITE_END()

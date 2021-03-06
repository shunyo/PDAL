/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>

#include <pdal/drivers/icebridge/Reader.hpp>

#include "Support.hpp"

void checkDimension(
        const pdal::PointBuffer& data,
        const std::size_t index,
        const std::string& name,
        const float expected)
{
    const pdal::Dimension& dimension = data.getSchema().getDimension(name);
    float actual = data.getField<float>(dimension, index);
    BOOST_CHECK_CLOSE(expected, actual, 0.000001);
}

void checkDimension(
        const pdal::PointBuffer& data,
        const std::size_t index,
        const std::string& name,
        const int expected)
{
    const pdal::Dimension& dimension = data.getSchema().getDimension(name);
    int actual = data.getField<int>(dimension, index);
    BOOST_CHECK_EQUAL(expected, actual);
}

void checkPoint(
        const pdal::PointBuffer& data,
        std::size_t index,
        float time,
        float latitude,
        float longitude,
        float elevation,
        int xmtSig,
        int rcvSig,
        float azimuth,
        float pitch,
        float roll,
        float gpsPdop,
        float pulseWidth,
        float relTime)
{
    checkDimension(data, index, "Time", time);
    checkDimension(data, index, "Y", latitude);
    checkDimension(data, index, "X", longitude);
    checkDimension(data, index, "Z", elevation);
    checkDimension(data, index, "StartPulse", xmtSig);
    checkDimension(data, index, "ReflectedPulse", rcvSig);
    checkDimension(data, index, "ScanAngleRank", azimuth);
    checkDimension(data, index, "Pitch", pitch);
    checkDimension(data, index, "Roll", roll);
    checkDimension(data, index, "PDOP", gpsPdop);
    checkDimension(data, index, "PulseWidth", pulseWidth);
    checkDimension(data, index, "GpsTime", relTime);
}

std::string getFilePath()
{
    return Support::datapath("icebridge/twoPoints.h5");
}

BOOST_AUTO_TEST_SUITE(IcebridgeReaderTest)

BOOST_AUTO_TEST_CASE(testConstructor)
{
    pdal::Option filename("filename", getFilePath(), "");
    pdal::Options options(filename);
    pdal::drivers::icebridge::Reader reader(options);

    BOOST_CHECK(reader.getDescription() == "Icebridge Reader");
    BOOST_CHECK_EQUAL(reader.getName(), "drivers.icebridge.reader");

    reader.initialize();

    BOOST_CHECK_EQUAL(reader.getNumPoints(), 2);
}

BOOST_AUTO_TEST_CASE(testRead)
{
    pdal::Option filename("filename", getFilePath(), "");
    pdal::Options options(filename);
    pdal::drivers::icebridge::Reader reader(options);

    reader.initialize();

    const pdal::Schema& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 2);

    boost::scoped_ptr<pdal::StageSequentialIterator> it(
            reader.createSequentialIterator(data));
    BOOST_CHECK(!it->atEnd());

    const boost::uint32_t numRead = it->read(data);
    BOOST_CHECK_EQUAL(numRead, 2);
    BOOST_CHECK(it->atEnd());

    checkPoint(
            data,
            0,
            141437.548,     // time
            82.605319,      // latitude
            301.406196,     // longitude
            18.678,         // elevation
            2408,           // xmtSig
            181,            // rcvSig
            49.91,          // azimuth
            -4.376,         // pitch
            0.608,          // roll
            2.9,            // gpsPdop
            20.0,           // pulseWidth
            0.0);           // relTime

    checkPoint(
            data,
            1,
            141437.548,     // time
            82.605287,      // latitude
            301.404862,     // longitude
            18.688,         // elevation
            2642,           // xmtSig
            173,            // rcvSig
            52.006,         // azimuth
            -4.376,         // pitch
            0.609,          // roll
            2.9,            // gpsPdop
            17.0,           // pulseWidth
            0.0);           // relTime
}

BOOST_AUTO_TEST_CASE(testSkip)
{
    pdal::Option filename("filename", getFilePath(), "");
    pdal::Options options(filename);
    pdal::drivers::icebridge::Reader reader(options);

    reader.initialize();

    const pdal::Schema& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 2);

    boost::scoped_ptr<pdal::StageSequentialIterator> it(
            reader.createSequentialIterator(data));

    it->skip(1);
    it->read(data);

    checkPoint(
            data,
            0,
            141437.548,     // time
            82.605287,      // latitude
            301.404862,     // longitude
            18.688,         // elevation
            2642,           // xmtSig
            173,            // rcvSig
            52.006,         // azimuth
            -4.376,         // pitch
            0.609,          // roll
            2.9,            // gpsPdop
            17.0,           // pulseWidth
            0.0);           // relTime
}

BOOST_AUTO_TEST_CASE(testPipeline)
{
    pdal::PipelineManager manager;
    pdal::PipelineReader reader(manager);

    bool isWriter =
        reader.readPipeline(Support::datapath("icebridge/pipeline.xml"));
    BOOST_CHECK(isWriter);

    const boost::uint64_t numPoints = manager.execute();
    BOOST_CHECK_EQUAL(numPoints, 2);
    pdal::FileUtils::deleteFile(Support::datapath("icebridge/outfile.txt"));
}

BOOST_AUTO_TEST_CASE(testRandomIterator)
{
    pdal::Option filename("filename", getFilePath(), "");
    pdal::Options options(filename);
    pdal::drivers::icebridge::Reader reader(options);

    reader.initialize();

    const pdal::Schema& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 1);

    boost::scoped_ptr<pdal::StageRandomIterator> it(
            reader.createRandomIterator(data));

    const boost::uint64_t numSeek = it->seek(1);
    BOOST_CHECK_EQUAL(numSeek, 1);

    const boost::uint32_t numRead = it->read(data);
    BOOST_CHECK_EQUAL(numRead, 1);

    checkPoint(
            data,
            0,
            141437.548,     // time
            82.605287,      // latitude
            301.404862,     // longitude
            18.688,         // elevation
            2642,           // xmtSig
            173,            // rcvSig
            52.006,         // azimuth
            -4.376,         // pitch
            0.609,          // roll
            2.9,            // gpsPdop
            17.0,           // pulseWidth
            0.0);           // relTime
}

BOOST_AUTO_TEST_SUITE_END()


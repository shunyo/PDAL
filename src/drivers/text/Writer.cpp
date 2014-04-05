/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/drivers/text/Writer.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_macros.hpp>

#include <iostream>
#include <algorithm>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>


#ifdef USE_PDAL_PLUGIN_TEXT
MAKE_WRITER_CREATOR(textWriter, pdal::drivers::text::Writer)
CREATE_WRITER_PLUGIN(text, pdal::drivers::text::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace text
{


struct FileStreamDeleter
{

    template <typename T>
    void operator()(T* ptr)
    {
        ptr->flush();
        FileUtils::closeFile(ptr);
    }
};

Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , bWroteHeader(false)
    , bWroteFirstPoint(false)

{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();

    std::string filename = getOptions().getValueOrThrow<std::string>("filename");

    // This is so the stream gets closed down if we throw any sort of
    // exception
    m_stream = FileStreamPtr(FileUtils::createFile(filename, true), FileStreamDeleter());

    return;
}



Options Writer::getDefaultOptions()
{
    Options options;

    Option delimiter("delimiter", ",", "Delimiter to use for writing text");
    Option newline("newline", "\n", "Newline character to use for additional lines");
    Option quote_header("quote_header", true, "Write dimension names in quotes");
    Option filename("filename", "", "Filename to write CSV file to");


    options.add(filename);
    options.add(delimiter);
    options.add(newline);
    options.add(quote_header);

    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");
    if (bGeoJSON)
    {
        *m_stream << "]}";
        std::string callback = getOptions().getValueOrDefault<std::string>("jscallback", "");

        if (callback.size())
        {
            *m_stream  <<")";
        }
    }

    m_stream.reset();
    m_stream = FileStreamPtr();
    bWroteHeader = false;
    bWroteFirstPoint = false;
    return;
}

std::vector<boost::tuple<std::string, std::string> >  Writer::getDimensionOrder(Schema const& schema) const
{
    boost::char_separator<char> separator(",");
    std::string dimension_order = getOptions().getValueOrDefault<std::string>("order", "");

    boost::erase_all(dimension_order, " "); // Wipe off spaces
    std::vector<boost::tuple<std::string, std::string> >  output;
    if (dimension_order.size())
    {

        // Put all the names in a map. We'll add the names in order to our
        // output list as we find them, and we'll remove those that we've added
        // from the map as we go. We'll then add what's left at the end based on
        // whether or not the user wants to do so.
        std::map<std::string, bool> all_names;
        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
        schema::index_by_index::const_iterator iter = dims.begin();
//        schema::index_by_name const& name_index = schema.getDimensions().get<schema::name>();
        while (iter != dims.end())
        {
            all_names.insert(std::pair<std::string, bool>(iter->getName(), true));
            ++iter;
        }

        tokenizer parameters(dimension_order, separator);
        for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
        {
            boost::optional<Dimension const&> d = schema.getDimensionOptional(*t);
            if (d)
            {
                if (boost::equals(d->getName(), *t))
                {
                    output.push_back(boost::tuple<std::string, std::string>(d->getName(), d->getNamespace()));

                    std::map<std::string, bool>::iterator i = all_names.find(d->getName());
                    all_names.erase(i);
                }
            }
            else
            {
                std::ostringstream oss;
                oss << "Dimension not found with name '" << *t <<"'";
                throw pdal::dimension_not_found(oss.str());
            }
        }

        bool bKeep = getOptions().getValueOrDefault<bool>("keep_unspecified", true);
        if (bKeep)
        {
            std::map<std::string, bool>::const_iterator i = all_names.begin();
            while (i!= all_names.end())
            {
                output.push_back(boost::tuple<std::string, std::string>(i->first, ""));
                ++i;
            }
        }
        else
        {
            return output;
        }
    }
    else
    {
        // No order was specified, just use the order of the schema
        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
        schema::index_by_index::const_iterator iter = dims.begin();
        while (iter != dims.end())
        {
            if (!iter->isIgnored())
                output.push_back(iter->getName());
            ++iter;
        }
    }

    return output;

}

void Writer::WriteGeoJSONHeader(pdal::Schema const&)
{
    std::string callback = getOptions().getValueOrDefault<std::string>("jscallback", "");

    if (callback.size())
    {
        *m_stream << callback <<"(";
    }
    *m_stream << "{ \"type\": \"FeatureCollection\", \"features\": [";
    return;
}

void Writer::WriteCSVHeader(pdal::Schema const& schema)
{
    bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");

    // Spaces get stripped off, so if the user set a space,
    // we'll get a 0-sized value. We'll set to space in that
    // instance. This means you can't use \n as a delimiter.
    if (delimiter.size() == 0)
        delimiter = " ";

    std::string order = getOptions().getValueOrDefault<std::string>("order", "");
    boost::erase_all(order, " "); // Wipe off spaces

    log()->get(logDEBUG) << "Dimension order specified '" << order << "'" << std::endl;

    std::vector<boost::tuple<std::string, std::string> > dimensions = getDimensionOrder(schema);
    log()->get(logDEBUG) << "dimensions.size(): " << dimensions.size() <<std::endl;
    std::ostringstream oss;
    oss << "Dimension order obtained '";
    for (std::vector<boost::tuple<std::string, std::string> >::const_iterator i = dimensions.begin(); i != dimensions.end(); i++)
    {
        std::string name = i->get<0>();
        std::string namespc = i->get<1>();

        Dimension const& d = schema.getDimension(name, namespc);
        if (d.isIgnored())
        {
            i++;
            if (i == dimensions.end())
                break;
            else
                continue;
        }

        if (i != dimensions.begin())
            oss <<  ",";
        oss <<i->get<0>();
    }
    log()->get(logDEBUG) << oss.str() << std::endl;


    std::vector<boost::tuple<std::string, std::string> >::const_iterator iter = dimensions.begin();

    bool bWroteProperty(false);
    while (iter != dimensions.end())
    {
        if (bWroteProperty)
            *m_stream << delimiter;

        Dimension const& d = schema.getDimension(iter->get<0>(), iter->get<1>());
        if (d.isIgnored())
        {
            iter++;
            bWroteProperty = false;
            if (iter == dimensions.end())
                break;
            else
                continue;
        }
        if (isQuoted)
            *m_stream << "\"";
        *m_stream << iter->get<0>();
        if (isQuoted)
            *m_stream<< "\"";
        iter++;
        bWroteProperty = true;
    }
    *m_stream << newline;


}


void Writer::WriteHeader(pdal::Schema const& schema)
{

    //schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

//    bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
    bool bWriteHeader = getOptions().getValueOrDefault<bool>("write_header", true);
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");

    // Spaces get stripped off, so if the user set a space,
    // we'll get a 0-sized value. We'll set to space in that
    // instance. This means you can't use \n as a delimiter.
    if (delimiter.size() == 0)
        delimiter = " ";

    if (!bWriteHeader)
    {
        log()->get(logDEBUG) << "Not writing header" << std::endl;
        bWroteHeader = true;
        return;

    }

    log()->get(logDEBUG) << "Writing header to filename: " << getOptions().getValueOrThrow<std::string>("filename") << std::endl;



    // If we're bGeoJSON, we're just going to write the preamble for FeatureCollection,
    // and let the rest happen in writeBuffer
    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");
    if (bGeoJSON)
    {
        WriteGeoJSONHeader(schema);
        return;
    }

    bool bCSV = boost::iequals(outputType, "CSV");
    if (bCSV)
    {
        WriteCSVHeader(schema);
        return;
    }


    return;
}

void Writer::putStringRepresentation(PointBuffer const& data,
                                     Dimension const& d,
                                     std::size_t pointIndex,
                                     std::ostream& output)
{
    std::streamsize old_precision = output.precision();

    bool bHaveScaling = !Utils::compare_distance(d.getNumericScale(), 1.0);

    // FIXME: Allow selective scaling of requested dimensions
    if (bHaveScaling)
    {
        output.setf(std::ios::fixed, std::ios::floatfield);
        output.precision(Utils::getStreamPrecision(d.getNumericScale()));
    }
    switch (d.getInterpretation())
    {
        case dimension::Float:
        case dimension::SignedInteger:
        case dimension::UnsignedInteger:
            output << data.getFieldAs<double>(d, pointIndex);
            break;
        case dimension::RawByte:
            {
                unsigned char* raw  = data.getData(pointIndex) +
                    d.getByteOffset();
                Utils::binary_to_hex_stream(raw, output, 0, d.getByteSize());
                break;
            }
        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            break;
    }

    if (bHaveScaling)
    {
        output.precision(old_precision);
        output.unsetf(std::ios::fixed);
        output.unsetf(std::ios::floatfield);
    }
}


void Writer::WriteCSVBuffer(const PointBuffer& data)
{
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");
    if (delimiter.size() == 0)
        delimiter = " ";

    boost::uint32_t pointIndex(0);

    pdal::Schema const& schema = data.getSchema();

    std::string order = getOptions().getValueOrDefault<std::string>("order", "");
    boost::erase_all(order, " "); // Wipe off spaces

    std::vector<boost::tuple<std::string, std::string> > dimensions = getDimensionOrder(schema);

    while (pointIndex != data.getNumPoints())
    {
        std::vector<boost::tuple<std::string, std::string> >::const_iterator iter =  dimensions.begin();

        bool bFirstProperty(true);
        while (iter != dimensions.end())
        {
            if (!bFirstProperty)
                *m_stream << delimiter;

            Dimension const& d = schema.getDimension(iter->get<0>(), iter->get<1>());
            if (d.isIgnored())
            {
                iter++;
                continue;
            }

            putStringRepresentation(data, d, pointIndex, *m_stream);

            bFirstProperty = false;
            iter++;

        }
        *m_stream << newline;

        pointIndex++;
        if (!bWroteFirstPoint)
        {
            bWroteFirstPoint = true;
        }

    }
}


void Writer::WriteGeoJSONBuffer(const PointBuffer& data)
{
    boost::uint32_t pointIndex(0);
    pdal::Schema const& schema = data.getSchema();
    std::vector<boost::tuple<std::string, std::string> > ordering = getDimensionOrder(schema);
    std::vector<boost::tuple<std::string, std::string> >::const_iterator ord =  ordering.begin();

    std::vector<Dimension const*> dimensions;
//    bool bFirstProperty(true);
    while (ord != ordering.end())
    {
        Dimension const& d = schema.getDimension(ord->get<0>(), ord->get<1>());
        dimensions.push_back(&d);
        ord++;
    }
    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    while (pointIndex != data.getNumPoints())
    {


        if (bWroteFirstPoint)
            *m_stream << ",";

        *m_stream << "{ \"type\":\"Feature\",\"geometry\": { \"type\": \"Point\", \"coordinates\": [";
        putStringRepresentation(data, dimX, pointIndex, *m_stream);
        *m_stream << ",";
        putStringRepresentation(data, dimY, pointIndex, *m_stream);
        *m_stream << ",";
        putStringRepresentation(data, dimZ, pointIndex, *m_stream);
        *m_stream << "]},";

        *m_stream << "\"properties\": {";

        std::vector<Dimension const* >::const_iterator iter =  dimensions.begin();

        bool bFirstProperty(true);
        while (iter != dimensions.end())
        {
            Dimension const* d = *iter;
            if (d->isIgnored())
            {
                iter++;
                continue;
            }

            if (!bFirstProperty)
                *m_stream << ",";

            *m_stream << "\"" << d->getName() << "\":";


            *m_stream << "\"";
            putStringRepresentation(data, *d, pointIndex, *m_stream);
            *m_stream <<"\"";

            iter++;
            bFirstProperty = false;
        }


        *m_stream << "}"; // end properties

        *m_stream << "}"; // end feature

        pointIndex++;
        if (!bWroteFirstPoint)
        {
            bWroteFirstPoint = true;
        }


    }
}

boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{

    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bCSV = boost::iequals(outputType, "CSV");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");

    if (!bWroteHeader)
    {
        WriteHeader(data.getSchema());
        bWroteHeader = true;
    }

    if (bCSV)
    {
        WriteCSVBuffer(data);

    }
    else if (bGeoJSON)
    {
        WriteGeoJSONBuffer(data);

    }



    return data.getNumPoints();
}


}
}
} // namespaces

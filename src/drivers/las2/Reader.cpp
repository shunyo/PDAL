/******************************************************************************
* Copyright (c) 2014, Howard Butler howard@hobu.co
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

#include <pdal/drivers/las2/Reader.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#ifdef PDAL_HAVE_LASZIP
#include <laszip/lasunzipper.hpp>
#endif

#include <pdal/FileUtils.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/Metadata.hpp>

#include <stdexcept>

#ifdef PDAL_HAVE_GDAL
#include "gdal.h"
#include "cpl_vsi.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#endif

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#if defined(max) && defined(PDAL_PLATFORM_WIN32)
#undef max
#endif

namespace pdal
{
namespace drivers
{
namespace las2
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{}


Reader::Reader(const std::string& filename)
    : pdal::Reader(Options::none())
{}




Reader::~Reader()
{
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    las_.open(getOptions().getValueOrThrow<std::string>("filename"));
    this->setNumPoints( las_.header.number_of_point_records);
    pdal::Bounds<double> bounds(las_.header.mins[0],
                                las_.header.mins[1],
                                las_.header.mins[2],
                                las_.header.maxs[0],
                                las_.header.maxs[1],
                                las_.header.maxs[2]);
    this->setBounds(bounds);
    las_.close();
    // 
    // LasHeaderReader lasHeaderReader(m_lasHeader, stream);
    // lasHeaderReader.read(*this, getSchemaRef());
    // 
    // this->setBounds(m_lasHeader.getBounds());
    // this->setNumPoints(m_lasHeader.GetPointRecordsCount());
    // 
    // // If the user is already overriding this by setting it on the stage, we'll
    // // take their overridden value
    // const SpatialReference& srs = getSpatialReference();
    // if (srs.getWKT(pdal::SpatialReference::eCompoundOK).empty())
    // {
    //     SpatialReference new_srs;
    //     m_lasHeader.getVLRs().constructSRS(new_srs);
    //     setSpatialReference(new_srs);
    // }
    readMetadata();
}


Options Reader::getDefaultOptions()
{
    Option option1("filename", "", "file to read from");
    Options options(option1);
    return options;
}





pdal::StageSequentialIterator*
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::las2::iterators::sequential::Reader(*this, buffer, getNumPoints());
}


pdal::StageRandomIterator*
Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::las2::iterators::random::Reader(*this, buffer, getNumPoints());
}


// boost::uint32_t Reader::processBuffer(PointBuffer& data, std::istream& stream,
//     boost::uint64_t numPointsLeft, LASunzipper* unzipper, ZipPoint* zipPoint,
//     PointDimensions* dimensions, std::vector<boost::uint8_t>& read_buffer) const
// {
//     if (!dimensions)
//     {
//         throw pdal_error("No dimension positions are available!");
//     } 
//  
//     // we must not read more points than are left in the file
//     boost::uint64_t numPoints64 =
//         std::min<boost::uint64_t>(data.getCapacity(), numPointsLeft);
//     boost::uint64_t numPoints =
//         std::min<boost::uint64_t>(numPoints64,
//             std::numeric_limits<boost::uint32_t>::max());
//     
//     if (numPoints64 >= std::numeric_limits<boost::uint32_t>::max())
//     {
//         throw pdal_error("Unable to read more than 2**32 points at a time");
//     }
//     
//     const LasHeader& lasHeader = getLasHeader();
//     const PointFormat pointFormat = lasHeader.getPointFormat();
// 
//     const bool hasTime = Support::hasTime(pointFormat);
//     const bool hasColor = Support::hasColor(pointFormat);
//     pointbuffer::PointBufferByteSize pointByteCount =
//         Support::getPointDataSize(pointFormat);
//     
//     pointbuffer::PointBufferByteSize numBytesToRead = pointByteCount * numPoints;
//     if (read_buffer.size() < numBytesToRead)
//     {
//         read_buffer.resize(numBytesToRead);
//     }
// 
// 
//     if (zipPoint)
//     {
// #ifdef PDAL_HAVE_LASZIP
//         boost::uint8_t* p = &(read_buffer.front());
// 
//         bool ok = false;
//         for (boost::uint32_t i=0; i<numPoints; i++)
//         {
//             ok = unzipper->read(zipPoint->m_lz_point);
//             if (!ok)
//             {
//                 std::ostringstream oss;
//                 const char* err = unzipper->get_error();
//                 if (err==NULL) err="(unknown error)";
//                 oss << "Error reading compressed point data: " <<
//                     std::string(err);
//                 throw pdal_error(oss.str());
//             }
// 
//             memcpy(p, zipPoint->m_lz_point_data.get(),
//                 zipPoint->m_lz_point_size);
//             p += zipPoint->m_lz_point_size;
//         }
// #else
//         boost::ignore_unused_variable_warning(unzipper);
//         throw pdal_error("LASzip is not enabled for this "
//             "pdal::drivers::las::Reader::processBuffer");
// #endif
//     }
//     else
//     {
//         try
//         {
//             Utils::read_n(read_buffer.front(), stream, numBytesToRead);
//         } catch (std::out_of_range&)
//         {
//             if (stream.gcount())
//             {
//                 // We weren't able to read as many bytes as asked 
//                 // The header must have lied or something, but we 
//                 // do have some data here. Figure out how many 
//                 // points we read and set things to that
//                 numPoints = stream.gcount()/pointByteCount;
// 
//             } else
//             {
//                 throw;
//             }
//         } catch (pdal::invalid_stream&)
//         {
//             numPoints = 0;
//         }
//         
//     }
// 
//     pdal::Bounds<double> bounds;
//     pdal::Vector<double> point(0.0, 0.0, 0.0);
//     bool bFirstPoint(true);
//     for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
//     {
//         boost::uint8_t* p = &(read_buffer.front()) +
//             static_cast<pointbuffer::PointBufferByteSize>(pointByteCount) *
//             static_cast<pointbuffer::PointBufferByteSize>(pointIndex);
// 
//         {
//             const boost::int32_t x = Utils::read_field<boost::int32_t>(p);
//             const boost::int32_t y = Utils::read_field<boost::int32_t>(p);
//             const boost::int32_t z = Utils::read_field<boost::int32_t>(p);
//             
//             if (dimensions->X && dimensions->Y && dimensions->Z)
//             {
//                 double X = dimensions->X->applyScaling(x);
//                 double Y = dimensions->Y->applyScaling(y);
//                 double Z = dimensions->Z->applyScaling(z);
// 
//                 if (bFirstPoint)
//                 {
//                     bounds = pdal::Bounds<double>(X, Y, Z, X, Y, Z);
//                     bFirstPoint = false;
//                 }
// 
//                 point.set(0, X);
//                 point.set(1, Y);
//                 point.set(2, Z);
//                 bounds.grow(point);
//             }
// 
// 
//             boost::uint16_t intensity = Utils::read_field<boost::uint16_t>(p);
//             boost::uint8_t flags = Utils::read_field<boost::uint8_t>(p);
//             boost::uint8_t classification =
//                 Utils::read_field<boost::uint8_t>(p) & 31;
//             boost::int8_t scanAngleRank = Utils::read_field<boost::int8_t>(p);
//             boost::uint8_t user = Utils::read_field<boost::uint8_t>(p);
//             boost::uint16_t pointSourceId =
//                 Utils::read_field<boost::uint16_t>(p);
// 
//             boost::uint8_t returnNum = flags & 0x07;
//             boost::uint8_t numReturns = (flags >> 3) & 0x07;
//             boost::uint8_t scanDirFlag = (flags >> 6) & 0x01;
//             boost::uint8_t flight = (flags >> 7) & 0x01;
//             
//             if (dimensions->X)
//                 data.setField<boost::int32_t>(*dimensions->X, pointIndex, x);
//             
//             if (dimensions->Y)
//                 data.setField<boost::int32_t>(*dimensions->Y, pointIndex, y);
//             
//             if (dimensions->Z)
//                 data.setField<boost::int32_t>(*dimensions->Z, pointIndex, z);
// 
//             if (dimensions->Intensity)
//                 data.setField<boost::uint16_t>(*dimensions->Intensity,
//                     pointIndex, intensity);
// 
//             if (dimensions->ReturnNumber)
//                 data.setField<boost::uint8_t>(*dimensions->ReturnNumber,
//                     pointIndex, returnNum);
// 
//             if (dimensions->NumberOfReturns)
//                 data.setField<boost::uint8_t>(*dimensions->NumberOfReturns,
//                     pointIndex, numReturns);
// 
//             if (dimensions->ScanDirectionFlag)
//                 data.setField<boost::uint8_t>(*dimensions->ScanDirectionFlag,
//                     pointIndex, scanDirFlag);
// 
//             if (dimensions->EdgeOfFlightLine)
//                 data.setField<boost::uint8_t>(*dimensions->EdgeOfFlightLine,
//                     pointIndex, flight);
// 
//             if (dimensions->Classification)
//                 data.setField<boost::uint8_t>(*dimensions->Classification,
//                     pointIndex, classification);
// 
//             if (dimensions->ScanAngleRank)
//                 data.setField<boost::int8_t>(*dimensions->ScanAngleRank,
//                     pointIndex, scanAngleRank);
// 
//             if (dimensions->UserData)
//                 data.setField<boost::uint8_t>(*dimensions->UserData,
//                     pointIndex, user);
// 
//             if (dimensions->PointSourceId)
//                 data.setField<boost::uint16_t>(*dimensions->PointSourceId,
//                     pointIndex, pointSourceId);
//         }
// 
//         if (hasTime)
//         {
//             double time = Utils::read_field<double>(p);
// 
//             if (dimensions->Time)
//                 data.setField<double>(*dimensions->Time, pointIndex, time);
//         }
// 
//         if (hasColor)
//         {
//             boost::uint16_t red = Utils::read_field<boost::uint16_t>(p);
//             boost::uint16_t green = Utils::read_field<boost::uint16_t>(p);
//             boost::uint16_t blue = Utils::read_field<boost::uint16_t>(p);
// 
//             if (dimensions->Red)
//                 data.setField<boost::uint16_t>(*dimensions->Red,
//                     pointIndex, red);
// 
//             if (dimensions->Green)
//                 data.setField<boost::uint16_t>(*dimensions->Green,
//                     pointIndex, green);
// 
//             if (dimensions->Blue)
//                 data.setField<boost::uint16_t>(*dimensions->Blue,
//                     pointIndex, blue);
//         }
// 
//         data.setNumPoints(pointIndex+1);
//     }
// 
//     data.setSpatialBounds(bounds);
//     return numPoints;
// }

void Reader::readMetadata()
{
    // LasHeader const& header = getLasHeader();
    // Metadata& metadata = getMetadataRef();
    // 
    // metadata.addMetadata<bool>("compressed",
    //     header.Compressed(), "true if this LAS file is compressed");
    // metadata.addMetadata<boost::uint32_t>("dataformat_id",
    //     static_cast<boost::uint32_t>(header.getPointFormat()),
    //     "The Point Format ID as specified in the LAS specification");
    // metadata.addMetadata<boost::uint32_t>("major_version",
    //     static_cast<boost::uint32_t>(header.GetVersionMajor()),
    //     "The major LAS version for the file, always 1 for now");
    // metadata.addMetadata<boost::uint32_t>("minor_version",
    //     static_cast<boost::uint32_t>(header.GetVersionMinor()),
    //     "The minor LAS version for the file");
    // metadata.addMetadata<boost::uint32_t>("filesource_id",
    //     static_cast<boost::uint32_t>(header.GetFileSourceId()),
    //     "File Source ID (Flight Line Number if this file was derived from "
    //     "an original flight line): This field should be set to a value "
    //     "between 1 and 65,535, inclusive. A value of zero (0) is interpreted "
    //     "to mean that an ID has not been assigned. In this case, processing "
    //     "software is free to assign any valid number. Note that this scheme "
    //     "allows a LIDAR project to contain up to 65,535 unique sources. A "
    //     "source can be considered an original flight line or it can be the "
    //     "result of merge and/or extract operations.");
    // boost::uint16_t reserved = header.GetReserved();
    // //ABELL  Byte order assumptions?
    // boost::uint8_t* start = (boost::uint8_t*)&reserved;
    // std::vector<boost::uint8_t> raw_bytes;
    // for (std::size_t i = 0 ; i < sizeof(boost::uint16_t); ++i)
    // {
    //   raw_bytes.push_back(start[i]);
    // }
    // pdal::ByteArray bytearray(raw_bytes);
    // 
    // pdal::Metadata entry("global_encoding", bytearray);
    // 
    // entry.setDescription("Global Encoding: This is a bit field used to "
    //     "indicate certain global properties about the file. In LAS 1.2 "
    //     "(the version in which this field was introduced), only the low bit "
    //     "is defined (this is the bit, that if set, would have the unsigned "
    //     "integer yield a value of 1).");
    // metadata.addMetadata(entry);                                       
    // metadata.addMetadata<boost::uuids::uuid>("project_id",
    //      header.GetProjectId(), "Project ID (GUID data): The four fields "
    //      "that comprise a complete Globally Unique Identifier (GUID) are now "
    //      "reserved for use as a Project Identifier (Project ID). The field "
    //      "remains optional. The time of assignment of the Project ID is at "
    //      "the discretion of processing software. The Project ID should be "
    //      "the same for all files that are associated with a unique project. "
    //      "By assigning a Project ID and using a File Source ID (defined above) "         "every file within a project and every point within a file can be "
    //      "uniquely identified, globally.");
    // metadata.addMetadata<std::string>("system_id", header.GetSystemId(false));
    // metadata.addMetadata<std::string>("software_id",
    //     header.GetSoftwareId(false), "This information is ASCII data "
    //     "describing the generating software itself. This field provides a "
    //     "mechanism for specifying which generating software package and "
    //     "version was used during LAS file creation (e.g. \"TerraScan V-10.8\","
    //     " \"REALM V-4.2\" and etc.).");
    // metadata.addMetadata<boost::uint32_t>("creation_doy",
    //     static_cast<boost::uint32_t>(header.GetCreationDOY()),
    //     "Day, expressed as an unsigned short, on which this file was created. "
    //     "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
    //     "considered day 1.");
    // metadata.addMetadata<boost::uint32_t>("creation_year",
    //     static_cast<boost::uint32_t>(header.GetCreationYear()),
    //     "The year, expressed as a four digit number, in which the file was "
    //     "created.");
    // metadata.addMetadata<boost::uint32_t>("header_size",
    //     static_cast<boost::uint32_t>(header.GetHeaderSize()),
    //     "The size, in bytes, of the Public Header Block itself. In the event "
    //     "that the header is extended by a software application through the "
    //     "addition of data at the end of the header, the Header Size field "
    //     "must be updated with the new header size. Extension of the Public "
    //     "Header Block is discouraged; the Variable Length Records should be "
    //     "used whenever possible to add custom header data. In the event a "
    //     "generating software package adds data to the Public Header Block, "
    //     "this data must be placed at the end of the structure and the Header "
    //     "Size must be updated to reflect the new size.");
    // metadata.addMetadata<boost::uint32_t>("dataoffset",
    //     static_cast<boost::uint32_t>(header.GetDataOffset()),
    //     "The actual number of bytes from the beginning of the file to the "
    //     "first field of the first point record data field. This data offset "
    //     "must be updated if any software adds data from the Public Header "
    //     "Block or adds/removes data to/from the Variable Length Records.");
    // metadata.addMetadata<double>("scale_x", header.GetScaleX(),
    //     "The scale factor fields contain a double floating point value that "
    //     "is used to scale the corresponding X, Y, and Z long values within "
    //     "the point records. The corresponding X, Y, and Z scale factor must "
    //     "be multiplied by the X, Y, or Z point record value to get the actual "
    //     "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
    //     "are intended to have two decimal point values, then each scale factor "
    //     "will contain the number 0.01.");
    // metadata.addMetadata<double>("scale_y", header.GetScaleY(),
    //     "The scale factor fields contain a double floating point value that "
    //     "is used to scale the corresponding X, Y, and Z long values within "
    //     "the point records. The corresponding X, Y, and Z scale factor must "
    //     "be multiplied by the X, Y, or Z point record value to get the "
    //     "actual X, Y, or Z coordinate. For example, if the X, Y, and Z "
    //     "coordinates are intended to have two decimal point values, then each "
    //     "scale factor will contain the number 0.01.");
    // metadata.addMetadata<double>("scale_z", header.GetScaleZ(),
    //     "The scale factor fields contain a double floating point value that "
    //     "is used to scale the corresponding X, Y, and Z long values within "
    //     "the point records. The corresponding X, Y, and Z scale factor must "
    //     "be multiplied by the X, Y, or Z point record value to get the actual "
    //     "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
    //     "are intended to have two decimal point values, then each scale factor "
    //     "will contain the number 0.01.");
    // metadata.addMetadata<double>("offset_x", header.GetOffsetX(),
    //     "The offset fields should be used to set the overall offset for the "
    //     "point records. In general these numbers will be zero, but for "
    //     "certain cases the resolution of the point data may not be large "
    //     "enough for a given projection system. However, it should always be "
    //     "assumed that these numbers are used.");
    // metadata.addMetadata<double>("offset_y", header.GetOffsetY(),
    //     "The offset fields should be used to set the overall offset for the "
    //     "point records. In general these numbers will be zero, but for "
    //     "certain cases the resolution of the point data may not be large "
    //     "enough for a given projection system. However, it should always be "
    //     "assumed that these numbers are used.");
    // metadata.addMetadata<double>("offset_z", header.GetOffsetZ(),
    //     "The offset fields should be used to set the overall offset for the "
    //     "point records. In general these numbers will be zero, but for certain "
    //     "cases the resolution of the point data may not be large enough for "
    //     "a given projection system. However, it should always be assumed that "
    //     "these numbers are used.");
    // metadata.addMetadata<double>("minx", header.GetMinX(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<double>("miny", header.GetMinY(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<double>("minz", header.GetMinZ(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<double>("maxx", header.GetMaxX(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<double>("maxy", header.GetMaxY(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<double>("maxz", header.GetMaxZ(),
    //     "The max and min data fields are the actual unscaled extents of the "
    //     "LAS point file data, specified in the coordinate system of the LAS "
    //     "data.");
    // metadata.addMetadata<boost::uint32_t>("count",
    //     header.GetPointRecordsCount(), "This field contains the total number "
    //     "of point records within the file.");
    // 
    // std::vector<VariableLengthRecord> const& vlrs = header.getVLRs().getAll();
    // for (std::vector<VariableLengthRecord>::size_type t = 0;
    //     t < vlrs.size(); ++t)
    // {
    //     VariableLengthRecord const& v = vlrs[t];
    // 
    //     std::vector<boost::uint8_t> raw_bytes;
    //     for (std::size_t i = 0 ; i < v.getLength(); ++i)
    //     {
    //         raw_bytes.push_back(v.getBytes()[i]);
    //     }
    //     pdal::ByteArray bytearray(raw_bytes);
    // 
    //     std::ostringstream name;
    //     name << "vlr_" << t;
    //     pdal::Metadata entry(name.str(), bytearray);
    // 
    //     entry.addMetadata<boost::uint32_t>("reserved", v.getReserved(),
    //         "Two bytes of padded, unused space. Some softwares expect the "
    //         "values of these bytes to be 0xAABB as specified in the 1.0 "
    //         "version of the LAS specification");
    //     entry.addMetadata<std::string>("user_id", v.getUserId(),
    //         "The User ID field is ASCII character data that identifies the "
    //         "user which created the variable length record. It is possible to "
    //         "have many Variable Length Records from different sources with "
    //         "different User IDs. If the character data is less than 16 "
    //         "characters, the remaining data must be null. The User ID must be "
    //         "registered with the LAS specification managing body. The "
    //         "management of these User IDs ensures that no two individuals "
    //         "accidentally use the same User ID. The specification will "
    //         "initially use two IDs: one for globally specified records "
    //         "(LASF_Spec), and another for projection types (LASF_Projection). "
    //         "Keys may be requested at "
    //         "http://www.asprs.org/lasform/keyform.html.");
    //     entry.addMetadata<boost::uint32_t>("record_id", v.getRecordId(),
    //         "The Record ID is dependent upon the User ID. There can be "
    //         "0 to 65535 Record IDs for every User ID. The LAS specification "
    //         "manages its own Record IDs (User IDs owned by the specification), "
    //         "otherwise Record IDs will be managed by the owner of the given "
    //         "User ID. Thus each User ID is allowed to assign 0 to 65535 Record "
    //         "IDs in any manner they desire. Publicizing the meaning of a given "
    //         "Record ID is left to the owner of the given User ID. Unknown User "
    //         "ID/Record ID combinations should be ignored.");
    //     entry.addMetadata<std::string>("description", v.getDescription());
    //     entry.setDescription(v.getDescription());
    // 
    //     std::ostringstream n;
    //     n << "vlr." << v.getUserId() << "." << v.getRecordId();
    //     metadata.addMetadata(entry);
    // }
}

std::vector<Dimension> Reader::getDefaultDimensions()
{

#   define DIMENSION_BOOKKEEPING(name, uid) \
    name.setUUID(uid); \
    name.setNamespace(s_getName()); \
    output.push_back(name);
    
    std::vector<Dimension> output;
    Dimension x("X", dimension::SignedInteger, 4, "X coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    DIMENSION_BOOKKEEPING(x, "2ee118d1-119e-4906-99c3-42934203f872")

    Dimension y("Y", dimension::SignedInteger, 4, "Y coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    DIMENSION_BOOKKEEPING(y, "87707eee-2f30-4979-9987-8ef747e30275")

    Dimension z("Z", dimension::SignedInteger, 4, "Z coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    DIMENSION_BOOKKEEPING(z, "e74b5e41-95e6-4cf2-86ad-e3f5a996da5d")

    Dimension time("Time", dimension::Float, 8, "The GPS Time is the double "
        "floating point time tag value at which the point was acquired. It "
        "is GPS Week Time if the Global Encoding low bit is clear and Adjusted "
        "Standard GPS Time if the Global Encoding low bit is set (see Global "
        "Encoding in the Public Header Block description).");
    DIMENSION_BOOKKEEPING(time, "aec43586-2711-4e59-9df0-65aca78a4ffc")

    Dimension intensity("Intensity", dimension::UnsignedInteger, 2,
        "The intensity value is the integer representation of the pulse "
        "return magnitude. This value is optional and system specific. "
        "However, it should always be included if available.");
    DIMENSION_BOOKKEEPING(intensity, "61e90c9a-42fc-46c7-acd3-20d67bd5626f")

    Dimension return_number("ReturnNumber", dimension::UnsignedInteger, 1,
        "Return Number: The Return Number is the pulse return number for "
        "a given output pulse. A given output laser pulse can have many "
        "returns, and they must be marked in sequence of return. The first "
        "return will have a Return Number of one, the second a Return "
        "Number of two, and so on up to five returns.");
    DIMENSION_BOOKKEEPING(return_number, "ffe5e5f8-4cec-4560-abf0-448008f7b89e")

    Dimension number_of_returns("NumberOfReturns", dimension::UnsignedInteger,
        1, "Number of Returns (for this emitted pulse): The Number of Returns "
        "is the total number of returns for a given pulse. For example, "
        "a laser data point may be return two (Return Number) within a "
        "total number of five returns.");
    DIMENSION_BOOKKEEPING(number_of_returns, "7c28bfd4-a9ed-4fb2-b07f-931c076fbaf0")

    Dimension scan_direction("ScanDirectionFlag", dimension::UnsignedInteger, 1,
        "The Scan Direction Flag denotes the direction at which the "
        "scanner mirror was traveling at the time of the output pulse. "
        "A bit value of 1 is a positive scan direction, and a bit value "
        "of 0 is a negative scan direction (where positive scan direction "
        "is a scan moving from the left side of the in-track direction to "
        "the right side and negative the opposite).");
    DIMENSION_BOOKKEEPING(scan_direction, "13019a2c-cf88-480d-a995-0162055fe5f9")

    Dimension edge("EdgeOfFlightLine", dimension::UnsignedInteger, 1,
        "The Edge of Flight Line data bit has a value of 1 only when "
        "the point is at the end of a scan. It is the last point on "
        "a given scan line before it changes direction.");
    DIMENSION_BOOKKEEPING(edge, "108c18f2-5cc0-4669-ae9a-f41eb4006ea5")

    Dimension classification("Classification", dimension::UnsignedInteger, 1,
        "Classification in LAS 1.0 was essentially user defined and optional. "
        "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
        "the field is now mandatory. If a point has never been classified, "
        "this byte must be set to zero. There are no user defined classes "
        "since both point format 0 and point format 1 supply 8 bits per point "
        "for user defined operations. Note that the format for classification "
        "is a bit encoded field with the lower five bits used for class and "
        "the three high bits used for flags.");
    DIMENSION_BOOKKEEPING(classification, "b4c67de9-cef1-432c-8909-7c751b2a4e0b")

    Dimension scan_angle("ScanAngleRank", dimension::SignedInteger, 1,
        "The Scan Angle Rank is a signed one-byte number with a "
        "valid range from -90 to +90. The Scan Angle Rank is the "
        "angle (rounded to the nearest integer in the absolute "
        "value sense) at which the laser point was output from the "
        "laser system including the roll of the aircraft. The scan "
        "angle is within 1 degree of accuracy from +90 to 90 degrees. "
        "The scan angle is an angle based on 0 degrees being nadir, "
        "and 90 degrees to the left side of the aircraft in the "
        "direction of flight.");
    DIMENSION_BOOKKEEPING(scan_angle, "aaadaf77-e0c9-4df0-81a7-27060794cd69")

    Dimension scan_angle2("ScanAngleRank2", dimension::SignedInteger, 2,
        "The Scan Angle Rank is a signed one-byte number with a "
        "valid range from -90 to +90. The Scan Angle Rank is the "
        "angle (rounded to the nearest integer in the absolute "
        "value sense) at which the laser point was output from the "
        "laser system including the roll of the aircraft. The scan "
        "angle is within 1 degree of accuracy from +90 to 90 degrees. "
        "The scan angle is an angle based on 0 degrees being nadir, "
        "and 90 degrees to the left side of the aircraft in the "
        "direction of flight.");
    DIMENSION_BOOKKEEPING(scan_angle2, "de5e353f-916c-46b3-8c4a-f44fb6f4bf5f")
            
    Dimension user_data("UserData", dimension::UnsignedInteger, 1,
        "This field may be used at the users discretion");
    DIMENSION_BOOKKEEPING(user_data, "70eb558e-63d4-4804-b1db-fc2fd716927c")

    Dimension point_source("PointSourceId", dimension::UnsignedInteger, 2,
        "This value indicates the file from which this point originated. "
        "Valid values for this field are 1 to 65,535 inclusive with zero "
        "being used for a special case discussed below. The numerical value "
        "corresponds to the File Source ID from which this point originated. "
        "Zero is reserved as a convenience to system implementers. A Point "
        "Source ID of zero implies that this point originated in this file. "
        "This implies that processing software should set the Point Source "
        "ID equal to the File Source ID of the file containing this point "
        "at some time during processing. ");
    DIMENSION_BOOKKEEPING(point_source, "4e42e96a-6af0-4fdd-81cb-6216ff47bf6b")

    Dimension packet_descriptor("WavePacketDescriptorIndex",
        dimension::UnsignedInteger, 1);
    DIMENSION_BOOKKEEPING(packet_descriptor, "1d095eb0-099f-4800-abb6-2272be486f81")

    Dimension packet_offset("WaveformDataOffset", dimension::UnsignedInteger, 8);
    DIMENSION_BOOKKEEPING(packet_offset, "6dee8edf-0c2a-4554-b999-20c9d5f0e7b9")

    Dimension ReturnPointWaveformLocation("ReturnPointWaveformLocation",
        dimension::UnsignedInteger, 4);
    DIMENSION_BOOKKEEPING(ReturnPointWaveformLocation, "f0f37962-2563-4c3e-858d-28ec15a1103f")

    Dimension wave_x("WaveformXt", dimension::Float, 4);
    DIMENSION_BOOKKEEPING(wave_x, "c0ec76eb-9121-4127-b3d7-af92ef871a2d")

    Dimension wave_y("WaveformYt", dimension::Float, 4);
    DIMENSION_BOOKKEEPING(wave_y, "b3f5bb56-3c25-42eb-9476-186bb6b78e3c")

    Dimension wave_z("WaveformZt", dimension::Float, 4);
    DIMENSION_BOOKKEEPING(wave_z, "7499ae66-462f-4d0b-a449-6e5c721fb087")

    Dimension red("Red", dimension::UnsignedInteger, 2,
        "The red image channel value associated with this point");
    DIMENSION_BOOKKEEPING(red, "a42ce297-6aa2-4a62-bd29-2db19ba862d5")

    Dimension blue("Blue", dimension::UnsignedInteger, 2,
        "The blue image channel value associated with this point");
    DIMENSION_BOOKKEEPING(blue, "5c1a99c8-1829-4d5b-8735-4f6f393a7970")

    Dimension green("Green", dimension::UnsignedInteger, 2,
        "The green image channel value associated with this point");
    DIMENSION_BOOKKEEPING(green, "7752759d-5713-48cd-9842-51db350cc979")

    Dimension nir("NIR", dimension::UnsignedInteger, 2,
        "The NIR image channel value associated with this point");
    DIMENSION_BOOKKEEPING(nir, "7efe5465-4ed5-4c4f-892b-0e8532fb2540")


    
    return output;
}

namespace iterators
{

Base::Base(pdal::drivers::las2::Reader const& reader)
    : m_reader(reader)
{
//     m_istream.seekg(m_reader.getLasHeader().GetDataOffset());
// 
//     if (m_reader.getLasHeader().Compressed())
//     {
// #ifdef PDAL_HAVE_LASZIP
//         initialize();
// #else
//         throw pdal_error("LASzip is not enabled for this "
//             "pdal::drivers::las2::IteratorBase!");
// #endif
//     }
}


Base::~Base()
{
// #ifdef PDAL_HAVE_LASZIP
//     m_zipPoint.reset();
//     m_unzipper.reset();
// #endif

}


void Base::initialize()
{
// #ifdef PDAL_HAVE_LASZIP
//     if (!m_zipPoint)
//     {
//         PointFormat format = m_reader.getLasHeader().getPointFormat();
//         boost::scoped_ptr<ZipPoint> z(new ZipPoint(format,
//             getReader().getLasHeader().getVLRs().getAll(), true));
//         m_zipPoint.swap(z);
//     }
// 
//     if (!m_unzipper)
//     {
//         boost::scoped_ptr<LASunzipper> z(new LASunzipper());
//         m_unzipper.swap(z);
// 
//         bool stat(false);
// 
//         m_istream.seekg(static_cast<std::streampos>(
//             m_reader.getLasHeader().GetDataOffset()), std::ios::beg);
//         stat = m_unzipper->open(m_istream, m_zipPoint->GetZipper());
// 
//         // Martin moves the stream on us
//         m_zipReadStartPosition = m_istream.tellg();
//         if (!stat)
//         {
//             std::ostringstream oss;
//             const char* err = m_unzipper->get_error();
//             if (err==NULL)
//                 err="(unknown error)";
//             oss << "Failed to open LASzip stream: " << std::string(err);
//             throw pdal_error(oss.str());
//         }
//     }
// #endif
}

void Base::read(PointBuffer&)
{}

bool Base::open()
{
    las_.open(getReader().getOptions().getValueOrThrow<std::string>("filename"));
    getReader().log()->get(logDEBUG) << "Header point count: " << las_.header.number_of_point_records <<std::endl;

    return true;
}

namespace sequential
{


Reader::Reader(pdal::drivers::las2::Reader const& reader, PointBuffer& buffer,
        boost::uint32_t numPoints)
    : Base(reader), pdal::ReaderSequentialIterator(buffer),
    m_numPoints(numPoints)
{}


Reader::~Reader()
{}

void Reader::readBeginImpl()
{
    getReader().log()->get(logDEBUG) << "readBeginImpl: " <<std::endl;

    Base::open();
}


void Reader::readBufferBeginImpl(PointBuffer& buffer)
{}

boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
// #ifdef PDAL_HAVE_LASZIP
//     if (m_unzipper)
//     {
//         const boost::uint32_t pos32 =
//             Utils::safeconvert64to32(getIndex() + count);
//         m_unzipper->seek(pos32);
//     }
//     else
//     {
//         pointbuffer::PointBufferByteSize delta =
//             Support::getPointDataSize(m_reader.getLasHeader().getPointFormat());
//         m_istream.seekg(delta * count, std::ios::cur);
//     }
// #else
//     pointbuffer::PointBufferByteSize delta =
//         Support::getPointDataSize(m_reader.getLasHeader().getPointFormat());
//     m_istream.seekg(delta * count, std::ios::cur);
// #endif
    return count;
}


bool Reader::atEndImpl() const
{
    return getIndex() >= m_numPoints;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{

    getReader().log()->get(logDEBUG) << "readBufferImpl: " <<std::endl;    
    // PointDimensions cachedDimensions(data.getSchema(), m_reader.getName());
// #ifdef PDAL_HAVE_LASZIP
//     return m_reader.processBuffer(data, m_istream,
//         getStage().getNumPoints()-this->getIndex(), m_unzipper.get(),
//         m_zipPoint.get(), &cachedDimensions, m_read_buffer);
// #else
//     return m_reader.processBuffer(data, m_istream,
//         getStage().getNumPoints()-this->getIndex(), NULL, NULL,
//         &cachedDimensions, m_read_buffer);
// #endif
    data.setNumPoints(data.getCapacity());
}

} // sequential

namespace random
{

Reader::Reader(pdal::drivers::las2::Reader const& reader, PointBuffer& buffer,
        boost::uint32_t numPoints)
    : Base(reader) 
    , pdal::ReaderRandomIterator(buffer)
    , m_numPoints(numPoints)
{}


Reader::~Reader()
{}


void Reader::readBufferBeginImpl(PointBuffer& /* buffer*/)
{}


void Reader::readBeginImpl()
{
}

boost::uint64_t Reader::seekImpl(boost::uint64_t count)
{
// #ifdef PDAL_HAVE_LASZIP
//     if (m_unzipper)
//     {
//         const boost::uint32_t pos32 = Utils::safeconvert64to32(count);
//         m_unzipper->seek(pos32);
//     }
//     else
//     {
//         pointbuffer::PointBufferByteSize delta =
//             Support::getPointDataSize(m_reader.getLasHeader().getPointFormat());
//         m_istream.seekg(m_reader.getLasHeader().GetDataOffset() +
//             delta * count);
//     }
// #else
//     pointbuffer::PointBufferByteSize delta =
//         Support::getPointDataSize(m_reader.getLasHeader().getPointFormat());
//     m_istream.seekg(m_reader.getLasHeader().GetDataOffset() + delta * count);
// #endif
    return count;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    // PointDimensions cachedDimensions(data.getSchema(), m_reader.getName());
// #ifdef PDAL_HAVE_LASZIP
//     return m_reader.processBuffer(data, m_istream,
//         getStage().getNumPoints()-this->getIndex(), m_unzipper.get(),
//         m_zipPoint.get(), &cachedDimensions, m_read_buffer);
// #else
//     return m_reader.processBuffer(data, m_istream,
//          getStage().getNumPoints()-this->getIndex(), NULL, NULL,
//          &cachedDimensions, m_read_buffer);
// #endif
}

} // random
} // iterators

}
}
} // namespaces

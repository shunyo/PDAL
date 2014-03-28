/*****************************************************************************

    (c) 2013 Hobu, Inc. hobu.inc@gmail.com

    Author: Uday Verma uday dot karan at gmail dot com

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/



#ifndef INCLUDED_LAS_HPP
#define INCLUDED_LAS_HPP

#include <boost/noncopyable.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <stdexcept>

namespace las {

enum GeotiffKeyType
{
    Geotiff_KeyType_SHORT=1,
    Geotiff_KeyType_DOUBLE=2,
    Geotiff_KeyType_ASCII=3
};

typedef struct geokey
{
  uint16_t key_id;
  uint16_t tiff_tag_location;
  uint16_t count;
  uint16_t value_offset;
} geokey_struct;

typedef struct vlr
{
  uint16_t reserved;
  char user_id[16]; 
  uint16_t record_id;
  uint16_t record_length_after_header;
  char description[32];
  uint8_t* data;
} vlr_struct;

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


typedef struct header
{
  uint16_t file_source_ID;
  uint16_t global_encoding;
  uint32_t project_ID_GUID_data_1;
  uint16_t project_ID_GUID_data_2;
  uint16_t project_ID_GUID_data_3;
  char project_ID_GUID_data_4[8];
  uint8_t version_major;
  uint8_t version_minor;
  char system_identifier[32];
  char generating_software[32];
  uint16_t file_creation_day;
  uint16_t file_creation_year;
  uint16_t header_size;
  uint32_t offset_to_point_data;
  uint32_t number_of_variable_length_records;
  uint8_t point_data_format;
  uint16_t point_data_record_length;
  uint32_t number_of_point_records;
  uint32_t number_of_points_by_return[5];
  double scale[3];
  double offset[3];
  double maxs[3];
  double mins[3];

  // LAS 1.3 and higher only
  uint64_t start_of_waveform_data_packet_record;

  // LAS 1.4 and higher only
  uint64_t start_of_first_extended_variable_length_record;
  uint32_t number_of_extended_variable_length_records;
  uint64_t extended_number_of_point_records;
  uint64_t extended_number_of_points_by_return[15];

  // optional
  uint32_t user_data_in_header_size;
  uint8_t* user_data_in_header;

  // optional VLRs
  vlr_struct* vlrs;

  // optional
  uint32_t user_data_after_header_size;
  uint8_t* user_data_after_header;


  bool hasRGB() const
  {
      return ( (point_data_format == PointFormat2) 
          || (point_data_format == PointFormat3) 
          || (point_data_format == PointFormat5) 
          || (point_data_format == PointFormat7)
          || (point_data_format == PointFormat8)
              );
  }

  bool hasNIR() const
  {
      return ( point_data_format == PointFormat7)
          || ( point_data_format == PointFormat8)
          || ( point_data_format == PointFormat10)
              ;
  }

  bool hasTime() const
  {
      return (point_data_format == PointFormat1) 
          || (point_data_format == PointFormat3) 
          || (point_data_format == PointFormat4) 
          || (point_data_format == PointFormat5)
          || (point_data_format == PointFormat6)
          || (point_data_format == PointFormat7)
          || (point_data_format == PointFormat8)
          || (point_data_format == PointFormat9)
          || (point_data_format == PointFormat10)
              ;
  }

  bool hasWave() const
  {
      return (point_data_format == PointFormat4) 
          || (point_data_format == PointFormat5)
          || (point_data_format == PointFormat9)
          || (point_data_format == PointFormat10)
              ;
  }
} header_struct;

typedef struct laszip_point
{
  int32_t X;
  int32_t Y;
  int32_t Z;
  uint16_t intensity;
  uint8_t return_number : 3;
  uint8_t number_of_returns_of_given_pulse : 3;
  uint8_t scan_direction_flag : 1;
  uint8_t edge_of_flight_line : 1;
  uint8_t classification;
  int8_t scan_angle_rank;
  uint8_t user_data;
  uint16_t point_source_ID;

  double gps_time;
  uint16_t rgb[4];
  uint8_t wave_packet[29];

  // LAS 1.4 only
  uint8_t extended_point_type : 2;
  uint8_t extended_scanner_channel : 2;
  uint8_t extended_classification_flags : 4;
  uint8_t extended_classification;
  uint8_t extended_return_number : 4;
  uint8_t extended_number_of_returns_of_given_pulse : 4;
  int16_t extended_scan_angle;

  int32_t num_extra_bytes;
  uint8_t* extra_bytes;

} point_struct;

class las_file : public boost::noncopyable {
public:
	las_file() 
        
    : count_(0)
    , is_open_(false) 
    {
    }

	~las_file() {
		close();
	}
    
    bool readHeader()
    {
		void *addr = pregion_->get_address();
        size_t position(0);
		std::string magic((char *)addr, (char *)addr + 4);
		if (!boost::iequals(magic, "LASF")) {
			throw std::runtime_error("Not a las file");
		}
        position += 4;
        header.file_source_ID = readAs<uint16_t>(position);
        position += sizeof(header.file_source_ID);
        
        header.global_encoding = readAs<uint16_t>(position);
        position += sizeof(header.global_encoding);
        
        
        // head
		header.version_major = readAs<char>(24);
		header.version_minor = readAs<char>(25);

		if ((int)(header.version_major * 10 + header.version_minor) >= 15) {
			throw std::runtime_error("Only version 1.0-1.4 files are supported");
		}

		header.offset_to_point_data = readAs<unsigned int>(32*3);
		header.point_data_format = readAs<unsigned char>(32*3 + 8);
		header.point_data_record_length = readAs<unsigned short>(32*3 + 8 + 1);
		header.number_of_point_records = readAs<unsigned int>(32*3 + 11);

        // std::cerr << "points count: " << points_count_ << std::endl;

		size_t start = 32*3 + 35;
		readN(start, header.scale, 3); start += sizeof(double) * 3;
		readN(start, header.offset, 3); start += sizeof(double) * 3;

		header.maxs[0] = readAs<double>(start); header.mins[0] = readAs<double>(start + sizeof(double)); start += 2*sizeof(double);
		header.maxs[1] = readAs<double>(start); header.mins[1] = readAs<double>(start + sizeof(double)); start += 2*sizeof(double);
		header.maxs[2] = readAs<double>(start); header.mins[2] = readAs<double>(start + sizeof(double)); start += 2*sizeof(double);

        // std::cerr << "region size: " << pregion_->get_size() << std::endl;
        // std::cerr << "points offset: " << points_offset_ << std::endl;


		uint64_t diff = pregion_->get_size() - header.offset_to_point_data;

		if (diff % stride() != 0)
			throw std::runtime_error("Point record data size is inconsistent");

		if (diff / stride() != header.number_of_point_records)
			throw std::runtime_error("Point record count is inconsistent with computed point records size");
        
        updateMinsMaxes();
        return true;
        
    }
	void open(const std::string& filename, int offset = 0, int count = -1) {
		using namespace boost::interprocess;
        
        start_offset_ = offset;
        count_ = (size_t)count;

		pmapping_.reset(new file_mapping(filename.c_str(), read_only));
		pregion_.reset(new mapped_region(*pmapping_, read_only));
        

        
        readHeader();
        is_open_ = true;
	}

	size_t size() {
		return pregion_->get_size();
	}

	void *points_offset() {
		return (char *)pregion_->get_address() + header.offset_to_point_data + start_offset_;
	}

	void close() {
		pregion_.reset();
		pmapping_.reset();
        is_open_ = false;
	}

	size_t stride() 
    {
        return header.point_data_record_length;
    }

	size_t points_count() const 
    {
        return header.number_of_point_records;
	}
    
    bool is_open() { return is_open_; }
    
    inline double getX(size_t point)
    {
        char *position = (char*)points_offset() + stride()* point;
        
        int *xi = reinterpret_cast<int *>(position);
        
        double x = *xi * scale_[0] + offset_[0];
        
        return x;
    }

    inline double getY(size_t point)
    {
        char *position = (char *)points_offset() + stride() * point + sizeof(int);
        
        int *yi = reinterpret_cast<int *>(position);
        
        double y = *yi * scale_[1] + offset_[1];
        
        return y;
    }

    inline double getZ(size_t point)
    {
        char *position = (char *)points_offset() + stride() * point + sizeof(int) + sizeof(int);
        
        int *zi = reinterpret_cast<int *>(position);
        
        double z = *zi * scale_[2] + offset_[2];
        
        return z;
    }

    
private:
    void updateMinsMaxes() {
        if (start_offset_ == 0 && count_ == -1)
            return; // no update required if no subrange is requested
        
        int largest = std::numeric_limits<int>::max();
        int smallest = std::numeric_limits<int>::min();
        
        int n[3] = { largest, largest, largest };
        int x[3] = { smallest, smallest, smallest };
        
        char *ip = (char *)points_offset();
        for (size_t i = 0 ; i < points_count() ; i ++) {
            int *p = reinterpret_cast<int *>(ip);		
            for (int j = 0 ; j < 3 ; j ++) {
                n[j] = std::min(n[j], p[j]);
                x[j] = std::max(x[j], p[j]);
            }
            
            ip += stride();
        }
        
        for (int i = 0 ; i < 3 ; i++) {
            mins_[i] = n[i] * scale_[i] + offset_[i];
            maxs_[i] = x[i] * scale_[i] + offset_[i];
        }
    }

	template<typename T>
	T readAs(size_t offset) {
		return *(reinterpret_cast<T*>(((char*)pregion_->get_address() + offset)));
	}

	template<typename T>
	void readN(size_t offset, T* dest, size_t n) {
		char *buf = (char *)pregion_->get_address() + offset;
		for(size_t i = 0 ; i < n ; i ++) {
			dest[i] = *(reinterpret_cast<T*>(buf + sizeof(T) * i));
		}
	}

public:
    header_struct header;
    
private:
	boost::shared_ptr<boost::interprocess::file_mapping> pmapping_;
	boost::shared_ptr<boost::interprocess::mapped_region> pregion_;

    unsigned int start_offset_;
    int64_t count_;

	double scale_[3], offset_[3], mins_[3], maxs_[3];
    
    bool is_open_;

};

} // namespace las

#endif // __HEXER_LAS_H__

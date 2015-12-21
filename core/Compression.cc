// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Compression.h"

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <sstream>
#include <string>


namespace dv { namespace comp {


std::string compress(const std::string &toCompress, int compressionLevel) {
  std::stringstream dataToCompress;
  dataToCompress << toCompress;

  boost::iostreams::filtering_streambuf<boost::iostreams::input> filter;
  boost::iostreams::zlib_params params(compressionLevel);

  filter.push(boost::iostreams::zlib_compressor(params));
  filter.push(dataToCompress);

  std::stringstream compressedData;

  boost::iostreams::copy(filter, compressedData);

  return compressedData.str();
}

std::string decompress(const std::string &toDecompress, int compressionLevel) {
  std::stringstream compressed, decompressed;

  compressed << toDecompress;

  boost::iostreams::filtering_streambuf<boost::iostreams::input> filter;
  boost::iostreams::zlib_params params(compressionLevel);

  filter.push(boost::iostreams::zlib_decompressor(params));
  filter.push(compressed);

  boost::iostreams::copy(filter, decompressed);

  return decompressed.str();
}

}}
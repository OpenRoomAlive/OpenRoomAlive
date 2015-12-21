// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <string>

namespace dv { namespace comp {

/**
 * Compresses the string using the specified compression level.
 */
std::string compress(const std::string &toCompress, int compressionLevel = 6);

/**
 * Decompresses the string.
 */
std::string decompress(const std::string &compressed, int compressionLevel = 6);

}}
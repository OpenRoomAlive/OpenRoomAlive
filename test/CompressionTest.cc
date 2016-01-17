// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <random>
#include <iostream>

#include <gtest/gtest.h>

#include "core/Compression.h"

using namespace dv;

constexpr auto kSeed = 12769;
constexpr auto kAlphabetSize = 26;
constexpr auto kSmallLettersStart = 97;

/**
 * Check that when a small word is compressed and subsequently decompressed,
 * it gives the initial value.
 */
TEST(CompressionTest, EqualCompressDecompressSmall) {
  const std::string word = "testWord";

  const auto compressed = comp::compress(word);
  const auto decompressed = comp::decompress(compressed);

  ASSERT_EQ(decompressed, word);
}

/**
 * Check that when a large work is compressed and subsequently decompressed,
 * it gives the initial value.
 */
TEST(CompressionTest, EqualCompressDecompressLarge) {
  std::string word;
  std::mt19937 generator(kSeed);

  for (size_t i = 0; i < 1000000; i++) {
    word += (char) (kSmallLettersStart + generator() % kAlphabetSize);
  }

  const auto compressed = comp::compress(word);
  const auto decompressed = comp::decompress(compressed);

  // Check that compression was performed.
  ASSERT_LT(compressed.size(), word.size());
  ASSERT_EQ(decompressed, word);
}
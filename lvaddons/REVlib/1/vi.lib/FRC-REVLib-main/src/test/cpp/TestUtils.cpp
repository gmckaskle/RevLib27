/*
 * Copyright (c) 2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "TestUtils.h"

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

#include "gtest/gtest.h"

////////////////////////////////////////////////

void ValidateFlattenedSubString(std::string& flattened, std::string&& sub) {
    auto pos = flattened.find(sub);
    EXPECT_NE(pos, std::string::npos) << sub << " not found in " << flattened;
    if (pos != std::string::npos) {
        flattened.erase(pos, sub.size());
    }
}

////////////////////////////////////////////////

namespace {  // unnamed

// https://stackoverflow.com/questions/16749069/c-split-string-by-regex/28142357#28142357
std::vector<std::string> SplitStrings(const std::string& s,
                                      const std::regex& sep_regex = std::regex{
                                          "\\n+"}) {
    std::sregex_token_iterator iter(s.begin(), s.end(), sep_regex, -1);
    std::sregex_token_iterator end;
    return {iter, end};
}

}  // namespace

void ValidateFlattenedStrings(const std::string& fs1, const std::string& fs2) {
    auto results1 = SplitStrings(fs1);
    std::sort(results1.begin(), results1.end());

    auto results2 = SplitStrings(fs2);
    std::sort(results2.begin(), results2.end());

    EXPECT_EQ(results1, results2);
}

////////////////////////////////////////////////

void ValidateFlattenedStringsExcludingIds(const std::string& fs1,
                                          const std::string& fs2) {
    auto removeIds = [](std::string& str) { str = str.substr(str.find(",")); };

    auto results1 = SplitStrings(fs1);
    std::sort(results1.begin(), results1.end());
    std::for_each(results1.begin(), results1.end(), removeIds);

    auto results2 = SplitStrings(fs2);
    std::sort(results2.begin(), results2.end());
    std::for_each(results2.begin(), results2.end(), removeIds);

    EXPECT_EQ(results1, results2);
}

////////////////////////////////////////////////

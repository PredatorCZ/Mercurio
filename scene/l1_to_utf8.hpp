/*  Copyright: 2015 Eliezio Oliveira <ebo@pobox.com>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef L1U8RECODE_H
#define L1U8RECODE_H

#include <cstdint>
#include <cstdlib>

class KmpSearch;

class L1U8Recode {
public:
    L1U8Recode();

    L1U8Recode(const char *beginText, const char *endText);

    L1U8Recode(const uint8_t *beginText, size_t beginTextLen, const uint8_t *endText, size_t endTextLen);

    ~L1U8Recode();

    void init();

    size_t translate(const uint8_t *input, size_t inputLen, uint8_t * const output);

    size_t finish(uint8_t *output);

    size_t getChangesCount() const {
        return changesCount;
    }

private:
    KmpSearch * kmpSearches[2];
    bool        inText;
    uint8_t     u8Esc;
    size_t      changesCount;
};

#endif
